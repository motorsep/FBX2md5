#include "fbx2md5.h"
#include "fbx_loader.h"
#include "skeleton.h"
#include "mesh_data.h"
#include "anim_data.h"
#include "md5_exporter.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

static void PrintUsage(const char* progName)
{
    printf("Usage: %s <input.fbx> [options]\n\n", progName);
    printf("Output modes:\n");
    printf("  (default)        Convert to md5mesh + md5anim\n");
    printf("  -fbx             Convert to FBX (apply scale/rotate, re-export)\n");
    printf("\nOptions:\n");
    printf("  -md5v10          Output MD5 version 10 (no vertex normals) [default]\n");
    printf("  -md5v11          Output MD5 version 11 (vertex normals for MikkTSpace)\n");
    printf("  -armatureFix     Remove Blender's extra Armature root bone\n");
    printf("  -scale <value>   Scale factor (default: 1.0)\n");
    printf("  -rotate <X> <Y> <Z>  Rotation in degrees around each axis\n");
    printf("  -outdir <path>   Output directory (default: same as input file)\n");
    printf("  -name <n>        Override base name for output files\n");
    printf("  -epsilon <value> Animation component threshold (default: 0.0001)\n");
    printf("  -verbose         Print detailed progress information\n");
    printf("  -help            Show this help\n");
    printf("\nExamples:\n");
    printf("  %s character.fbx -md5v11 -armatureFix\n", progName);
    printf("  %s character.fbx -fbx -scale 0.01 -rotate 0 0 90\n", progName);
    printf("  %s character.fbx -fbx -rotate -90 0 0\n", progName);
}

static bool ParseArgs(int argc, char* argv[], fbx2md5::Config& config)
{
    if (argc < 2) {
        PrintUsage(argv[0]);
        return false;
    }

    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];

        if (arg[0] != '-') {
            if (config.inputFile.empty()) {
                config.inputFile = arg;
            } else {
                fprintf(stderr, "Error: Unexpected argument '%s'\n", arg);
                return false;
            }
            continue;
        }

        if (strcmp(arg, "-fbx") == 0) {
            config.fbxOutput = true;
        } else if (strcmp(arg, "-md5v10") == 0) {
            config.md5Version = 10;
        } else if (strcmp(arg, "-md5v11") == 0) {
            config.md5Version = 11;
        } else if (strcmp(arg, "-armatureFix") == 0) {
            config.armatureFix = true;
        } else if (strcmp(arg, "-scale") == 0) {
            if (++i >= argc) { fprintf(stderr, "Error: -scale requires a value\n"); return false; }
            config.scaleFactor = atof(argv[i]);
        } else if (strcmp(arg, "-rotate") == 0) {
            if (i + 3 >= argc) { fprintf(stderr, "Error: -rotate requires X Y Z values\n"); return false; }
            config.rotateX = atof(argv[++i]);
            config.rotateY = atof(argv[++i]);
            config.rotateZ = atof(argv[++i]);
        } else if (strcmp(arg, "-outdir") == 0) {
            if (++i >= argc) { fprintf(stderr, "Error: -outdir requires a path\n"); return false; }
            config.outputDir = argv[i];
        } else if (strcmp(arg, "-name") == 0) {
            if (++i >= argc) { fprintf(stderr, "Error: -name requires a value\n"); return false; }
            config.meshName = argv[i];
        } else if (strcmp(arg, "-epsilon") == 0) {
            if (++i >= argc) { fprintf(stderr, "Error: -epsilon requires a value\n"); return false; }
            config.animEpsilon = atof(argv[i]);
        } else if (strcmp(arg, "-verbose") == 0) {
            config.verbose = true;
        } else if (strcmp(arg, "-help") == 0 || strcmp(arg, "--help") == 0) {
            PrintUsage(argv[0]);
            return false;
        } else {
            fprintf(stderr, "Error: Unknown option '%s'\n", arg);
            PrintUsage(argv[0]);
            return false;
        }
    }

    if (config.inputFile.empty()) {
        fprintf(stderr, "Error: No input file specified\n");
        PrintUsage(argv[0]);
        return false;
    }

    return true;
}

static std::string SanitizeName(const std::string& name)
{
    std::string result;
    result.reserve(name.size());
    for (char c : name) {
        if (c == ' ' || c == '|' || c == '\\' || c == '/' || c == ':' ||
            c == '*' || c == '?' || c == '"' || c == '<' || c == '>') {
            result += '_';
        } else {
            result += c;
        }
    }
    return result;
}

// ─── FBX-to-FBX export ─────────────────────────────────────────────────────────

static bool ExportFbx(const fbx2md5::Config& config, const std::string& outputPath)
{
    FbxManager* manager = FbxManager::Create();
    if (!manager) {
        fprintf(stderr, "Error: Failed to create FBX SDK manager\n");
        return false;
    }

    FbxIOSettings* ios = FbxIOSettings::Create(manager, IOSROOT);
    manager->SetIOSettings(ios);

    // Import
    FbxImporter* importer = FbxImporter::Create(manager, "");
    if (!importer->Initialize(config.inputFile.c_str(), -1, manager->GetIOSettings())) {
        fprintf(stderr, "Error: Failed to initialize FBX importer: %s\n",
                importer->GetStatus().GetErrorString());
        manager->Destroy();
        return false;
    }

    FbxScene* scene = FbxScene::Create(manager, "scene");
    if (!importer->Import(scene)) {
        fprintf(stderr, "Error: Failed to import FBX scene\n");
        importer->Destroy();
        manager->Destroy();
        return false;
    }
    importer->Destroy();

    // Triangulate
    FbxGeometryConverter geomConverter(manager);
    geomConverter.Triangulate(scene, true);

    // Apply scale and rotation to the scene root node
    FbxNode* rootNode = scene->GetRootNode();

    if (config.scaleFactor != 1.0) {
        FbxVector4 curScale = rootNode->LclScaling.Get();
        rootNode->LclScaling.Set(FbxVector4(
            curScale[0] * config.scaleFactor,
            curScale[1] * config.scaleFactor,
            curScale[2] * config.scaleFactor));
        printf("  Applied scale: %.6f\n", config.scaleFactor);
    }

    bool hasRotation = (config.rotateX != 0.0 || config.rotateY != 0.0 || config.rotateZ != 0.0);
    if (hasRotation) {
        FbxVector4 curRotation = rootNode->LclRotation.Get();
        rootNode->LclRotation.Set(FbxVector4(
            curRotation[0] + config.rotateX,
            curRotation[1] + config.rotateY,
            curRotation[2] + config.rotateZ));
        printf("  Applied rotation: (%.2f, %.2f, %.2f) degrees\n",
               config.rotateX, config.rotateY, config.rotateZ);
    }

    // Export as binary FBX
    FbxExporter* exporter = FbxExporter::Create(manager, "");
    int formatIndex = manager->GetIOPluginRegistry()->GetNativeWriterFormat();

    if (!exporter->Initialize(outputPath.c_str(), formatIndex, manager->GetIOSettings())) {
        fprintf(stderr, "Error: Failed to initialize FBX exporter: %s\n",
                exporter->GetStatus().GetErrorString());
        manager->Destroy();
        return false;
    }

    bool success = exporter->Export(scene);
    if (!success) {
        fprintf(stderr, "Error: Failed to export FBX: %s\n",
                exporter->GetStatus().GetErrorString());
    }

    exporter->Destroy();
    manager->Destroy();

    return success;
}

// ─── main ───────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    printf("fbx2md5 - FBX to MD5 Model Converter\n");
    printf("  For id Tech 4 / StormEngine2\n\n");

    fbx2md5::Config config;
    if (!ParseArgs(argc, argv, config))
        return fbx2md5::EXIT_BAD_ARGS;

    fs::path inputPath(config.inputFile);
    std::string baseName = config.meshName.empty()
        ? inputPath.stem().string()
        : config.meshName;

    if (config.outputDir == ".") {
        config.outputDir = inputPath.parent_path().string();
        if (config.outputDir.empty()) config.outputDir = ".";
    }

    fs::create_directories(config.outputDir);

    printf("Input:   %s\n", config.inputFile.c_str());

    // ═══ FBX OUTPUT MODE ═══════════════════════════════════════════════════
    if (config.fbxOutput) {
        std::string outPath = config.outputDir + "/" + baseName + "_out.fbx";
        printf("Output:  %s (FBX re-export)\n", outPath.c_str());
        if (config.scaleFactor != 1.0)
            printf("Scale:   %.6f\n", config.scaleFactor);
        if (config.rotateX != 0.0 || config.rotateY != 0.0 || config.rotateZ != 0.0)
            printf("Rotate:  (%.2f, %.2f, %.2f)\n", config.rotateX, config.rotateY, config.rotateZ);
        printf("\n");

        if (!ExportFbx(config, outPath)) {
            return fbx2md5::EXIT_WRITE_FAIL;
        }

        printf("\nFBX export complete: %s\n", outPath.c_str());
        return fbx2md5::EXIT_OK;
    }

    // ═══ MD5 OUTPUT MODE ═══════════════════════════════════════════════════
    printf("Output:  %s/%s.md5mesh (v%d)\n", config.outputDir.c_str(),
           baseName.c_str(), config.md5Version);
    if (config.armatureFix) printf("Armature fix: enabled\n");
    if (config.scaleFactor != 1.0) printf("User scale factor: %.4f\n", config.scaleFactor);
    printf("\n");

    printf("Loading FBX...\n");
    fbx2md5::FbxContext fbxCtx;
    if (!fbx2md5::LoadFbxScene(config.inputFile, fbxCtx, config.verbose)) {
        return fbx2md5::EXIT_FBX_LOAD_FAIL;
    }

    printf("Extracting skeleton...\n");
    fbx2md5::Skeleton skeleton;
    if (!fbx2md5::ExtractSkeleton(fbxCtx.scene, config, skeleton)) {
        return fbx2md5::EXIT_NO_SKELETON;
    }

    printf("Extracting meshes...\n");
    std::vector<fbx2md5::Mesh> meshes;
    if (!fbx2md5::ExtractMeshes(fbxCtx.scene, skeleton, config, meshes)) {
        return fbx2md5::EXIT_NO_MESH;
    }

    std::string meshPath = config.outputDir + "/" + baseName + ".md5mesh";
    printf("Writing %s...\n", meshPath.c_str());
    if (!fbx2md5::WriteMD5Mesh(meshPath, skeleton, meshes, config)) {
        return fbx2md5::EXIT_WRITE_FAIL;
    }
    printf("  OK\n");

    printf("Extracting animations...\n");
    std::vector<fbx2md5::AnimData> anims;
    if (!fbx2md5::ExtractAnimations(fbxCtx.scene, skeleton, meshes, config, anims)) {
        fprintf(stderr, "Warning: Animation extraction failed\n");
    }

    for (auto& anim : anims) {
        std::string animName = SanitizeName(anim.name);
        std::string animPath = config.outputDir + "/" + baseName + "_" + animName + ".md5anim";
        printf("Writing %s...\n", animPath.c_str());
        if (!fbx2md5::WriteMD5Anim(animPath, skeleton, anim, config)) {
            fprintf(stderr, "Warning: Failed to write animation '%s'\n", anim.name.c_str());
        } else {
            printf("  OK (%d frames @ %d fps)\n", anim.frameCount, anim.frameRate);
        }
    }

    printf("\nConversion complete.\n");
    printf("  md5mesh: %s\n", meshPath.c_str());
    printf("  Animations: %d\n", (int)anims.size());

    return fbx2md5::EXIT_OK;
}
