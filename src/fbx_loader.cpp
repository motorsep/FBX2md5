#include "fbx_loader.h"
#include <cstdio>

namespace fbx2md5 {

void FbxContext::Destroy()
{
    if (importer) { importer->Destroy(); importer = nullptr; }
    if (scene)    { scene->Destroy();    scene = nullptr; }
    if (manager)  { manager->Destroy();  manager = nullptr; }
}

bool LoadFbxScene(const std::string& filepath, FbxContext& ctx, bool verbose)
{
    ctx.manager = FbxManager::Create();
    if (!ctx.manager) {
        fprintf(stderr, "Error: Failed to create FBX Manager\n");
        return false;
    }

    FbxIOSettings* ios = FbxIOSettings::Create(ctx.manager, IOSROOT);
    ctx.manager->SetIOSettings(ios);

    ctx.importer = FbxImporter::Create(ctx.manager, "");
    if (!ctx.importer->Initialize(filepath.c_str(), -1, ctx.manager->GetIOSettings())) {
        fprintf(stderr, "Error: Failed to initialize FBX importer: %s\n",
                ctx.importer->GetStatus().GetErrorString());
        return false;
    }

    ctx.scene = FbxScene::Create(ctx.manager, "scene");
    if (!ctx.importer->Import(ctx.scene)) {
        fprintf(stderr, "Error: Failed to import FBX scene: %s\n",
                ctx.importer->GetStatus().GetErrorString());
        return false;
    }

    if (verbose) {
        int major, minor, revision;
        ctx.importer->GetFileVersion(major, minor, revision);
        printf("  FBX file version: %d.%d.%d\n", major, minor, revision);
    }

    // Triangulate the entire scene upfront
    FbxGeometryConverter converter(ctx.manager);
    converter.Triangulate(ctx.scene, true);

    if (verbose) {
        printf("  Scene triangulated\n");
    }

    return true;
}

} // namespace fbx2md5
