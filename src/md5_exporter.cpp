#include "md5_exporter.h"
#include <cstdio>
#include <cmath>

namespace fbx2md5 {

// ─── md5mesh writer ─────────────────────────────────────────────────────────────

bool WriteMD5Mesh(const std::string& filepath, const Skeleton& skeleton,
                  const std::vector<Mesh>& meshes, const Config& config)
{
    FILE* f = fopen(filepath.c_str(), "w");
    if (!f) {
        fprintf(stderr, "Error: Cannot open '%s' for writing\n", filepath.c_str());
        return false;
    }

    int version = config.md5Version;

    fprintf(f, "MD5Version %d\n", version);
    fprintf(f, "commandline \"Exported by fbx2md5\"\n\n");

    fprintf(f, "numJoints %d\n", (int)skeleton.joints.size());
    fprintf(f, "numMeshes %d\n\n", (int)meshes.size());

    // ─── Joints (world space) ───────────────────────────────────────────
    fprintf(f, "joints {\n");
    for (int i = 0; i < (int)skeleton.joints.size(); i++) {
        const Joint& j = skeleton.joints[i];

        // Ensure quaternion has positive w before writing
        Quat q = j.worldRot.Normalized().EnsureW();

        fprintf(f, "\t\"%s\"\t%d ( %.10f %.10f %.10f ) ( %.10f %.10f %.10f )\n",
                j.name.c_str(), j.parentIndex,
                j.worldPos.x, j.worldPos.y, j.worldPos.z,
                q.x, q.y, q.z);
    }
    fprintf(f, "}\n\n");

    // ─── Meshes ─────────────────────────────────────────────────────────
    for (int mi = 0; mi < (int)meshes.size(); mi++) {
        const Mesh& mesh = meshes[mi];

        fprintf(f, "mesh {\n");
        fprintf(f, "\tshader \"%s\"\n\n", mesh.materialName.c_str());

        // Build flat weight list (all weights for all vertices, sequentially)
        struct FlatWeight {
            int jointIndex;
            double weight;
            Vec3 offset;
        };
        std::vector<FlatWeight> flatWeights;
        std::vector<int> firstWeight;   // per vertex
        std::vector<int> weightCount;   // per vertex

        for (auto& vert : mesh.vertices) {
            firstWeight.push_back((int)flatWeights.size());
            weightCount.push_back((int)vert.weights.size());
            for (auto& w : vert.weights) {
                flatWeights.push_back({w.jointIndex, w.weight, w.offset});
            }
        }

        // Vertices
        fprintf(f, "\tnumverts %d\n", (int)mesh.vertices.size());
        for (int vi = 0; vi < (int)mesh.vertices.size(); vi++) {
            fprintf(f, "\tvert %d ( %.10f %.10f ) %d %d\n",
                    vi, mesh.vertices[vi].uv.x, mesh.vertices[vi].uv.y,
                    firstWeight[vi], weightCount[vi]);
        }
        fprintf(f, "\n");

        // Triangles
        fprintf(f, "\tnumtris %d\n", (int)mesh.triangles.size());
        for (int ti = 0; ti < (int)mesh.triangles.size(); ti++) {
            const Triangle& tri = mesh.triangles[ti];
            fprintf(f, "\ttri %d %d %d %d\n", ti,
                    tri.indices[0], tri.indices[1], tri.indices[2]);
        }
        fprintf(f, "\n");

        // Weights
        fprintf(f, "\tnumweights %d\n", (int)flatWeights.size());
        for (int wi = 0; wi < (int)flatWeights.size(); wi++) {
            auto& w = flatWeights[wi];
            fprintf(f, "\tweight %d %d %.10f ( %.10f %.10f %.10f )\n",
                    wi, w.jointIndex, w.weight,
                    w.offset.x, w.offset.y, w.offset.z);
        }

        // V11: optional normals block
        if (version >= 11) {
            fprintf(f, "\n\tnormals {\n");
            fprintf(f, "\t\tnumverts %d\n", (int)mesh.vertices.size());
            for (int vi = 0; vi < (int)mesh.vertices.size(); vi++) {
                const Vec3& n = mesh.vertices[vi].normal;
                fprintf(f, "\t\tnormal %d ( %.10f %.10f %.10f )\n",
                        vi, n.x, n.y, n.z);
            }
            fprintf(f, "\t}\n");
        }

        fprintf(f, "}\n\n");
    }

    fclose(f);
    return true;
}

// ─── md5anim writer ─────────────────────────────────────────────────────────────

bool WriteMD5Anim(const std::string& filepath, const Skeleton& skeleton,
                  const AnimData& anim, const Config& config)
{
    FILE* f = fopen(filepath.c_str(), "w");
    if (!f) {
        fprintf(stderr, "Error: Cannot open '%s' for writing\n", filepath.c_str());
        return false;
    }

    // md5anim is always version 10 (no format changes for v11)
    fprintf(f, "MD5Version 10\n");
    fprintf(f, "commandline \"Exported by fbx2md5\"\n\n");

    fprintf(f, "numFrames %d\n", anim.frameCount);
    fprintf(f, "numJoints %d\n", (int)skeleton.joints.size());
    fprintf(f, "frameRate %d\n", anim.frameRate);
    fprintf(f, "numAnimatedComponents %d\n\n", anim.numAnimatedComponents);

    // ─── Hierarchy ──────────────────────────────────────────────────────
    fprintf(f, "hierarchy {\n");
    for (int i = 0; i < (int)skeleton.joints.size(); i++) {
        const Joint& j = skeleton.joints[i];
        const JointAnimInfo& info = anim.jointInfo[i];
        fprintf(f, "\t\"%s\"\t%d %d %d\n",
                j.name.c_str(), j.parentIndex, info.flags, info.startIndex);
    }
    fprintf(f, "}\n\n");

    // ─── Bounds ─────────────────────────────────────────────────────────
    fprintf(f, "bounds {\n");
    for (int f2 = 0; f2 < anim.frameCount; f2++) {
        const AnimBounds& b = anim.frameBounds[f2];
        fprintf(f, "\t( %.10f %.10f %.10f ) ( %.10f %.10f %.10f )\n",
                b.mins.x, b.mins.y, b.mins.z,
                b.maxs.x, b.maxs.y, b.maxs.z);
    }
    fprintf(f, "}\n\n");

    // ─── Baseframe ──────────────────────────────────────────────────────
    fprintf(f, "baseframe {\n");
    for (int i = 0; i < (int)skeleton.joints.size(); i++) {
        const JointFrame& bf = anim.jointInfo[i].baseframe;
        Quat q = bf.rot.EnsureW();
        fprintf(f, "\t( %.10f %.10f %.10f ) ( %.10f %.10f %.10f )\n",
                bf.pos.x, bf.pos.y, bf.pos.z,
                q.x, q.y, q.z);
    }
    fprintf(f, "}\n\n");

    // ─── Frames ─────────────────────────────────────────────────────────
    for (int fi = 0; fi < anim.frameCount; fi++) {
        fprintf(f, "frame %d {\n", fi);
        const auto& components = anim.frameComponents[fi];
        // Write 6 values per line for readability
        for (int ci = 0; ci < (int)components.size(); ci++) {
            if (ci % 6 == 0) fprintf(f, "\t");
            fprintf(f, "%.10f ", components[ci]);
            if (ci % 6 == 5 || ci == (int)components.size() - 1)
                fprintf(f, "\n");
        }
        fprintf(f, "}\n\n");
    }

    fclose(f);
    return true;
}

} // namespace fbx2md5
