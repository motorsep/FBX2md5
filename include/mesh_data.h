#pragma once

#include "math_types.h"
#include "skeleton.h"
#include <string>
#include <vector>
#include <fbxsdk.h>

namespace fbx2md5 {

struct VertexWeight {
    int    jointIndex = 0;
    double weight     = 0;
    Vec3   offset;          // weight offset in joint-local space
};

struct Vertex {
    Vec3  position;         // bind-pose world position (id Tech space)
    Vec2  uv;
    Vec3  normal;           // bind-pose normal (for v11)
    std::vector<VertexWeight> weights;
};

struct Triangle {
    int indices[3] = {};
};

struct Mesh {
    std::string  materialName;   // id Tech material path
    std::vector<Vertex>   vertices;
    std::vector<Triangle> triangles;
};

// Extract all skinned meshes from FBX scene, split by material.
// Triangulates, resolves skin clusters, computes weight offsets.
bool ExtractMeshes(FbxScene* scene, const Skeleton& skeleton,
                   const Config& config, std::vector<Mesh>& outMeshes);

// Reduce vertex weights to MAX_VERTEX_WEIGHTS, renormalize.
void ReduceWeights(Vertex& vert);

} // namespace fbx2md5
