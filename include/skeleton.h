#pragma once

#include "math_types.h"
#include "fbx2md5.h"
#include <string>
#include <vector>
#include <fbxsdk.h>

namespace fbx2md5 {

struct Joint {
    std::string name;
    int         parentIndex = -1;    // -1 = root
    FbxNode*    fbxNode = nullptr;   // back-reference for anim sampling

    // World-space bind pose in id Tech coordinates (for md5mesh)
    Vec3 worldPos;
    Quat worldRot;

    // Local-space bind pose (parent-relative, for md5anim baseframe)
    Vec3 localPos;
    Quat localRot;
};

struct Skeleton {
    std::vector<Joint> joints;

    // Auto-detected FBX scale factor (e.g. 100 for Blender exports).
    // Positions are divided by this during conversion.
    double fbxScale = 1.0;
    double scaleFactor = 1.0;

    int FindJointByName(const std::string& name) const;
    int FindJointByNode(FbxNode* node) const;

    // Compute local-space transforms from world-space ones
    void ComputeLocalFromWorld();
};

// Extract skeleton from FBX scene.
// If armatureFix is true, skip the top-level Armature node that Blender inserts.
bool ExtractSkeleton(FbxScene* scene, const Config& config, Skeleton& outSkeleton);

} // namespace fbx2md5
