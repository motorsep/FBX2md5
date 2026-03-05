#pragma once

#include "math_types.h"
#include "skeleton.h"
#include "mesh_data.h"
#include <string>
#include <vector>
#include <fbxsdk.h>

namespace fbx2md5 {

// Per-joint per-frame data (local space)
struct JointFrame {
    Vec3 pos;
    Quat rot;   // only x,y,z stored in md5anim; w reconstructed
};

// md5anim component flags (which components are animated)
enum AnimFlags : int {
    ANIM_TX = 1 << 0,
    ANIM_TY = 1 << 1,
    ANIM_TZ = 1 << 2,
    ANIM_QX = 1 << 3,
    ANIM_QY = 1 << 4,
    ANIM_QZ = 1 << 5,
};

struct JointAnimInfo {
    int    flags = 0;           // bitmask of AnimFlags
    int    startIndex = 0;      // index into per-frame component array
    JointFrame baseframe;       // reference pose for this anim
};

struct AnimBounds {
    Vec3 mins;
    Vec3 maxs;
};

struct AnimData {
    std::string name;
    int   frameCount = 0;
    int   frameRate  = 24;
    int   numAnimatedComponents = 0;

    std::vector<JointAnimInfo> jointInfo;               // per joint
    std::vector<std::vector<double>> frameComponents;   // [frame][component]
    std::vector<AnimBounds> frameBounds;                 // per frame AABB
};

// Extract all animation stacks/takes from FBX.
// Samples at the native frame rate, bakes constraints/IK.
bool ExtractAnimations(FbxScene* scene, const Skeleton& skeleton,
                       const std::vector<Mesh>& meshes,
                       const Config& config,
                       std::vector<AnimData>& outAnims);

// Compute per-frame world-space AABB by evaluating skinned vertices
AnimBounds ComputeFrameBounds(const Skeleton& skeleton,
                              const std::vector<Mesh>& meshes,
                              const std::vector<JointFrame>& worldFrameJoints);

} // namespace fbx2md5
