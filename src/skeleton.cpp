#include "skeleton.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <map>

namespace fbx2md5 {

// ─── FBX → id Tech coordinate conversion ─────────────────────────────────────
//
// Verified numerically against imp.md5mesh ↔ d3_imp_slash1.fbx:
//   Position:   id = ( fbx_x/S, -fbx_z/S,  fbx_y/S )
//   Quaternion: id = ( fbx_qx,  -fbx_qz,   fbx_qy,  fbx_qw )
//   where S = scale factor (auto-detected from TransformLink column magnitude)

static Vec3 FbxPosToId(const FbxVector4& t, double scale)
{
    double invS = 1.0 / scale;
    return { t[0] * invS, -t[2] * invS, t[1] * invS };
}

static Quat FbxQuatToId(const FbxQuaternion& q)
{
    // id_q = conj(fbx_q) * conj(Q_90x)
    // Strips the armature pre-rotation from TransformLink global quaternions.
    // NOT multiplicative (old(A*B) ≠ old(A)*old(B)), but the world→local
    // decomposition still roundtrips perfectly since:
    //   local = conj(old(parent)) * old(child)
    //   reconstruct: old(parent) * local = old(child)  ✓
    static const double S = 0.70710678118654752;
    double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    Quat r(
        -S * (qw + qx),
         S * (qz - qy),
        -S * (qy + qz),
         S * (qw - qx)
    );
    return r.Normalized().EnsureW();
}

static Vec3 FbxDirToId(const FbxVector4& d)
{
    // Direction (no scale): same axis remap
    return Vec3( d[0], -d[2], d[1] );
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

int Skeleton::FindJointByName(const std::string& name) const
{
    for (int i = 0; i < (int)joints.size(); i++)
        if (joints[i].name == name) return i;
    return -1;
}

int Skeleton::FindJointByNode(FbxNode* node) const
{
    for (int i = 0; i < (int)joints.size(); i++)
        if (joints[i].fbxNode == node) return i;
    return -1;
}

void Skeleton::ComputeLocalFromWorld()
{
    for (int i = 0; i < (int)joints.size(); i++) {
        Joint& j = joints[i];
        if (j.parentIndex < 0) {
            j.localPos = j.worldPos;
            j.localRot = j.worldRot;
        } else {
            const Joint& parent = joints[j.parentIndex];

            // Position: decompose from id Tech world
            j.localPos = parent.worldRot.Conjugate().RotatePoint(j.worldPos - parent.worldPos);

            // Quaternion: decompose in FBX space, then similarity transform
            if (j.fbxNode && parent.fbxNode) {
                FbxAMatrix parentBind = parent.fbxNode->EvaluateGlobalTransform(FBXSDK_TIME_INFINITE);
                FbxAMatrix childBind = j.fbxNode->EvaluateGlobalTransform(FBXSDK_TIME_INFINITE);
                FbxAMatrix localFbx = parentBind.Inverse() * childBind;
                FbxQuaternion lq = localFbx.GetQ();
                j.localRot = Quat(lq[0], lq[1], lq[2], lq[3]).Normalized().EnsureW();
            } else {
                j.localRot = (parent.worldRot.Conjugate() * j.worldRot).Normalized().EnsureW();
            }
        }
    }
}

// ─── Bone detection ──────────────────────────────────────────────────────────

static bool IsBoneNode(FbxNode* node)
{
    FbxNodeAttribute* attr = node->GetNodeAttribute();
    if (!attr) return false;
    return attr->GetAttributeType() == FbxNodeAttribute::eSkeleton;
}

static bool IsArmatureRoot(FbxNode* node)
{
    FbxNodeAttribute* attr = node->GetNodeAttribute();
    if (!attr) return false;
    if (attr->GetAttributeType() != FbxNodeAttribute::eSkeleton) return false;

    FbxSkeleton* skel = (FbxSkeleton*)attr;
    if (skel->GetSkeletonType() != FbxSkeleton::eRoot) return false;

    int boneChildren = 0;
    for (int i = 0; i < node->GetChildCount(); i++) {
        if (IsBoneNode(node->GetChild(i))) boneChildren++;
    }
    return boneChildren > 0;
}

// ─── Collect bind pose from skin clusters ─────────────────────────────────────

static void CollectClusterBindPoses(FbxScene* scene,
                                    std::map<FbxNode*, FbxAMatrix>& outBindPoses,
                                    bool verbose)
{
    int nodeCount = scene->GetNodeCount();
    for (int ni = 0; ni < nodeCount; ni++) {
        FbxNode* node = scene->GetNode(ni);
        if (!node) continue;

        FbxMesh* mesh = node->GetMesh();
        if (!mesh) continue;

        int skinCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
        for (int si = 0; si < skinCount; si++) {
            FbxSkin* skin = (FbxSkin*)mesh->GetDeformer(si, FbxDeformer::eSkin);
            if (!skin) continue;

            int clusterCount = skin->GetClusterCount();
            for (int ci = 0; ci < clusterCount; ci++) {
                FbxCluster* cluster = skin->GetCluster(ci);
                FbxNode* linkNode = cluster->GetLink();
                if (!linkNode) continue;

                FbxAMatrix bindPoseMat;
                cluster->GetTransformLinkMatrix(bindPoseMat);
                outBindPoses[linkNode] = bindPoseMat;
            }
        }
    }
}

// ─── Auto-detect scale factor from TransformLink column magnitudes ────────────

static double DetectFbxScale(const std::map<FbxNode*, FbxAMatrix>& bindPoses, bool verbose)
{
    // The TransformLink matrix contains rotation * scale.
    // For Blender exports, each column of the rotation 3x3 has magnitude ~100.
    // For Maya/other exports, magnitude ~1.

    if (bindPoses.empty()) return 1.0;

    // Use first cluster to detect scale
    const FbxAMatrix& TL = bindPoses.begin()->second;

    // Column 0 magnitude:  sqrt( TL[0][0]^2 + TL[1][0]^2 + TL[2][0]^2 )
    double col0 = std::sqrt(TL[0][0]*TL[0][0] + TL[1][0]*TL[1][0] + TL[2][0]*TL[2][0]);
    double col1 = std::sqrt(TL[0][1]*TL[0][1] + TL[1][1]*TL[1][1] + TL[2][1]*TL[2][1]);
    double col2 = std::sqrt(TL[0][2]*TL[0][2] + TL[1][2]*TL[1][2] + TL[2][2]*TL[2][2]);

    double avgScale = (col0 + col1 + col2) / 3.0;

    if (verbose) {
        printf("  TransformLink column magnitudes: (%.2f, %.2f, %.2f) avg=%.2f\n",
               col0, col1, col2, avgScale);
    }

    // Round to nearest common value
    if (avgScale > 50.0) {
        double rounded = std::round(avgScale);
        if (verbose) printf("  Auto-detected FBX scale factor: %.0f\n", rounded);
        return rounded;
    }

    if (verbose) printf("  FBX scale factor: 1.0 (no baked scale detected)\n");
    return 1.0;
}

// ─── Recursive bone collection ────────────────────────────────────────────────

static void CollectBones(FbxNode* node, int parentIndex,
                         const std::map<FbxNode*, FbxAMatrix>& bindPoses,
                         double fbxScale, const Config& config, Skeleton& skel)
{
    bool isBone = IsBoneNode(node);

    if (isBone && config.armatureFix && parentIndex == -1 && IsArmatureRoot(node)) {
        if (config.verbose)
            printf("  Armature fix: skipping '%s' (Blender armature root)\n", node->GetName());

        for (int i = 0; i < node->GetChildCount(); i++) {
            CollectBones(node->GetChild(i), -1, bindPoses, fbxScale, config, skel);
        }
        return;
    }

    if (isBone) {
        Joint joint;
        joint.name = node->GetName();
        joint.parentIndex = parentIndex;
        joint.fbxNode = node;

        // Get bind pose from cluster, fall back to EvaluateGlobalTransform
        auto it = bindPoses.find(node);
        FbxAMatrix boneMat;
        if (it != bindPoses.end()) {
            boneMat = it->second;
        } else {
            boneMat = node->EvaluateGlobalTransform(FBXSDK_TIME_ZERO);
            if (config.verbose)
                printf("  Note: Joint '%s' has no skin cluster, using EvaluateGlobalTransform\n",
                       joint.name.c_str());
        }

        // Convert FBX → id Tech using GetT/GetQ + axis remap
        // GetQ() automatically normalizes away any scale in the matrix
        joint.worldPos = FbxPosToId(boneMat.GetT(), fbxScale) * config.scaleFactor;
        joint.worldRot = FbxQuatToId(boneMat.GetQ());

        if (config.verbose) {
            printf("  Joint[%d] '%s' parent=%d pos=(%.4f, %.4f, %.4f) rot=(%.4f, %.4f, %.4f)\n",
                   (int)skel.joints.size(), joint.name.c_str(), parentIndex,
                   joint.worldPos.x, joint.worldPos.y, joint.worldPos.z,
                   joint.worldRot.x, joint.worldRot.y, joint.worldRot.z);
        }

        int thisIndex = (int)skel.joints.size();
        skel.joints.push_back(joint);

        for (int i = 0; i < node->GetChildCount(); i++) {
            CollectBones(node->GetChild(i), thisIndex, bindPoses, fbxScale, config, skel);
        }
    } else {
        for (int i = 0; i < node->GetChildCount(); i++) {
            CollectBones(node->GetChild(i), parentIndex, bindPoses, fbxScale, config, skel);
        }
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

bool ExtractSkeleton(FbxScene* scene, const Config& config, Skeleton& outSkeleton)
{
    FbxNode* root = scene->GetRootNode();
    if (!root) {
        fprintf(stderr, "Error: No root node in FBX scene\n");
        return false;
    }

    // Report FBX axis system
    FbxAxisSystem sceneAxis = scene->GetGlobalSettings().GetAxisSystem();
    int upSign, frontSign;
    FbxAxisSystem::EUpVector upVec = sceneAxis.GetUpVector(upSign);
    FbxAxisSystem::ECoordSystem coordSys = sceneAxis.GetCoorSystem();
    (void)sceneAxis.GetFrontVector(frontSign);

    const char* upName = (upVec == FbxAxisSystem::eXAxis) ? "X" :
                         (upVec == FbxAxisSystem::eYAxis) ? "Y" : "Z";
    printf("  FBX axis system: %s%s-up, %s\n",
           upSign > 0 ? "+" : "-", upName,
           coordSys == FbxAxisSystem::eRightHanded ? "right-handed" : "left-handed");

    // Collect cluster bind poses
    std::map<FbxNode*, FbxAMatrix> bindPoses;
    CollectClusterBindPoses(scene, bindPoses, config.verbose);

    if (config.verbose)
        printf("  Found %d cluster bind poses\n", (int)bindPoses.size());

    // Auto-detect scale factor from TransformLink column magnitudes
    double fbxScale = DetectFbxScale(bindPoses, config.verbose);
    outSkeleton.fbxScale = fbxScale;
    outSkeleton.scaleFactor = config.scaleFactor;

    // Walk the scene graph and collect bones
    for (int i = 0; i < root->GetChildCount(); i++) {
        CollectBones(root->GetChild(i), -1, bindPoses, fbxScale, config, outSkeleton);
    }

    if (outSkeleton.joints.empty()) {
        fprintf(stderr, "Error: No skeleton bones found in FBX file\n");
        return false;
    }

    if ((int)outSkeleton.joints.size() > MAX_MD5_JOINTS) {
        fprintf(stderr, "Error: Too many bones (%d). MD5 supports max %d.\n",
                (int)outSkeleton.joints.size(), MAX_MD5_JOINTS);
        return false;
    }

    outSkeleton.ComputeLocalFromWorld();

    printf("  Skeleton: %d joints (FBX scale: %.0f)\n",
           (int)outSkeleton.joints.size(), fbxScale);
    return true;
}

} // namespace fbx2md5
