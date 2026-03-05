#include "anim_data.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace fbx2md5 {

// Same conversion functions as skeleton.cpp
static Vec3 FbxPosToId(const FbxVector4& t, double scale)
{
    double invS = 1.0 / scale;
    return { t[0] * invS, -t[2] * invS, t[1] * invS };
}

static Quat FbxQuatToId(const FbxQuaternion& q)
{
    // id_q = conj(fbx_q) * conj(Q_90x)
    // Same as skeleton.cpp. Applied to EvaluateGlobalTransform quaternions.
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

AnimBounds ComputeFrameBounds(const Skeleton& skeleton,
                              const std::vector<Mesh>& meshes,
                              const std::vector<JointFrame>& worldFrameJoints)
{
    AnimBounds bounds;
    bounds.mins = { 1e18,  1e18,  1e18};
    bounds.maxs = {-1e18, -1e18, -1e18};

    // For each vertex, compute skinned position at this frame using
    // the weight offset approach: v = sum( weight * (R_frame * offset + T_frame) )
    for (auto& mesh : meshes) {
        for (auto& vert : mesh.vertices) {
            Vec3 skinnedPos = {0, 0, 0};
            for (auto& w : vert.weights) {
                const JointFrame& jf = worldFrameJoints[w.jointIndex];
                // id Tech: ProjectVector(Q, offset) = conj(Q) * offset * Q
                Vec3 transformed = jf.rot.Conjugate().RotatePoint(w.offset) + jf.pos;
                skinnedPos = skinnedPos + transformed * w.weight;
            }
            bounds.mins.x = std::min(bounds.mins.x, skinnedPos.x);
            bounds.mins.y = std::min(bounds.mins.y, skinnedPos.y);
            bounds.mins.z = std::min(bounds.mins.z, skinnedPos.z);
            bounds.maxs.x = std::max(bounds.maxs.x, skinnedPos.x);
            bounds.maxs.y = std::max(bounds.maxs.y, skinnedPos.y);
            bounds.maxs.z = std::max(bounds.maxs.z, skinnedPos.z);
        }
    }

    return bounds;
}

bool ExtractAnimations(FbxScene* scene, const Skeleton& skeleton,
                       const std::vector<Mesh>& meshes,
                       const Config& config,
                       std::vector<AnimData>& outAnims)
{
    int animStackCount = scene->GetSrcObjectCount<FbxAnimStack>();
    if (animStackCount == 0) {
        printf("  No animation stacks found\n");
        return true; // not an error — mesh-only export is fine
    }

    double fbxScale = skeleton.fbxScale;

    for (int stackIdx = 0; stackIdx < animStackCount; stackIdx++) {
        FbxAnimStack* animStack = scene->GetSrcObject<FbxAnimStack>(stackIdx);
        if (!animStack) continue;

        scene->SetCurrentAnimationStack(animStack);

        FbxTakeInfo* takeInfo = scene->GetTakeInfo(animStack->GetName());
        FbxTime start, stop;
        if (takeInfo) {
            start = takeInfo->mLocalTimeSpan.GetStart();
            stop = takeInfo->mLocalTimeSpan.GetStop();
        } else {
            FbxTimeSpan timeSpan;
            scene->GetGlobalSettings().GetTimelineDefaultTimeSpan(timeSpan);
            start = timeSpan.GetStart();
            stop = timeSpan.GetStop();
        }

        FbxTime::EMode timeMode = scene->GetGlobalSettings().GetTimeMode();
        double frameRateD = FbxTime::GetFrameRate(timeMode);
        int frameRate = (int)(frameRateD + 0.5);
        if (frameRate <= 0) frameRate = 24;

        FbxTime frameDuration;
        frameDuration.SetTime(0, 0, 0, 1, 0, timeMode);

        // Frame count: number of frames in the animation range (exclusive end)
        long long startFrame = start.GetFrameCount(timeMode);
        long long stopFrame = stop.GetFrameCount(timeMode);
        int frameCount = (int)(stopFrame - startFrame);
        if (frameCount <= 0) continue;

        AnimData anim;
        anim.name = animStack->GetName();
        anim.frameCount = frameCount;
        anim.frameRate = frameRate;
        anim.jointInfo.resize(skeleton.joints.size());

        if (config.verbose) {
            printf("  Animation '%s': %d frames @ %d fps\n",
                   anim.name.c_str(), frameCount, frameRate);
        }

        // ─── Sample all frames ──────────────────────────────────────────
        // Use EvaluateGlobalTransform for ALL bones.
        // Apply FbxQuatToId (old formula) which works when EvalGlobal includes armature.
        //
        // For FBX files where EvalGlobal doesn't include armature (detected by
        // comparing root TL.Q with EG_rest.Q), we pre-compose the armature
        // rotation into each bone's EvalGlobal BEFORE FbxQuatToId/FbxPosToId.
        // This is done per-bone using the bone's own TransformLink:
        //   corrected(t) = TransformLink * inverse(EvalGlobal_rest) * EvalGlobal(t)
        // At rest: corrected = TransformLink ✓
        // At any frame: captures animation delta correctly.
        //
        // To avoid scale issues from matrix multiplication, we decompose into
        // rotation-only correction applied to position and quaternion separately.

        // Detect if armature correction is needed
        bool needArmatureCorrection = false;
        std::vector<FbxAMatrix> boneRestGlobal(skeleton.joints.size());
        std::vector<FbxAMatrix> boneTL(skeleton.joints.size());
        {
            FbxQuaternion tlQ, egQ;
            tlQ.Set(0,0,0,1); egQ.Set(0,0,0,1);

            // Get all TransformLink matrices
            int meshCount2 = scene->GetSrcObjectCount<FbxMesh>();
            for (int mi = 0; mi < meshCount2; mi++) {
                FbxMesh* mesh2 = scene->GetSrcObject<FbxMesh>(mi);
                if (!mesh2) continue;
                for (int di = 0; di < mesh2->GetDeformerCount(FbxDeformer::eSkin); di++) {
                    FbxSkin* skin2 = (FbxSkin*)mesh2->GetDeformer(di, FbxDeformer::eSkin);
                    if (!skin2) continue;
                    for (int ci = 0; ci < skin2->GetClusterCount(); ci++) {
                        FbxCluster* cl = skin2->GetCluster(ci);
                        if (!cl || !cl->GetLink()) continue;
                        FbxAMatrix tlMat;
                        cl->GetTransformLinkMatrix(tlMat);
                        int idx = skeleton.FindJointByNode(cl->GetLink());
                        if (idx >= 0) boneTL[idx] = tlMat;
                    }
                }
            }

            // Get rest-pose globals and detect armature
            for (int ji = 0; ji < (int)skeleton.joints.size(); ji++) {
                boneRestGlobal[ji] = skeleton.joints[ji].fbxNode->EvaluateGlobalTransform(FBXSDK_TIME_INFINITE);
            }

            tlQ = boneTL[0].GetQ();
            egQ = boneRestGlobal[0].GetQ();
            double d = 0, dN = 0;
            for (int k = 0; k < 4; k++) { d += std::abs(tlQ[k]-egQ[k]); dN += std::abs(tlQ[k]+egQ[k]); }
            needArmatureCorrection = (std::min(d, dN) > 0.01);

            if (config.verbose)
                printf("  Armature correction: %s\n", needArmatureCorrection ? "YES" : "NO");
        }

        std::vector<std::vector<JointFrame>> allFrames(frameCount);
        std::vector<std::vector<JointFrame>> allWorldFrames(frameCount);

        for (int f = 0; f < frameCount; f++) {
            FbxTime evalTime = start + frameDuration * f;
            allFrames[f].resize(skeleton.joints.size());
            allWorldFrames[f].resize(skeleton.joints.size());

            for (int ji = 0; ji < (int)skeleton.joints.size(); ji++) {
                const Joint& joint = skeleton.joints[ji];
                FbxNode* node = joint.fbxNode;

                FbxAMatrix globalMat = node->EvaluateGlobalTransform(evalTime);

                if (needArmatureCorrection) {
                    // corrected = TL * inverse(EG_rest) * EG(t)
                    // Scale cancels: TL has scale S, inv(EG_rest) has scale 1/S,
                    // so offset = TL*inv(EG_rest) is scale-free.
                    // offset * EG(t) then has scale S (from EG), matching FbxPosToId expectations.
                    FbxAMatrix restInv = boneRestGlobal[ji].Inverse();
                    FbxAMatrix offset = boneTL[ji] * restInv;
                    globalMat = offset * globalMat;
                }

                Vec3 worldPos = FbxPosToId(globalMat.GetT(), fbxScale) * config.scaleFactor;
                Quat worldRot = FbxQuatToId(globalMat.GetQ());

                // Store world for bounds computation
                allWorldFrames[f][ji] = {worldPos, worldRot};

                // Compute local transforms
                if (joint.parentIndex < 0) {
                    allFrames[f][ji] = {worldPos, worldRot.EnsureW()};
                } else {
                    // Decompose quaternion in FBX space, then similarity transform
                    const Joint& parentJoint = skeleton.joints[joint.parentIndex];
                    FbxAMatrix parentGlobal = parentJoint.fbxNode->EvaluateGlobalTransform(evalTime);
                    FbxAMatrix localFbx = parentGlobal.Inverse() * globalMat;

                    // Quaternion: FBX local decomposition gives correct parent-relative
                    // rotation. No axis remap needed — the coordinate system change
                    // from the root propagates through the hierarchy.
                    FbxQuaternion lq = localFbx.GetQ();
                    Quat localRot = Quat(lq[0], lq[1], lq[2], lq[3]).Normalized().EnsureW();

                    // Position from id Tech world decomposition
                    auto& pw = allWorldFrames[f][joint.parentIndex];
                    Vec3 localPos = pw.rot.Conjugate().RotatePoint(worldPos - pw.pos);

                    allFrames[f][ji] = {localPos, localRot};
                }
            }
        }

        // ─── Determine baseframe and animated component flags ───────────
        double eps = config.animEpsilon;

        int totalAnimatedComponents = 0;
        for (int ji = 0; ji < (int)skeleton.joints.size(); ji++) {
            JointAnimInfo& info = anim.jointInfo[ji];
            info.baseframe = allFrames[0][ji];
            info.baseframe.rot = info.baseframe.rot.EnsureW();

            // Check which components change across frames
            int flags = 0;
            for (int f = 1; f < frameCount; f++) {
                auto& frame = allFrames[f][ji];
                auto& base = info.baseframe;
                if (std::abs(frame.pos.x - base.pos.x) > eps) flags |= ANIM_TX;
                if (std::abs(frame.pos.y - base.pos.y) > eps) flags |= ANIM_TY;
                if (std::abs(frame.pos.z - base.pos.z) > eps) flags |= ANIM_TZ;
                if (std::abs(frame.rot.x - base.rot.x) > eps) flags |= ANIM_QX;
                if (std::abs(frame.rot.y - base.rot.y) > eps) flags |= ANIM_QY;
                if (std::abs(frame.rot.z - base.rot.z) > eps) flags |= ANIM_QZ;
            }
            info.flags = flags;
            info.startIndex = totalAnimatedComponents;

            for (int b = 0; b < 6; b++) {
                if (flags & (1 << b)) totalAnimatedComponents++;
            }
        }

        anim.numAnimatedComponents = totalAnimatedComponents;

        // ─── Build per-frame component arrays ───────────────────────────
        anim.frameComponents.resize(frameCount);
        anim.frameBounds.resize(frameCount);

        for (int f = 0; f < frameCount; f++) {
            anim.frameComponents[f].reserve(totalAnimatedComponents);

            for (int ji = 0; ji < (int)skeleton.joints.size(); ji++) {
                auto& info = anim.jointInfo[ji];
                auto& frame = allFrames[f][ji];

                Quat q = frame.rot.EnsureW();

                if (info.flags & ANIM_TX) anim.frameComponents[f].push_back(frame.pos.x);
                if (info.flags & ANIM_TY) anim.frameComponents[f].push_back(frame.pos.y);
                if (info.flags & ANIM_TZ) anim.frameComponents[f].push_back(frame.pos.z);
                if (info.flags & ANIM_QX) anim.frameComponents[f].push_back(q.x);
                if (info.flags & ANIM_QY) anim.frameComponents[f].push_back(q.y);
                if (info.flags & ANIM_QZ) anim.frameComponents[f].push_back(q.z);
            }

            // Compute bounds for this frame
            anim.frameBounds[f] = ComputeFrameBounds(skeleton, meshes, allWorldFrames[f]);
        }

        printf("  Animation '%s': %d animated components out of %d possible\n",
               anim.name.c_str(), totalAnimatedComponents,
               (int)skeleton.joints.size() * 6);

        outAnims.push_back(std::move(anim));
    }

    return true;
}

} // namespace fbx2md5
