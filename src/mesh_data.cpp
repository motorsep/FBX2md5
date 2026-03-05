#include "mesh_data.h"
#include <cstdio>
#include <cmath>
#include <map>
#include <set>
#include <algorithm>

namespace fbx2md5 {

// FBX to id Tech coordinate conversion (must match skeleton.cpp)

static Vec3 FbxPosToId(const FbxVector4& t, double scale)
{
    double invS = 1.0 / scale;
    return { t[0] * invS, -t[2] * invS, t[1] * invS };
}

static Vec3 FbxDirToId(const FbxVector4& d)
{
    return Vec3( d[0], -d[2], d[1] );
}

// Skin data collection

struct ClusterInfo {
    int jointIndex;
    FbxAMatrix transformLink;      // bone's world transform at bind time
    FbxAMatrix transform;          // mesh's world transform at bind time
};

struct SkinInfo {
    // Per control point: list of (clusterInfoIndex, weight)
    std::vector< std::vector< std::pair<int, double> > > cpWeights;
    std::vector<ClusterInfo> clusters;
    // Mesh's world transform at bind time (from first cluster's GetTransformMatrix)
    FbxAMatrix meshBindTransform;
    bool hasMeshBindTransform = false;
};

static SkinInfo CollectSkinData(FbxMesh* mesh, const Skeleton& skeleton,
                                const Config& config)
{
    SkinInfo info;
    int cpCount = mesh->GetControlPointsCount();
    info.cpWeights.resize(cpCount);

    int skinCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
    for (int si = 0; si < skinCount; si++) {
        FbxSkin* skin = (FbxSkin*)mesh->GetDeformer(si, FbxDeformer::eSkin);
        if (!skin) continue;

        int clusterCount = skin->GetClusterCount();
        for (int ci = 0; ci < clusterCount; ci++) {
            FbxCluster* cluster = skin->GetCluster(ci);
            if (!cluster) continue;

            FbxNode* linkNode = cluster->GetLink();
            if (!linkNode) continue;

            int jointIdx = skeleton.FindJointByNode(linkNode);
            if (jointIdx < 0)
                jointIdx = skeleton.FindJointByName(linkNode->GetName());
            if (jointIdx < 0) {
                if (config.verbose)
                    printf("    Warning: cluster '%s' not found in skeleton\n",
                           linkNode->GetName());
                continue;
            }

            int clusterInfoIdx = (int)info.clusters.size();
            ClusterInfo cinfo;
            cinfo.jointIndex = jointIdx;
            cluster->GetTransformLinkMatrix(cinfo.transformLink);
            cluster->GetTransformMatrix(cinfo.transform);
            info.clusters.push_back(cinfo);

            // Store the mesh bind-time transform from the first cluster
            if (!info.hasMeshBindTransform) {
                info.meshBindTransform = cinfo.transform;
                info.hasMeshBindTransform = true;
            }

            int* indices = cluster->GetControlPointIndices();
            double* weights = cluster->GetControlPointWeights();
            int count = cluster->GetControlPointIndicesCount();

            for (int k = 0; k < count; k++) {
                int cpIdx = indices[k];
                double w = weights[k];
                if (cpIdx >= 0 && cpIdx < cpCount && w > 1e-8) {
                    info.cpWeights[cpIdx].push_back({ clusterInfoIdx, w });
                }
            }
        }
    }

    return info;
}

// UV helper

static Vec2 GetUV(FbxMesh* mesh, int polyIdx, int vertIdx, int uvLayer)
{
    FbxGeometryElementUV* uvElem = mesh->GetElementUV(uvLayer);
    if (!uvElem) return { 0, 0 };

    int idx = 0;
    if (uvElem->GetMappingMode() == FbxGeometryElement::eByPolygonVertex) {
        if (uvElem->GetReferenceMode() == FbxGeometryElement::eDirect)
            idx = polyIdx * 3 + vertIdx;
        else
            idx = uvElem->GetIndexArray().GetAt(polyIdx * 3 + vertIdx);
    } else if (uvElem->GetMappingMode() == FbxGeometryElement::eByControlPoint) {
        int cpIdx = mesh->GetPolygonVertex(polyIdx, vertIdx);
        if (uvElem->GetReferenceMode() == FbxGeometryElement::eDirect)
            idx = cpIdx;
        else
            idx = uvElem->GetIndexArray().GetAt(cpIdx);
    }

    FbxVector2 uv = uvElem->GetDirectArray().GetAt(idx);
    return { uv[0], 1.0 - uv[1] };
}

// Weight reduction

void ReduceWeights(Vertex& vert)
{
    auto& w = vert.weights;

    std::sort(w.begin(), w.end(), [](const VertexWeight& a, const VertexWeight& b) {
        return a.weight > b.weight;
    });

    if ((int)w.size() > MAX_VERTEX_WEIGHTS)
        w.resize(MAX_VERTEX_WEIGHTS);

    double total = 0;
    for (auto& vw : w) total += vw.weight;
    if (total > 1e-12) {
        for (auto& vw : w) vw.weight /= total;
    }
}

// Vertex deduplication key

struct VertexKey {
    int posIndex;
    int uvIndex;
    int normalIndex;

    bool operator<(const VertexKey& o) const {
        if (posIndex != o.posIndex) return posIndex < o.posIndex;
        if (uvIndex  != o.uvIndex)  return uvIndex  < o.uvIndex;
        return normalIndex < o.normalIndex;
    }
};

// Verify skinning correctness.
// Note: vert.position comes from meshGlobal.MultT(cp) which may differ from 
// the position reconstructed from cluster-based offsets. Mismatches here indicate
// the meshGlobal transform differs from the cluster Transform matrix, which is
// expected for FBX files with axis conversion baked into the scene graph.
static void VerifySkinning(const Skeleton& skeleton, const std::vector<Mesh>& meshes,
                           bool verbose)
{
    printf("  Verifying skinning correctness...\n");

    double maxError = 0;
    int errorCount = 0;
    int totalVerts = 0;

    for (auto& mesh : meshes) {
        for (int vi = 0; vi < (int)mesh.vertices.size(); vi++) {
            auto& vert = mesh.vertices[vi];
            totalVerts++;

            Vec3 computed = {0, 0, 0};
            for (auto& w : vert.weights) {
                const Joint& j = skeleton.joints[w.jointIndex];
                // id Tech reconstructs: ProjectVector(Q, offset) = conj(Q) * offset * Q
                Vec3 rotated = j.worldRot.Conjugate().RotatePoint(w.offset);
                computed.x += w.weight * (rotated.x + j.worldPos.x);
                computed.y += w.weight * (rotated.y + j.worldPos.y);
                computed.z += w.weight * (rotated.z + j.worldPos.z);
            }

            Vec3 diff = computed - vert.position;
            double err = diff.Length();
            if (err > maxError) maxError = err;
            if (err > 0.01) {
                errorCount++;
                if (verbose && errorCount <= 3) {
                    printf("    MISMATCH vert[%d]: expected=(%.4f, %.4f, %.4f) computed=(%.4f, %.4f, %.4f) err=%.6f\n",
                           vi, vert.position.x, vert.position.y, vert.position.z,
                           computed.x, computed.y, computed.z, err);
                    for (auto& w : vert.weights) {
                        const Joint& j = skeleton.joints[w.jointIndex];
                        printf("      weight: joint=%d('%s') bias=%.6f offset=(%.4f, %.4f, %.4f)\n",
                               w.jointIndex, j.name.c_str(), w.weight,
                               w.offset.x, w.offset.y, w.offset.z);
                        printf("        jointPos=(%.4f, %.4f, %.4f) jointRot=(%.4f, %.4f, %.4f, %.4f)\n",
                               j.worldPos.x, j.worldPos.y, j.worldPos.z,
                               j.worldRot.x, j.worldRot.y, j.worldRot.z, j.worldRot.w);
                    }
                }
            }
        }
    }

    printf("    %d vertices, max reconstruction error: %.6f", totalVerts, maxError);
    if (maxError < 0.01) {
        printf(" (PASS)\n");
    } else {
        printf(" *** FAIL: %d verts with >0.01 error ***\n", errorCount);
    }
}

// Diagnostic dump: print first few vertices for comparison with reference
static void DumpDiagnostics(const Skeleton& skeleton, const std::vector<Mesh>& meshes,
                            FbxMesh* fbxMesh, FbxAMatrix& meshGlobal, double fbxScale,
                            double scaleFactor)
{
    printf("\n  === DIAGNOSTIC OUTPUT (compare with reference) ===\n");

    // Print first 5 joints
    printf("  First 5 joints:\n");
    for (int i = 0; i < std::min(5, (int)skeleton.joints.size()); i++) {
        const Joint& j = skeleton.joints[i];
        Quat q = j.worldRot.Normalized().EnsureW();
        printf("    [%d] '%s' parent=%d\n", i, j.name.c_str(), j.parentIndex);
        printf("         pos=(%.6f, %.6f, %.6f)\n", j.worldPos.x, j.worldPos.y, j.worldPos.z);
        printf("         quat=(%.6f, %.6f, %.6f) w=%.6f\n", q.x, q.y, q.z, q.w);
    }

    // Print meshGlobal transform
    FbxVector4 mT = meshGlobal.GetT();
    FbxQuaternion mQ = meshGlobal.GetQ();
    FbxVector4 mS = meshGlobal.GetS();
    printf("  meshGlobal: T=(%.4f, %.4f, %.4f) S=(%.4f, %.4f, %.4f)\n",
           mT[0], mT[1], mT[2], mS[0], mS[1], mS[2]);
    printf("  fbxScale=%.2f scaleFactor=%.4f\n", fbxScale, scaleFactor);

    // Print first 3 control points (raw FBX, before any transform)
    FbxVector4* cps = fbxMesh->GetControlPoints();
    int cpCount = fbxMesh->GetControlPointsCount();
    printf("  First 3 raw control points (FBX mesh-local):\n");
    for (int i = 0; i < std::min(3, cpCount); i++) {
        printf("    cp[%d] = (%.4f, %.4f, %.4f)\n", i, cps[i][0], cps[i][1], cps[i][2]);
    }

    // Print first 3 control points transformed to id Tech world space
    printf("  First 3 control points in id Tech world space:\n");
    for (int i = 0; i < std::min(3, cpCount); i++) {
        FbxVector4 worldPos = meshGlobal.MultT(cps[i]);
        Vec3 idPos = FbxPosToId(worldPos, fbxScale) * scaleFactor;
        printf("    cp[%d] fbx_world=(%.4f, %.4f, %.4f) -> id=(%.4f, %.4f, %.4f)\n",
               i, worldPos[0], worldPos[1], worldPos[2], idPos.x, idPos.y, idPos.z);
    }

    // Print first 5 vertices from first mesh
    if (!meshes.empty()) {
        auto& mesh = meshes[0];
        printf("  First 5 vertices (mesh '%s'):\n", mesh.materialName.c_str());
        for (int i = 0; i < std::min(5, (int)mesh.vertices.size()); i++) {
            auto& v = mesh.vertices[i];
            printf("    vert[%d] pos=(%.4f, %.4f, %.4f) uv=(%.4f, %.4f) %d weights\n",
                   i, v.position.x, v.position.y, v.position.z,
                   v.uv.x, v.uv.y, (int)v.weights.size());
            for (auto& w : v.weights) {
                printf("      weight: joint=%d('%s') bias=%.6f offset=(%.4f, %.4f, %.4f)\n",
                       w.jointIndex, skeleton.joints[w.jointIndex].name.c_str(),
                       w.weight, w.offset.x, w.offset.y, w.offset.z);
            }
        }
    }

    printf("  === END DIAGNOSTICS ===\n\n");
}

// Main extraction

bool ExtractMeshes(FbxScene* scene, const Skeleton& skeleton,
                   const Config& config, std::vector<Mesh>& outMeshes)
{
    double fbxScale = skeleton.fbxScale;

    int nodeCount = scene->GetNodeCount();
    for (int ni = 0; ni < nodeCount; ni++) {
        FbxNode* node = scene->GetNode(ni);
        if (!node) continue;

        FbxMesh* fbxMesh = node->GetMesh();
        if (!fbxMesh) continue;

        // Triangulate if needed
        if (!fbxMesh->IsTriangleMesh()) {
            FbxGeometryConverter conv(scene->GetFbxManager());
            fbxMesh = (FbxMesh*)conv.Triangulate(fbxMesh, true);
            if (!fbxMesh) continue;
        }

        // Must have skin
        if (fbxMesh->GetDeformerCount(FbxDeformer::eSkin) <= 0) continue;

        printf("  Processing mesh: '%s' (%d polygons)\n",
               node->GetName(), fbxMesh->GetPolygonCount());

        // Collect skin weights (joint indices and bias from clusters)
        SkinInfo skinData = CollectSkinData(fbxMesh, skeleton, config);

        // Use the skin cluster's Transform matrix for vertex positions.
        // This is the mesh's world transform at bind time, guaranteed consistent
        // with TransformLink (used for joint extraction). Using node->EvaluateGlobalTransform
        // can differ if the FBX SDK applies axis conversion or if the scene was modified.
        FbxAMatrix meshGlobal = skinData.hasMeshBindTransform
            ? skinData.meshBindTransform
            : node->EvaluateGlobalTransform(FBXSDK_TIME_ZERO);

        FbxVector4* controlPoints = fbxMesh->GetControlPoints();
        int polyCount = fbxMesh->GetPolygonCount();

        // Split by material
        FbxGeometryElementMaterial* matElem = fbxMesh->GetElementMaterial();
        std::map<std::string, std::pair<Mesh, std::map<VertexKey, int>>> meshMap;

        for (int pi = 0; pi < polyCount; pi++) {
            if (fbxMesh->GetPolygonSize(pi) != 3) continue;

            std::string matName = "default";
            if (matElem) {
                int matIdx = matElem->GetIndexArray().GetAt(pi);
                FbxSurfaceMaterial* mat = node->GetMaterial(matIdx);
                if (mat) matName = mat->GetName();
            }

            auto& [mesh, vmap] = meshMap[matName];
            mesh.materialName = matName;

            Triangle tri;

            // Flip winding: FBX uses opposite front-face convention from id Tech.
            // Swap indices 1 and 2 to reverse triangle facing.
            static const int windingRemap[3] = {0, 2, 1};

            for (int fvi = 0; fvi < 3; fvi++) {
                int vi = windingRemap[fvi];
                int cpIndex = fbxMesh->GetPolygonVertex(pi, vi);

                VertexKey key;
                key.posIndex = cpIndex;
                key.uvIndex = (fbxMesh->GetElementUVCount() > 0)
                              ? fbxMesh->GetTextureUVIndex(pi, vi) : 0;
                key.normalIndex = (config.md5Version >= 11) ? (pi * 3 + vi) : -1;

                auto it = vmap.find(key);
                if (it != vmap.end()) {
                    tri.indices[fvi] = it->second;
                } else {
                    Vertex vert;

                    // Vertex world position in id Tech space
                    // Note: meshGlobal may contain an axis-conversion rotation
                    // not present in TransformLink. We use meshGlobal here for 
                    // vert.position (UVs, normals, display only). The weight offsets
                    // are computed separately using the corrected position.
                    FbxVector4 cp = controlPoints[cpIndex];
                    FbxVector4 worldPos = meshGlobal.MultT(cp);
                    vert.position = FbxPosToId(worldPos, fbxScale) * config.scaleFactor;

                    // UV
                    if (fbxMesh->GetElementUVCount() > 0) {
                        vert.uv = GetUV(fbxMesh, pi, vi, 0);
                    }

                    // Normal (for v11)
                    if (config.md5Version >= 11) {
                        FbxVector4 fbxNormal;
                        fbxMesh->GetPolygonVertexNormal(pi, vi, fbxNormal);
                        FbxVector4 worldNormal = meshGlobal.MultR(fbxNormal);
                        Vec3 n = FbxDirToId(worldNormal);
                        double len = n.Length();
                        if (len > 1e-12) n = n * (1.0 / len);
                        vert.normal = n;
                    }

                    // Weight offsets: compute from FBX cluster matrices directly.
                    //
                    // FBX skinning: vertPos_fbx = TransformLink * (TransformLink^-1 * Transform * cp)
                    //                           = Transform * cp
                    // The bone-local FBX position is: offset_fbx = (TransformLink^-1 * Transform * cp).GetT()
                    //
                    // To convert to id Tech bone-local:
                    //   vertPos_id = Q_id * offset_id + T_id
                    //              = AxisRemap(TransformLink * offset_fbx)  [since AxisRemap is linear]
                    //              = AxisRemap(R_fbx * offset_fbx + T_fbx)
                    // And T_id = AxisRemap(T_fbx), so:
                    //   Q_id * offset_id = AxisRemap(R_fbx * offset_fbx)
                    //   offset_id = conj(Q_id) * AxisRemap(R_fbx * offset_fbx)
                    //
                    // This is equivalent to:
                    //   worldPos_id = AxisRemap(Transform * cp)
                    //   offset_id = conj(Q_id) * (worldPos_id - T_id)
                    // ...but using Transform instead of meshGlobal/EvaluateGlobalTransform.
                    //
                    // Since Transform may equal meshGlobal in some cases, we go through
                    // the full TransformLink chain to guarantee correctness:
                    // Weight offsets: compute in id Tech space.
                    // id Tech reconstructs: vertPos = ProjectVector(Q, offset) + T
                    // where ProjectVector does conj(Q) * offset * Q
                    // So: offset = Q * (vertPos - T) * conj(Q) = Q.RotatePoint(vertPos - T)
                    if (cpIndex < (int)skinData.cpWeights.size()) {
                        for (auto& cpw : skinData.cpWeights[cpIndex]) {
                            int clusterIdx = cpw.first;
                            const ClusterInfo& ci = skinData.clusters[clusterIdx];

                            VertexWeight vw;
                            vw.jointIndex = ci.jointIndex;
                            vw.weight = cpw.second;

                            const Joint& joint = skeleton.joints[ci.jointIndex];
                            vw.offset = joint.worldRot.RotatePoint(
                                vert.position - joint.worldPos);

                            vert.weights.push_back(vw);
                        }
                    }

                    ReduceWeights(vert);

                    // Fallback: if no weights, bind to root
                    if (vert.weights.empty()) {
                        VertexWeight vw;
                        vw.jointIndex = 0;
                        vw.weight = 1.0;
                        vw.offset = skeleton.joints[0].worldRot.RotatePoint(
                            vert.position - skeleton.joints[0].worldPos);
                        vert.weights.push_back(vw);
                    }

                    int newIdx = (int)mesh.vertices.size();
                    mesh.vertices.push_back(vert);
                    vmap[key] = newIdx;
                    tri.indices[fvi] = newIdx;
                }
            }

            mesh.triangles.push_back(tri);
        }

        for (auto& [name, pair] : meshMap) {
            auto& [mesh, vmap] = pair;
            if (!mesh.vertices.empty()) {
                printf("    Material '%s': %d verts, %d tris\n",
                       name.c_str(), (int)mesh.vertices.size(), (int)mesh.triangles.size());
                outMeshes.push_back(std::move(mesh));
            }
        }

        // Always print diagnostics for debugging
        DumpDiagnostics(skeleton, outMeshes, fbxMesh, meshGlobal, fbxScale,
                        config.scaleFactor);
    }

    if (!outMeshes.empty()) {
        VerifySkinning(skeleton, outMeshes, config.verbose);
    }

    return !outMeshes.empty();
}

} // namespace fbx2md5
