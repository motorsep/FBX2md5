/*
 * fbx2md5 -- FBX to idTech4 MD5 mesh and animation converter.
 *
 * Copyright (c) 2026 motorsep
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This tool uses ufbx by Samuli Raivio (https://github.com/ufbx/ufbx),
 * bundled as src/ufbx.{c,h} under its own MIT license.
 */

// =============================================================================
// fbx2md5.cpp
// FBX to idTech4 MD5 mesh/anim converter using the ufbx library.
//
// Build (MSVC 2022, single translation unit + ufbx.c):
//   - Add fbx2md5.cpp, ufbx.c, ufbx.h to a Console Application project.
//   - For ufbx.c:  Properties -> C/C++ -> Advanced -> Compile As -> Compile as C Code (/TC)
//   - C++ language standard: /std:c++17 or later.
//   - Multi-byte or Unicode character set both fine.
//
// Usage:
//   fbx2md5 input.fbx output [-scale X.X] [-fps N] [-noaxes] [-v12] [-noAnimPrefix]
//
// Produces:
//   output.md5mesh
//   output_<stackname>.md5anim        (one per FBX animation stack)
//
// Defaults:
//   -scale   1.0        (world-unit multiplier applied to all positions)
//   -fps     <from FBX scene>   (falls back to 24 if the FBX doesn't report one)
//   axes     convert to Z-up, X-forward, right-handed (idTech4 convention)
//
// Correctness notes:
//   * Weight offsets use ufbx_skin_cluster.geometry_to_bone directly --
//     that matrix transforms geometry-space positions into bone-local space,
//     which is exactly what MD5 wants.
//   * Bind pose joint transforms come from ufbx_skin_cluster.bind_to_world
//     when available, falling back to ufbx_node.node_to_world otherwise.
//   * Quaternions are normalized and flipped to w >= 0 before dropping w.
//   * Face winding is reversed (FBX CCW -> MD5 CW).
//   * V texture coordinate is flipped (1.0 - v).
//   * Per-joint animBits delta compression, identical to the Maya exporter.
// =============================================================================

#define _CRT_SECURE_NO_WARNINGS

#include "ufbx.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <cctype>

#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

// -----------------------------------------------------------------------------
// MD5 constants
// -----------------------------------------------------------------------------

static const char *MD5_VERSION_STRING = "MD5Version";
static const int   MD5_VERSION        = 10;
static const int   MD5_VERSION_V12    = 12;  // per-vertex normals + MikkTSpace tangents

enum {
	ANIM_TX = 1 << 0,
	ANIM_TY = 1 << 1,
	ANIM_TZ = 1 << 2,
	ANIM_QX = 1 << 3,
	ANIM_QY = 1 << 4,
	ANIM_QZ = 1 << 5,
};
static const char *ANIM_BIT_NAMES[6] = { "Tx", "Ty", "Tz", "Qx", "Qy", "Qz" };

// Thresholds (match the Maya exporter's DEFAULT_ANIM_EPSILON / QUAT_EPSILON)
static const float XYZ_EPSILON  = 0.1f;        // legacy idTech4 threshold
static const float QUAT_EPSILON = 0.000001f;
static const float VERT_EPSILON = 0.001f;
static const float UV_EPSILON   = 0.001f;
static const float MIN_WEIGHT   = 0.001f;
static const float NORMAL_EPSILON = 0.001f;

// -----------------------------------------------------------------------------
// Small math helpers
// -----------------------------------------------------------------------------

struct CQuat { float x, y, z; };  // compressed quat (w derived as sqrt(1 - x^2 - y^2 - z^2))

// Compress a quaternion into MD5's 3-component form.
//
// IMPORTANT - handedness convention conversion:
// ufbx returns quaternions in the column-vector convention (M*v), which is
// the standard math / modern graphics convention. idTech4's idQuat uses the
// row-vector convention (v*M). The same rotation, expressed in the two
// conventions, has x/y/z signs flipped (conjugate quaternion). So we
// conjugate before compressing. Then we force w >= 0 and drop w, because the
// MD5 reader reconstructs w via sqrt(1 - x^2 - y^2 - z^2) (always positive).
static CQuat CompressQuat(ufbx_quat q) {
	// Normalize for safety.
	double len = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	if (len > 0.0) {
		q.x = (ufbx_real)(q.x / len);
		q.y = (ufbx_real)(q.y / len);
		q.z = (ufbx_real)(q.z / len);
		q.w = (ufbx_real)(q.w / len);
	} else {
		q.x = q.y = q.z = 0; q.w = 1;
	}
	// Conjugate: ufbx column convention -> idTech4 row convention.
	q.x = -q.x; q.y = -q.y; q.z = -q.z;
	// Force w >= 0 for the compressed form.
	if (q.w < 0.0f) {
		q.x = -q.x; q.y = -q.y; q.z = -q.z;
	}
	CQuat r = { (float)q.x, (float)q.y, (float)q.z };
	return r;
}

// Inverse of CompressQuat. Undoes the w>=0 projection (w reconstructs as
// positive root) and undoes the conjugation so the result is back in ufbx's
// column-vector convention. Used for internal math that works in ufbx space.
static ufbx_quat DecompressQuat(CQuat c) {
	double t = 1.0 - (double)c.x*c.x - (double)c.y*c.y - (double)c.z*c.z;
	ufbx_quat q;
	q.x = -c.x; q.y = -c.y; q.z = -c.z;   // un-conjugate
	q.w = (ufbx_real)(t > 0.0 ? sqrt(t) : 0.0);
	return q;
}

static ufbx_matrix MatFromTR(ufbx_vec3 t, ufbx_quat q) {
	ufbx_transform tf = { t, q, { 1, 1, 1 } };
	return ufbx_transform_to_matrix(&tf);
}

// Transform a direction vector by the 3x3 rotation part of a matrix (no translation).
static ufbx_vec3 TransformDir(const ufbx_matrix &m, ufbx_vec3 v) {
	return {
		(ufbx_real)(m.cols[0].x*v.x + m.cols[1].x*v.y + m.cols[2].x*v.z),
		(ufbx_real)(m.cols[0].y*v.x + m.cols[1].y*v.y + m.cols[2].y*v.z),
		(ufbx_real)(m.cols[0].z*v.x + m.cols[1].z*v.y + m.cols[2].z*v.z)
	};
}

static ufbx_vec3 NormalizeVec3(ufbx_vec3 v) {
	double len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
	if (len > 1e-12) { v.x = (ufbx_real)(v.x/len); v.y = (ufbx_real)(v.y/len); v.z = (ufbx_real)(v.z/len); }
	return v;
}

static ufbx_vec3 CrossVec3(ufbx_vec3 a, ufbx_vec3 b) {
	return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}

static double DotVec3(ufbx_vec3 a, ufbx_vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

// -----------------------------------------------------------------------------
// Name sanitization for the MD5 "shader" field.
// Keep alnum, underscore, dash, slash, dot. Convert whitespace to underscore.
// Drop everything else.
// -----------------------------------------------------------------------------

static std::string SanitizeShaderName(const char *src, size_t len) {
	std::string out;
	out.reserve(len + 1);
	for (size_t i = 0; i < len; ++i) {
		char c = src[i];
		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		    (c >= '0' && c <= '9') || c == '_' || c == '/' || c == '-' || c == '.') {
			out += c;
		} else if (c == ' ' || c == '\t' || c == '\\') {
			out += '_';
		}
	}
	if (out.empty()) out = "default";
	return out;
}

// -----------------------------------------------------------------------------
// Export data structures
// -----------------------------------------------------------------------------

struct ExportJoint {
	std::string  name;
	int          parentIdx;      // -1 for root
	ufbx_vec3    bindPos;        // world-space
	CQuat        bindQuat;       // world-space, compressed
	ufbx_node   *node;           // original scene node
};

struct ExportWeight {
	int        jointIdx;
	float      weight;
	ufbx_vec3  offset;           // bone-local (geometry_to_bone * pos)
};

struct ExportVertex {
	float      uv[2];
	int        startWeight;
	int        numWeights;
	// v12 per-vertex data (stored in bone-rotated space for the dominant joint)
	float      normal[3];    // ( Nx Ny Nz )
	float      tangent[4];   // ( Tx Ty Tz Tw ) -- Tw = bitangent sign
	float      color[4];     // ( R G B A )  -- vertex color, default (1,1,1,1)
	bool       hasColor;
};

struct ExportTri { int v[3]; };

struct ExportMesh {
	std::string               name;
	std::string               shader;
	std::vector<ExportVertex> verts;
	std::vector<ExportTri>    tris;
	std::vector<ExportWeight> weights;
};

// Per-joint, per-frame transform (parent-local)
struct JointFrame {
	ufbx_vec3 t;
	CQuat     q;
};

// -----------------------------------------------------------------------------
// Joint collection
// -----------------------------------------------------------------------------

// Collect every scene node that is either (a) referenced by a skin cluster
// or (b) carries a ufbx_bone attribute. Flatten non-joint ancestors: each
// joint's parentIdx points to its nearest joint ancestor, or -1 if none.
// Output is topologically sorted (parents before children).
static std::vector<ExportJoint> CollectJoints(ufbx_scene *scene) {
	size_t nNodes = scene->nodes.count;
	std::vector<bool>         isJoint(nNodes, false);
	std::vector<ufbx_matrix>  bindToWorld(nNodes);
	std::vector<bool>         hasBindToWorld(nNodes, false);

	// Any node with a bone attribute is a joint.
	for (size_t i = 0; i < nNodes; ++i) {
		ufbx_node *n = scene->nodes.data[i];
		if (n->bone) isJoint[i] = true;
	}

	// Any node referenced by a skin cluster is a joint; record its bind_to_world.
	for (size_t d = 0; d < scene->skin_deformers.count; ++d) {
		ufbx_skin_deformer *skin = scene->skin_deformers.data[d];
		for (size_t c = 0; c < skin->clusters.count; ++c) {
			ufbx_skin_cluster *cl = skin->clusters.data[c];
			if (!cl->bone_node) continue;
			uint32_t idx = cl->bone_node->typed_id;
			isJoint[idx] = true;
			if (!hasBindToWorld[idx]) {
				bindToWorld[idx] = cl->bind_to_world;
				hasBindToWorld[idx] = true;
			}
		}
	}

	std::vector<ExportJoint> joints;

	// Iterative DFS so parents land before children.
	struct Frame { ufbx_node *node; int parentJoint; size_t childIdx; };
	std::vector<Frame> stack;
	stack.push_back({ scene->root_node, -1, 0 });

	while (!stack.empty()) {
		Frame &top = stack.back();
		if (top.childIdx == 0 && top.node && top.node != scene->root_node) {
			uint32_t idx = top.node->typed_id;
			if (isJoint[idx]) {
				ExportJoint ej = {};
				ej.name.assign(top.node->name.data, top.node->name.length);
				if (ej.name.empty()) {
					char buf[32];
					snprintf(buf, sizeof(buf), "joint_%u", idx);
					ej.name = buf;
				}
				ej.parentIdx = top.parentJoint;
				ej.node      = top.node;

				ufbx_matrix bind = hasBindToWorld[idx]
					? bindToWorld[idx]
					: top.node->node_to_world;
				ufbx_transform bt = ufbx_matrix_to_transform(&bind);
				ej.bindPos  = bt.translation;
				ej.bindQuat = CompressQuat(bt.rotation);

				top.parentJoint = (int)joints.size();  // children of this node inherit this joint as parent
				joints.push_back(ej);
			}
		}

		if (top.node && top.childIdx < top.node->children.count) {
			ufbx_node *child = top.node->children.data[top.childIdx++];
			stack.push_back({ child, top.parentJoint, 0 });
		} else {
			stack.pop_back();
		}
	}

	return joints;
}

// If the FBX has no skeleton at all, synthesize a single "origin" joint at
// world identity so static meshes still produce a valid MD5.
static void EnsureAtLeastOneJoint(std::vector<ExportJoint> &joints) {
	if (!joints.empty()) return;
	ExportJoint j = {};
	j.name      = "origin";
	j.parentIdx = -1;
	j.bindPos   = { 0, 0, 0 };
	j.bindQuat  = { 0, 0, 0 };
	j.node      = nullptr;
	joints.push_back(j);
}

// -----------------------------------------------------------------------------
// Mesh extraction
// -----------------------------------------------------------------------------

// Key for vertex deduplication: UV + exact sequence of (jointIdx, weight, offset)
// triplets. Two FBX corners collapse to one MD5 vert when they agree on UV AND
// produce the same weight list after normalization/filtering.
struct DedupeKey {
	float u, v;
	std::vector<ExportWeight> ws;
	// v12 fields for normal-aware deduplication
	float normal[3];
	float tangent[4];
	bool Equals(const DedupeKey &o, bool v12) const {
		if (std::fabs(u - o.u) > UV_EPSILON) return false;
		if (std::fabs(v - o.v) > UV_EPSILON) return false;
		if (ws.size() != o.ws.size()) return false;
		for (size_t i = 0; i < ws.size(); ++i) {
			if (ws[i].jointIdx != o.ws[i].jointIdx) return false;
			if (std::fabs(ws[i].weight - o.ws[i].weight) > 1e-4f) return false;
			if (std::fabs(ws[i].offset.x - o.ws[i].offset.x) > VERT_EPSILON) return false;
			if (std::fabs(ws[i].offset.y - o.ws[i].offset.y) > VERT_EPSILON) return false;
			if (std::fabs(ws[i].offset.z - o.ws[i].offset.z) > VERT_EPSILON) return false;
		}
		if (v12) {
			for (int i = 0; i < 3; ++i)
				if (std::fabs(normal[i] - o.normal[i]) > NORMAL_EPSILON) return false;
			for (int i = 0; i < 4; ++i)
				if (std::fabs(tangent[i] - o.tangent[i]) > NORMAL_EPSILON) return false;
		}
		return true;
	}
};

// Build ExportMesh from an FBX mesh. Applies winding reversal and V-flip.
// Returns true on success (mesh had at least one triangle).
static bool BuildExportMesh(ufbx_mesh *mesh,
                            ufbx_node *meshNode,
                            const std::vector<ExportJoint> &joints,
                            ExportMesh &out,
                            const std::string &shaderOverride,
                            bool v12)
{
	// Find the first skin deformer on this mesh (if any).
	ufbx_skin_deformer *skin = nullptr;
	if (mesh->skin_deformers.count > 0) {
		skin = mesh->skin_deformers.data[0];
	}

	// Map cluster index -> (jointIdx, geometry_to_bone)
	struct ClusterBinding {
		int         jointIdx;
		ufbx_matrix geometry_to_bone;
		ufbx_matrix bind_to_world;   // needed for v12 normal/tangent transform
	};
	std::vector<ClusterBinding> clusterBindings;
	if (skin) {
		clusterBindings.resize(skin->clusters.count, { -1, {}, {} });
		for (size_t c = 0; c < skin->clusters.count; ++c) {
			ufbx_skin_cluster *cl = skin->clusters.data[c];
			clusterBindings[c].geometry_to_bone = cl->geometry_to_bone;
			clusterBindings[c].bind_to_world    = cl->bind_to_world;
			if (!cl->bone_node) continue;
			for (size_t j = 0; j < joints.size(); ++j) {
				if (joints[j].node == cl->bone_node) {
					clusterBindings[c].jointIdx = (int)j;
					break;
				}
			}
		}
	}

	// Precompute fallback geometry_to_bone for joint 0. Used either when the
	// mesh has no skin deformer at all (static mesh) or when a specific vertex
	// ended up with no valid weights (bad rig). offset = inverse(joint0_world)
	// * mesh_geometry_to_world * pos so joint0 * offset reproduces bind world.
	ufbx_matrix fallbackGeomToBone = ufbx_identity_matrix;
	bool haveFallback = false;
	if (!joints.empty()) {
		ufbx_quat j0q = DecompressQuat(joints[0].bindQuat);
		ufbx_matrix j0World = MatFromTR(joints[0].bindPos, j0q);
		ufbx_matrix j0Inv   = ufbx_matrix_invert(&j0World);
		fallbackGeomToBone  = ufbx_matrix_mul(&j0Inv, &meshNode->geometry_to_world);
		haveFallback = true;
	}

	// Material/shader resolution for this mesh.
	std::string shader = shaderOverride;
	if (shader.empty()) {
		if (mesh->materials.count > 0 && mesh->materials.data[0]) {
			ufbx_material *m = mesh->materials.data[0];
			shader = SanitizeShaderName(m->name.data, m->name.length);
		} else {
			shader = "default";
		}
	}
	out.shader = shader;
	out.name.assign(meshNode->name.data, meshNode->name.length);
	if (out.name.empty()) out.name = "mesh";

	out.verts.clear();
	out.tris.clear();
	out.weights.clear();

	std::vector<DedupeKey> uniqueVerts;
	uniqueVerts.reserve(mesh->num_vertices);

	// Iterate faces, fan-triangulate, emit triangles with reversed winding.
	for (size_t fi = 0; fi < mesh->faces.count; ++fi) {
		ufbx_face face = mesh->faces.data[fi];
		if (face.num_indices < 3) continue;

		// Fan (0, k, k+1) for k in [1, n-2], reverse winding -> (0, k+1, k).
		for (uint32_t k = 1; k + 1 < face.num_indices; ++k) {
			uint32_t corners[3] = {
				face.index_begin + 0,
				face.index_begin + k + 1,
				face.index_begin + k
			};

			ExportTri tri = {};

			for (int ci = 0; ci < 3; ++ci) {
				uint32_t corner = corners[ci];
				uint32_t vIdx   = mesh->vertex_indices.data[corner];
				ufbx_vec3 pos   = mesh->vertices.data[vIdx];

				ufbx_vec2 uv = { 0, 0 };
				if (mesh->vertex_uv.exists) {
					uv = ufbx_get_vertex_vec2(&mesh->vertex_uv, corner);
				}

				// Build the weight list for this corner.
				DedupeKey key;
				key.u = (float)uv.x;
				key.v = 1.0f - (float)uv.y;   // MD5 V-flip

				if (skin && vIdx < skin->vertices.count) {
					ufbx_skin_vertex sv = skin->vertices.data[vIdx];
					float total = 0.0f;
					for (uint32_t w = 0; w < sv.num_weights; ++w) {
						ufbx_skin_weight sw = skin->weights.data[sv.weight_begin + w];
						if (sw.weight < MIN_WEIGHT) continue;
						if (sw.cluster_index >= clusterBindings.size()) continue;
						const ClusterBinding &cb = clusterBindings[sw.cluster_index];
						if (cb.jointIdx < 0) continue;

						ExportWeight ew;
						ew.jointIdx = cb.jointIdx;
						ew.weight   = (float)sw.weight;
						ew.offset   = ufbx_transform_position(&cb.geometry_to_bone, pos);
						key.ws.push_back(ew);
						total += ew.weight;
					}
					if (total > 0.0f) {
						float inv = 1.0f / total;
						for (auto &w : key.ws) w.weight *= inv;
					}
				}

				// Fallback: static mesh, bind every vert to joint 0.
				if (key.ws.empty() && haveFallback) {
					ExportWeight ew;
					ew.jointIdx = 0;
					ew.weight   = 1.0f;
					ew.offset   = ufbx_transform_position(&fallbackGeomToBone, pos);
					key.ws.push_back(ew);
				}

				// v12: extract normal, tangent, bitangent sign, vertex color.
				// Normal/tangent are transformed into true bone-local space
				// using geometry_to_bone (same matrix used for weight offsets),
				// matching Blender's reference exporter convention.
				// In idTech4, idMat3 * idVec3 uses row-vector convention, so
				// ToMat3() * storedN effectively applies R_bind, meaning stored
				// must be R_bind^(-1) * n_world = the actual bone-local normal.
				float storedNormal[3]  = { 0, 0, 1 };
				float storedTangent[4] = { 1, 0, 0, 1 };
				float storedColor[4]   = { 1, 1, 1, 1 };
				bool  hasVertexColor   = false;

				if (v12) {
					// Find dominant joint (highest weight).
					int   domCluster = -1;
					float domWeight  = 0.0f;
					if (skin && vIdx < skin->vertices.count) {
						ufbx_skin_vertex sv = skin->vertices.data[vIdx];
						for (uint32_t w = 0; w < sv.num_weights; ++w) {
							ufbx_skin_weight sw = skin->weights.data[sv.weight_begin + w];
							if (sw.weight > domWeight && sw.cluster_index < clusterBindings.size()
							    && clusterBindings[sw.cluster_index].jointIdx >= 0) {
								domWeight  = (float)sw.weight;
								domCluster = (int)sw.cluster_index;
							}
						}
					}

					// Get geometry-space normal.
					ufbx_vec3 gNrm = { 0, 0, 1 };
					if (mesh->vertex_normal.exists) {
						gNrm = ufbx_get_vertex_vec3(&mesh->vertex_normal, corner);
					}

					// Get geometry-space tangent + compute bitangent sign.
					ufbx_vec3 gTan = { 1, 0, 0 };
					float     bSign = 1.0f;
					if (mesh->vertex_tangent.exists) {
						gTan = ufbx_get_vertex_vec3(&mesh->vertex_tangent, corner);
						if (mesh->vertex_bitangent.exists) {
							ufbx_vec3 gBtn = ufbx_get_vertex_vec3(&mesh->vertex_bitangent, corner);
							ufbx_vec3 cBtn = CrossVec3(gNrm, gTan);
							bSign = (float)(DotVec3(cBtn, gBtn) < 0.0 ? -1.0 : 1.0);
						}
					}

					// Transform into bone-local space. geometry_to_bone already
					// equals inverse(bind_to_world) * geometry_to_world, so its
					// rotation takes geometry-space directions to bone-local.
					if (domCluster >= 0) {
						const ufbx_matrix &g2b = clusterBindings[domCluster].geometry_to_bone;
						ufbx_vec3 sN = NormalizeVec3(TransformDir(g2b, gNrm));
						ufbx_vec3 sT = NormalizeVec3(TransformDir(g2b, gTan));
						storedNormal[0]  = (float)sN.x;
						storedNormal[1]  = (float)sN.y;
						storedNormal[2]  = (float)sN.z;
						storedTangent[0] = (float)sT.x;
						storedTangent[1] = (float)sT.y;
						storedTangent[2] = (float)sT.z;
					} else {
						// Fallback (static mesh / joint 0): apply the same
						// transform used for fallback weight offsets.
						ufbx_vec3 sN = NormalizeVec3(TransformDir(fallbackGeomToBone, gNrm));
						ufbx_vec3 sT = NormalizeVec3(TransformDir(fallbackGeomToBone, gTan));
						storedNormal[0]  = (float)sN.x;
						storedNormal[1]  = (float)sN.y;
						storedNormal[2]  = (float)sN.z;
						storedTangent[0] = (float)sT.x;
						storedTangent[1] = (float)sT.y;
						storedTangent[2] = (float)sT.z;
					}
					storedTangent[3] = bSign;

					// Vertex color.
					if (mesh->vertex_color.exists) {
						ufbx_vec4 c = ufbx_get_vertex_vec4(&mesh->vertex_color, corner);
						storedColor[0] = (float)c.x;
						storedColor[1] = (float)c.y;
						storedColor[2] = (float)c.z;
						storedColor[3] = (float)c.w;
						hasVertexColor = true;
					}

					// Pack into DedupeKey for normal-aware dedup.
					for (int i = 0; i < 3; ++i) key.normal[i]  = storedNormal[i];
					for (int i = 0; i < 4; ++i) key.tangent[i] = storedTangent[i];
				}

				// Deduplicate.
				int foundIdx = -1;
				for (size_t u = 0; u < uniqueVerts.size(); ++u) {
					if (uniqueVerts[u].Equals(key, v12)) { foundIdx = (int)u; break; }
				}
				if (foundIdx < 0) {
					foundIdx = (int)uniqueVerts.size();

					ExportVertex ev = {};
					ev.uv[0]       = key.u;
					ev.uv[1]       = key.v;
					ev.startWeight = (int)out.weights.size();
					ev.numWeights  = (int)key.ws.size();
					for (int i = 0; i < 3; ++i) ev.normal[i]  = storedNormal[i];
					for (int i = 0; i < 4; ++i) ev.tangent[i] = storedTangent[i];
					for (int i = 0; i < 4; ++i) ev.color[i]   = storedColor[i];
					ev.hasColor    = hasVertexColor;
					for (auto &w : key.ws) out.weights.push_back(w);
					out.verts.push_back(ev);
					uniqueVerts.push_back(std::move(key));
				}
				tri.v[ci] = foundIdx;
			}

			out.tris.push_back(tri);
		}
	}

	return !out.tris.empty();
}

// -----------------------------------------------------------------------------
// MD5 mesh writer
// -----------------------------------------------------------------------------

static bool WriteMD5Mesh(const char *path,
                         const std::vector<ExportJoint> &joints,
                         const std::vector<ExportMesh>  &meshes,
                         float scale,
                         float meshScale,
                         const std::string &commandLine,
                         bool v12)
{
	FILE *f = fopen(path, "wb");
	if (!f) { fprintf(stderr, "Cannot open '%s' for writing.\n", path); return false; }

	// Joint bind positions come out of ufbx in raw scene-space units (e.g. cm
	// for a Maya/Blender default FBX) because they're just translation columns
	// of the bind matrices. They use meshScale so they end up in output units.
	// Weight offsets are in bone-local space where the ancestor-chain scaling
	// already cancels out, so they use only the user-supplied scale.

	int version = v12 ? MD5_VERSION_V12 : MD5_VERSION;
	fprintf(f, "%s %d\n", MD5_VERSION_STRING, version);
	fprintf(f, "commandline \"%s\"\n\n", commandLine.c_str());
	fprintf(f, "numJoints %zu\n", joints.size());
	fprintf(f, "numMeshes %zu\n\n", meshes.size());

	fprintf(f, "joints {\n");
	for (size_t i = 0; i < joints.size(); ++i) {
		const ExportJoint &j = joints[i];
		const char *parentName = (j.parentIdx >= 0) ? joints[j.parentIdx].name.c_str() : "";
		fprintf(f, "\t\"%s\"\t%d ( %f %f %f ) ( %f %f %f )\t\t// %s\n",
		        j.name.c_str(), j.parentIdx,
		        j.bindPos.x * meshScale, j.bindPos.y * meshScale, j.bindPos.z * meshScale,
		        j.bindQuat.x, j.bindQuat.y, j.bindQuat.z,
		        parentName);
	}
	fprintf(f, "}\n");

	for (const ExportMesh &m : meshes) {
		fprintf(f, "\nmesh {\n");
		fprintf(f, "\t// meshes: %s\n", m.name.c_str());
		fprintf(f, "\tshader \"%s\"\n", m.shader.c_str());

		fprintf(f, "\n\tnumverts %zu\n", m.verts.size());
		for (size_t i = 0; i < m.verts.size(); ++i) {
			const ExportVertex &v = m.verts[i];
			if (v12) {
				fprintf(f, "\tvert %zu ( %f %f ) %d %d ( %f %f %f ) ( %f %f %f %f )\n",
				        i, v.uv[0], v.uv[1], v.startWeight, v.numWeights,
				        v.normal[0], v.normal[1], v.normal[2],
				        v.tangent[0], v.tangent[1], v.tangent[2], v.tangent[3]);
			} else {
				fprintf(f, "\tvert %zu ( %f %f ) %d %d\n",
				        i, v.uv[0], v.uv[1], v.startWeight, v.numWeights);
			}
		}

		fprintf(f, "\n\tnumtris %zu\n", m.tris.size());
		for (size_t i = 0; i < m.tris.size(); ++i) {
			const ExportTri &t = m.tris[i];
			fprintf(f, "\ttri %zu %d %d %d\n", i, t.v[0], t.v[1], t.v[2]);
		}

		fprintf(f, "\n\tnumweights %zu\n", m.weights.size());
		for (size_t i = 0; i < m.weights.size(); ++i) {
			const ExportWeight &w = m.weights[i];
			fprintf(f, "\tweight %zu %d %f ( %f %f %f )\n",
			        i, w.jointIdx, w.weight,
			        w.offset.x * scale, w.offset.y * scale, w.offset.z * scale);
		}

		// v12: optional vertex colors section (after weights, before closing brace).
		if (v12) {
			bool anyColor = false;
			for (const auto &v : m.verts) { if (v.hasColor) { anyColor = true; break; } }
			if (anyColor) {
				fprintf(f, "\n\tnumvertexcolors %zu\n", m.verts.size());
				for (size_t i = 0; i < m.verts.size(); ++i) {
					const ExportVertex &v = m.verts[i];
					fprintf(f, "\tvertexcolor %zu ( %f %f %f %f )\n",
					        i, v.color[0], v.color[1], v.color[2], v.color[3]);
				}
			}
		}

		fprintf(f, "}\n");
	}

	fclose(f);
	return true;
}

// -----------------------------------------------------------------------------
// Animation sampling + MD5 anim writer
// -----------------------------------------------------------------------------

// Compute per-frame world-space pos for a single skinned vertex given the
// frame's joint world matrices.
static ufbx_vec3 SkinVertex(const ExportMesh &m,
                            size_t vertIdx,
                            const std::vector<ufbx_matrix> &jointWorldAtFrame)
{
	const ExportVertex &v = m.verts[vertIdx];
	ufbx_vec3 p = { 0, 0, 0 };
	for (int i = 0; i < v.numWeights; ++i) {
		const ExportWeight &w = m.weights[v.startWeight + i];
		ufbx_vec3 wp = ufbx_transform_position(&jointWorldAtFrame[w.jointIdx], w.offset);
		p.x += wp.x * w.weight;
		p.y += wp.y * w.weight;
		p.z += wp.z * w.weight;
	}
	return p;
}

static bool WriteMD5Anim(const char *path,
                         ufbx_scene *scene,
                         ufbx_anim_stack *stack,
                         const std::vector<ExportJoint> &joints,
                         const std::vector<ExportMesh>  &meshes,
                         double fps,
                         float scale,
                         float meshScale,
                         const std::string &commandLine)
{
	// Two scales are needed here:
	//   meshScale -- applies to world-space quantities (frame bounds, root-
	//                joint translations). For a cm FBX this is ~0.01 times the
	//                user scale, so world positions come out in idTech4 units.
	//   scale     -- applies to parent-local joint translations. These come
	//                out of (parent_world)^-1 * child_world with the bone
	//                hierarchy's scale already cancelled, so only the user's
	//                -scale multiplier remains to be applied.

	// --- Determine frame count ------------------------------------------------
	double t0 = stack->time_begin;
	double t1 = stack->time_end;
	if (t1 <= t0) {
		// Stack with no explicit time range -- fall back to scene anim range.
		if (scene->anim && scene->anim->time_end > scene->anim->time_begin) {
			t0 = scene->anim->time_begin;
			t1 = scene->anim->time_end;
		}
	}
	if (t1 <= t0) {
		fprintf(stderr, "  skipping '%.*s': zero-length animation\n",
		        (int)stack->name.length, stack->name.data);
		return false;
	}

	int numFrames = (int)floor((t1 - t0) * fps + 0.5) + 1;
	if (numFrames < 1) numFrames = 1;

	// --- Sample every frame ---------------------------------------------------
	// frames[f][j] holds parent-local transform of joint j at frame f.
	std::vector<std::vector<JointFrame>> frames(numFrames,
	                                            std::vector<JointFrame>(joints.size()));

	// Per-frame bounds in world space.
	struct AABB { ufbx_vec3 lo, hi; bool valid; };
	std::vector<AABB> bounds(numFrames, { {0,0,0}, {0,0,0}, false });

	printf("  sampling %d frames at %.3f fps...\n", numFrames, fps);

	for (int f = 0; f < numFrames; ++f) {
		double time = t0 + (double)f / fps;
		if (time > t1) time = t1;

		ufbx_evaluate_opts eopts = {};
		eopts.evaluate_skinning = false;  // we do our own LBS for bounds
		ufbx_error err;
		ufbx_scene *ev = ufbx_evaluate_scene(scene, stack->anim, time, &eopts, &err);
		if (!ev) {
			fprintf(stderr, "ufbx_evaluate_scene failed at t=%f: %s\n", time, err.description.data);
			return false;
		}

		// Collect world-space transforms for every joint at this frame.
		std::vector<ufbx_matrix> jointWorld(joints.size());
		for (size_t j = 0; j < joints.size(); ++j) {
			const ExportJoint &ej = joints[j];
			ufbx_matrix m;
			if (ej.node) {
				ufbx_node *en = ev->nodes.data[ej.node->typed_id];
				m = en->node_to_world;
			} else {
				m = ufbx_identity_matrix;  // synthetic origin joint
			}
			jointWorld[j] = m;

			// Convert to local relative to MD5 parent joint.
			ufbx_matrix local;
			if (ej.parentIdx < 0) {
				local = m;
			} else {
				ufbx_matrix parentInv = ufbx_matrix_invert(&jointWorld[ej.parentIdx]);
				local = ufbx_matrix_mul(&parentInv, &m);
			}
			ufbx_transform lt = ufbx_matrix_to_transform(&local);
			frames[f][j].t = lt.translation;
			frames[f][j].q = CompressQuat(lt.rotation);
		}

		// Compute bounds at this frame via LBS.
		AABB &bb = bounds[f];
		for (const ExportMesh &m : meshes) {
			for (size_t vi = 0; vi < m.verts.size(); ++vi) {
				ufbx_vec3 p = SkinVertex(m, vi, jointWorld);
				p.x *= meshScale; p.y *= meshScale; p.z *= meshScale;
				if (!bb.valid) {
					bb.lo = bb.hi = p;
					bb.valid = true;
				} else {
					if (p.x < bb.lo.x) bb.lo.x = p.x;
					if (p.y < bb.lo.y) bb.lo.y = p.y;
					if (p.z < bb.lo.z) bb.lo.z = p.z;
					if (p.x > bb.hi.x) bb.hi.x = p.x;
					if (p.y > bb.hi.y) bb.hi.y = p.y;
					if (p.z > bb.hi.z) bb.hi.z = p.z;
				}
			}
		}
		if (!bb.valid) {
			// No mesh -- use joint positions (in world space, so meshScale).
			for (size_t j = 0; j < joints.size(); ++j) {
				ufbx_vec3 p = jointWorld[j].cols[3];
				p.x *= meshScale; p.y *= meshScale; p.z *= meshScale;
				if (!bb.valid) {
					bb.lo = bb.hi = p;
					bb.valid = true;
				} else {
					if (p.x < bb.lo.x) bb.lo.x = p.x;
					if (p.y < bb.lo.y) bb.lo.y = p.y;
					if (p.z < bb.lo.z) bb.lo.z = p.z;
					if (p.x > bb.hi.x) bb.hi.x = p.x;
					if (p.y > bb.hi.y) bb.hi.y = p.y;
					if (p.z > bb.hi.z) bb.hi.z = p.z;
				}
			}
		}

		ufbx_free_scene(ev);
	}

	// --- Compute animBits per joint (delta compress) --------------------------
	std::vector<int> animBits(joints.size(), 0);
	std::vector<int> firstComp(joints.size(), 0);
	std::vector<JointFrame> baseFrame(joints.size());

	int numAnimatedComponents = 0;
	for (size_t j = 0; j < joints.size(); ++j) {
		baseFrame[j] = frames[0][j];
		int bits = 0;
		for (int f = 1; f < numFrames; ++f) {
			const JointFrame &a = baseFrame[j];
			const JointFrame &b = frames[f][j];
			if (std::fabs(b.t.x - a.t.x) > XYZ_EPSILON)  bits |= ANIM_TX;
			if (std::fabs(b.t.y - a.t.y) > XYZ_EPSILON)  bits |= ANIM_TY;
			if (std::fabs(b.t.z - a.t.z) > XYZ_EPSILON)  bits |= ANIM_TZ;
			if (std::fabs(b.q.x - a.q.x) > QUAT_EPSILON) bits |= ANIM_QX;
			if (std::fabs(b.q.y - a.q.y) > QUAT_EPSILON) bits |= ANIM_QY;
			if (std::fabs(b.q.z - a.q.z) > QUAT_EPSILON) bits |= ANIM_QZ;
			if ((bits & 63) == 63) break;
		}
		animBits[j] = bits;
		if (bits) {
			firstComp[j] = numAnimatedComponents;
			for (int b = 0; b < 6; ++b) if (bits & (1 << b)) ++numAnimatedComponents;
		}
	}

	// --- Write file -----------------------------------------------------------
	FILE *f = fopen(path, "wb");
	if (!f) { fprintf(stderr, "Cannot open '%s' for writing.\n", path); return false; }

	fprintf(f, "%s %d\n", MD5_VERSION_STRING, MD5_VERSION);
	fprintf(f, "commandline \"%s\"\n\n", commandLine.c_str());
	fprintf(f, "numFrames %d\n", numFrames);
	fprintf(f, "numJoints %zu\n", joints.size());
	fprintf(f, "frameRate %d\n", (int)(fps + 0.5));
	fprintf(f, "numAnimatedComponents %d\n", numAnimatedComponents);

	// hierarchy
	fprintf(f, "\nhierarchy {\n");
	for (size_t j = 0; j < joints.size(); ++j) {
		const ExportJoint &ej = joints[j];
		const char *parentName = (ej.parentIdx >= 0) ? joints[ej.parentIdx].name.c_str() : "";
		fprintf(f, "\t\"%s\"\t%d %d %d\t// %s",
		        ej.name.c_str(), ej.parentIdx, animBits[j], firstComp[j], parentName);
		if (!animBits[j]) {
			fprintf(f, "\n");
		} else {
			fprintf(f, " ( ");
			for (int b = 0; b < 6; ++b) {
				if (animBits[j] & (1 << b)) fprintf(f, "%s ", ANIM_BIT_NAMES[b]);
			}
			fprintf(f, ")\n");
		}
	}
	fprintf(f, "}\n");

	// bounds
	fprintf(f, "\nbounds {\n");
	for (int fi = 0; fi < numFrames; ++fi) {
		const AABB &b = bounds[fi];
		fprintf(f, "\t( %f %f %f ) ( %f %f %f )\n",
		        b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z);
	}
	fprintf(f, "}\n");

	// baseframe -- root joint is in world space (meshScale); non-root joints
	// are parent-local (self-cancelled through parentInv, so just scale).
	fprintf(f, "\nbaseframe {\n");
	for (size_t j = 0; j < joints.size(); ++j) {
		const JointFrame &bf = baseFrame[j];
		float ts = (joints[j].parentIdx < 0) ? meshScale : scale;
		fprintf(f, "\t( %f %f %f ) ( %f %f %f )\n",
		        bf.t.x * ts, bf.t.y * ts, bf.t.z * ts,
		        bf.q.x, bf.q.y, bf.q.z);
	}
	fprintf(f, "}\n");

	// frames
	for (int fi = 0; fi < numFrames; ++fi) {
		fprintf(f, "\nframe %d {\n", fi);
		for (size_t j = 0; j < joints.size(); ++j) {
			if (!animBits[j]) continue;
			const JointFrame &fr = frames[fi][j];
			float ts = (joints[j].parentIdx < 0) ? meshScale : scale;
			fprintf(f, "\t");
			if (animBits[j] & ANIM_TX) fprintf(f, " %f", fr.t.x * ts);
			if (animBits[j] & ANIM_TY) fprintf(f, " %f", fr.t.y * ts);
			if (animBits[j] & ANIM_TZ) fprintf(f, " %f", fr.t.z * ts);
			if (animBits[j] & ANIM_QX) fprintf(f, " %f", fr.q.x);
			if (animBits[j] & ANIM_QY) fprintf(f, " %f", fr.q.y);
			if (animBits[j] & ANIM_QZ) fprintf(f, " %f", fr.q.z);
			fprintf(f, "\n");
		}
		fprintf(f, "}\n");
	}

	fclose(f);
	return true;
}

// -----------------------------------------------------------------------------
// Filename sanitization for anim stack names
// -----------------------------------------------------------------------------

static std::string SanitizeFilenameStem(const char *src, size_t len) {
	// FBX anim stacks are typically named "<objectname>|<actionname>" (Blender,
	// Maya, MotionBuilder all follow this convention -- '|' is FBX's namespace
	// separator). Strip everything up to and including the LAST '|' so the
	// filename is just the action name.
	for (size_t i = len; i > 0; --i) {
		if (src[i - 1] == '|') {
			src += i;
			len -= i;
			break;
		}
	}

	std::string out;
	out.reserve(len + 1);
	for (size_t i = 0; i < len; ++i) {
		char c = src[i];
		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		    (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.') {
			out += c;
		} else if (c == ' ') {
			out += '_';
		}
	}
	if (out.empty()) out = "anim";
	return out;
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------

static void PrintUsage() {
	fprintf(stderr,
		"fbx2md5 -- FBX to idTech4 MD5 mesh/anim converter\n\n"
		"Usage:\n"
		"  fbx2md5 input.fbx output [-scale X.X] [-fps N] [-noaxes] [-nounits]\n"
		"                           [-v12] [-noAnimPrefix]\n\n"
		"Options:\n"
		"  -scale X.X    Extra multiplier applied to all positions (default 1.0).\n"
		"  -fps N        Override sample rate (default: FBX scene fps).\n"
		"  -noaxes       Skip idTech4 axis conversion (keep FBX axes as-is).\n"
		"  -nounits      Skip FBX unit conversion (keep FBX units as-is).\n"
		"                Without this flag, scene is normalized so 1 FBX\n"
		"                world unit = 1 output unit (cm FBXs end up /100).\n"
		"  -v12          Output MD5 Version 12 mesh (per-vertex normals,\n"
		"                MikkTSpace tangents, optional vertex colors).\n"
		"                Anim output remains Version 10.\n"
		"  -noAnimPrefix Write anims as <stack>.md5anim instead of\n"
		"                <output>_<stack>.md5anim. The output stem's directory\n"
		"                (if any) is preserved.\n\n"
		"Note on anim stack names:\n"
		"  FBX tools (Blender, Maya, etc.) often prefix stack names with the\n"
		"  object namespace, e.g. \"imp|slash1\". Everything up to and including\n"
		"  the last '|' is stripped automatically, so you get 'slash1', not\n"
		"  'imp_slash1', in the output filename.\n\n"
		"Outputs:\n"
		"  output.md5mesh\n"
		"  output_<stackname>.md5anim (one per FBX animation stack)\n");
}

int main(int argc, char **argv) {
	if (argc < 3) { PrintUsage(); return 1; }

	const char *inputPath  = argv[1];
	const char *outputStem = argv[2];

	float  scale      = 1.0f;
	double fpsOverride = 0.0;
	bool   convertAxes = true;
	bool   convertUnits = true;
	bool   v12 = false;
	bool   noAnimPrefix = false;

	for (int i = 3; i < argc; ++i) {
		if (!strcmp(argv[i], "-scale") && i + 1 < argc) {
			scale = (float)atof(argv[++i]);
		} else if (!strcmp(argv[i], "-fps") && i + 1 < argc) {
			fpsOverride = atof(argv[++i]);
		} else if (!strcmp(argv[i], "-noaxes")) {
			convertAxes = false;
		} else if (!strcmp(argv[i], "-nounits")) {
			convertUnits = false;
		} else if (!strcmp(argv[i], "-v12")) {
			v12 = true;
		} else if (!strcmp(argv[i], "-noAnimPrefix") || !strcmp(argv[i], "-noanimprefix")) {
			noAnimPrefix = true;
		} else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			PrintUsage();
			return 0;
		} else {
			fprintf(stderr, "Unknown argument: %s\n", argv[i]);
			PrintUsage();
			return 1;
		}
	}

	// Reconstruct a command line string for the MD5 "commandline" field.
	std::string cmdLine;
	for (int i = 0; i < argc; ++i) {
		if (i) cmdLine += ' ';
		cmdLine += argv[i];
	}

	// --- Load FBX -------------------------------------------------------------
	ufbx_load_opts opts = {};
	opts.evaluate_skinning         = false;
	opts.generate_missing_normals  = v12;   // v12 needs normals; v10 doesn't
	opts.geometry_transform_handling = UFBX_GEOMETRY_TRANSFORM_HANDLING_MODIFY_GEOMETRY;
	opts.space_conversion          = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
	if (convertAxes) {
		opts.target_axes = ufbx_axes_right_handed_z_up;
	}
	// NOTE: we do NOT set target_unit_meters. Letting ufbx do unit conversion
	// interacts badly with ufbx_evaluate_scene (evaluated local transforms end
	// up scaled by an extra factor vs the static ones). Instead we compute a
	// unit-scale factor ourselves and apply it to mesh positions/offsets only
	// -- animation-time local transforms self-cancel the bone-hierarchy scale
	// when we compute (parent_world)^-1 * child_world, so they come out in the
	// right magnitude naturally.

	ufbx_error err;
	ufbx_scene *scene = ufbx_load_file(inputPath, &opts, &err);
	if (!scene) {
		char buf[512];
		ufbx_format_error(buf, sizeof(buf), &err);
		fprintf(stderr, "Failed to load '%s':\n%s\n", inputPath, buf);
		return 1;
	}

	printf("Loaded '%s'\n", inputPath);
	printf("  %zu nodes, %zu meshes, %zu skin deformers, %zu anim stacks\n",
	       scene->nodes.count, scene->meshes.count,
	       scene->skin_deformers.count, scene->anim_stacks.count);
	printf("  scene unit: %.6f m/unit, fps: %.3f\n",
	       (double)scene->settings.unit_meters,
	       scene->settings.frames_per_second);

	// Fold the FBX unit scale into the user scale so it applies uniformly to
	// mesh joint positions and weight offsets. Anim local-transform
	// translations self-correct via the parent-inverse cancellation and
	// should NOT be scaled again -- WriteMD5Anim skips meshScale for them.
	float meshScale = scale;
	if (convertUnits) {
		double u = scene->settings.unit_meters;
		if (u > 0.0) meshScale = (float)(scale * u);
	}
	printf("  scaling mesh positions by %.6f (unit %s, user -scale %g)\n",
	       (double)meshScale, convertUnits ? "on" : "off", (double)scale);
	if (v12) {
		printf("  output mode: MD5 Version 12 (per-vertex normals + tangents)\n");
	}

	// --- Joints ---------------------------------------------------------------
	std::vector<ExportJoint> joints = CollectJoints(scene);
	EnsureAtLeastOneJoint(joints);
	printf("  %zu joints\n", joints.size());

	// --- Meshes ---------------------------------------------------------------
	std::vector<ExportMesh> meshes;
	for (size_t mi = 0; mi < scene->meshes.count; ++mi) {
		ufbx_mesh *mesh = scene->meshes.data[mi];
		// A mesh may be instanced across multiple nodes; emit one ExportMesh per instance.
		for (size_t ni = 0; ni < mesh->instances.count; ++ni) {
			ufbx_node *meshNode = mesh->instances.data[ni];
			ExportMesh em;
			if (BuildExportMesh(mesh, meshNode, joints, em, "", v12)) {
				printf("  mesh '%s': %zu verts, %zu tris, %zu weights, shader '%s'\n",
				       em.name.c_str(), em.verts.size(), em.tris.size(),
				       em.weights.size(), em.shader.c_str());
				meshes.push_back(std::move(em));
			}
		}
	}

	if (meshes.empty()) {
		printf("  (no meshes -- skeleton/anim only output)\n");
	}

	// --- Write mesh -----------------------------------------------------------
	std::string meshPath = std::string(outputStem) + ".md5mesh";
	if (!WriteMD5Mesh(meshPath.c_str(), joints, meshes, scale, meshScale, cmdLine, v12)) {
		ufbx_free_scene(scene);
		return 1;
	}
	printf("Wrote %s\n", meshPath.c_str());

	// --- Animations -----------------------------------------------------------
	double fps = fpsOverride;
	if (fps <= 0.0) {
		fps = scene->settings.frames_per_second;
		if (fps <= 0.0) fps = 24.0;
	}
	printf("  using %.3f fps\n", fps);

	// Precompute the directory part of outputStem (everything up to the last
	// '/' or '\\', inclusive) so -noAnimPrefix writes anims into the same
	// directory as the mesh rather than the current working directory.
	std::string animDir;
	{
		const char *s = outputStem;
		const char *lastSep = nullptr;
		for (const char *p = s; *p; ++p) {
			if (*p == '/' || *p == '\\') lastSep = p;
		}
		if (lastSep) animDir.assign(s, (size_t)(lastSep - s + 1));
	}

	int animsWritten = 0;
	for (size_t si = 0; si < scene->anim_stacks.count; ++si) {
		ufbx_anim_stack *stack = scene->anim_stacks.data[si];

		std::string stackName = SanitizeFilenameStem(stack->name.data, stack->name.length);
		std::string animPath  = noAnimPrefix
			? (animDir + stackName + ".md5anim")
			: (std::string(outputStem) + "_" + stackName + ".md5anim");

		printf("anim stack '%.*s' -> %s\n",
		       (int)stack->name.length, stack->name.data, animPath.c_str());

		// Anim uses user `scale` only; the FBX unit conversion is already
		// implicit in the parent-inverse math used to form local transforms.
		if (WriteMD5Anim(animPath.c_str(), scene, stack, joints, meshes, fps, scale, meshScale, cmdLine)) {
			printf("Wrote %s\n", animPath.c_str());
			++animsWritten;
		}
	}

	if (scene->anim_stacks.count == 0) {
		printf("  (no animation stacks in FBX)\n");
	}

	ufbx_free_scene(scene);
	printf("Done. %zu mesh(es), %d anim(s).\n", meshes.size(), animsWritten);
	return 0;
}
