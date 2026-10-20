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

// MikkTSpace (canonical reference impl by Morten S. Mikkelsen, zlib license).
// Used as a fallback to compute tangents when the source FBX has none that
// ufbx can read. Some FBX exporters (and some DCC round-trips) drop the
// tangent layer or write it in a form ufbx doesn't surface. Since Substance
// Painter also bakes against MikkTSpace, recomputing here yields a tangent
// basis that matches the bake -- eliminating UV-shell shading seams that a
// constant (1,0,0) fallback would otherwise cause.
#include "mikktspace.h"

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

// MD5 joints carry no scale. Rigs frequently do -- Cascadeur parents the
// whole skeleton under a root scaled 0.01 (its cm/m conversion), Blender
// exports can carry the unit scale on the armature object -- and that scale
// propagates into every bone's world matrix. Everything we write must agree
// on ONE convention, so: joint world transforms are translation + rotation
// only (the scale is simply dropped), and anything expressed relative to a
// joint (weight offsets, parent-local anim transforms) is computed against
// that scale-free frame. Dropping the scale here and keeping it in the
// offsets is exactly the mismatch that produced 100x weight offsets against
// 1x joints.
static ufbx_matrix StripScale(const ufbx_matrix &m) {
	ufbx_transform t = ufbx_matrix_to_transform(&m);
	return MatFromTR(t.translation, t.rotation);
}

// Scale-free version of a cluster's geometry_to_bone. With
// bind_to_world = T*R*S we have geometry_to_bone = S^-1 * R^-1 * T^-1 * G,
// and the frame MD5 actually reconstructs is T*R, so the offset matrix we
// need is R^-1 * T^-1 * G = S * geometry_to_bone. Exact for non-uniform
// scale as well, and the identity when the rig carries none.
static ufbx_matrix ScaleFreeGeometryToBone(const ufbx_skin_cluster *cl) {
	ufbx_transform bt = ufbx_matrix_to_transform(&cl->bind_to_world);
	ufbx_transform s  = { { 0, 0, 0 }, ufbx_identity_quat, bt.scale };
	ufbx_matrix sm = ufbx_transform_to_matrix(&s);
	return ufbx_matrix_mul(&sm, &cl->geometry_to_bone);
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
// ---------------------------------------------------------------------------
// MikkTSpace fallback for FBX files with no readable tangents.
//
// We present the mesh to MikkTSpace using the SAME fan-triangulation and
// reversed winding that BuildExportMesh's main loop uses, so the per-corner
// tangents computed here line up 1:1 with the corners processed there. The
// output is indexed by a flat triangle-corner counter (tri*3 + ci), which the
// main loop reconstructs identically.
//
// Everything is in geometry space (raw ufbx positions/normals/UVs). The main
// loop then applies geometry_to_bone + Gram-Schmidt to these exactly as it
// does for source-provided tangents, so no special-casing downstream.

struct MikkFbxCtx {
    ufbx_mesh *mesh;
    // Flattened triangle list: each triangle stores its 3 source corner indices
    // (indices into mesh->vertex_indices / the vertex_* attribute streams),
    // in the reversed-winding order the main loop emits.
    std::vector<uint32_t> triCorners;   // size = numTris*3
    std::vector<ufbx_vec4> outTangent;  // size = numTris*3, xyz + sign
};

static int mkf_getNumFaces(const SMikkTSpaceContext *c) {
    MikkFbxCtx *ctx = (MikkFbxCtx*)c->m_pUserData;
    return (int)(ctx->triCorners.size() / 3);
}
static int mkf_getNumVerticesOfFace(const SMikkTSpaceContext *, const int) { return 3; }

static void mkf_getPosition(const SMikkTSpaceContext *c, float out[], const int f, const int v) {
    MikkFbxCtx *ctx = (MikkFbxCtx*)c->m_pUserData;
    uint32_t corner = ctx->triCorners[(size_t)f*3 + v];
    uint32_t vIdx = ctx->mesh->vertex_indices.data[corner];
    ufbx_vec3 p = ctx->mesh->vertices.data[vIdx];
    out[0]=(float)p.x; out[1]=(float)p.y; out[2]=(float)p.z;
}
static void mkf_getNormal(const SMikkTSpaceContext *c, float out[], const int f, const int v) {
    MikkFbxCtx *ctx = (MikkFbxCtx*)c->m_pUserData;
    uint32_t corner = ctx->triCorners[(size_t)f*3 + v];
    ufbx_vec3 n = { 0, 0, 1 };
    if (ctx->mesh->vertex_normal.exists) n = ufbx_get_vertex_vec3(&ctx->mesh->vertex_normal, corner);
    out[0]=(float)n.x; out[1]=(float)n.y; out[2]=(float)n.z;
}
static void mkf_getTexCoord(const SMikkTSpaceContext *c, float out[], const int f, const int v) {
    MikkFbxCtx *ctx = (MikkFbxCtx*)c->m_pUserData;
    uint32_t corner = ctx->triCorners[(size_t)f*3 + v];
    ufbx_vec2 uv = { 0, 0 };
    if (ctx->mesh->vertex_uv.exists) uv = ufbx_get_vertex_vec2(&ctx->mesh->vertex_uv, corner);
    out[0]=(float)uv.x; out[1]=(float)uv.y;
}
static void mkf_setTSpaceBasic(const SMikkTSpaceContext *c, const float t[], const float sign,
                               const int f, const int v) {
    MikkFbxCtx *ctx = (MikkFbxCtx*)c->m_pUserData;
    size_t idx = (size_t)f*3 + v;
    // MikkTSpace's sign convention: bitangent = sign * cross(N, T).
    // idTech4 MD5v12 stores Tw with the same meaning, so pass it through.
    ctx->outTangent[idx] = ufbx_vec4{ t[0], t[1], t[2], sign };
}

// Compute per-triangle-corner tangents for a mesh lacking source tangents.
// triCorners must be filled with the same corner ordering the main loop uses.
// Returns false if the mesh lacks the inputs MikkTSpace needs.
static bool ComputeMikkTangentsFBX(ufbx_mesh *mesh, MikkFbxCtx &ctx) {
    if (!mesh->vertex_normal.exists || !mesh->vertex_uv.exists) {
        std::fprintf(stderr,
            "  WARN: cannot compute MikkTSpace fallback (needs normals + UVs)\n");
        return false;
    }
    // Build the triangle-corner list with the SAME fan + reversed winding
    // as BuildExportMesh: for each face, fan (0, k+1, k) for k in [1, n-2].
    ctx.mesh = mesh;
    ctx.triCorners.clear();
    for (size_t fi = 0; fi < mesh->faces.count; ++fi) {
        ufbx_face face = mesh->faces.data[fi];
        if (face.num_indices < 3) continue;
        for (uint32_t k = 1; k + 1 < face.num_indices; ++k) {
            ctx.triCorners.push_back(face.index_begin + 0);
            ctx.triCorners.push_back(face.index_begin + k + 1);
            ctx.triCorners.push_back(face.index_begin + k);
        }
    }
    ctx.outTangent.assign(ctx.triCorners.size(), ufbx_vec4{1,0,0,1});

    SMikkTSpaceInterface iface = {};
    iface.m_getNumFaces          = mkf_getNumFaces;
    iface.m_getNumVerticesOfFace = mkf_getNumVerticesOfFace;
    iface.m_getPosition          = mkf_getPosition;
    iface.m_getNormal            = mkf_getNormal;
    iface.m_getTexCoord          = mkf_getTexCoord;
    iface.m_setTSpaceBasic       = mkf_setTSpaceBasic;

    SMikkTSpaceContext mctx;
    mctx.m_pInterface = &iface;
    mctx.m_pUserData  = &ctx;

    if (!genTangSpaceDefault(&mctx)) {
        std::fprintf(stderr, "  WARN: MikkTSpace genTangSpaceDefault failed\n");
        return false;
    }
    return true;
}

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
			clusterBindings[c].geometry_to_bone = ScaleFreeGeometryToBone(cl);
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

	// If v12 is requested but the FBX carries no tangents ufbx can read,
	// precompute them with MikkTSpace (the algorithm Substance/Blender/glTF
	// all use). Without this, we'd fall back to a constant (1,0,0) tangent on
	// every vertex, which the engine turns into UV-shell seams. The computed
	// tangents are indexed by a flat triangle-corner counter that advances in
	// lockstep with the emit loop below.
	MikkFbxCtx mikkCtx;
	bool useMikkFallback = false;
	if (v12 && !mesh->vertex_tangent.exists) {
		std::fprintf(stderr,
			"  INFO: FBX has no readable tangents -- computing MikkTSpace fallback\n");
		if (ComputeMikkTangentsFBX(mesh, mikkCtx)) {
			useMikkFallback = true;
			std::fprintf(stderr, "  INFO: MikkTSpace produced %zu corner tangents\n",
				mikkCtx.outTangent.size());
		}
	}
	size_t mikkTriCounter = 0;   // advances once per emitted triangle

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
					} else if (useMikkFallback) {
						// Pull the precomputed MikkTSpace tangent for this exact
						// triangle-corner. The flat index matches because the
						// fallback builder used the same fan + winding.
						size_t flatIdx = mikkTriCounter * 3 + (size_t)ci;
						if (flatIdx < mikkCtx.outTangent.size()) {
							ufbx_vec4 mt = mikkCtx.outTangent[flatIdx];
							gTan  = ufbx_vec3{ mt.x, mt.y, mt.z };
							bSign = (float)mt.w;
						}
					}

					// Transform into bone-local space. geometry_to_bone already
					// equals inverse(bind_to_world) * geometry_to_world, so its
					// rotation takes geometry-space directions to bone-local.
					//
					// After both N and T are in bone-local space, apply a
					// Gram-Schmidt step to T against N. This guarantees the
					// written tangent is perpendicular to the written normal
					// even if (a) the dominant-bone matrix carries scale, or
					// (b) the source FBX tangents were slightly off-perpendi-
					// cular to begin with (some DCC exports cut corners). The
					// engine's bitangent reconstruction `B = cross(N,T)*tw`
					// only produces a clean orthonormal frame -- and thus only
					// shades normal maps cleanly across UV-shell seams -- when
					// T is exactly perpendicular to N. See MD5v12 spec 2.1.
					ufbx_vec3 sN, sT;
					if (domCluster >= 0) {
						const ufbx_matrix &g2b = clusterBindings[domCluster].geometry_to_bone;
						sN = NormalizeVec3(TransformDir(g2b, gNrm));
						sT = NormalizeVec3(TransformDir(g2b, gTan));
					} else {
						// Fallback (static mesh / joint 0).
						sN = NormalizeVec3(TransformDir(fallbackGeomToBone, gNrm));
						sT = NormalizeVec3(TransformDir(fallbackGeomToBone, gTan));
					}

					// Gram-Schmidt: project T off N, renormalize.
					double NdotT = sN.x*sT.x + sN.y*sT.y + sN.z*sT.z;
					sT.x = (ufbx_real)(sT.x - sN.x * NdotT);
					sT.y = (ufbx_real)(sT.y - sN.y * NdotT);
					sT.z = (ufbx_real)(sT.z - sN.z * NdotT);
					double tLen = sqrt(sT.x*sT.x + sT.y*sT.y + sT.z*sT.z);
					if (tLen > 1e-9) {
						sT.x = (ufbx_real)(sT.x / tLen);
						sT.y = (ufbx_real)(sT.y / tLen);
						sT.z = (ufbx_real)(sT.z / tLen);
					} else {
						// Degenerate: pick any perpendicular to N.
						ufbx_vec3 axis = (fabs(sN.z) < 0.9) ? ufbx_vec3{0,0,1} : ufbx_vec3{1,0,0};
						sT = NormalizeVec3(CrossVec3(sN, axis));
					}

					storedNormal[0]  = (float)sN.x;
					storedNormal[1]  = (float)sN.y;
					storedNormal[2]  = (float)sN.z;
					storedTangent[0] = (float)sT.x;
					storedTangent[1] = (float)sT.y;
					storedTangent[2] = (float)sT.z;
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
			++mikkTriCounter;   // keep MikkTSpace flat-corner index in lockstep
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
			// Weight offsets are in bone-local geometry space and must be in the
			// SAME output units as the joint bind positions (which use meshScale).
			// Using plain `scale` here left offsets in raw cm while joints were in
			// inches -- a 1/unit_meters mismatch that made the whole mesh ~25.4x
			// oversized (and, downstream, the anim bounds wrong). meshScale folds
			// in both the unit conversion and the user -scale.
			fprintf(f, "\tweight %zu %d %f ( %f %f %f )\n",
			        i, w.jointIdx, w.weight,
			        w.offset.x * meshScale, w.offset.y * meshScale, w.offset.z * meshScale);
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
                         const std::string &commandLine,
                         const std::string &rootMotionJoint,
                         bool rootMotionXYZ)
{
	// Scale handling:
	//   meshScale (= user -scale * scene unit_meters) converts EVERY position
	//   from raw FBX file units into output units. Because we never call ufbx
	//   with target_unit_meters, world AND parent-local translations are both
	//   in raw file units, so both use meshScale. (Genuine per-bone hierarchy
	//   scale is already baked into the parent-local translation by ufbx and is
	//   orthogonal to this uniform unit conversion.)
	//

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
	printf("  stack range %.4f .. %.4f s (scene anim %.4f .. %.4f s)\n",
	       stack->time_begin, stack->time_end,
	       scene->anim ? scene->anim->time_begin : 0.0,
	       scene->anim ? scene->anim->time_end : 0.0);

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
				// Scale-free (see StripScale): the parent-local transform
				// below must be formed between the same T*R frames the
				// md5mesh joints use, or a scaled rig root leaks its scale
				// into every child translation.
				ufbx_node *en = ev->nodes.data[ej.node->typed_id];
				m = StripScale(en->node_to_world);
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
			// No mesh -- use joint positions (world space, meshScale).
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

	// --- Root motion transfer -------------------------------------------------
	// idTech4 reads locomotion from joint 0 ("origin") only: on load it strips
	// origin's translation from every frame and keeps the last-frame value as
	// the anim's total move delta (idMD5Anim::LoadAnim). Any other joint's
	// translation is played back literally, so a pelvis that travels makes the
	// mesh run away from its entity. Some tools (Cascadeur) always animate the
	// hips and leave the root parked. This moves the travel of <joint>
	// (its displacement from frame 0, XY by default, Z bob stays) onto origin.
	// Both joints end up in the same world pose, so the visible result is
	// unchanged -- only which joint carries it, and bounds need no update.
	if (!rootMotionJoint.empty()) {
		int k = -1;
		for (size_t j = 0; j < joints.size(); ++j)
			if (joints[j].name == rootMotionJoint) { k = (int)j; break; }
		if (k < 0) {
			fprintf(stderr, "  WARN: rootmotion joint '%s' not found -- no transfer\n", rootMotionJoint.c_str());
		} else if (joints[k].parentIdx != 0) {
			fprintf(stderr, "  WARN: rootmotion joint '%s' is not a direct child of '%s' -- no transfer\n",
			        rootMotionJoint.c_str(), joints[0].name.c_str());
		} else {
			ufbx_vec3 base = frames[0][k].t;
			for (int f = 0; f < numFrames; ++f) {
				ufbx_vec3 d = { frames[f][k].t.x - base.x, frames[f][k].t.y - base.y,
				                rootMotionXYZ ? frames[f][k].t.z - base.z : 0 };
				// d is in origin's local frame; origin's own translation is in
				// world space (it has no parent), so rotate by origin's rotation.
				ufbx_vec3 dw = ufbx_quat_rotate_vec3(DecompressQuat(frames[f][0].q), d);
				frames[f][0].t.x += dw.x; frames[f][0].t.y += dw.y; frames[f][0].t.z += dw.z;
				frames[f][k].t.x -= d.x;  frames[f][k].t.y -= d.y;  frames[f][k].t.z -= d.z;
			}
			ufbx_vec3 total = frames[numFrames - 1][0].t;
			printf("  root motion: moved '%s' travel onto '%s', total delta ( %.2f %.2f %.2f )\n",
			       rootMotionJoint.c_str(), joints[0].name.c_str(),
			       total.x * meshScale, total.y * meshScale, total.z * meshScale);
		}
	}

	// --- Bounds relative to root travel --------------------------------------
	// The engine strips origin's translation on load (frame minus baseframe)
	// and, when it lets the anim move the entity, adds that per-frame origin
	// offset back onto the stored bounds (idAnimBlend::AddBounds). So the file
	// must hold bounds with the root travel taken out, or the box walks away
	// from the mesh (or is counted twice). Applies whether the root was
	// animated by the tool or by the transfer above.
	{
		ufbx_vec3 base = frames[0][0].t;
		for (int f = 0; f < numFrames; ++f) {
			if (!bounds[f].valid) continue;
			ufbx_vec3 d = { (frames[f][0].t.x - base.x) * meshScale,
			                (frames[f][0].t.y - base.y) * meshScale,
			                (frames[f][0].t.z - base.z) * meshScale };
			bounds[f].lo.x -= d.x; bounds[f].lo.y -= d.y; bounds[f].lo.z -= d.z;
			bounds[f].hi.x -= d.x; bounds[f].hi.y -= d.y; bounds[f].hi.z -= d.z;
		}
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

	// A stack where no joint moves is a pose, not an animation. Some tools
	// (Cascadeur) always export a stack of some minimum length, so an "init"
	// pose would otherwise come out as dozens of identical frames.
	if (numAnimatedComponents == 0 && numFrames > 1) {
		printf("  no joint animates -- collapsing %d identical frames to 1\n", numFrames);
		numFrames = 1;
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

	// baseframe -- all joint translations are in raw FBX file units and are
	// converted to output units with meshScale (= user scale * unit_meters).
	fprintf(f, "\nbaseframe {\n");
	for (size_t j = 0; j < joints.size(); ++j) {
		const JointFrame &bf = baseFrame[j];
		// Every translation ufbx returns is in raw FBX file units (we deliberately
		// do NOT set target_unit_meters), so ALL of them -- root and non-root,
		// world and parent-local -- take the same meshScale unit conversion. The
		// old root/non-root split wrongly assumed parent-inverse cancels the FBX
		// unit scale; it does not, which flung skinned meshes ~1/unit_meters away.
		float ts = meshScale;
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
			float ts = meshScale;  // raw file units -> output units (see baseframe note)
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
// Options: command line and config file
// -----------------------------------------------------------------------------

// One entry per "shader" directive: any mesh whose NODE name or MATERIAL name
// matches `match` (case-insensitive) is written with `shader` instead.
struct ShaderMap { std::string match; std::string shader; };

// One entry per "anim" directive. An empty name means "derive from the FBX
// stack name" (same cleanup as the command-line path).
struct AnimSpec { std::string fbx; std::string name; };

struct Options {
	float       scale        = 1.0f;
	double      fps          = 0.0;      // 0 = take it from the FBX
	bool        convertAxes  = true;
	bool        convertUnits = true;
	bool        v12          = false;
	bool        noAnimPrefix = false;
	bool        noMesh       = false;    // skip the .md5mesh (anims only)
	std::string meshFbx;                 // FBX that carries the skin + skeleton
	std::string outputStem;
	std::vector<ShaderMap>   shaders;
	std::vector<std::string> skipMeshes; // mesh node names to leave out
	std::vector<AnimSpec>    anims;      // explicit anim list; empty = stacks of meshFbx
	std::string rootMotionJoint;         // move this joint's travel onto joint 0
	bool        rootMotionXYZ = false;   // include Z (default: horizontal only)
};

static bool IEquals(const std::string &a, const std::string &b) {
	if (a.size() != b.size()) return false;
	for (size_t i = 0; i < a.size(); ++i)
		if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return false;
	return true;
}

static std::string DirOf(const std::string &path) {
	size_t p = path.find_last_of("/\\");
	return (p == std::string::npos) ? std::string() : path.substr(0, p + 1);
}

static bool IsAbsolutePath(const std::string &p) {
	if (p.empty()) return false;
	if (p[0] == '/' || p[0] == '\\') return true;
	return p.size() > 1 && p[1] == ':';
}

// Paths inside a config file are relative to the config file itself, so a
// character's folder can carry its own config and be converted from anywhere.
static std::string ResolveAgainst(const std::string &dir, const std::string &p) {
	return IsAbsolutePath(p) || dir.empty() ? p : dir + p;
}

// Split a config line into tokens. Double quotes group words (for paths with
// spaces); '#' and "//" start a comment.
static std::vector<std::string> TokenizeLine(const std::string &line) {
	std::vector<std::string> out;
	std::string cur;
	bool inQuote = false, have = false;
	for (size_t i = 0; i < line.size(); ++i) {
		char c = line[i];
		if (inQuote) {
			if (c == '"') inQuote = false; else cur += c;
			continue;
		}
		if (c == '#' || (c == '/' && i + 1 < line.size() && line[i + 1] == '/')) break;
		if (c == '"') { inQuote = true; have = true; continue; }
		if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
			if (have) { out.push_back(cur); cur.clear(); have = false; }
			continue;
		}
		cur += c; have = true;
	}
	if (have) out.push_back(cur);
	return out;
}

// Config file grammar (one directive per line, see README):
//   mesh <file.fbx>            output <stem>            nomesh
//   scale <f>   fps <n>   noaxes   nounits   v12   noanimprefix
//   skipmesh <node name>
//   shader <node or material name> <shader>
//   anim <file.fbx> [<anim name>]
static bool ParseConfig(const char *path, Options &o) {
	FILE *f = fopen(path, "rb");
	if (!f) { fprintf(stderr, "Cannot open config '%s'.\n", path); return false; }
	std::string dir = DirOf(path);
	char buf[4096];
	int lineNo = 0;
	bool ok = true;
	while (fgets(buf, sizeof(buf), f)) {
		++lineNo;
		std::vector<std::string> t = TokenizeLine(buf);
		if (t.empty()) continue;
		std::string key = t[0];
		for (auto &c : key) c = (char)tolower((unsigned char)c);
		size_t n = t.size() - 1;
		auto need = [&](size_t want) {
			if (n < want) {
				fprintf(stderr, "%s(%d): '%s' needs %zu argument(s)\n", path, lineNo, t[0].c_str(), want);
				ok = false;
				return false;
			}
			return true;
		};
		if      (key == "mesh")         { if (need(1)) o.meshFbx = ResolveAgainst(dir, t[1]); }
		else if (key == "output")       { if (need(1)) o.outputStem = ResolveAgainst(dir, t[1]); }
		else if (key == "scale")        { if (need(1)) o.scale = (float)atof(t[1].c_str()); }
		else if (key == "fps")          { if (need(1)) o.fps = atof(t[1].c_str()); }
		else if (key == "noaxes")       { o.convertAxes = false; }
		else if (key == "nounits")      { o.convertUnits = false; }
		else if (key == "v12")          { o.v12 = true; }
		else if (key == "noanimprefix") { o.noAnimPrefix = true; }
		else if (key == "nomesh")       { o.noMesh = true; }
		else if (key == "skipmesh")     { if (need(1)) o.skipMeshes.push_back(t[1]); }
		else if (key == "shader")       { if (need(2)) o.shaders.push_back({ t[1], t[2] }); }
		else if (key == "anim")         { if (need(1)) o.anims.push_back({ ResolveAgainst(dir, t[1]), n >= 2 ? t[2] : "" }); }
		else if (key == "rootmotion")   { if (need(1)) { o.rootMotionJoint = t[1]; o.rootMotionXYZ = (n >= 2 && IEquals(t[2], "xyz")); } }
		else {
			fprintf(stderr, "%s(%d): unknown directive '%s'\n", path, lineNo, t[0].c_str());
			ok = false;
		}
	}
	fclose(f);
	if (o.meshFbx.empty() && o.anims.empty()) {
		fprintf(stderr, "%s: needs at least a 'mesh' or an 'anim' line\n", path);
		ok = false;
	}
	if (o.outputStem.empty()) {
		fprintf(stderr, "%s: 'output' is required\n", path);
		ok = false;
	}
	return ok;
}

static void PrintUsage() {
	fprintf(stderr,
		"fbx2md5 -- FBX to idTech4 MD5 mesh/anim converter\n\n"
		"Usage:\n"
		"  fbx2md5 input.fbx output [options]\n"
		"  fbx2md5 -cfg character.cfg [options]\n"
		"  fbx2md5 -listMaterials input.fbx\n\n"
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
		"                (if any) is preserved.\n"
		"  -shader <name>=<shader>\n"
		"                Write meshes whose node or material name is <name>\n"
		"                with <shader> instead. Repeatable.\n"
		"  -skipMesh <name>\n"
		"                Leave the mesh node <name> out of the md5mesh. The\n"
		"                skeleton is unaffected. Repeatable.\n"
		"  -anim <file.fbx>[=<name>]\n"
		"                Convert the animation in <file.fbx> (which may carry\n"
		"                no mesh) against input.fbx's skeleton, written as\n"
		"                <name>.md5anim next to the mesh. Repeatable; when\n"
		"                given, input.fbx's own stacks are NOT exported.\n"
		"  -noMesh       Skip the md5mesh; only write anims.\n"
		"  -rootMotion <joint>[,xyz]\n"
		"                Move <joint>'s travel (displacement from frame 0,\n"
		"                horizontal unless ',xyz') onto the root joint, which\n"
		"                is the only joint idTech4 reads locomotion from.\n"
		"  -cfg <file>   Read the same settings from a config file (see\n"
		"                README). Command-line options are applied after it.\n"
		"  -listMaterials <file.fbx>\n"
		"                Print each mesh node and its material names as ready\n"
		"                to paste 'shader' lines, then exit.\n\n"
		"Note on anim stack names:\n"
		"  FBX tools (Blender, Maya, etc.) often prefix stack names with the\n"
		"  object namespace, e.g. \"imp|slash1\". Everything up to and including\n"
		"  the last '|' is stripped automatically, so you get 'slash1', not\n"
		"  'imp_slash1', in the output filename.\n\n"
		"Outputs:\n"
		"  output.md5mesh\n"
		"  output_<stackname>.md5anim (one per FBX animation stack)\n");
}

// `fbx2md5 -listMaterials file.fbx`: print every mesh node with its material
// names, in the form the config's "shader" directive wants, so the names can
// be pasted straight into a .cfg (or checked to be already right).
static int ListMaterials(const char *path) {
	ufbx_load_opts opts = {};
	opts.ignore_geometry  = false;
	opts.ignore_animation = true;
	ufbx_error err;
	ufbx_scene *scene = ufbx_load_file(path, &opts, &err);
	if (!scene) {
		char buf[512];
		ufbx_format_error(buf, sizeof(buf), &err);
		fprintf(stderr, "Failed to load '%s':\n%s\n", path, buf);
		return 1;
	}
	printf("%s: %zu mesh node(s)\n\n", path, scene->meshes.count);
	for (size_t mi = 0; mi < scene->meshes.count; ++mi) {
		ufbx_mesh *mesh = scene->meshes.data[mi];
		for (size_t ni = 0; ni < mesh->instances.count; ++ni) {
			ufbx_node *n = mesh->instances.data[ni];
			printf("mesh \"%.*s\"  (%zu verts, %zu faces, %zu material slot(s))\n",
			       (int)n->name.length, n->name.data,
			       mesh->num_vertices, mesh->faces.count, mesh->materials.count);
			if (mesh->materials.count == 0) {
				printf("    no material -- md5mesh shader will be the node name:\n");
				printf("    shader \"%.*s\" \"<md5 material>\"\n", (int)n->name.length, n->name.data);
			}
			for (size_t m = 0; m < mesh->materials.count; ++m) {
				ufbx_material *mat = mesh->materials.data[m];
				if (!mat) continue;
				std::string san = SanitizeShaderName(mat->name.data, mat->name.length);
				printf("    shader \"%.*s\" \"<md5 material>\"%s\n",
				       (int)mat->name.length, mat->name.data,
				       m ? "   # slot >0: NOT used, only the first material is written" : "");
				if (san != std::string(mat->name.data, mat->name.length))
					printf("        (written as \"%s\" when not remapped)\n", san.c_str());
			}
		}
	}
	ufbx_free_scene(scene);
	return 0;
}

static bool ParseCommandLine(int argc, char **argv, Options &o) {
	int i = 1;
	bool haveCfg = false;
	if (i < argc && !strcmp(argv[i], "-cfg")) {
		if (i + 1 >= argc) { PrintUsage(); return false; }
		if (!ParseConfig(argv[i + 1], o)) return false;
		haveCfg = true;
		i += 2;
	} else {
		if (argc < 3) { PrintUsage(); return false; }
		o.meshFbx    = argv[1];
		o.outputStem = argv[2];
		i = 3;
	}

	for (; i < argc; ++i) {
		const char *a = argv[i];
		if (!strcmp(a, "-scale") && i + 1 < argc) {
			o.scale = (float)atof(argv[++i]);
		} else if (!strcmp(a, "-fps") && i + 1 < argc) {
			o.fps = atof(argv[++i]);
		} else if (!strcmp(a, "-noaxes")) {
			o.convertAxes = false;
		} else if (!strcmp(a, "-nounits")) {
			o.convertUnits = false;
		} else if (!strcmp(a, "-v12")) {
			o.v12 = true;
		} else if (!strcmp(a, "-noAnimPrefix") || !strcmp(a, "-noanimprefix")) {
			o.noAnimPrefix = true;
		} else if (!strcmp(a, "-noMesh") || !strcmp(a, "-nomesh")) {
			o.noMesh = true;
		} else if ((!strcmp(a, "-rootMotion") || !strcmp(a, "-rootmotion")) && i + 1 < argc) {
			std::string s = argv[++i];
			size_t c = s.find(',');
			o.rootMotionJoint = s.substr(0, c);
			o.rootMotionXYZ = (c != std::string::npos && IEquals(s.substr(c + 1), "xyz"));
		} else if (!strcmp(a, "-shader") && i + 1 < argc) {
			std::string s = argv[++i];
			size_t eq = s.find('=');
			if (eq == std::string::npos || eq == 0 || eq + 1 >= s.size()) {
				fprintf(stderr, "-shader expects <name>=<shader>, got '%s'\n", s.c_str());
				return false;
			}
			o.shaders.push_back({ s.substr(0, eq), s.substr(eq + 1) });
		} else if ((!strcmp(a, "-skipMesh") || !strcmp(a, "-skipmesh")) && i + 1 < argc) {
			o.skipMeshes.push_back(argv[++i]);
		} else if (!strcmp(a, "-anim") && i + 1 < argc) {
			std::string s = argv[++i];
			size_t eq = s.find('=');
			if (eq == std::string::npos) o.anims.push_back({ s, "" });
			else o.anims.push_back({ s.substr(0, eq), s.substr(eq + 1) });
		} else if (!strcmp(a, "-cfg")) {
			fprintf(stderr, "-cfg must be the first argument\n");
			return false;
		} else if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
			PrintUsage();
			return false;
		} else {
			fprintf(stderr, "Unknown argument: %s\n", a);
			PrintUsage();
			return false;
		}
	}
	(void)haveCfg;
	return true;
}

// -----------------------------------------------------------------------------
// Scene loading
// -----------------------------------------------------------------------------

static ufbx_scene *LoadScene(const std::string &path, const Options &o, float &meshScale) {
	ufbx_load_opts opts = {};
	opts.evaluate_skinning         = false;
	opts.generate_missing_normals  = o.v12;   // v12 needs normals; v10 doesn't
	opts.geometry_transform_handling = UFBX_GEOMETRY_TRANSFORM_HANDLING_MODIFY_GEOMETRY;
	opts.space_conversion          = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
	if (o.convertAxes) {
		opts.target_axes = ufbx_axes_right_handed_z_up;
	}
	// NOTE: we do NOT set target_unit_meters. Letting ufbx do unit conversion
	// interacts badly with ufbx_evaluate_scene (evaluated local transforms end
	// up scaled by an extra factor vs the static ones). Instead we compute a
	// unit-scale factor ourselves and apply it to every position we write.

	ufbx_error err;
	ufbx_scene *scene = ufbx_load_file(path.c_str(), &opts, &err);
	if (!scene) {
		char buf[512];
		ufbx_format_error(buf, sizeof(buf), &err);
		fprintf(stderr, "Failed to load '%s':\n%s\n", path.c_str(), buf);
		return nullptr;
	}

	printf("Loaded '%s'\n", path.c_str());
	printf("  %zu nodes, %zu meshes, %zu skin deformers, %zu anim stacks\n",
	       scene->nodes.count, scene->meshes.count,
	       scene->skin_deformers.count, scene->anim_stacks.count);
	printf("  scene unit: %.6f m/unit, fps: %.3f\n",
	       (double)scene->settings.unit_meters,
	       scene->settings.frames_per_second);

	// Fold the FBX unit scale into the user scale so it applies uniformly to
	// joint positions, weight offsets and anim translations.
	meshScale = o.scale;
	if (o.convertUnits) {
		double u = scene->settings.unit_meters;
		if (u > 0.0) meshScale = (float)(o.scale * u);
	}
	printf("  scaling positions by %.6f (unit %s, user -scale %g)\n",
	       (double)meshScale, o.convertUnits ? "on" : "off", (double)o.scale);
	return scene;
}

// An anim FBX exported separately from the mesh must carry the same skeleton:
// the engine matches md5anim joints to the md5mesh by name AND parent, in
// order, and the bounds we compute skin the mesh with this file's joints.
static bool SkeletonsMatch(const std::vector<ExportJoint> &mesh,
                           const std::vector<ExportJoint> &anim,
                           const std::string &animPath)
{
	if (mesh.size() != anim.size()) {
		fprintf(stderr, "  ERROR: '%s' has %zu joints, mesh has %zu -- skipping\n",
		        animPath.c_str(), anim.size(), mesh.size());
		return false;
	}
	for (size_t j = 0; j < mesh.size(); ++j) {
		if (mesh[j].name != anim[j].name || mesh[j].parentIdx != anim[j].parentIdx) {
			fprintf(stderr, "  ERROR: '%s' joint %zu is '%s' (parent %d), mesh has '%s' (parent %d) -- skipping\n",
			        animPath.c_str(), j, anim[j].name.c_str(), anim[j].parentIdx,
			        mesh[j].name.c_str(), mesh[j].parentIdx);
			return false;
		}
	}
	return true;
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------

int main(int argc, char **argv) {
	if (argc >= 3 && (!strcmp(argv[1], "-listMaterials") || !strcmp(argv[1], "-listmaterials"))) {
		return ListMaterials(argv[2]);
	}

	Options o;
	if (!ParseCommandLine(argc, argv, o)) return 1;

	// Reconstruct a command line string for the MD5 "commandline" field.
	std::string cmdLine;
	for (int i = 0; i < argc; ++i) {
		if (i) cmdLine += ' ';
		cmdLine += argv[i];
	}

	double fps = o.fps;

	// --- Mesh scene: skeleton + skin ------------------------------------------
	// With no 'mesh' the first anim FBX supplies the skeleton and the anims get
	// joint-only bounds (warned below).
	std::string skelPath = !o.meshFbx.empty() ? o.meshFbx : o.anims[0].fbx;
	float meshScale = 1.0f;
	ufbx_scene *scene = LoadScene(skelPath, o, meshScale);
	if (!scene) return 1;

	std::vector<ExportJoint> joints = CollectJoints(scene);
	EnsureAtLeastOneJoint(joints);
	printf("  %zu joints\n", joints.size());

	std::vector<ExportMesh> meshes;
	for (size_t mi = 0; mi < scene->meshes.count; ++mi) {
		ufbx_mesh *mesh = scene->meshes.data[mi];
		// A mesh may be instanced across multiple nodes; emit one ExportMesh per instance.
		for (size_t ni = 0; ni < mesh->instances.count; ++ni) {
			ufbx_node *meshNode = mesh->instances.data[ni];
			std::string nodeName(meshNode->name.data, meshNode->name.length);
			std::string matName;
			if (mesh->materials.count > 0 && mesh->materials.data[0]) {
				matName.assign(mesh->materials.data[0]->name.data, mesh->materials.data[0]->name.length);
			}

			bool skip = false;
			for (const auto &s : o.skipMeshes) if (IEquals(s, nodeName)) skip = true;
			if (skip) {
				printf("  mesh '%s': skipped\n", nodeName.c_str());
				continue;
			}

			std::string shaderOverride;
			for (const auto &s : o.shaders) {
				if (IEquals(s.match, nodeName) || (!matName.empty() && IEquals(s.match, matName))) {
					shaderOverride = s.shader;
					break;
				}
			}

			ExportMesh em;
			if (BuildExportMesh(mesh, meshNode, joints, em, shaderOverride, o.v12)) {
				printf("  mesh '%s': %zu verts, %zu tris, %zu weights, shader '%s'%s\n",
				       em.name.c_str(), em.verts.size(), em.tris.size(),
				       em.weights.size(), em.shader.c_str(),
				       shaderOverride.empty() ? "" : " (override)");
				meshes.push_back(std::move(em));
			}
		}
	}

	// Every 'shader' / 'skipmesh' line should have hit something; a typo in a
	// mesh name would otherwise fail silently.
	for (const auto &s : o.shaders) {
		bool hit = false;
		for (const auto &m : meshes) if (s.shader == m.shader) hit = true;
		if (!hit) fprintf(stderr, "  WARN: shader override '%s' matched no mesh\n", s.match.c_str());
	}

	if (meshes.empty()) {
		printf("  (no meshes -- skeleton/anim only output; anim bounds will be joint-only)\n");
	}

	// --- Write mesh -----------------------------------------------------------
	if (!o.noMesh && !meshes.empty()) {
		std::string meshPath = o.outputStem + ".md5mesh";
		if (!WriteMD5Mesh(meshPath.c_str(), joints, meshes, o.scale, meshScale, cmdLine, o.v12)) {
			ufbx_free_scene(scene);
			return 1;
		}
		printf("Wrote %s\n", meshPath.c_str());
	}

	// --- Animations -----------------------------------------------------------
	// Directory part of outputStem (everything up to the last separator) so
	// anims land next to the mesh.
	std::string animDir = DirOf(o.outputStem);

	// With no explicit anim list, every stack of the mesh FBX is an anim.
	std::vector<AnimSpec> anims = o.anims;
	if (anims.empty()) anims.push_back({ skelPath, "" });

	int animsWritten = 0;
	for (const AnimSpec &spec : anims) {
		ufbx_scene *as = scene;
		float animScale = meshScale;
		std::vector<ExportJoint> *aj = &joints;
		std::vector<ExportJoint> animJoints;
		if (spec.fbx != skelPath) {
			as = LoadScene(spec.fbx, o, animScale);
			if (!as) continue;
			animJoints = CollectJoints(as);
			EnsureAtLeastOneJoint(animJoints);
			if (!SkeletonsMatch(joints, animJoints, spec.fbx)) { ufbx_free_scene(as); continue; }
			aj = &animJoints;
		}

		double useFps = fps;
		if (useFps <= 0.0) {
			useFps = as->settings.frames_per_second;
			if (useFps <= 0.0) useFps = 24.0;
		}

		if (as->anim_stacks.count == 0) {
			fprintf(stderr, "  WARN: '%s' has no animation stacks\n", spec.fbx.c_str());
		}
		for (size_t si = 0; si < as->anim_stacks.count; ++si) {
			ufbx_anim_stack *stack = as->anim_stacks.data[si];
			std::string stackName = SanitizeFilenameStem(stack->name.data, stack->name.length);

			std::string animPath;
			if (!spec.name.empty()) {
				// Explicit name: one file per stack is still possible, so
				// stacks past the first keep their own name as a suffix.
				animPath = animDir + spec.name + (si ? "_" + stackName : "") + ".md5anim";
			} else if (o.noAnimPrefix) {
				animPath = animDir + stackName + ".md5anim";
			} else {
				animPath = o.outputStem + "_" + stackName + ".md5anim";
			}

			printf("anim stack '%.*s' -> %s (%.3f fps)\n",
			       (int)stack->name.length, stack->name.data, animPath.c_str(), useFps);

			if (WriteMD5Anim(animPath.c_str(), as, stack, *aj, meshes, useFps, o.scale, animScale, cmdLine,
			                 o.rootMotionJoint, o.rootMotionXYZ)) {
				printf("Wrote %s\n", animPath.c_str());
				++animsWritten;
			}
		}

		if (as != scene) ufbx_free_scene(as);
	}

	ufbx_free_scene(scene);
	printf("Done. %zu mesh(es), %d anim(s).\n", o.noMesh ? (size_t)0 : meshes.size(), animsWritten);
	return 0;
}
