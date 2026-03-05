#pragma once

#include <string>
#include <vector>

namespace fbx2md5 {

// ─── Configuration ──────────────────────────────────────────────────────────────

struct Config {
    std::string inputFile;
    std::string outputDir = ".";     // output directory
    std::string meshName;            // override output mesh filename (without ext)
    int  md5Version     = 10;        // 10 or 11
    bool armatureFix    = false;     // remove Blender's extra Armature root bone
    bool fbxOutput      = false;     // output FBX instead of md5
    double scaleFactor  = 1.0;       // additional scale factor (applied on top of auto-detected)
    double rotateX      = 0.0;       // rotation around X axis in degrees
    double rotateY      = 0.0;       // rotation around Y axis in degrees
    double rotateZ      = 0.0;       // rotation around Z axis in degrees
    double animEpsilon  = 0.0001;    // threshold for detecting animated components
    bool verbose        = false;
};

// Return codes
enum ExitCode {
    EXIT_OK             = 0,
    EXIT_BAD_ARGS       = 1,
    EXIT_FBX_LOAD_FAIL  = 2,
    EXIT_NO_SKELETON    = 3,
    EXIT_TOO_MANY_BONES = 4,
    EXIT_NO_MESH        = 5,
    EXIT_WRITE_FAIL     = 6,
};

static constexpr int MAX_MD5_JOINTS = 255;
static constexpr int MAX_VERTEX_WEIGHTS = 4;

// ─── FBX → id Tech coordinate conversion ───────────────────────────────────────
//
// FBX Y-up to id Tech Z-up:
//   id_x =  fbx_x / scale
//   id_y = -fbx_z / scale
//   id_z =  fbx_y / scale
//
// Same remap on quaternion imaginary parts:
//   id_qx =  fbx_qx
//   id_qy = -fbx_qz
//   id_qz =  fbx_qy
//   id_qw =  fbx_qw

} // namespace fbx2md5
