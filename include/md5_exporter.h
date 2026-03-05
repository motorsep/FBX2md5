#pragma once

#include "fbx2md5.h"
#include "skeleton.h"
#include "mesh_data.h"
#include "anim_data.h"
#include <string>
#include <vector>

namespace fbx2md5 {

// Write md5mesh file (v10 or v11).
bool WriteMD5Mesh(const std::string& filepath, const Skeleton& skeleton,
                  const std::vector<Mesh>& meshes, const Config& config);

// Write md5anim file (always v10 format — anim doesn't change between v10/v11).
bool WriteMD5Anim(const std::string& filepath, const Skeleton& skeleton,
                  const AnimData& anim, const Config& config);

} // namespace fbx2md5
