#pragma once

#include "fbx2md5.h"
#include <fbxsdk.h>
#include <string>

namespace fbx2md5 {

struct FbxContext {
    FbxManager*    manager  = nullptr;
    FbxScene*      scene    = nullptr;
    FbxImporter*   importer = nullptr;

    ~FbxContext() { Destroy(); }
    void Destroy();
};

// Initialize FBX SDK and load a scene from file.
bool LoadFbxScene(const std::string& filepath, FbxContext& ctx, bool verbose);

} // namespace fbx2md5
