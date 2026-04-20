cmake_minimum_required(VERSION 3.15)
project(fbx2md5
    VERSION 1.0.0
    DESCRIPTION "FBX to Doom 3 MD5 mesh/anim converter"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to Release when using a single-config generator (Make, Ninja). For
# multi-config generators (Visual Studio, Xcode) the user picks at build time.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

add_executable(fbx2md5
    src/fbx2md5.cpp
    src/ufbx.c
    src/ufbx.h
)

target_include_directories(fbx2md5 PRIVATE src)

# ufbx.c is deliberately compiled as C (not C++). CMake does this by default
# based on the .c extension; no special handling required.

if(MSVC)
    # /W3 is MSVC's reasonable-warnings default; /MP parallel-compiles TUs.
    target_compile_options(fbx2md5 PRIVATE /W3 /MP)
    # Suppress noisy C4244 (ufbx_real -> float) warnings that show up with
    # double-precision ufbx. The source uses std::fabs where it matters.
    target_compile_options(fbx2md5 PRIVATE /wd4244)
else()
    target_compile_options(fbx2md5 PRIVATE -Wall)
    # ufbx uses threads and libm on POSIX.
    find_package(Threads REQUIRED)
    target_link_libraries(fbx2md5 PRIVATE Threads::Threads m)
endif()

install(TARGETS fbx2md5 RUNTIME DESTINATION bin)
