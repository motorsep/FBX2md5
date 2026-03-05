cmake_minimum_required(VERSION 3.16)
project(fbx2md5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# Options
# =============================================================================

option(FBX_STATIC "Link FBX SDK statically (recommended)" ON)

# FBX_SDK_ROOT — path to the Autodesk FBX SDK installation.
# Examples:
#   Windows:  cmake -DFBX_SDK_ROOT="C:/Program Files/Autodesk/FBX/FBX SDK/2020.3.7" ..
#   Linux:    cmake -DFBX_SDK_ROOT=/home/user/fbxsdk ..
set(FBX_SDK_ROOT "" CACHE PATH "Path to Autodesk FBX SDK installation")

# =============================================================================
# Auto-detect FBX SDK if not specified
# =============================================================================

if(NOT FBX_SDK_ROOT OR FBX_SDK_ROOT STREQUAL "")
    if(WIN32)
        # Scan common install locations, pick the newest version
        file(GLOB _fbx_candidates
            "C:/Program Files/Autodesk/FBX/FBX SDK/*"
            "C:/Program Files (x86)/Autodesk/FBX/FBX SDK/*"
        )
        if(_fbx_candidates)
            list(SORT _fbx_candidates ORDER DESCENDING)
            list(GET _fbx_candidates 0 FBX_SDK_ROOT)
            message(STATUS "Auto-detected FBX SDK: ${FBX_SDK_ROOT}")
        endif()
    else()
        # Common Linux locations
        foreach(_path "/usr" "/usr/local" "/opt/fbxsdk" "$ENV{HOME}/fbxsdk")
            if(EXISTS "${_path}/include/fbxsdk.h")
                set(FBX_SDK_ROOT "${_path}")
                message(STATUS "Auto-detected FBX SDK: ${FBX_SDK_ROOT}")
                break()
            endif()
        endforeach()
    endif()
endif()

if(NOT FBX_SDK_ROOT OR FBX_SDK_ROOT STREQUAL "")
    message(FATAL_ERROR
        "FBX SDK not found. Please set FBX_SDK_ROOT:\n"
        "  cmake -DFBX_SDK_ROOT=\"C:/Program Files/Autodesk/FBX/FBX SDK/2020.3.7\" ..\n"
        "  cmake -DFBX_SDK_ROOT=/path/to/fbxsdk ..\n"
        "\n"
        "Download the FBX SDK from:\n"
        "  https://aps.autodesk.com/developer/overview/fbx-sdk\n"
    )
endif()

message(STATUS "FBX SDK root: ${FBX_SDK_ROOT}")

# =============================================================================
# Find FBX SDK include directory
# =============================================================================

find_path(FBX_INCLUDE_DIR
    NAMES fbxsdk.h
    PATHS
        "${FBX_SDK_ROOT}/include"
    NO_DEFAULT_PATH
)

if(NOT FBX_INCLUDE_DIR)
    message(FATAL_ERROR
        "Cannot find fbxsdk.h in ${FBX_SDK_ROOT}/include\n"
        "Verify your FBX_SDK_ROOT path is correct."
    )
endif()

message(STATUS "FBX SDK include: ${FBX_INCLUDE_DIR}")

# =============================================================================
# Find FBX SDK libraries (platform-specific)
# =============================================================================

if(WIN32)
    # ─── Windows / Visual Studio ─────────────────────────────────────────
    #
    # FBX SDK on Windows provides:
    #   Static:  libfbxsdk-md.lib  (links with /MD runtime)
    #            libfbxsdk-mt.lib  (links with /MT runtime)
    #   Shared:  libfbxsdk.lib + libfbxsdk.dll
    #
    # Layout: lib/<vs_version>/<arch>/<config>/
    #   e.g. lib/vs2022/x64/release/libfbxsdk-md.lib
    #        lib/x64/release/libfbxsdk-md.lib  (older SDKs)

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_fbx_arch "x64")
    else()
        set(_fbx_arch "x86")
    endif()

    # Build a list of search paths, newest VS first
    set(_fbx_lib_paths
        "${FBX_SDK_ROOT}/lib/vs2022/${_fbx_arch}"
        "${FBX_SDK_ROOT}/lib/vs2019/${_fbx_arch}"
        "${FBX_SDK_ROOT}/lib/vs2017/${_fbx_arch}"
        "${FBX_SDK_ROOT}/lib/${_fbx_arch}"
    )

    if(FBX_STATIC)
        # Static library: libfbxsdk-md (matches /MD runtime, CMake default)
        set(_fbx_lib_names "libfbxsdk-md" "libfbxsdk-mt")

        # Search release
        set(_fbx_release_paths)
        foreach(_p ${_fbx_lib_paths})
            list(APPEND _fbx_release_paths "${_p}/release")
        endforeach()

        find_library(FBX_LIBRARY_RELEASE
            NAMES ${_fbx_lib_names}
            PATHS ${_fbx_release_paths}
            NO_DEFAULT_PATH
        )

        # Search debug
        set(_fbx_debug_paths)
        foreach(_p ${_fbx_lib_paths})
            list(APPEND _fbx_debug_paths "${_p}/debug")
        endforeach()

        find_library(FBX_LIBRARY_DEBUG
            NAMES ${_fbx_lib_names}
            PATHS ${_fbx_debug_paths}
            NO_DEFAULT_PATH
        )
    else()
        # Shared library: libfbxsdk.lib (import library)
        set(_fbx_lib_names "libfbxsdk")

        set(_fbx_release_paths)
        foreach(_p ${_fbx_lib_paths})
            list(APPEND _fbx_release_paths "${_p}/release")
        endforeach()

        find_library(FBX_LIBRARY_RELEASE
            NAMES ${_fbx_lib_names}
            PATHS ${_fbx_release_paths}
            NO_DEFAULT_PATH
        )

        set(_fbx_debug_paths)
        foreach(_p ${_fbx_lib_paths})
            list(APPEND _fbx_debug_paths "${_p}/debug")
        endforeach()

        find_library(FBX_LIBRARY_DEBUG
            NAMES ${_fbx_lib_names}
            PATHS ${_fbx_debug_paths}
            NO_DEFAULT_PATH
        )
    endif()

    # Validate we found at least the release library
    if(NOT FBX_LIBRARY_RELEASE)
        message(FATAL_ERROR
            "Cannot find FBX SDK library in ${FBX_SDK_ROOT}/lib\n"
            "Searched for: ${_fbx_lib_names}\n"
            "In paths: ${_fbx_release_paths}\n"
            "Try: cmake -DFBX_STATIC=OFF .. (for shared/DLL linking)"
        )
    endif()

    # Use release for both if debug not found
    if(NOT FBX_LIBRARY_DEBUG)
        set(FBX_LIBRARY_DEBUG ${FBX_LIBRARY_RELEASE})
    endif()

    message(STATUS "FBX SDK library (release): ${FBX_LIBRARY_RELEASE}")
    message(STATUS "FBX SDK library (debug):   ${FBX_LIBRARY_DEBUG}")

else()
    # ─── Linux / GCC / Clang ─────────────────────────────────────────────
    #
    # FBX SDK on Linux provides:
    #   Static:  libfbxsdk.a
    #   Shared:  libfbxsdk.so
    #
    # Layout: lib/<compiler>/<arch>/<config>/
    #   e.g. lib/gcc/x64/release/libfbxsdk.a
    #        lib/x64/release/libfbxsdk.a  (simplified)

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_fbx_arch "x64")
    else()
        set(_fbx_arch "x86")
    endif()

    set(_fbx_lib_paths
        "${FBX_SDK_ROOT}/lib/gcc/${_fbx_arch}/release"
        "${FBX_SDK_ROOT}/lib/gcc4/${_fbx_arch}/release"
        "${FBX_SDK_ROOT}/lib/clang/${_fbx_arch}/release"
        "${FBX_SDK_ROOT}/lib/${_fbx_arch}/release"
        "${FBX_SDK_ROOT}/lib/release"
        "${FBX_SDK_ROOT}/lib"
    )

    if(FBX_STATIC)
        find_library(FBX_LIBRARY_RELEASE
            NAMES libfbxsdk.a fbxsdk
            PATHS ${_fbx_lib_paths}
            NO_DEFAULT_PATH
        )
    else()
        find_library(FBX_LIBRARY_RELEASE
            NAMES fbxsdk libfbxsdk.so
            PATHS ${_fbx_lib_paths}
            NO_DEFAULT_PATH
        )
    endif()

    if(NOT FBX_LIBRARY_RELEASE)
        message(FATAL_ERROR
            "Cannot find FBX SDK library in ${FBX_SDK_ROOT}/lib\n"
            "Searched in: ${_fbx_lib_paths}\n"
            "Try: cmake -DFBX_STATIC=OFF .. (for shared library linking)"
        )
    endif()

    set(FBX_LIBRARY_DEBUG ${FBX_LIBRARY_RELEASE})

    message(STATUS "FBX SDK library: ${FBX_LIBRARY_RELEASE}")
endif()

# =============================================================================
# Sources
# =============================================================================

set(SOURCES
    src/main.cpp
    src/fbx_loader.cpp
    src/md5_exporter.cpp
    src/skeleton.cpp
    src/mesh_data.cpp
    src/anim_data.cpp
)

set(HEADERS
    include/fbx2md5.h
    include/fbx_loader.h
    include/md5_exporter.h
    include/skeleton.h
    include/mesh_data.h
    include/anim_data.h
    include/math_types.h
)

# =============================================================================
# Target
# =============================================================================

add_executable(fbx2md5 ${SOURCES} ${HEADERS})

target_include_directories(fbx2md5 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${FBX_INCLUDE_DIR}
)

# Use generator expressions for correct Debug/Release library selection.
# This is the ONLY place we link the FBX library — no double-linking.
target_link_libraries(fbx2md5 PRIVATE
    $<$<CONFIG:Debug>:${FBX_LIBRARY_DEBUG}>
    $<$<NOT:$<CONFIG:Debug>>:${FBX_LIBRARY_RELEASE}>
)

# =============================================================================
# Platform-specific configuration
# =============================================================================

if(WIN32)
    # ─── MSVC settings ──────────────────────────────────────────────────
    if(MSVC)
        # Suppress C4996 warnings for fopen, fprintf, etc.
        target_compile_definitions(fbx2md5 PRIVATE
            _CRT_SECURE_NO_WARNINGS
        )

        # Enable standards-compliant preprocessor (VS2019 16.6+)
        target_compile_options(fbx2md5 PRIVATE /Zc:preprocessor)

        # Warning level 3 (reasonable without being overwhelming)
        target_compile_options(fbx2md5 PRIVATE /W3)
    endif()

    if(FBX_STATIC)
        # Static FBX SDK bundles its own libxml2 and zlib in the same lib directory.
        # We need to find and link them — they have the same -md/-mt suffix.
        
        # Find libxml2-md.lib (or libxml2-mt.lib)
        find_library(FBX_XML2_LIBRARY_RELEASE
            NAMES libxml2-md libxml2-mt libxml2
            PATHS ${_fbx_release_paths}
            NO_DEFAULT_PATH
        )
        find_library(FBX_XML2_LIBRARY_DEBUG
            NAMES libxml2-md libxml2-mt libxml2
            PATHS ${_fbx_debug_paths}
            NO_DEFAULT_PATH
        )
        if(NOT FBX_XML2_LIBRARY_DEBUG)
            set(FBX_XML2_LIBRARY_DEBUG ${FBX_XML2_LIBRARY_RELEASE})
        endif()
        
        # Find zlib-md.lib (or zlib-mt.lib)
        find_library(FBX_ZLIB_LIBRARY_RELEASE
            NAMES zlib-md zlib-mt zlib
            PATHS ${_fbx_release_paths}
            NO_DEFAULT_PATH
        )
        find_library(FBX_ZLIB_LIBRARY_DEBUG
            NAMES zlib-md zlib-mt zlib
            PATHS ${_fbx_debug_paths}
            NO_DEFAULT_PATH
        )
        if(NOT FBX_ZLIB_LIBRARY_DEBUG)
            set(FBX_ZLIB_LIBRARY_DEBUG ${FBX_ZLIB_LIBRARY_RELEASE})
        endif()
        
        if(FBX_XML2_LIBRARY_RELEASE)
            message(STATUS "FBX bundled libxml2 (release): ${FBX_XML2_LIBRARY_RELEASE}")
            target_link_libraries(fbx2md5 PRIVATE
                $<$<CONFIG:Debug>:${FBX_XML2_LIBRARY_DEBUG}>
                $<$<NOT:$<CONFIG:Debug>>:${FBX_XML2_LIBRARY_RELEASE}>
            )
        else()
            message(WARNING "Could not find libxml2-md.lib bundled with FBX SDK")
        endif()
        
        if(FBX_ZLIB_LIBRARY_RELEASE)
            message(STATUS "FBX bundled zlib (release): ${FBX_ZLIB_LIBRARY_RELEASE}")
            target_link_libraries(fbx2md5 PRIVATE
                $<$<CONFIG:Debug>:${FBX_ZLIB_LIBRARY_DEBUG}>
                $<$<NOT:$<CONFIG:Debug>>:${FBX_ZLIB_LIBRARY_RELEASE}>
            )
        else()
            message(WARNING "Could not find zlib-md.lib bundled with FBX SDK")
        endif()
        
        # Windows system libs the FBX SDK also needs
        target_link_libraries(fbx2md5 PRIVATE
            advapi32
            wininet
        )
        # When linking statically, do NOT define FBXSDK_SHARED
    else()
        # Shared/DLL: tell FBX headers to use __declspec(dllimport)
        target_compile_definitions(fbx2md5 PRIVATE FBXSDK_SHARED)
    endif()

else()
    # ─── Linux / GCC / Clang settings ────────────────────────────────────

    # FBX SDK on Linux depends on these system libraries
    find_package(Threads REQUIRED)
    target_link_libraries(fbx2md5 PRIVATE Threads::Threads)

    if(FBX_STATIC)
        # Static FBX SDK on Linux needs: libxml2, zlib, pthreads, dl
        # Use pkg-config for reliable detection
        find_package(PkgConfig QUIET)

        if(PkgConfig_FOUND)
            pkg_check_modules(LIBXML2 QUIET libxml-2.0)
            pkg_check_modules(ZLIB_PKG QUIET zlib)
        endif()

        if(LIBXML2_FOUND)
            target_include_directories(fbx2md5 PRIVATE ${LIBXML2_INCLUDE_DIRS})
            target_link_libraries(fbx2md5 PRIVATE ${LIBXML2_LINK_LIBRARIES})
        else()
            # Fallback: try CMake's find module
            find_package(LibXml2 QUIET)
            if(LibXml2_FOUND)
                target_include_directories(fbx2md5 PRIVATE ${LIBXML2_INCLUDE_DIR})
                target_link_libraries(fbx2md5 PRIVATE ${LIBXML2_LIBRARIES})
            else()
                # Last resort: link by name
                target_link_libraries(fbx2md5 PRIVATE xml2)
                message(WARNING
                    "libxml2 not found via pkg-config or find_package; linking -lxml2 directly.\n"
                    "Install with: sudo apt install libxml2-dev  (Debian/Ubuntu)\n"
                    "              sudo dnf install libxml2-devel (Fedora/RHEL)"
                )
            endif()
        endif()

        if(ZLIB_PKG_FOUND)
            target_link_libraries(fbx2md5 PRIVATE ${ZLIB_PKG_LINK_LIBRARIES})
        else()
            find_package(ZLIB QUIET)
            if(ZLIB_FOUND)
                target_link_libraries(fbx2md5 PRIVATE ZLIB::ZLIB)
            else()
                target_link_libraries(fbx2md5 PRIVATE z)
                message(WARNING
                    "zlib not found via pkg-config or find_package; linking -lz directly.\n"
                    "Install with: sudo apt install zlib1g-dev  (Debian/Ubuntu)\n"
                    "              sudo dnf install zlib-devel  (Fedora/RHEL)"
                )
            endif()
        endif()

        target_link_libraries(fbx2md5 PRIVATE dl)
    else()
        # Shared FBX SDK doesn't need the extra deps (they're in the .so)
    endif()

    # Warnings
    target_compile_options(fbx2md5 PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# =============================================================================
# Visual Studio specific: set working directory and startup project
# =============================================================================

if(MSVC)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT fbx2md5)

    # Put the executable in a predictable location
    set_target_properties(fbx2md5 PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_BINARY_DIR}/bin"
    )

    # Organize source files into filters matching the folder structure
    source_group("Source Files" FILES ${SOURCES})
    source_group("Header Files" FILES ${HEADERS})
endif()

# =============================================================================
# Install
# =============================================================================

install(TARGETS fbx2md5 RUNTIME DESTINATION bin)

# =============================================================================
# Summary
# =============================================================================

message(STATUS "")
message(STATUS "=== fbx2md5 build configuration ===")
message(STATUS "  Platform:      ${CMAKE_SYSTEM_NAME}")
if(CMAKE_CXX_COMPILER_ID)
    message(STATUS "  Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
endif()
message(STATUS "  Architecture:  ${_fbx_arch}")
message(STATUS "  FBX SDK:       ${FBX_SDK_ROOT}")
message(STATUS "  FBX linking:   $<IF:$<BOOL:${FBX_STATIC}>,static,shared>")
message(STATUS "  Output dir:    ${CMAKE_BINARY_DIR}/bin")
message(STATUS "===================================")
message(STATUS "")
