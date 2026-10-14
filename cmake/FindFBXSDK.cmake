# FindFBXSDK.cmake
# Locates the Autodesk FBX SDK and creates an IMPORTED target FBXSDK::FBXSDK.
#
# Usage:
#   set(FBXSDK_ROOT "C:/Program Files/Autodesk/FBX/FBX SDK/2020.3.9")
#   find_package(FBXSDK REQUIRED)
#   target_link_libraries(myapp PRIVATE FBXSDK::FBXSDK)
#
# The module searches for the SDK in the following order:
#   1. FBXSDK_ROOT CMake variable
#   2. FBXSDK_ROOT environment variable
#   3. Default install path (Windows) / auto-detection (macOS)
#
# macOS auto-detection scans /Applications/Autodesk/FBX SDK/* and picks the
# newest version directory containing include/fbxsdk.h and both
# lib/clang/{release,debug}/libfbxsdk.a. Explicitly specified roots are never
# silently replaced: if they are invalid, configuration fails.

# --- Determine root path ---------------------------------------------------
if(NOT FBXSDK_ROOT)
    if(DEFINED ENV{FBXSDK_ROOT})
        set(FBXSDK_ROOT "$ENV{FBXSDK_ROOT}")
    elseif(APPLE)
        # macOS 標準インストール先から自動検出する。
        # バージョン形式 (数字とドットのみ) のディレクトリだけを対象にし、
        # ヘッダと release/debug 両方の静的ライブラリが揃うもののうち
        # 数値バージョンが最大のものを選ぶ。バージョンのハードコードはしない。
        file(GLOB _FBXSDK_CANDIDATE_DIRS CONFIGURE_DEPENDS "/Applications/Autodesk/FBX SDK/*")
        set(_FBXSDK_BEST_VERSION "")
        set(_FBXSDK_DETECTED_ROOT "")
        foreach(_FBXSDK_DIR IN LISTS _FBXSDK_CANDIDATE_DIRS)
            if(NOT IS_DIRECTORY "${_FBXSDK_DIR}")
                continue()
            endif()
            get_filename_component(_FBXSDK_NAME "${_FBXSDK_DIR}" NAME)
            if(NOT _FBXSDK_NAME MATCHES "^[0-9]+(\\.[0-9]+)*$")
                continue()
            endif()
            if(NOT EXISTS "${_FBXSDK_DIR}/include/fbxsdk.h")
                continue()
            endif()
            if(NOT EXISTS "${_FBXSDK_DIR}/lib/clang/release/libfbxsdk.a")
                continue()
            endif()
            if(NOT EXISTS "${_FBXSDK_DIR}/lib/clang/debug/libfbxsdk.a")
                continue()
            endif()
            if(_FBXSDK_BEST_VERSION STREQUAL ""
               OR _FBXSDK_NAME VERSION_GREATER _FBXSDK_BEST_VERSION)
                set(_FBXSDK_BEST_VERSION "${_FBXSDK_NAME}")
                set(_FBXSDK_DETECTED_ROOT "${_FBXSDK_DIR}")
            endif()
        endforeach()
        unset(_FBXSDK_DIR)
        unset(_FBXSDK_NAME)
        unset(_FBXSDK_CANDIDATE_DIRS)
        set(FBXSDK_ROOT "${_FBXSDK_DETECTED_ROOT}")
        unset(_FBXSDK_DETECTED_ROOT)
        if(FBXSDK_ROOT)
            message(STATUS "FBXSDK_ROOT auto-detected: ${FBXSDK_ROOT} (version ${_FBXSDK_BEST_VERSION})")
        endif()
        unset(_FBXSDK_BEST_VERSION)
    else()
        set(FBXSDK_ROOT "C:/Program Files/Autodesk/FBX/FBX SDK/2020.3.9")
    endif()
endif()

if(APPLE)
    # --- macOS: clang 静的ライブラリ配置 ------------------------------------
    # include/libxml2 は存在しない (system libxml2 は dylib) ので追加しない。
    # LIBXML_STATIC も定義しない。
    find_path(FBXSDK_INCLUDE_DIR
        NAMES fbxsdk.h
        PATHS "${FBXSDK_ROOT}/include"
        NO_DEFAULT_PATH
    )

    # 拡張子 .a を明示し、libfbxsdk.dylib を拾わないようにする。
    find_library(FBXSDK_LIBRARY_RELEASE
        NAMES libfbxsdk.a
        PATHS "${FBXSDK_ROOT}/lib/clang/release"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_LIBRARY_DEBUG
        NAMES libfbxsdk.a
        PATHS "${FBXSDK_ROOT}/lib/clang/debug"
        NO_DEFAULT_PATH
    )

    # SDK 同梱 samples/fbxsdk.cmake の APPLE 分岐に準拠したリンク依存。
    # xml2/z/dl/pthread/iconv はシステムライブラリ名でリンクする。
    # framework は個別に解決し、返ったパスをリンクする。
    find_library(FBXSDK_FRAMEWORK_SYSTEM
        NAMES System
    )
    find_library(FBXSDK_FRAMEWORK_CORESERVICES
        NAMES CoreServices
    )
    find_library(FBXSDK_FRAMEWORK_COREFOUNDATION
        NAMES CoreFoundation
    )
    find_library(FBXSDK_FRAMEWORK_SYSTEMCONFIGURATION
        NAMES SystemConfiguration
    )

    # --- Standard find_package handling -------------------------------------
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(FBXSDK
        REQUIRED_VARS
            FBXSDK_INCLUDE_DIR
            FBXSDK_LIBRARY_RELEASE
            FBXSDK_LIBRARY_DEBUG
            FBXSDK_FRAMEWORK_SYSTEM
            FBXSDK_FRAMEWORK_CORESERVICES
            FBXSDK_FRAMEWORK_COREFOUNDATION
            FBXSDK_FRAMEWORK_SYSTEMCONFIGURATION
    )

    # --- Create IMPORTED target ---------------------------------------------
    if(FBXSDK_FOUND AND NOT TARGET FBXSDK::FBXSDK)
        add_library(FBXSDK::FBXSDK INTERFACE IMPORTED)

        # Debug 構成は debug ライブラリ、それ以外は release ライブラリを使う。
        set_target_properties(FBXSDK::FBXSDK PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${FBXSDK_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES
                "$<IF:$<CONFIG:Debug>,${FBXSDK_LIBRARY_DEBUG},${FBXSDK_LIBRARY_RELEASE}>;xml2;z;dl;pthread;iconv;${FBXSDK_FRAMEWORK_SYSTEM};${FBXSDK_FRAMEWORK_CORESERVICES};${FBXSDK_FRAMEWORK_COREFOUNDATION};${FBXSDK_FRAMEWORK_SYSTEMCONFIGURATION}"
        )
    endif()

    mark_as_advanced(
        FBXSDK_INCLUDE_DIR
        FBXSDK_LIBRARY_RELEASE
        FBXSDK_LIBRARY_DEBUG
        FBXSDK_FRAMEWORK_SYSTEM
        FBXSDK_FRAMEWORK_CORESERVICES
        FBXSDK_FRAMEWORK_COREFOUNDATION
        FBXSDK_FRAMEWORK_SYSTEMCONFIGURATION
    )
else()
    # --- Windows: 従来の配置 (挙動変更なし) ----------------------------------
    find_path(FBXSDK_INCLUDE_DIR
        NAMES fbxsdk.h
        PATHS "${FBXSDK_ROOT}/include"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_LIBRARY_RELEASE
        NAMES libfbxsdk-md
        PATHS "${FBXSDK_ROOT}/lib/x64/release"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_LIBRARY_DEBUG
        NAMES libfbxsdk-md
        PATHS "${FBXSDK_ROOT}/lib/x64/debug"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_XML2_RELEASE
        NAMES libxml2-md
        PATHS "${FBXSDK_ROOT}/lib/x64/release"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_XML2_DEBUG
        NAMES libxml2-md
        PATHS "${FBXSDK_ROOT}/lib/x64/debug"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_ZLIB_RELEASE
        NAMES zlib-md
        PATHS "${FBXSDK_ROOT}/lib/x64/release"
        NO_DEFAULT_PATH
    )

    find_library(FBXSDK_ZLIB_DEBUG
        NAMES zlib-md
        PATHS "${FBXSDK_ROOT}/lib/x64/debug"
        NO_DEFAULT_PATH
    )

    # --- Standard find_package handling -------------------------------------
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(FBXSDK
        REQUIRED_VARS
            FBXSDK_INCLUDE_DIR
            FBXSDK_LIBRARY_RELEASE
            FBXSDK_LIBRARY_DEBUG
    )

    # --- Create IMPORTED target ---------------------------------------------
    if(FBXSDK_FOUND AND NOT TARGET FBXSDK::FBXSDK)
        add_library(FBXSDK::FBXSDK INTERFACE IMPORTED)

        set_target_properties(FBXSDK::FBXSDK PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${FBXSDK_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES
                "$<IF:$<CONFIG:Debug>,${FBXSDK_LIBRARY_DEBUG};${FBXSDK_XML2_DEBUG};${FBXSDK_ZLIB_DEBUG},${FBXSDK_LIBRARY_RELEASE};${FBXSDK_XML2_RELEASE};${FBXSDK_ZLIB_RELEASE}>"
        )
    endif()

    mark_as_advanced(
        FBXSDK_INCLUDE_DIR
        FBXSDK_LIBRARY_RELEASE
        FBXSDK_LIBRARY_DEBUG
        FBXSDK_XML2_RELEASE
        FBXSDK_XML2_DEBUG
        FBXSDK_ZLIB_RELEASE
        FBXSDK_ZLIB_DEBUG
    )
endif()
