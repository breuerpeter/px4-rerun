cmake_minimum_required(VERSION 3.16...3.27)
project(px4-rerun VERSION 0.1.0 LANGUAGES CXX) # x-release-please-version

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Rerun SDK
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/download/0.34.1/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Suppress strict warnings for third-party Rerun SDK code
target_compile_options(rerun_sdk PRIVATE
    -Wno-double-promotion
    -Wno-float-equal
    -Wno-shadow
    -Wno-error
)

# ULog data loader (optional — builds rerun-loader-ulog executable)
option(PX4_RERUN_LOADER "Build the ULog data loader for Rerun" ON)

set(PX4_RERUN_SOURCES
    src/coordinates.cpp
    src/loggers.cpp
)

if(PX4_RERUN_LOADER)
    FetchContent_Declare(ulog_cpp
        GIT_REPOSITORY https://github.com/PX4/ulog_cpp.git
        GIT_TAG v1.0.1
    )
    FetchContent_MakeAvailable(ulog_cpp)
    list(APPEND PX4_RERUN_SOURCES src/ulog.cpp)
endif()

# Terrain fetching (optional, requires PROJ + libtiff + OpenSSL)
option(PX4_RERUN_TERRAIN "Enable native USGS terrain fetching" ON)

if(PX4_RERUN_TERRAIN)
    # cpp-httplib (header-only HTTPS client)
    FetchContent_Declare(httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG v0.18.3)
    set(HTTPLIB_REQUIRE_OPENSSL ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(httplib)

    # stb (header-only image loading/writing)
    FetchContent_Declare(stb
        GIT_REPOSITORY https://github.com/nothings/stb.git
        GIT_TAG f0569113c93ad095470c54bf34a17b36646bbbb5)
    FetchContent_MakeAvailable(stb)

    # tinygltf (header-only GLB writer)
    FetchContent_Declare(tinygltf
        GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
        GIT_TAG v2.9.4)
    set(TINYGLTF_BUILD_LOADER_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(TINYGLTF_HEADER_ONLY ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(tinygltf)

    # System libraries
    find_package(TIFF REQUIRED)
    find_package(OpenSSL REQUIRED)

    # PROJ — try find_package, fall back to pkg-config
    find_package(PROJ QUIET)
    if(NOT PROJ_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)
    endif()

    list(APPEND PX4_RERUN_SOURCES src/terrain_fetcher.cpp)
endif()

add_library(px4_rerun ${PX4_RERUN_SOURCES})
target_include_directories(px4_rerun PUBLIC include)
target_link_libraries(px4_rerun PUBLIC rerun_sdk)

if(PX4_RERUN_LOADER)
    target_link_libraries(px4_rerun PRIVATE ulog_cpp::ulog_cpp)
endif()

if(PX4_RERUN_TERRAIN)
    target_include_directories(px4_rerun PRIVATE ${stb_SOURCE_DIR})
    target_link_libraries(px4_rerun PRIVATE
        httplib::httplib
        TIFF::TIFF
        tinygltf
    )
    if(TARGET PROJ::proj)
        target_link_libraries(px4_rerun PRIVATE PROJ::proj)
    else()
        target_link_libraries(px4_rerun PRIVATE PkgConfig::PROJ)
    endif()
    target_compile_definitions(px4_rerun PRIVATE PX4_RERUN_TERRAIN)

    # Suppress strict warnings from third-party headers pulled in by
    # terrain_fetcher.cpp (stb_image, stb_image_write, tinygltf, httplib).
    set_source_files_properties(src/terrain_fetcher.cpp PROPERTIES
        COMPILE_OPTIONS "-Wno-error;-Wno-double-promotion;-Wno-shadow;-Wno-float-equal"
    )
endif()

if(PX4_RERUN_LOADER)
    add_subdirectory(rerun-loader-ulog)

    # Auto-install blueprints to ~/.local/share/px4-rerun/blueprints so users
    # can reference them as `rerun flight.ulg ~/.local/share/px4-rerun/blueprints/<name>.rbl`
    file(GLOB BLUEPRINT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/blueprints/*.rbl")
    if(BLUEPRINT_FILES)
        add_custom_target(auto_install_blueprints ALL
            COMMAND ${CMAKE_COMMAND} -E make_directory $ENV{HOME}/.local/share/px4-rerun/blueprints
            COMMAND ${CMAKE_COMMAND} -E copy ${BLUEPRINT_FILES} $ENV{HOME}/.local/share/px4-rerun/blueprints
            COMMENT "Auto-installing blueprints to ~/.local/share/px4-rerun/blueprints"
        )
    endif()
endif()
