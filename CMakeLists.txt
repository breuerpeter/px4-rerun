cmake_minimum_required(VERSION 3.16...3.27)
project(px4-rerun LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Rerun SDK
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# ULog support (optional)
option(PX4_RERUN_ULOG "Enable ULog file parsing support" ON)

set(PX4_RERUN_SOURCES
    src/coordinates.cpp
    src/loggers.cpp
)

if(PX4_RERUN_ULOG)
    FetchContent_Declare(ulog_cpp
        GIT_REPOSITORY https://github.com/PX4/ulog_cpp.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(ulog_cpp)
    list(APPEND PX4_RERUN_SOURCES src/ulog.cpp)
endif()

add_library(px4_rerun ${PX4_RERUN_SOURCES})
target_include_directories(px4_rerun PUBLIC include)
target_link_libraries(px4_rerun PUBLIC rerun_sdk)

if(PX4_RERUN_ULOG)
    target_link_libraries(px4_rerun PRIVATE ulog_cpp::ulog_cpp)
    target_compile_definitions(px4_rerun PUBLIC PX4_RERUN_HAS_ULOG)
endif()
