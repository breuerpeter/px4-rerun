cmake_minimum_required(VERSION 3.16...3.27)
project(rerun-loader-ulog LANGUAGES CXX)

add_executable(rerun-loader-ulog main.cpp)
target_link_libraries(rerun-loader-ulog PRIVATE px4_rerun)

# Install target: copy executable to ~/.local/bin so the Rerun Viewer finds it
install(TARGETS rerun-loader-ulog
        RUNTIME DESTINATION $ENV{HOME}/.local/bin)

# Auto-install after every build
add_custom_target(auto_install ALL
    DEPENDS rerun-loader-ulog
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> $ENV{HOME}/.local/bin
    COMMENT "Auto-installing rerun-loader-ulog to ~/.local/bin"
)
