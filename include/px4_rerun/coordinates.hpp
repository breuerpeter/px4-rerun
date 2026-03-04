#pragma once

#include <array>

namespace px4_rerun::coords {

/// Convert NED position to Rerun z-up: {x, y, -z}
std::array<float, 3> ned_to_zup(float x, float y, float z);

/// Convert NED quaternion (w,x,y,z) to Rerun z-up: {w, x, -y, -z}
std::array<float, 4> ned_quat_to_zup(float qw, float qx, float qy, float qz);

} // namespace px4_rerun::coords
