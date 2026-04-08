#include <px4_rerun/coordinates.hpp>

namespace px4_rerun::coords {

std::array<float, 3> ned_to_zup(float x, float y, float z)
{
    // NED (x=North, y=East, z=Down) → ENU (x=East, y=North, z=Up)
    return {y, x, -z};
}

std::array<float, 4> ned_quat_to_zup(float qw, float qx, float qy, float qz)
{
    // PX4 quaternion is world-to-body (NED→FRD).
    // Rerun Transform3D expects body-to-world (child→parent).
    // Conjugate + NED→ENU axis swap (x↔y, z→-z): {qw, -qy, -qx, qz}
    return {qw, -qy, -qx, qz};
}

} // namespace px4_rerun::coords
