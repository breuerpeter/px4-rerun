#include <px4_rerun/coordinates.hpp>

namespace px4_rerun::coords {

std::array<float, 3> ned_to_zup(float x, float y, float z)
{
    return {x, y, -z};
}

std::array<float, 4> ned_quat_to_zup(float qw, float qx, float qy, float qz)
{
    // PX4 quaternion is world-to-body (NED→FRD).
    // Rerun Transform3D expects body-to-world (child→parent).
    // Conjugate + NED-to-zup coordinate transform: {qw, -qx, qy, qz}
    return {qw, -qx, qy, qz};
}

} // namespace px4_rerun::coords
