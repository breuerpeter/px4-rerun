#include <px4_rerun/loggers.hpp>
#include <px4_rerun/coordinates.hpp>

#include <cmath>
#include <cstdio>

namespace px4_rerun {

static void set_timestamp(rerun::RecordingStream& rec, int64_t timestamp_us)
{
    if (timestamp_us >= 0) {
        rec.set_time_duration_nanos("timestamp", timestamp_us * 1000);
    }
}

static std::array<float, 3> last_pos = {0, 0, 0};
static std::array<float, 4> last_quat = {1, 0, 0, 0}; // identity

void set_vehicle_position(float x, float y, float z)
{
    last_pos = coords::ned_to_zup(x, y, z);
}

void set_vehicle_attitude(float qw, float qx, float qy, float qz)
{
    last_quat = coords::ned_quat_to_zup(qw, qx, qy, qz);
}

static std::vector<std::array<float, 3>> trajectory_points;

void log_vehicle_pose(rerun::RecordingStream& rec, int64_t timestamp_us)
{
    set_timestamp(rec, timestamp_us);

    rec.log("px4/body", rerun::Transform3D::from_translation_rotation(
        {last_pos[0], last_pos[1], last_pos[2]},
        rerun::datatypes::Quaternion::from_wxyz(last_quat[0], last_quat[1], last_quat[2], last_quat[3])));

    rec.log("px4/body/frd", rerun::Arrows3D::from_vectors({{0.3f, 0, 0}, {0, -0.3f, 0}, {0, 0, -0.3f}})
        .with_colors({rerun::Color(255, 0, 0), rerun::Color(0, 255, 0), rerun::Color(0, 0, 255)}));

    rec.log("px4/body/label", rerun::Points3D({{0, 0, 0}})
        .with_labels({"Vehicle"})
        .with_show_labels(true)
        .with_colors({rerun::Color(255, 255, 255)})
        .with_radii({0.02f}));

    trajectory_points.push_back(last_pos);
}

void log_velocity(rerun::RecordingStream& rec, int64_t timestamp_us,
                  float x, float y, float z, float vx, float vy, float vz)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);
    constexpr float scale = 0.03f; // 10 m/s → 0.3 (body axes length)
    constexpr float min_len = 0.02f;
    float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    float len = std::max(speed * scale, min_len);
    float s = speed > 0 ? len / speed : 0;
    auto vel = coords::ned_to_zup(vx * s, vy * s, vz * s);
    char label[16];
    std::snprintf(label, sizeof(label), "%.1f m/s", speed);

    rec.log("px4/world/velocity", rerun::Arrows3D::from_vectors({{vel[0], vel[1], vel[2]}})
        .with_origins({{pos[0], pos[1], pos[2]}})
        .with_colors({rerun::Color(0, 255, 255)})
        .with_labels({label})
        .with_show_labels(true));
}

void log_setpoint_pose(rerun::RecordingStream& rec, int64_t timestamp_us,
                       float x, float y, float z, float yaw)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);

    // Yaw direction vector in z-up: NED yaw is CW from north (+x)
    float fwd_x = std::cos(yaw);
    float fwd_y = std::sin(yaw);
    auto fwd = coords::ned_to_zup(fwd_x * 0.3f, fwd_y * 0.3f, 0);

    rec.log("px4/world/setpoint", rerun::Points3D({{pos[0], pos[1], pos[2]}})
        .with_labels({"Setpoint"})
        .with_show_labels(true)
        .with_colors({rerun::Color(255, 255, 255, 80)})
        .with_radii({0.02f}));

    rec.log("px4/world/setpoint_yaw", rerun::Arrows3D::from_vectors({{fwd[0], fwd[1], fwd[2]}})
        .with_origins({{pos[0], pos[1], pos[2]}})
        .with_colors({rerun::Color(255, 0, 0)}));

    rec.log("px4/world/setpoint_yaw_label", rerun::Points3D({{pos[0] + fwd[0], pos[1] + fwd[1], pos[2] + fwd[2]}})
        .with_labels({"Yaw setpoint"})
        .with_show_labels(true)
        .with_colors({rerun::Color(255, 255, 255, 80)})
        .with_radii({0.0f}));
}

void flush_trajectory(rerun::RecordingStream& rec)
{
    if (trajectory_points.empty()) return;

    rec.log_static("px4/world/trajectory",
        rerun::LineStrips3D(rerun::components::LineStrip3D(trajectory_points))
            .with_colors({rerun::Color(255, 255, 255, 128)}));

    trajectory_points.clear();
}

static rerun::Color mag_color(uint8_t mag_idx)
{
    static const rerun::Color colors[] = {
        rerun::Color(239, 85, 59),   // red-orange
        rerun::Color(0, 204, 150),   // teal
        rerun::Color(99, 110, 250),  // blue-violet
        rerun::Color(254, 203, 82),  // gold
    };
    return colors[mag_idx % 4];
}

void log_home_position(rerun::RecordingStream& rec, int64_t timestamp_us,
                       float x, float y, float z)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);

    auto point = rerun::Points3D({{pos[0], pos[1], pos[2]}})
        .with_colors({rerun::Color(0, 255, 0)})
        .with_radii({0.05f})
        .with_labels({"Home"})
        .with_show_labels(true);

    rec.log("px4/world/home", point);
}

void log_mission_item(rerun::RecordingStream& rec, int64_t timestamp_us,
                      float x, float y, float z, uint16_t nav_cmd)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);
    const char* label = nav_cmd_label(nav_cmd);

    auto point = rerun::Points3D({{pos[0], pos[1], pos[2]}})
        .with_colors({rerun::Color(255, 165, 0)})
        .with_radii({0.2f})
        .with_labels({label})
        .with_show_labels(true);

    rec.log("px4/world/mission_item", point);
}

void log_text(rerun::RecordingStream& rec, int64_t timestamp_us,
              const std::string& text, uint8_t severity)
{
    set_timestamp(rec, timestamp_us);

    // Prefix module name: "[module] msg" → "[px4/module] msg"
    std::string display = text;

    if (display.size() > 1 && display[0] == '[') {
        auto close = display.find(']', 1);

        if (close != std::string::npos) {
            display.insert(1, "px4/");
        }
    }

    rec.log("logs/px4", rerun::TextLog(display).with_level(severity_to_rerun_level(severity)));
}

void log_scalar(rerun::RecordingStream& rec, const std::string& entity_path,
                int64_t timestamp_us, double value)
{
    set_timestamp(rec, timestamp_us);
    rec.log(entity_path, rerun::Scalars(value));
}

void log_sensor_mag(rerun::RecordingStream& rec, int64_t timestamp_us,
                    uint8_t mag_idx, float x, float y, float z)
{
    set_timestamp(rec, timestamp_us);
    std::string path = "px4/body/sensor_mag/" + std::to_string(mag_idx);
    rec.log(path, rerun::Arrows3D::from_vectors({{x, y, z}})
        .with_colors({mag_color(mag_idx)})
        .with_labels({"Mag"}));
}

void log_mag_cal_samples(rerun::RecordingStream& rec, int64_t timestamp_us,
                         uint8_t mag_idx,
                         const std::vector<std::array<float, 3>>& samples)
{
    set_timestamp(rec, timestamp_us);
    std::string path = "px4/body/mag_worker_data/" + std::to_string(mag_idx);
    rec.log(path, rerun::Points3D(samples)
        .with_colors({mag_color(mag_idx)})
        .with_radii({0.01f}));
}

const char* nav_cmd_label(uint16_t nav_cmd)
{
    switch (nav_cmd) {
    case 16: return "WP";
    case 22: return "TAKEOFF";
    case 21: return "LAND";
    case 20: return "RTL";
    case 17: return "LOITER";
    case 19: return "LOITER_T";
    case 84: return "VTOL_TAKEOFF";
    case 85: return "VTOL_LAND";
    default: return "CMD";
    }
}

rerun::components::TextLogLevel severity_to_rerun_level(uint8_t severity)
{
    switch (severity) {
    case 0: return rerun::TextLogLevel::Critical;
    case 1:
    case 2:
    case 3: return rerun::TextLogLevel::Error;
    case 4: return rerun::TextLogLevel::Warning;
    case 5:
    case 6: return rerun::TextLogLevel::Info;
    case 7: return rerun::TextLogLevel::Debug;
    default: return rerun::TextLogLevel::Trace;
    }
}

} // namespace px4_rerun
