#include <px4_rerun/loggers.hpp>
#include <px4_rerun/coordinates.hpp>

namespace px4_rerun {

static void set_timestamp(rerun::RecordingStream& rec, int64_t timestamp_us)
{
    if (timestamp_us >= 0) {
        rec.set_time_duration_nanos("timestamp", timestamp_us * 1000);
    }
}

void log_vehicle_position(rerun::RecordingStream& rec, int64_t timestamp_us,
                          float x, float y, float z)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);

    rec.log("px4/body", rerun::Transform3D::from_translation({pos[0], pos[1], pos[2]}));
}

void log_vehicle_attitude(rerun::RecordingStream& rec, int64_t timestamp_us,
                          float qw, float qx, float qy, float qz)
{
    set_timestamp(rec, timestamp_us);

    auto q = coords::ned_quat_to_zup(qw, qx, qy, qz);

    rec.log("px4/body", rerun::Transform3D::from_rotation(
        rerun::datatypes::Quaternion::from_wxyz(q[0], q[1], q[2], q[3])
    ));

    rec.log("px4/body/frd", rerun::Arrows3D::from_vectors({{0.3f, 0, 0}, {0, -0.3f, 0}, {0, 0, -0.3f}})
        .with_colors({rerun::Color(255, 0, 0), rerun::Color(0, 255, 0), rerun::Color(0, 0, 255)}));
}

void log_home_position(rerun::RecordingStream& rec, int64_t timestamp_us,
                       float x, float y, float z)
{
    set_timestamp(rec, timestamp_us);

    auto pos = coords::ned_to_zup(x, y, z);

    auto point = rerun::Points3D({{pos[0], pos[1], pos[2]}})
        .with_colors({rerun::Color(0, 255, 0)})
        .with_radii({0.1f})
        .with_labels({"Home"})
        .with_show_labels(true);

    rec.log("world/home", point);
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

    rec.log("world/mission/current", point);
}

void log_text(rerun::RecordingStream& rec, int64_t timestamp_us,
              const std::string& text, uint8_t severity)
{
    set_timestamp(rec, timestamp_us);
    rec.log("logs/console", rerun::TextLog(text).with_level(severity_to_rerun_level(severity)));
}

void log_scalar(rerun::RecordingStream& rec, const std::string& entity_path,
                int64_t timestamp_us, double value)
{
    set_timestamp(rec, timestamp_us);
    rec.log(entity_path, rerun::Scalars(value));
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
