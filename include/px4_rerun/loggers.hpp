#pragma once

#include <cstdint>
#include <string>

#include <rerun.hpp>

namespace px4_rerun {

/// Log vehicle pose (NED position + quaternion). Converts to z-up internally.
void log_vehicle_pose(rerun::RecordingStream& rec, int64_t timestamp_us,
                      float x, float y, float z,
                      float qw, float qx, float qy, float qz);

/// Log home position (NED). Converts to z-up internally.
void log_home_position(rerun::RecordingStream& rec, int64_t timestamp_us,
                       float x, float y, float z);

/// Log mission item at local NED position. Converts to z-up internally.
void log_mission_item(rerun::RecordingStream& rec, int64_t timestamp_us,
                      float x, float y, float z, uint16_t nav_cmd);

/// Log a text message with PX4 syslog severity (0-7).
void log_text(rerun::RecordingStream& rec, int64_t timestamp_us,
              const std::string& text, uint8_t severity);

/// Log a scalar value at an arbitrary entity path.
void log_scalar(rerun::RecordingStream& rec, const std::string& entity_path,
                int64_t timestamp_us, double value);

/// Convert NAV_CMD to human-readable label.
const char* nav_cmd_label(uint16_t nav_cmd);

/// Convert PX4 syslog severity (0-7) to Rerun TextLogLevel.
rerun::components::TextLogLevel severity_to_rerun_level(uint8_t severity);

} // namespace px4_rerun
