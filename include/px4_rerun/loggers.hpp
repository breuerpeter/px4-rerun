#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <rerun.hpp>

namespace px4_rerun {

/// Cache vehicle position (NED). Converts to z-up internally.
void set_vehicle_position(float x, float y, float z);

/// Cache vehicle attitude (NED quaternion). Converts to z-up internally.
void set_vehicle_attitude(float qw, float qx, float qy, float qz);

/// Log cached position + attitude as a combined Transform3D. Also accumulates trajectory.
void log_vehicle_pose(rerun::RecordingStream& rec, int64_t timestamp_us);

/// Log setpoint pose (NED position + yaw). Converts to z-up internally.
void log_setpoint_pose(rerun::RecordingStream& rec, int64_t timestamp_us,
                       float x, float y, float z, float yaw);

/// Log all accumulated trajectory points as a single static LineStrips3D.
void flush_trajectory(rerun::RecordingStream& rec);

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

/// Log raw magnetometer reading as a 3D point (body-frame Gauss).
void log_sensor_mag(rerun::RecordingStream& rec, int64_t timestamp_us,
                    uint8_t mag_idx, float x, float y, float z);

/// Log all accumulated mag calibration samples as a single Points3D batch.
void log_mag_cal_samples(rerun::RecordingStream& rec, int64_t timestamp_us,
                         uint8_t mag_idx,
                         const std::vector<std::array<float, 3>>& samples);

/// Convert NAV_CMD to human-readable label.
const char* nav_cmd_label(uint16_t nav_cmd);

/// Convert PX4 syslog severity (0-7) to Rerun TextLogLevel.
rerun::components::TextLogLevel severity_to_rerun_level(uint8_t severity);

} // namespace px4_rerun
