#pragma once

#ifdef PX4_RERUN_HAS_ULOG

#include <string>

#include <rerun.hpp>

namespace px4_rerun {

struct ULogOptions {
    bool log_3d = true;           ///< 3D spatial data (pose, home, mission)
    bool log_text = true;         ///< Text log messages
    bool log_all_scalars = true;  ///< Auto-log every numeric field as timeseries
};

/// Parse a ULog file and log its contents to a Rerun recording stream.
void log_ulog(rerun::RecordingStream& rec, const std::string& filepath,
              const ULogOptions& options = {});

} // namespace px4_rerun

#endif // PX4_RERUN_HAS_ULOG
