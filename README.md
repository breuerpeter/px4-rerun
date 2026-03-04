# px4-rerun

C++ library for visualizing PX4 data with [Rerun](https://rerun.io). Maps PX4 topics to Rerun archetypes with NED-to-z-up coordinate transforms.

Optionally parses ULog files directly (via `PX4_RERUN_ULOG`, default ON).

## Usage

Add as a subdirectory or submodule:

```cmake
add_subdirectory(px4-rerun)
target_link_libraries(my_target PRIVATE px4_rerun)
```

```cpp
#include <px4_rerun/px4_rerun.hpp>

// Log data (examples)
px4_rerun::log_vehicle_pose(rec, timestamp_us, x, y, z, qw, qx, qy, qz);
px4_rerun::log_text(rec, timestamp_us, text, severity);
px4_rerun::log_scalar(rec, "timeseries/topic/field", timestamp_us, value);

// Or parse an entire ULog file at once
px4_rerun::log_ulog(rec, "flight.ulg");
```

## See it in action

- PX4 RerunLogger module (to be publicly released soon) — real-time SITL visualization
- ULog dataloader for the Rerun viewer ([rerun-loader-ulog](https://github.com/breuerpeter/rerun-loader-ulog))
