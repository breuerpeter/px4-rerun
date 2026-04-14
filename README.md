# px4-rerun

[![Build](https://github.com/breuerpeter/px4-rerun/actions/workflows/build.yml/badge.svg)](https://github.com/breuerpeter/px4-rerun/actions/workflows/build.yml)
[![Pre-commit](https://github.com/breuerpeter/px4-rerun/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/breuerpeter/px4-rerun/actions/workflows/pre-commit.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

C++ library for visualizing PX4 data with [Rerun](https://rerun.io).

## Usage

### 1. FetchContent (recommended)

```cmake
include(FetchContent)
FetchContent_Declare(px4_rerun
    GIT_REPOSITORY https://github.com/breuerpeter/px4-rerun.git
    GIT_TAG v0.1.0)
FetchContent_MakeAvailable(px4_rerun)
target_link_libraries(my_target PRIVATE px4_rerun)
```

### 2. Git submodule

```bash
git submodule add https://github.com/breuerpeter/px4-rerun.git
```

```cmake
add_subdirectory(px4-rerun)
target_link_libraries(my_target PRIVATE px4_rerun)
```

### 3. Prebuilt loader binary

Download `rerun-loader-ulog` from the [latest release](https://github.com/breuerpeter/px4-rerun/releases/latest) and place it in `~/.local/bin/`. The Rerun Viewer will auto-discover it, letting you open `.ulg` files directly:

```bash
rerun flight.ulg
```

## Library API

```cpp
#include <px4_rerun/px4_rerun.hpp>

// Log data (examples)
px4_rerun::log_vehicle_pose(rec, timestamp_us, x, y, z, qw, qx, qy, qz);
px4_rerun::log_text(rec, timestamp_us, text, severity);
px4_rerun::log_scalar(rec, "timeseries/topic/field", timestamp_us, value);

// Or parse an entire ULog file at once
px4_rerun::log_ulog(rec, "flight.ulg");
```

## Build options

| Option | Default | Description |
|---|---|---|
| `PX4_RERUN_LOADER` | `ON` | Build the `rerun-loader-ulog` executable and ULog parsing support |
| `PX4_RERUN_TERRAIN` | `ON` | Enable USGS terrain fetching (requires `libproj-dev libtiff-dev libssl-dev`) |

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for build instructions, commit conventions, and the release flow.
