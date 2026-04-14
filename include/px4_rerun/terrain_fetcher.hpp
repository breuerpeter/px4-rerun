#pragma once

#include <cstdint>
#include <vector>

namespace terrain_fetcher {

struct TerrainData {
  std::vector<uint8_t> glb;  // GLB file contents (mesh vertices in local ENU meters)
};

/// Fetch USGS elevation + imagery and return a textured terrain mesh as GLB.
///
/// @param ref_lat  Reference latitude (WGS84 degrees) — local frame origin
/// @param ref_lon  Reference longitude (WGS84 degrees)
/// @param ref_alt  Reference altitude (WGS84 ellipsoid height, meters)
/// @param lat_min  Southern boundary (WGS84 degrees)
/// @param lat_max  Northern boundary (WGS84 degrees)
/// @param lon_min  Western boundary (WGS84 degrees)
/// @param lon_max  Eastern boundary (WGS84 degrees)
/// @return TerrainData with GLB bytes, or empty glb on failure
TerrainData fetch_terrain(double ref_lat, double ref_lon, double ref_alt, double lat_min,
                          double lat_max, double lon_min, double lon_max);

}  // namespace terrain_fetcher
