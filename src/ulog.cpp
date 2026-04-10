#include <px4_rerun/ulog.hpp>

#ifdef PX4_RERUN_HAS_ULOG

#include <px4_rerun/loggers.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <ulog_cpp/data_container.hpp>
#include <ulog_cpp/reader.hpp>

namespace px4_rerun {

namespace {

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double EARTH_RADIUS_M = 6371000.0;

uint64_t ts_diff(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

std::shared_ptr<ulog_cpp::DataContainer> parse_file(const std::string& filepath)
{
    auto container = std::make_shared<ulog_cpp::DataContainer>(
        ulog_cpp::DataContainer::StorageConfig::FullLog);
    ulog_cpp::Reader reader{container};

    FILE* file = fopen(filepath.c_str(), "rb");
    if (!file) return nullptr;

    uint8_t buffer[4096];
    int bytes_read;
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        reader.readChunk(buffer, bytes_read);
    }
    fclose(file);

    if (container->hadFatalError()) return nullptr;
    return container;
}

bool has_sub(const std::shared_ptr<ulog_cpp::DataContainer>& data, const std::string& name)
{
    return data->subscriptionNames().count(name) > 0;
}

void log_pose(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    if (!has_sub(data, "vehicle_local_position") || !has_sub(data, "vehicle_attitude")) return;

    // Collect attitude samples for nearest-neighbor timestamp join
    auto att_sub = data->subscription("vehicle_attitude");
    struct AttSample { uint64_t ts; float q[4]; };
    std::vector<AttSample> att_samples;

    for (const auto& s : *att_sub) {
        try {
            AttSample a;
            a.ts = s["timestamp"].as<uint64_t>();
            a.q[0] = s["q"][0].as<float>();
            a.q[1] = s["q"][1].as<float>();
            a.q[2] = s["q"][2].as<float>();
            a.q[3] = s["q"][3].as<float>();
            att_samples.push_back(a);
        } catch (const std::exception&) {}
    }
    if (att_samples.empty()) return;

    auto pos_sub = data->subscription("vehicle_local_position");
    size_t j = 0;

    for (const auto& s : *pos_sub) {
        try {
            auto ts = s["timestamp"].as<uint64_t>();
            float x = s["x"].as<float>();
            float y = s["y"].as<float>();
            float z = s["z"].as<float>();

            // Advance attitude pointer to nearest timestamp
            while (j + 1 < att_samples.size() &&
                   ts_diff(att_samples[j + 1].ts, ts) <= ts_diff(att_samples[j].ts, ts)) {
                ++j;
            }

            float vx = s["vx"].as<float>();
            float vy = s["vy"].as<float>();
            float vz = s["vz"].as<float>();

            set_vehicle_position(x, y, z);
            set_vehicle_attitude(
                att_samples[j].q[0], att_samples[j].q[1],
                att_samples[j].q[2], att_samples[j].q[3]);
            log_vehicle_pose(rec, static_cast<int64_t>(ts));
            log_velocity(rec, static_cast<int64_t>(ts), x, y, z, vx, vy, vz);
        } catch (const std::exception&) {}
    }

    flush_trajectory(rec);
}

void log_home(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    if (!has_sub(data, "home_position")) return;

    auto sub = data->subscription("home_position");
    for (const auto& s : *sub) {
        try {
            auto ts = static_cast<int64_t>(s["timestamp"].as<uint64_t>());
            float x = s["x"].as<float>();
            float y = s["y"].as<float>();
            float z = s["z"].as<float>();
            log_home_position(rec, ts, x, y, z);
        } catch (const std::exception&) {}
    }
}

void log_setpoint(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    if (!has_sub(data, "vehicle_local_position_setpoint")) return;

    auto sub = data->subscription("vehicle_local_position_setpoint");
    for (const auto& s : *sub) {
        try {
            auto ts = static_cast<int64_t>(s["timestamp"].as<uint64_t>());
            float x = s["x"].as<float>();
            float y = s["y"].as<float>();
            float z = s["z"].as<float>();
            float yaw = s["yaw"].as<float>();
            log_setpoint_pose(rec, ts, x, y, z, yaw);
        } catch (const std::exception&) {}
    }
}

void log_mission(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    if (!has_sub(data, "navigator_mission_item")) return;

    // Get map reference from vehicle_local_position
    double ref_lat = 0, ref_lon = 0;
    float ref_alt = 0;
    bool have_ref = false;

    if (has_sub(data, "vehicle_local_position")) {
        auto vlp = data->subscription("vehicle_local_position");
        for (const auto& s : *vlp) {
            try {
                if (s["xy_global"].as<int>() != 0) {
                    ref_lat = s["ref_lat"].as<double>();
                    ref_lon = s["ref_lon"].as<double>();
                    ref_alt = s["ref_alt"].as<float>();
                    have_ref = true;
                    break;
                }
            } catch (const std::exception&) {}
        }
    }
    if (!have_ref) return;

    auto sub = data->subscription("navigator_mission_item");
    for (const auto& s : *sub) {
        try {
            auto ts = static_cast<int64_t>(s["timestamp"].as<uint64_t>());
            double lat = s["latitude"].as<double>();
            double lon = s["longitude"].as<double>();
            float alt = s["altitude"].as<float>();
            uint16_t nav_cmd = s["nav_cmd"].as<uint16_t>();

            // Equirectangular projection to local NED
            float local_x = static_cast<float>((lat - ref_lat) * DEG_TO_RAD * EARTH_RADIUS_M);
            float local_y = static_cast<float>((lon - ref_lon) * DEG_TO_RAD * EARTH_RADIUS_M *
                                               cos(ref_lat * DEG_TO_RAD));

            bool alt_relative = false;
            try { alt_relative = s["altitude_is_relative"].as<int>() != 0; } catch (const std::exception&) {}

            float local_z = alt_relative ? -alt : -(alt - ref_alt);

            log_mission_item(rec, ts, local_x, local_y, local_z, nav_cmd);
        } catch (const std::exception&) {}
    }
}

void log_text_messages(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    for (const auto& entry : data->logging()) {
        uint8_t severity = static_cast<uint8_t>(entry.logLevel()) - '0';
        log_text(rec, static_cast<int64_t>(entry.timestamp()),
                entry.message(), severity);
    }
}

void log_all_scalars(rerun::RecordingStream& rec, const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    for (const auto& name : data->subscriptionNames()) {
        auto sub = data->subscription(name);
        auto fields = sub->fieldNames();

        // Determine numeric fields from first sample
        std::vector<std::string> numeric_fields;
        bool first = true;

        for (const auto& sample : *sub) {
            if (first) {
                for (const auto& field : fields) {
                    if (field == "timestamp" ||
                        field.find("_padding") != std::string::npos) continue;
                    try {
                        sample[field].as<double>();
                        numeric_fields.push_back(field);
                    } catch (const std::exception&) {}
                }
                first = false;
                if (numeric_fields.empty()) break;
            }

            try {
                auto ts = static_cast<int64_t>(sample["timestamp"].as<uint64_t>());
                for (const auto& field : numeric_fields) {
                    try {
                        log_scalar(rec, "timeseries/" + name + "/" + field,
                                  ts, sample[field].as<double>());
                    } catch (const std::exception&) {}
                }
            } catch (const std::exception&) {}
        }
    }
}

constexpr double TERRAIN_PADDING_M = 50.0;
constexpr double DEG_PER_METER_LAT = 1.0 / 111000.0;

void fetch_and_log_terrain(
    rerun::RecordingStream& rec,
    const std::shared_ptr<ulog_cpp::DataContainer>& data)
{
    if (data->subscriptionNames().count("vehicle_local_position") == 0) return;

    double ref_lat = 0, ref_lon = 0;
    float ref_alt = 0;
    bool have_ref = false;

    auto vlp = data->subscription("vehicle_local_position");
    for (const auto& s : *vlp) {
        try {
            if (s["xy_global"].as<int>() != 0) {
                ref_lat = s["ref_lat"].as<double>();
                ref_lon = s["ref_lon"].as<double>();
                ref_alt = s["ref_alt"].as<float>();
                have_ref = true;
                break;
            }
        } catch (const std::exception&) {}
    }
    if (!have_ref) return;

    if (data->subscriptionNames().count("vehicle_global_position") == 0) return;

    double lat_min = 90, lat_max = -90, lon_min = 180, lon_max = -180;
    auto vgp = data->subscription("vehicle_global_position");
    int count = 0;

    for (const auto& s : *vgp) {
        try {
            double lat = s["lat"].as<double>();
            double lon = s["lon"].as<double>();
            if (lat < lat_min) lat_min = lat;
            if (lat > lat_max) lat_max = lat;
            if (lon < lon_min) lon_min = lon;
            if (lon > lon_max) lon_max = lon;
            ++count;
        } catch (const std::exception&) {}
    }
    if (count == 0) return;

    double deg_per_meter_lon = DEG_PER_METER_LAT / std::cos(ref_lat * M_PI / 180.0);
    lat_min -= TERRAIN_PADDING_M * DEG_PER_METER_LAT;
    lat_max += TERRAIN_PADDING_M * DEG_PER_METER_LAT;
    lon_min -= TERRAIN_PADDING_M * deg_per_meter_lon;
    lon_max += TERRAIN_PADDING_M * deg_per_meter_lon;

    auto terrain_fetcher_dir = std::filesystem::path(__FILE__).parent_path().parent_path() / "terrain-fetcher";
    auto glb_path = std::filesystem::temp_directory_path() / "px4_rerun_terrain.glb";

    std::array<char, 512> ref_lat_s, ref_lon_s, ref_alt_s;
    std::array<char, 512> lat_min_s, lat_max_s, lon_min_s, lon_max_s;
    std::snprintf(ref_lat_s.data(), ref_lat_s.size(), "%.10f", ref_lat);
    std::snprintf(ref_lon_s.data(), ref_lon_s.size(), "%.10f", ref_lon);
    std::snprintf(ref_alt_s.data(), ref_alt_s.size(), "%.4f", static_cast<double>(ref_alt));
    std::snprintf(lat_min_s.data(), lat_min_s.size(), "%.10f", lat_min);
    std::snprintf(lat_max_s.data(), lat_max_s.size(), "%.10f", lat_max);
    std::snprintf(lon_min_s.data(), lon_min_s.size(), "%.10f", lon_min);
    std::snprintf(lon_max_s.data(), lon_max_s.size(), "%.10f", lon_max);

    std::string cmd = "uv run --directory " + terrain_fetcher_dir.string() +
        " export-glb"
        " --ref-lat " + ref_lat_s.data() +
        " --ref-lon " + ref_lon_s.data() +
        " --ref-alt " + ref_alt_s.data() +
        " --lat-min " + lat_min_s.data() +
        " --lat-max " + lat_max_s.data() +
        " --lon-min " + lon_min_s.data() +
        " --lon-max " + lon_max_s.data() +
        " -o " + glb_path.string();

    std::cerr << "Fetching terrain: " << cmd << "\n";
    int ret = std::system((cmd + " >/dev/null").c_str());

    if (ret != 0 || !std::filesystem::exists(glb_path)) {
        std::cerr << "Terrain fetch failed (exit code " << ret << ")\n";
        return;
    }

    auto asset = rerun::Asset3D::from_file_path(glb_path.string());
    if (asset.is_ok()) {
        rec.log_static("terrain", asset.value);
    } else {
        std::cerr << "Failed to load terrain GLB\n";
    }
    std::filesystem::remove(glb_path);
    std::cerr << "Terrain logged\n";
}

} // anonymous namespace

void log_ulog(rerun::RecordingStream& rec, const std::string& filepath, const ULogOptions& options)
{
    rec.log_static("/", rerun::ViewCoordinates::RFU);

    auto data = parse_file(filepath);
    if (!data) return;

    if (options.log_3d) {
        log_pose(rec, data);
        log_setpoint(rec, data);
        log_home(rec, data);
        log_mission(rec, data);
    }
    if (options.log_text) {
        log_text_messages(rec, data);
    }
    if (options.log_all_scalars) {
        log_all_scalars(rec, data);
    }
    if (options.log_terrain) {
        fetch_and_log_terrain(rec, data);
    }
}

} // namespace px4_rerun

#endif // PX4_RERUN_HAS_ULOG
