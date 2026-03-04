#include <px4_rerun/ulog.hpp>

#ifdef PX4_RERUN_HAS_ULOG

#include <px4_rerun/loggers.hpp>

#include <cmath>
#include <cstdio>
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
        } catch (...) {}
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

            log_vehicle_pose(rec, static_cast<int64_t>(ts),
                            x, y, z,
                            att_samples[j].q[0], att_samples[j].q[1],
                            att_samples[j].q[2], att_samples[j].q[3]);
        } catch (...) {}
    }
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
        } catch (...) {}
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
            } catch (...) {}
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
            try { alt_relative = s["altitude_is_relative"].as<int>() != 0; } catch (...) {}

            float local_z = alt_relative ? -alt : -(alt - ref_alt);

            log_mission_item(rec, ts, local_x, local_y, local_z, nav_cmd);
        } catch (...) {}
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
                    } catch (...) {}
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
                    } catch (...) {}
                }
            } catch (...) {}
        }
    }
}

} // anonymous namespace

void log_ulog(rerun::RecordingStream& rec, const std::string& filepath, const ULogOptions& options)
{
    auto data = parse_file(filepath);
    if (!data) return;

    if (options.log_3d) {
        log_pose(rec, data);
        log_home(rec, data);
        log_mission(rec, data);
    }
    if (options.log_text) {
        log_text_messages(rec, data);
    }
    if (options.log_all_scalars) {
        log_all_scalars(rec, data);
    }
}

} // namespace px4_rerun

#endif // PX4_RERUN_HAS_ULOG
