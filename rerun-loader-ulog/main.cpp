#include <filesystem>
#include <iostream>
#include <string>

#include <rerun.hpp>
#include <rerun/third_party/cxxopts.hpp>

#include <px4_rerun/px4_rerun.hpp>

int main(int argc, char* argv[]) {
    cxxopts::Options options("rerun-loader-ulog", "PX4 ULog data loader for Rerun");
    // clang-format off
    options.add_options()
        ("h,help", "Print usage")
        ("filepath", "The filepath to load", cxxopts::value<std::string>())
        ("application-id", "Application ID", cxxopts::value<std::string>())
        ("recording-id", "Recording ID", cxxopts::value<std::string>())
        ("entity-path-prefix", "Entity path prefix", cxxopts::value<std::string>())
        ("static", "Log as static", cxxopts::value<bool>()->default_value("false"))
        ("time_sequence", "Time sequences", cxxopts::value<std::vector<std::string>>())
        ("time_duration_nanos", "Time durations", cxxopts::value<std::vector<std::string>>())
        ("time_timestamp_nanos", "Time timestamps", cxxopts::value<std::vector<std::string>>());
    // clang-format on
    options.parse_positional({"filepath"});
    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    const auto filepath = args["filepath"].as<std::string>();
    if (!std::filesystem::is_regular_file(filepath) ||
        std::filesystem::path(filepath).extension() != ".ulg") {
        return rerun::EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE;
    }

    std::string app_id = "rerun-loader-ulog";
    if (args.count("application-id")) app_id = args["application-id"].as<std::string>();
    std::string rec_id;
    if (args.count("recording-id")) rec_id = args["recording-id"].as<std::string>();

    auto rec = rerun::RecordingStream(app_id, rec_id);
    rec.to_stdout().exit_on_failure();

    px4_rerun::log_ulog(rec, filepath);
    return 0;
}
