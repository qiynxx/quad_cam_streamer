#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include "config.h"

// Runtime camera parameters (can be updated via ZMQ)
struct RuntimeCameraParams {
    std::atomic<bool> auto_exposure{true};
    std::atomic<int> exposure_us{10000};
    std::atomic<int> analogue_gain{16};
    std::atomic<int> jpeg_quality{80};
    std::atomic<uint64_t> version{0};  // Incremented on each update
};

// Parameter controller: receives ZMQ commands and updates runtime params
class ParamController {
public:
    ParamController(AppConfig &config, RuntimeCameraParams *cam_params, int num_cams);
    ~ParamController();

    void run(std::atomic<bool> &running);

private:
    std::string handle_command(const std::string &json_str);
    void save_to_config(int cam_index, const CameraConfig &cam_cfg);

    AppConfig &config_;
    RuntimeCameraParams *cam_params_;
    int num_cams_;
    std::string config_path_;
    std::mutex config_mutex_;  // Protects config file writes
};
