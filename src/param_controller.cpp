#include "param_controller.h"
#include "zmq_streamer.h"
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <cstdio>
#include <fstream>
#include <thread>
#include <chrono>

using json = nlohmann::json;

ParamController::ParamController(AppConfig &config, RuntimeCameraParams *cam_params, int num_cams)
    : config_(config), cam_params_(cam_params), num_cams_(num_cams)
{
    config_path_ = "/etc/quad_cam_streamer/config.json";
}

ParamController::~ParamController() = default;

void ParamController::run(std::atomic<bool> &running)
{
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::rep);

    try {
        sock.bind("tcp://*:5570");
    } catch (const zmq::error_t &e) {
        fprintf(stderr, "[param_ctrl] Failed to bind port 5570: %s\n", e.what());
        return;
    }

    fprintf(stderr, "[param_ctrl] Listening on tcp://*:5570\n");

    while (running) {
        zmq::message_t msg;
        zmq::recv_result_t res = sock.recv(msg, zmq::recv_flags::dontwait);

        if (!res) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::string cmd((char *)msg.data(), msg.size());

        try {
            std::string reply = handle_command(cmd);
            sock.send(zmq::buffer(reply), zmq::send_flags::none);
        } catch (const std::exception &e) {
            std::string reply = json{{"status", "error"}, {"message", e.what()}}.dump();
            sock.send(zmq::buffer(reply), zmq::send_flags::none);
            fprintf(stderr, "[param_ctrl] Error: %s\n", e.what());
        }
    }

    fprintf(stderr, "[param_ctrl] Stopping\n");
}

std::string ParamController::handle_command(const std::string &json_str)
{
    json cmd = json::parse(json_str);

    // Handle query command
    if (cmd.contains("query") && cmd["query"].get<bool>()) {
        json response;
        response["status"] = "ok";
        response["cameras"] = json::array();

        for (int i = 0; i < num_cams_; i++) {
            RuntimeCameraParams &params = cam_params_[i];
            json cam_info;
            cam_info["index"] = i;
            cam_info["auto_exposure"] = params.auto_exposure.load();
            cam_info["exposure_us"] = params.exposure_us.load();
            cam_info["analogue_gain"] = params.analogue_gain.load();
            cam_info["jpeg_quality"] = params.jpeg_quality.load();
            response["cameras"].push_back(cam_info);
        }

        return response.dump();
    }

    // Handle parameter update command
    int cam_idx = cmd.value("cam", -1);
    if (cam_idx < 0 || cam_idx >= num_cams_) {
        throw std::runtime_error("Invalid camera index");
    }

    RuntimeCameraParams &params = cam_params_[cam_idx];
    bool changed = false;

    // Update runtime parameters
    if (cmd.contains("auto_exposure")) {
        params.auto_exposure = cmd["auto_exposure"].get<bool>();
        changed = true;
    }
    if (cmd.contains("exposure_us")) {
        params.exposure_us = cmd["exposure_us"].get<int>();
        changed = true;
    }
    if (cmd.contains("analogue_gain")) {
        params.analogue_gain = cmd["analogue_gain"].get<int>();
        changed = true;
    }
    if (cmd.contains("jpeg_quality")) {
        int q = cmd["jpeg_quality"].get<int>();
        if (q < 1 || q > 100) throw std::runtime_error("jpeg_quality must be 1-100");
        params.jpeg_quality = q;
        changed = true;
    }

    if (changed) {
        params.version.fetch_add(1);
        fprintf(stderr, "[param_ctrl] cam%d updated: auto_exp=%d exp=%dus gain=%d q=%d\n",
                cam_idx, params.auto_exposure.load(), params.exposure_us.load(),
                params.analogue_gain.load(), params.jpeg_quality.load());
    }

    // Persist to config.json if requested
    if (cmd.value("persist", false)) {
        std::lock_guard<std::mutex> lock(config_mutex_);

        CameraConfig &cam_cfg = config_.cameras[cam_idx];
        cam_cfg.auto_exposure = params.auto_exposure;
        cam_cfg.exposure_us = params.exposure_us;
        cam_cfg.analogue_gain = params.analogue_gain;

        if (cmd.contains("jpeg_quality")) {
            config_.stream.jpeg_quality = params.jpeg_quality;
        }

        save_to_config(cam_idx, cam_cfg);
        fprintf(stderr, "[param_ctrl] cam%d persisted to %s\n", cam_idx, config_path_.c_str());
    }

    return R"({"status":"ok"})";
}

void ParamController::save_to_config(int cam_index, const CameraConfig &cam_cfg)
{
    // Read current config
    std::ifstream ifs(config_path_);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open config file for reading");
    }
    json j = json::parse(ifs);
    ifs.close();

    // Update camera entry
    j["cameras"][cam_index]["auto_exposure"] = cam_cfg.auto_exposure;
    j["cameras"][cam_index]["exposure_us"] = cam_cfg.exposure_us;
    j["cameras"][cam_index]["analogue_gain"] = cam_cfg.analogue_gain;
    j["stream"]["jpeg_quality"] = config_.stream.jpeg_quality;

    // Write back atomically (write to temp, then rename)
    std::string tmp_path = config_path_ + ".tmp";
    std::ofstream ofs(tmp_path);
    if (!ofs.is_open()) {
        throw std::runtime_error("Cannot open temp config file for writing");
    }
    ofs << j.dump(2) << std::endl;
    ofs.close();

    if (rename(tmp_path.c_str(), config_path_.c_str()) != 0) {
        throw std::runtime_error("Failed to rename temp config file");
    }
}
