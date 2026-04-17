#include "config.h"

#include <fstream>
#include <mutex>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

std::mutex g_config_file_mu;

bool write_json_atomic(const std::string &path, const json &j, std::string *error)
{
    const std::string tmp_path = path + ".tmp";

    std::ofstream ofs(tmp_path);
    if (!ofs.is_open()) {
        if (error) *error = "Cannot open temp config file for writing";
        return false;
    }
    ofs << j.dump(2) << std::endl;
    ofs.close();

    if (rename(tmp_path.c_str(), path.c_str()) != 0) {
        if (error) *error = "Failed to rename temp config file";
        return false;
    }
    return true;
}

bool load_json_file(const std::string &path, json &j, std::string *error)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (error) *error = "Cannot open config file: " + path;
        return false;
    }
    j = json::parse(ifs);
    return true;
}

}  // namespace

AppConfig load_config(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open config file: " + path);

    json j = json::parse(f);
    AppConfig cfg;
    cfg.config_path = path;

    if (j.contains("stream")) {
        auto &s = j["stream"];
        if (s.contains("host"))         cfg.stream.host = s["host"].get<std::string>();
        if (s.contains("base_port"))    cfg.stream.base_port = s["base_port"];
        if (s.contains("jpeg_quality")) cfg.stream.jpeg_quality = s["jpeg_quality"];
    }

    if (j.contains("cameras")) {
        for (auto &c : j["cameras"]) {
            CameraConfig cam;
            if (c.contains("name"))           cam.name = c["name"].get<std::string>();
            if (c.contains("enabled"))        cam.enabled = c["enabled"];
            if (c.contains("device"))         cam.device = c["device"].get<std::string>();
            if (c.contains("subdev"))         cam.subdev = c["subdev"].get<std::string>();
            if (c.contains("width"))          cam.width = c["width"];
            if (c.contains("height"))         cam.height = c["height"];
            if (c.contains("fps"))            cam.fps = c["fps"];
            if (c.contains("format"))         cam.format = c["format"].get<std::string>();
            if (c.contains("rotate_180"))     cam.rotate_180 = c["rotate_180"];
            if (c.contains("mirror"))         cam.mirror = c["mirror"];
            if (c.contains("auto_exposure"))  cam.auto_exposure = c["auto_exposure"];
            if (c.contains("exposure_us"))    cam.exposure_us = c["exposure_us"];
            if (c.contains("analogue_gain"))  cam.analogue_gain = c["analogue_gain"];
            if (c.contains("ae_target_brightness")) cam.ae_target_brightness = c["ae_target_brightness"];
            if (c.contains("ae_max_analogue_gain")) cam.ae_max_analogue_gain = c["ae_max_analogue_gain"];
            if (c.contains("use_rkaiq"))      cam.use_rkaiq = c["use_rkaiq"];
            if (c.contains("sensor_entity_name")) cam.sensor_entity_name = c["sensor_entity_name"].get<std::string>();
            if (c.contains("iq_file_dir"))    cam.iq_file_dir = c["iq_file_dir"].get<std::string>();
            cfg.cameras.push_back(cam);
        }
    }

    if (j.contains("imu")) {
        auto &im = j["imu"];
        if (im.contains("enabled"))             cfg.imu.enabled = im["enabled"];
        if (im.contains("i2c_device"))          cfg.imu.i2c_device = im["i2c_device"].get<std::string>();
        if (im.contains("i2c_addr"))            cfg.imu.i2c_addr = im["i2c_addr"];
        if (im.contains("sampling_frequency"))  cfg.imu.sampling_frequency = im["sampling_frequency"];
        if (im.contains("accel_range"))         cfg.imu.accel_range = im["accel_range"];
        if (im.contains("gyro_range"))          cfg.imu.gyro_range = im["gyro_range"];
        if (im.contains("zmq_port"))            cfg.imu.zmq_port = im["zmq_port"];
        if (im.contains("rotation_matrix")) {
            auto &rm = im["rotation_matrix"];
            for (int r = 0; r < 3 && r < (int)rm.size(); r++)
                for (int c = 0; c < 3 && c < (int)rm[r].size(); c++)
                    cfg.imu.rotation_matrix[r][c] = rm[r][c].get<double>();
        }
    }

    if (j.contains("serial_imus")) {
        for (auto &si : j["serial_imus"]) {
            SerialImuConfig sc;
            if (si.contains("enabled"))     sc.enabled = si["enabled"];
            if (si.contains("name"))        sc.name = si["name"].get<std::string>();
            if (si.contains("role"))        sc.role = si["role"].get<std::string>();
            if (si.contains("uart_device")) sc.uart_device = si["uart_device"].get<std::string>();
            if (si.contains("baudrate"))    sc.baudrate = si["baudrate"];
            if (si.contains("zmq_port"))    sc.zmq_port = si["zmq_port"];
            cfg.serial_imus.push_back(sc);
        }
    }

    if (j.contains("ble_imus")) {
        auto &b = j["ble_imus"];
        if (b.contains("enabled"))                     cfg.ble_imus.enabled = b["enabled"];
        if (b.contains("auto_resume"))                cfg.ble_imus.auto_resume = b["auto_resume"];
        if (b.contains("pair_long_press_ms"))         cfg.ble_imus.pair_long_press_ms = b["pair_long_press_ms"];
        if (b.contains("pairing_status_interval_ms")) cfg.ble_imus.pairing_status_interval_ms = b["pairing_status_interval_ms"];
        if (b.contains("disconnect_timeout_ms"))      cfg.ble_imus.disconnect_timeout_ms = b["disconnect_timeout_ms"];
        if (b.contains("disconnect_alarm_interval_ms")) cfg.ble_imus.disconnect_alarm_interval_ms = b["disconnect_alarm_interval_ms"];
        if (b.contains("paired_left_addr"))           cfg.ble_imus.paired_left_addr = b["paired_left_addr"].get<std::string>();
        if (b.contains("paired_right_addr"))          cfg.ble_imus.paired_right_addr = b["paired_right_addr"].get<std::string>();
        if (b.contains("paired_waist_addr"))          cfg.ble_imus.paired_waist_addr = b["paired_waist_addr"].get<std::string>();
    }

    if (j.contains("recording")) {
        auto &r = j["recording"];
        if (r.contains("enabled"))         cfg.recording.enabled = r["enabled"];
        if (r.contains("sd_mount_path"))   cfg.recording.sd_mount_path = r["sd_mount_path"].get<std::string>();
        if (r.contains("calib_dir"))       cfg.recording.calib_dir = r["calib_dir"].get<std::string>();
        if (r.contains("record_key_code")) cfg.recording.record_key_code = r["record_key_code"];
        if (r.contains("input_device"))    cfg.recording.input_device = r["input_device"].get<std::string>();
        if (r.contains("output_format"))   cfg.recording.output_format = r["output_format"].get<std::string>();
        if (r.contains("video_codec"))     cfg.recording.video_codec = r["video_codec"].get<std::string>();
        if (r.contains("video_bitrate_mbps")) cfg.recording.video_bitrate_mbps = r["video_bitrate_mbps"];
        if (r.contains("key_event_port"))  cfg.recording.key_event_port = r["key_event_port"];
    }

    if (j.contains("hardware_sync")) {
        auto &hs = j["hardware_sync"];
        if (hs.contains("enabled"))     cfg.hw_sync.enabled = hs["enabled"];
        if (hs.contains("fps"))         cfg.hw_sync.fps = hs["fps"];
        // ov9281_fps defaults to fps if not explicitly set
        cfg.hw_sync.ov9281_fps = cfg.hw_sync.fps;
        if (hs.contains("ov9281_fps"))  cfg.hw_sync.ov9281_fps = hs["ov9281_fps"];
    }

    return cfg;
}

bool persist_camera_config(const std::string &path,
                           int cam_index,
                           const CameraConfig &cam_cfg,
                           int jpeg_quality,
                           std::string *error)
{
    std::lock_guard<std::mutex> lock(g_config_file_mu);

    json j;
    if (!load_json_file(path, j, error))
        return false;

    if (!j.contains("cameras") || cam_index < 0 || cam_index >= (int)j["cameras"].size()) {
        if (error) *error = "Invalid camera index";
        return false;
    }

    j["cameras"][cam_index]["auto_exposure"] = cam_cfg.auto_exposure;
    j["cameras"][cam_index]["exposure_us"] = cam_cfg.exposure_us;
    j["cameras"][cam_index]["analogue_gain"] = cam_cfg.analogue_gain;
    j["stream"]["jpeg_quality"] = jpeg_quality;

    return write_json_atomic(path, j, error);
}

bool persist_ble_pairing_config(const std::string &path,
                                const BleImuConfig &ble_cfg,
                                std::string *error)
{
    std::lock_guard<std::mutex> lock(g_config_file_mu);

    json j;
    if (!load_json_file(path, j, error))
        return false;

    auto &b = j["ble_imus"];
    b["enabled"] = ble_cfg.enabled;
    b["auto_resume"] = ble_cfg.auto_resume;
    b["pair_long_press_ms"] = ble_cfg.pair_long_press_ms;
    b["pairing_status_interval_ms"] = ble_cfg.pairing_status_interval_ms;
    b["disconnect_timeout_ms"] = ble_cfg.disconnect_timeout_ms;
    b["disconnect_alarm_interval_ms"] = ble_cfg.disconnect_alarm_interval_ms;
    b["paired_left_addr"] = ble_cfg.paired_left_addr;
    b["paired_right_addr"] = ble_cfg.paired_right_addr;
    b["paired_waist_addr"] = ble_cfg.paired_waist_addr;

    return write_json_atomic(path, j, error);
}
