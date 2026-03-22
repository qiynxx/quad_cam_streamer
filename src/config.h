#pragma once

#include <string>
#include <vector>

struct CameraConfig {
    std::string name;
    bool enabled = true;
    std::string device;
    std::string subdev;  // sensor subdev node for exposure/gain control
    int width = 1920;
    int height = 1080;
    int fps = 30;
    std::string format = "NV12";
    bool rotate_180 = false;
    bool mirror = false;
    bool auto_exposure = true;
    int exposure_us = 10000;
    int analogue_gain = 16;
    bool use_rkaiq = false;
    std::string sensor_entity_name;
    std::string iq_file_dir;
};

struct StreamConfig {
    std::string host = "192.168.100.6";
    int base_port = 5550;
    int jpeg_quality = 80;
};

struct ImuConfig {
    bool enabled = false;
    std::string i2c_device = "/dev/i2c-3";
    int i2c_addr = 0x6b;
    int sampling_frequency = 208;
    int accel_range = 4;
    int gyro_range = 1000;
    int zmq_port = 5560;
    double rotation_matrix[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
};

struct RecordingConfig {
    bool enabled = true;
    std::string sd_mount_path = "/mnt/sdcard/SD";
    std::string calib_dir = "/etc/quad_cam_streamer/calib";
    int record_key_code = 115;  // KEY_VOLUMEUP (SARADC ~17mV = ground)
    std::string input_device;   // auto-detect "adc-keys" if empty
    std::string output_format = "video";
    std::string video_codec = "mjpeg";
    int video_bitrate_mbps = 10;
};

struct HwSyncConfig {
    bool enabled = false;
    int fps = 30;
};

struct AppConfig {
    StreamConfig stream;
    std::vector<CameraConfig> cameras;
    ImuConfig imu;
    RecordingConfig recording;
    HwSyncConfig hw_sync;
};

AppConfig load_config(const std::string &path);
