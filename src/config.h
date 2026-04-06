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
    int ae_target_brightness = 100;
    int ae_max_analogue_gain = 96;
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

struct SerialImuConfig {
    bool enabled = false;
    std::string name = "imu_left";
    std::string uart_device = "/dev/ttyS4";
    int baudrate = 921600;
    int zmq_port = 5561;
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
    int fps = 30;          // IMX334 frame rate (PWM ch1/ch2)
    int ov9281_fps = 30;   // OV9281 frame rate (PWM ch3/ch4); defaults to fps if not set
};

struct AppConfig {
    StreamConfig stream;
    std::vector<CameraConfig> cameras;
    ImuConfig imu;
    std::vector<SerialImuConfig> serial_imus;
    RecordingConfig recording;
    HwSyncConfig hw_sync;
};

AppConfig load_config(const std::string &path);
