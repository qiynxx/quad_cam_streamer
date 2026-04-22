#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "audio_player.h"
#include "config.h"
#include "imu_reader.h"

class EurocRecorder {
public:
    EurocRecorder(const AppConfig &cfg, AudioPlayer *audio = nullptr);
    ~EurocRecorder();

    bool is_recording() const { return recording_.load(std::memory_order_relaxed); }
    int sequence_number() const { return seq_num_; }

    // Called from camera threads - enqueues JPEG data into ring buffer
    void push_frame(int cam_index, uint64_t timestamp_ns,
                    const uint8_t *jpeg_data, size_t jpeg_size);

    // Called from IMU thread - enqueues IMU sample (I2C IMU -> imu0)
    void push_imu(const ImuSample &sample);

    // Called from serial IMU threads - enqueues sample for a named serial IMU
    void push_serial_imu(int serial_index, const ImuSample &sample);

    // Toggle recording on/off (called from key monitor)
    bool toggle_recording();

    // Stop recording if active
    void stop();

    int num_cameras() const { return num_cameras_; }

private:
    // Ring buffer depth per camera. At 30fps, 300 frames = 10 seconds of buffering.
    // Large buffer needed because FFmpeg pipe writes can block on SD card I/O.
    static constexpr int RING_BUF_SIZE = 300;
    static constexpr int IMU_QUEUE_SIZE = 600;
    static constexpr int STATS_INTERVAL = 30;  // log every N groups
    static constexpr int MAX_WRITE_QUEUE = 600; // max pending disk writes before dropping
    static constexpr int CSV_FLUSH_INTERVAL = 30; // flush CSV every N writes
    static constexpr uint64_t MIN_FREE_SPACE_MB = 500; // minimum free space to start recording

    struct FrameEntry {
        uint64_t timestamp_ns = 0;
        std::vector<uint8_t> jpeg;
    };

    bool start_recording();
    void stop_recording();
    void writer_thread_func();
    bool try_group_and_write();
    void write_grouped_frames(uint64_t avg_ts, std::vector<FrameEntry> &frames);
    void drain_imu();
    void drain_serial_imus();
    void disk_writer_thread_func();
    std::string find_sd_mount_path() const;
    void write_body_yaml(const std::string &base_dir);
    void copy_sensor_yaml(const std::string &src, const std::string &dst);

    const AppConfig &cfg_;
    AudioPlayer *audio_ = nullptr;
    int num_cameras_ = 0;
    uint64_t match_window_ns_ = 33333333;  // default 1 frame @ 30fps
    std::vector<int> enabled_indices_;      // config indices of enabled cameras
    std::atomic<bool> recording_{false};
    std::mutex toggle_mtx_;  // prevent concurrent toggle_recording() calls

    // Session directory
    std::string session_dir_;
    int seq_num_ = 0;  // euroc_NNN sequence number

    // Per-camera ring buffers
    struct CamRing {
        std::mutex mtx;
        std::deque<FrameEntry> buf;
    };
    std::vector<std::unique_ptr<CamRing>> cam_rings_;

    // Wake writer when new frame arrives
    std::mutex wake_mtx_;
    std::condition_variable wake_cv_;

    // IMU queue (I2C)
    std::mutex imu_mtx_;
    std::deque<ImuSample> imu_queue_;

    // Serial IMU queues
    struct SerialImuQueue {
        std::string name;
        std::mutex mtx;
        std::deque<ImuSample> queue;
        std::ofstream csv;
    };
    std::vector<std::unique_ptr<SerialImuQueue>> serial_imu_queues_;

    // CSV file handles
    std::vector<std::ofstream> cam_csv_;
    std::ofstream imu_csv_;

    // Video recording (FFmpeg pipes)
    std::vector<FILE*> video_pipes_;

    // Writer thread
    std::thread writer_thread_;
    std::atomic<bool> writer_running_{false};

    // Async disk writer
    struct WriteTask {
        int cam_idx;
        uint64_t timestamp_ns;
        std::vector<uint8_t> jpeg;
        std::string csv_line;
    };
    std::deque<WriteTask> write_queue_;
    std::mutex write_queue_mtx_;
    std::condition_variable write_queue_cv_;
    std::thread disk_writer_thread_;
    std::atomic<bool> disk_writer_running_{false};

    // Stats
    uint64_t groups_written_ = 0;
    uint64_t frames_dropped_ = 0;
    uint64_t write_queue_drops_ = 0;
    int csv_flush_counter_ = 0;
};
