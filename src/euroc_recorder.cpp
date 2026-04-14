#include "euroc_recorder.h"
#include "audio_player.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <sstream>

EurocRecorder::EurocRecorder(const AppConfig &cfg, AudioPlayer *audio)
    : cfg_(cfg), audio_(audio)
{
    for (size_t i = 0; i < cfg_.cameras.size(); i++) {
        if (cfg_.cameras[i].enabled) {
            num_cameras_++;
            enabled_indices_.push_back((int)i);
        }
    }

    cam_rings_.resize(cfg_.cameras.size());
    for (auto &r : cam_rings_)
        r = std::make_unique<CamRing>();
    cam_csv_.resize(cfg_.cameras.size());
    video_pipes_.resize(cfg_.cameras.size(), nullptr);

    for (auto &sc : cfg_.serial_imus) {
        if (sc.enabled) {
            auto q = std::make_unique<SerialImuQueue>();
            q->name = sc.name;
            serial_imu_queues_.push_back(std::move(q));
        }
    }

    // Match window = one frame period (from first enabled camera's fps)
    if (!enabled_indices_.empty()) {
        int fps = cfg_.cameras[enabled_indices_[0]].fps;
        if (fps > 0)
            match_window_ns_ = 1000000000ULL / fps;
    }
}

EurocRecorder::~EurocRecorder()
{
    stop();
}

void EurocRecorder::push_frame(int cam_index, uint64_t timestamp_ns,
                               const uint8_t *jpeg_data, size_t jpeg_size)
{
    if (cam_index < 0 || cam_index >= (int)cam_rings_.size()) return;

    auto &cr = *cam_rings_[cam_index];
    {
        std::lock_guard<std::mutex> lock(cr.mtx);
        if ((int)cr.buf.size() >= RING_BUF_SIZE) {
            cr.buf.pop_front();  // drop oldest
            fprintf(stderr, "[recorder] WARNING: Ring buffer full for cam%d, dropping frame (buffer overflow)\n", cam_index);
        }

        FrameEntry entry;
        entry.timestamp_ns = timestamp_ns;
        entry.jpeg.assign(jpeg_data, jpeg_data + jpeg_size);
        cr.buf.push_back(std::move(entry));
    }
    wake_cv_.notify_one();
}

void EurocRecorder::push_imu(const ImuSample &sample)
{
    std::lock_guard<std::mutex> lock(imu_mtx_);
    if ((int)imu_queue_.size() >= IMU_QUEUE_SIZE)
        imu_queue_.pop_front();
    imu_queue_.push_back(sample);
}

void EurocRecorder::push_serial_imu(int serial_index, const ImuSample &sample)
{
    if (serial_index < 0 || serial_index >= (int)serial_imu_queues_.size())
        return;
    auto &q = *serial_imu_queues_[serial_index];
    std::lock_guard<std::mutex> lock(q.mtx);
    if ((int)q.queue.size() >= IMU_QUEUE_SIZE)
        q.queue.pop_front();
    q.queue.push_back(sample);
}

bool EurocRecorder::toggle_recording()
{
    std::unique_lock<std::mutex> lk(toggle_mtx_, std::try_to_lock);
    if (!lk.owns_lock()) {
        fprintf(stderr, "[recorder] Toggle already in progress, ignoring key press\n");
        return recording_.load();
    }
    if (recording_.load()) {
        if (audio_) {
            audio_->stop_rec_ticker();
            audio_->play(AudioPlayer::Sound::REC_STOP);
        }
        stop_recording();
        return false;
    } else {
        bool ok = start_recording();
        if (audio_) {
            if (ok) {
                audio_->play(AudioPlayer::Sound::REC_START);
                audio_->start_rec_ticker();
            } else {
                audio_->play(AudioPlayer::Sound::REC_ERROR);
            }
        }
        return ok;
    }
}

void EurocRecorder::stop()
{
    if (recording_.load())
        stop_recording();
}

// Find SD card mount path. The configured sd_mount_path can be either:
//   1. An exact mount point (e.g. /mnt/sdcard/SD) — used directly if it's a mount point.
//   2. A parent directory (e.g. /mnt/sdcard) — auto-scans for the first mounted subdirectory.
// Returns the resolved path, or empty string if no SD card found.
std::string EurocRecorder::find_sd_mount_path() const
{
    const std::string &path = cfg_.recording.sd_mount_path;
    struct stat st_path, st_parent;

    if (stat(path.c_str(), &st_path) != 0) {
        fprintf(stderr, "[recorder] Path does not exist: %s\n", path.c_str());
        return "";
    }

    if (!S_ISDIR(st_path.st_mode)) {
        fprintf(stderr, "[recorder] Path is not a directory: %s\n", path.c_str());
        return "";
    }

    // Check if the configured path itself is a mount point
    std::string parent_of_path = path.substr(0, path.find_last_of('/'));
    if (parent_of_path.empty()) parent_of_path = "/";

    if (stat(parent_of_path.c_str(), &st_parent) == 0 &&
        st_path.st_dev != st_parent.st_dev) {
        // Configured path IS a mount point — use it directly
        fprintf(stderr, "[recorder] SD card found at configured path: %s\n", path.c_str());
        return path;
    }

    // Configured path is not a mount point — scan subdirectories for a mount point
    DIR *dir = opendir(path.c_str());
    if (!dir) {
        fprintf(stderr, "[recorder] Cannot open directory: %s\n", path.c_str());
        return "";
    }

    std::string found;
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_name[0] == '.') continue;  // skip . and ..

        std::string child = path + "/" + ent->d_name;
        struct stat st_child;
        if (stat(child.c_str(), &st_child) != 0) continue;
        if (!S_ISDIR(st_child.st_mode)) continue;

        // Check if this subdirectory is on a different device (= mount point)
        if (st_child.st_dev != st_path.st_dev) {
            found = child;
            fprintf(stderr, "[recorder] SD card auto-detected: %s\n", found.c_str());
            break;
        }
    }
    closedir(dir);

    if (found.empty()) {
        fprintf(stderr, "[recorder] No mounted SD card found under %s\n", path.c_str());
    }
    return found;
}

static void mkdir_p(const std::string &path)
{
    std::string tmp;
    for (char c : path) {
        tmp += c;
        if (c == '/')
            mkdir(tmp.c_str(), 0755);
    }
    mkdir(tmp.c_str(), 0755);
}

bool EurocRecorder::start_recording()
{
    if (!cfg_.recording.enabled) {
        fprintf(stderr, "[recorder] Recording disabled in config\n");
        return false;
    }

    std::string sd_path = find_sd_mount_path();
    if (sd_path.empty()) {
        fprintf(stderr, "[recorder] SD card not found (configured: %s)\n",
                cfg_.recording.sd_mount_path.c_str());
        return false;
    }

    // Check free disk space
    struct statvfs vfs;
    if (statvfs(sd_path.c_str(), &vfs) == 0) {
        uint64_t free_mb = (uint64_t)vfs.f_bavail * vfs.f_frsize / (1024 * 1024);
        if (free_mb < MIN_FREE_SPACE_MB) {
            fprintf(stderr, "[recorder] Insufficient disk space: %luMB free, need %luMB\n",
                    (unsigned long)free_mb, (unsigned long)MIN_FREE_SPACE_MB);
            return false;
        }
        fprintf(stderr, "[recorder] Disk space: %luMB free\n", (unsigned long)free_mb);
    }

    // Find next sequence number by scanning existing directories
    int seq_num = 1;
    DIR *dir = opendir(sd_path.c_str());
    if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != nullptr) {
            if (strncmp(ent->d_name, "euroc_", 6) == 0) {
                const char *suffix = ent->d_name + 6;
                bool all_digits = true;
                for (const char *p = suffix; *p; p++) {
                    if (*p < '0' || *p > '9') { all_digits = false; break; }
                }
                if (all_digits && *suffix) {
                    int num = atoi(suffix);
                    if (num >= seq_num)
                        seq_num = num + 1;
                }
            }
        }
        closedir(dir);
    }

    char seq_str[16];
    snprintf(seq_str, sizeof(seq_str), "%03d", seq_num);
    session_dir_ = sd_path + "/euroc_" + seq_str + "/mav0";

    // Create directory structure
    for (int idx : enabled_indices_)
        mkdir_p(session_dir_ + "/cam" + std::to_string(idx) + "/data");
    mkdir_p(session_dir_ + "/imu0");
    for (auto &sq : serial_imu_queues_)
        mkdir_p(session_dir_ + "/" + sq->name);

    // Open CSV files for cameras
    for (int idx : enabled_indices_) {
        std::string csv_path = session_dir_ + "/cam" + std::to_string(idx) + "/data.csv";
        cam_csv_[idx].open(csv_path);
        if (!cam_csv_[idx].is_open()) {
            fprintf(stderr, "[recorder] Failed to open %s\n", csv_path.c_str());
            stop_recording();
            return false;
        }
        cam_csv_[idx] << "#timestamp [ns],filename\n";
    }

    // Open IMU CSV (I2C)
    if (cfg_.imu.enabled) {
        std::string imu_csv_path = session_dir_ + "/imu0/data.csv";
        imu_csv_.open(imu_csv_path);
        if (!imu_csv_.is_open()) {
            fprintf(stderr, "[recorder] Failed to open %s\n", imu_csv_path.c_str());
            stop_recording();
            return false;
        }
        imu_csv_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                     "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
    }

    // Open serial IMU CSVs
    for (auto &sq : serial_imu_queues_) {
        std::string csv_path = session_dir_ + "/" + sq->name + "/data.csv";
        sq->csv.open(csv_path);
        if (!sq->csv.is_open()) {
            fprintf(stderr, "[recorder] Failed to open %s\n", csv_path.c_str());
            stop_recording();
            return false;
        }
        sq->csv << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                   "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
    }

    // Copy sensor.yaml files
    for (int idx : enabled_indices_) {
        std::string src = cfg_.recording.calib_dir + "/cam" + std::to_string(idx) + "_sensor.yaml";
        std::string dst = session_dir_ + "/cam" + std::to_string(idx) + "/sensor.yaml";
        copy_sensor_yaml(src, dst);
    }

    if (cfg_.imu.enabled) {
        std::string src = cfg_.recording.calib_dir + "/imu_sensor.yaml";
        std::string dst = session_dir_ + "/imu0/sensor.yaml";
        copy_sensor_yaml(src, dst);
    }

    for (auto &sq : serial_imu_queues_) {
        std::string src = cfg_.recording.calib_dir + "/" + sq->name + "_sensor.yaml";
        std::string dst = session_dir_ + "/" + sq->name + "/sensor.yaml";
        copy_sensor_yaml(src, dst);
    }

    // Write body.yaml
    write_body_yaml(session_dir_);

    // Start FFmpeg video pipes if output_format is "video"
    if (cfg_.recording.output_format == "video") {
        fprintf(stderr, "[recorder] Starting video recording mode (codec=%s)\n",
                cfg_.recording.video_codec.c_str());

        for (int idx : enabled_indices_) {
            std::string video_path = session_dir_ + "/cam" + std::to_string(idx) + "/video.avi";
            std::string codec = cfg_.recording.video_codec;
            int fps = cfg_.cameras[idx].fps;
            int width = cfg_.cameras[idx].width;
            int height = cfg_.cameras[idx].height;

            char cmd[512];
            if (codec == "h264") {
                snprintf(cmd, sizeof(cmd),
                    "ffmpeg -probesize 5000000 -f mjpeg -r %d -i - "
                    "-vcodec libx264 -preset fast -b:v %dM -pix_fmt yuv420p "
                    "-y '%s' 2>/dev/null",
                    fps, cfg_.recording.video_bitrate_mbps, video_path.c_str());
            } else {
                snprintf(cmd, sizeof(cmd),
                    "ffmpeg -probesize 5000000 -f mjpeg -r %d -i - "
                    "-vcodec copy -y '%s' 2>/dev/null",
                    fps, video_path.c_str());
            }

            fprintf(stderr, "[recorder] cam%d: %s\n", idx, cmd);
            video_pipes_[idx] = popen(cmd, "w");
            if (!video_pipes_[idx]) {
                fprintf(stderr, "[recorder] Failed to start FFmpeg for cam%d\n", idx);
                stop_recording();
                return false;
            }
        }
    }

    // Clear ring buffers before starting
    for (auto &cr : cam_rings_) {
        std::lock_guard<std::mutex> lock(cr->mtx);
        cr->buf.clear();
    }
    groups_written_ = 0;
    frames_dropped_ = 0;
    write_queue_drops_ = 0;
    csv_flush_counter_ = 0;

    // Start writer thread
    writer_running_ = true;
    recording_ = true;
    writer_thread_ = std::thread(&EurocRecorder::writer_thread_func, this);

    // Start async disk writer (only for image mode)
    if (cfg_.recording.output_format != "video") {
        disk_writer_running_ = true;
        disk_writer_thread_ = std::thread(&EurocRecorder::disk_writer_thread_func, this);
    }

    fprintf(stderr, "[recorder] Recording started: %s (window=%.1fms, ring=%d)\n",
            session_dir_.c_str(), match_window_ns_ / 1e6, RING_BUF_SIZE);
    return true;
}

void EurocRecorder::stop_recording()
{
    fprintf(stderr, "[recorder] Stopping recording...\n");

    // 1. Stop accepting new frames from camera threads
    recording_ = false;

    // 2. Let writer thread drain all remaining ring buffer frames
    //    Do NOT clear ring buffers here — writer thread needs them for drain.
    writer_running_ = false;
    wake_cv_.notify_all();

    if (writer_thread_.joinable())
        writer_thread_.join();

    // 3. Log residual frames that couldn't be drained
    {
        int residual = 0;
        for (auto &cr : cam_rings_) {
            std::lock_guard<std::mutex> lock(cr->mtx);
            residual += (int)cr->buf.size();
            cr->buf.clear();
        }
        if (residual > 0)
            fprintf(stderr, "[recorder] WARNING: %d residual frames discarded from ring buffers\n", residual);
    }

    // 4. Wait for disk writer to finish ALL queued writes (no timeout — data integrity first)
    disk_writer_running_ = false;
    write_queue_cv_.notify_all();
    if (disk_writer_thread_.joinable()) {
        fprintf(stderr, "[recorder] Waiting for disk writer to flush %zu pending writes...\n",
                write_queue_.size());
        disk_writer_thread_.join();
    }

    // 5. Close CSV files
    for (auto &f : cam_csv_) {
        if (f.is_open()) { f.flush(); f.close(); }
    }
    if (imu_csv_.is_open()) {
        imu_csv_.flush();
        imu_csv_.close();
    }
    for (auto &sq : serial_imu_queues_) {
        if (sq->csv.is_open()) { sq->csv.flush(); sq->csv.close(); }
    }

    // 6. Close video pipes
    bool has_video = false;
    for (auto &pipe : video_pipes_) {
        if (pipe) { has_video = true; break; }
    }
    if (has_video) {
        fprintf(stderr, "[recorder] Flushing video files to SD card, please wait...\n");
    }
    for (int idx = 0; idx < (int)video_pipes_.size(); idx++) {
        auto &pipe = video_pipes_[idx];
        if (pipe) {
            fflush(pipe);
            fprintf(stderr, "[recorder] Closing FFmpeg for cam%d...\n", idx);
            pclose(pipe);
            pipe = nullptr;
        }
    }
    if (has_video) {
        fprintf(stderr, "[recorder] All video files written to SD card\n");
    }

    fprintf(stderr, "[recorder] Recording stopped: %s (groups=%lu, dropped=%lu, queue_drops=%lu)\n",
            session_dir_.c_str(), (unsigned long)groups_written_,
            (unsigned long)frames_dropped_, (unsigned long)write_queue_drops_);
}

void EurocRecorder::writer_thread_func()
{
    fprintf(stderr, "[recorder] Writer thread started (ring=%d, window=%.1fms, cams=%d)\n",
            RING_BUF_SIZE, match_window_ns_ / 1e6, (int)enabled_indices_.size());

    while (writer_running_.load()) {
        bool did_work = false;
        while (writer_running_.load() && try_group_and_write())
            did_work = true;

        if (!writer_running_.load()) break;

        drain_imu();
        drain_serial_imus();

        if (!did_work) {
            std::unique_lock<std::mutex> lock(wake_mtx_);
            wake_cv_.wait_for(lock, std::chrono::milliseconds(5));
        }
    }

    // Drain all remaining complete groups from ring buffers (no timeout).
    // Incomplete groups (missing cameras) are left for stop_recording() to report.
    int drain_count = 0;
    while (try_group_and_write())
        drain_count++;
    if (drain_count > 0)
        fprintf(stderr, "[recorder] Final drain: flushed %d groups\n", drain_count);

    drain_imu();
    drain_serial_imus();

    fprintf(stderr, "[recorder] Writer thread stopped (groups=%lu, dropped=%lu)\n",
            (unsigned long)groups_written_, (unsigned long)frames_dropped_);
}

// Try to form one frame group from all cameras' ring buffer fronts.
// Returns true if work was done (group written or stale frame dropped).
// Returns false if not all cameras have data yet (caller should wait).
bool EurocRecorder::try_group_and_write()
{
    if (enabled_indices_.empty()) return false;

    // Check which cameras have data
    std::vector<int> cams_with_data;
    for (int idx : enabled_indices_) {
        std::lock_guard<std::mutex> lock(cam_rings_[idx]->mtx);
        if (!cam_rings_[idx]->buf.empty())
            cams_with_data.push_back(idx);
    }

    // Need ALL enabled cameras to have data for a complete synchronized group
    if ((int)cams_with_data.size() < (int)enabled_indices_.size())
        return false;

    // Collect front timestamps from cameras with data
    uint64_t min_ts = UINT64_MAX, max_ts = 0;
    int min_idx = -1;

    for (int idx : cams_with_data) {
        std::lock_guard<std::mutex> lock(cam_rings_[idx]->mtx);
        uint64_t ts = cam_rings_[idx]->buf.front().timestamp_ns;
        if (ts < min_ts) { min_ts = ts; min_idx = idx; }
        if (ts > max_ts) max_ts = ts;
    }

    uint64_t spread = max_ts - min_ts;

    if (spread > match_window_ns_) {
        // Oldest front is too stale to match - drop it and retry
        std::lock_guard<std::mutex> lock(cam_rings_[min_idx]->mtx);
        cam_rings_[min_idx]->buf.pop_front();
        frames_dropped_++;
        return true;  // caller should retry immediately
    }

    // Extract frames from all cameras
    std::vector<FrameEntry> group(cfg_.cameras.size());

    for (int idx : cams_with_data) {
        std::lock_guard<std::mutex> lock(cam_rings_[idx]->mtx);
        group[idx] = std::move(cam_rings_[idx]->buf.front());
        cam_rings_[idx]->buf.pop_front();
    }

    // Use sync master (first enabled camera = IMX334#0) timestamp as reference
    uint64_t ref_ts = group[enabled_indices_[0]].timestamp_ns;
    write_grouped_frames(ref_ts, group);
    groups_written_++;

    // Periodic stats log
    if (groups_written_ % STATS_INTERVAL == 0) {
        size_t wq_size = 0;
        {
            std::lock_guard<std::mutex> lock(write_queue_mtx_);
            wq_size = write_queue_.size();
        }
        fprintf(stderr, "[recorder] groups=%lu dropped=%lu queue_drops=%lu wq=%zu spread=%.2fms\n",
                (unsigned long)groups_written_, (unsigned long)frames_dropped_,
                (unsigned long)write_queue_drops_, wq_size, spread / 1e6);
    }

    return true;
}

void EurocRecorder::write_grouped_frames(uint64_t avg_ts,
                                          std::vector<FrameEntry> &frames)
{
    char filename[64];
    snprintf(filename, sizeof(filename), "%lu.jpg", (unsigned long)avg_ts);

    bool is_video_mode = (cfg_.recording.output_format == "video");

    for (int idx : enabled_indices_) {
        auto &entry = frames[idx];
        if (entry.jpeg.empty()) continue;

        if (is_video_mode) {
            if (video_pipes_[idx]) {
                auto t0 = std::chrono::steady_clock::now();
                size_t written = fwrite(entry.jpeg.data(), 1, entry.jpeg.size(), video_pipes_[idx]);
                fflush(video_pipes_[idx]);  // push data to FFmpeg immediately
                auto t1 = std::chrono::steady_clock::now();
                auto write_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

                if (write_ms > 100) {
                    fprintf(stderr, "[recorder] WARNING: FFmpeg pipe write took %ldms for cam%d (blocking detected)\n",
                            write_ms, idx);
                }
                if (written != entry.jpeg.size()) {
                    fprintf(stderr, "[recorder] ERROR: Short write to video pipe cam%d (%zu/%zu bytes)\n",
                            idx, written, entry.jpeg.size());
                }
                if (ferror(video_pipes_[idx])) {
                    fprintf(stderr, "[recorder] ERROR: Video pipe error for cam%d, closing pipe\n", idx);
                    pclose(video_pipes_[idx]);
                    video_pipes_[idx] = nullptr;
                }
            }
            // Write CSV entry
            if (cam_csv_[idx].is_open()) {
                cam_csv_[idx] << avg_ts << "," << filename << "\n";
                // Periodic CSV flush for video mode too
                if (groups_written_ % CSV_FLUSH_INTERVAL == 0)
                    cam_csv_[idx].flush();
            }
        } else {
            // Queue async write task with backpressure
            WriteTask task;
            task.cam_idx = idx;
            task.timestamp_ns = avg_ts;
            task.jpeg = std::move(entry.jpeg);
            task.csv_line = std::to_string(avg_ts) + "," + filename + "\n";

            std::lock_guard<std::mutex> lock(write_queue_mtx_);
            if ((int)write_queue_.size() >= MAX_WRITE_QUEUE) {
                write_queue_drops_++;
                if (write_queue_drops_ % 100 == 1)
                    fprintf(stderr, "[recorder] WARNING: Write queue full (%d), dropping frame for cam%d (total drops=%lu)\n",
                            MAX_WRITE_QUEUE, idx, (unsigned long)write_queue_drops_);
                continue;  // skip this frame
            }
            write_queue_.push_back(std::move(task));
            write_queue_cv_.notify_one();
        }
    }
}

void EurocRecorder::drain_imu()
{
    std::deque<ImuSample> batch;
    {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        batch.swap(imu_queue_);
    }

    for (auto &s : batch) {
        if (imu_csv_.is_open()) {
            char line[256];
            snprintf(line, sizeof(line),
                     "%lu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                     (unsigned long)s.timestamp_ns,
                     s.gyro[0], s.gyro[1], s.gyro[2],
                     s.accel[0], s.accel[1], s.accel[2]);
            imu_csv_ << line;
        }
    }
    if (!batch.empty() && imu_csv_.is_open())
        imu_csv_.flush();
}

void EurocRecorder::drain_serial_imus()
{
    for (auto &sq : serial_imu_queues_) {
        std::deque<ImuSample> batch;
        {
            std::lock_guard<std::mutex> lock(sq->mtx);
            batch.swap(sq->queue);
        }

        for (auto &s : batch) {
            if (sq->csv.is_open()) {
                char line[256];
                snprintf(line, sizeof(line),
                         "%lu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                         (unsigned long)s.timestamp_ns,
                         s.gyro[0], s.gyro[1], s.gyro[2],
                         s.accel[0], s.accel[1], s.accel[2]);
                sq->csv << line;
            }
        }
        if (!batch.empty() && sq->csv.is_open())
            sq->csv.flush();
    }
}

void EurocRecorder::write_body_yaml(const std::string &base_dir)
{
    std::string path = base_dir + "/body.yaml";
    std::ofstream f(path);
    if (!f.is_open()) return;

    f << "type: body\n"
      << "name: Purple Pi OH2\n"
      << "T_BS:\n"
      << "  cols: 4\n"
      << "  rows: 4\n"
      << "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
}

void EurocRecorder::copy_sensor_yaml(const std::string &src, const std::string &dst)
{
    std::ifstream in(src, std::ios::binary);
    if (!in.is_open()) {
        fprintf(stderr, "[recorder] Warning: cannot open %s, skipping copy\n", src.c_str());
        return;
    }
    std::ofstream out(dst, std::ios::binary);
    if (!out.is_open()) {
        fprintf(stderr, "[recorder] Warning: cannot create %s\n", dst.c_str());
        return;
    }
    out << in.rdbuf();
}

void EurocRecorder::disk_writer_thread_func()
{
    fprintf(stderr, "[recorder] Disk writer thread started\n");

    for (;;) {
        WriteTask task;
        {
            std::unique_lock<std::mutex> lock(write_queue_mtx_);

            // Wait for work or shutdown signal
            write_queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [this]() {
                return !write_queue_.empty() || !disk_writer_running_.load();
            });

            if (write_queue_.empty()) {
                if (!disk_writer_running_.load())
                    break;  // clean shutdown, queue fully drained
                continue;
            }

            task = std::move(write_queue_.front());
            write_queue_.pop_front();
        }
        // Lock released — do I/O outside critical section

        // Write JPEG file
        char filename[64];
        snprintf(filename, sizeof(filename), "%lu.jpg", (unsigned long)task.timestamp_ns);
        std::string filepath = session_dir_ + "/cam" + std::to_string(task.cam_idx) + "/data/" + filename;

        FILE *fp = fopen(filepath.c_str(), "wb");
        if (fp) {
            size_t written = fwrite(task.jpeg.data(), 1, task.jpeg.size(), fp);
            if (written != task.jpeg.size()) {
                fprintf(stderr, "[recorder] WARNING: Short write for %s (%zu/%zu bytes)\n",
                        filepath.c_str(), written, task.jpeg.size());
            }
            fclose(fp);
        } else {
            fprintf(stderr, "[recorder] ERROR: Cannot open %s: %s\n",
                    filepath.c_str(), strerror(errno));
        }

        // Write CSV entry
        if (cam_csv_[task.cam_idx].is_open()) {
            cam_csv_[task.cam_idx] << task.csv_line;

            // Periodic CSV flush to prevent data loss on crash
            csv_flush_counter_++;
            if (csv_flush_counter_ >= CSV_FLUSH_INTERVAL) {
                csv_flush_counter_ = 0;
                for (auto &csv : cam_csv_) {
                    if (csv.is_open()) csv.flush();
                }
            }
        }
    }

    // Final flush all CSV files
    for (auto &csv : cam_csv_) {
        if (csv.is_open()) csv.flush();
    }

    fprintf(stderr, "[recorder] Disk writer thread stopped\n");
}
