#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>

#include "audio_player.h"
#include "config.h"
#include "v4l2_camera.h"
#include "mpp_encoder.h"
#include "zmq_streamer.h"
#include "imu_reader.h"
#include "euroc_recorder.h"
#include "key_monitor.h"
#include "pwm_sync.h"
#include "param_controller.h"
#include "auto_exposure.h"
#include "rkaiq_controller.h"

static std::atomic<bool> g_running{true};

// STREAMON barrier: all camera threads wait until all are initialized,
// then start streaming simultaneously to catch the same FSIN edge.
static std::mutex g_streamon_mtx;
static std::condition_variable g_streamon_cv;
static int g_cameras_initialized = 0;
static int g_total_enabled_cams = 0;
static bool g_all_cams_ready = false;

static void signal_handler(int)
{
    g_running = false;
}

// ---- NV12 image transforms (applied before JPEG encoding) ----

// Rotate NV12 180 degrees: src -> dst (must not overlap)
// Handles stride (bytesperline may be > width)
static void nv12_rotate_180(const uint8_t *src, uint8_t *dst,
                            int width, int height, int stride)
{
    const uint8_t *src_y  = src;
    const uint8_t *src_uv = src + stride * height;
    uint8_t *dst_y  = dst;
    uint8_t *dst_uv = dst + stride * height;
    int uv_height = height / 2;

    // Y plane: pixel(x,y) -> pixel(W-1-x, H-1-y)
    for (int y = 0; y < height; y++) {
        const uint8_t *sr = src_y + (height - 1 - y) * stride;
        uint8_t *dr = dst_y + y * stride;
        for (int x = 0; x < width; x++)
            dr[x] = sr[width - 1 - x];
        // Clear padding area if stride > width
        if (stride > width)
            memset(dr + width, 0, stride - width);
    }

    // UV plane: UV pair(i,y) -> UV pair(W/2-1-i, H/2-1-y)
    for (int y = 0; y < uv_height; y++) {
        const uint8_t *sr = src_uv + (uv_height - 1 - y) * stride;
        uint8_t *dr = dst_uv + y * stride;
        for (int x = 0; x < width; x += 2) {
            dr[x]     = sr[width - 2 - x];
            dr[x + 1] = sr[width - 1 - x];
        }
        // Clear padding area if stride > width
        if (stride > width)
            memset(dr + width, 128, stride - width);
    }
}

// Mirror NV12 horizontally: src -> dst (must not overlap)
static void nv12_mirror(const uint8_t *src, uint8_t *dst,
                        int width, int height, int stride)
{
    const uint8_t *src_y  = src;
    const uint8_t *src_uv = src + stride * height;
    uint8_t *dst_y  = dst;
    uint8_t *dst_uv = dst + stride * height;
    int uv_height = height / 2;

    // Y plane: reverse each row
    for (int y = 0; y < height; y++) {
        const uint8_t *sr = src_y + y * stride;
        uint8_t *dr = dst_y + y * stride;
        for (int x = 0; x < width; x++)
            dr[x] = sr[width - 1 - x];
        // Clear padding area if stride > width
        if (stride > width)
            memset(dr + width, 0, stride - width);
    }

    // UV plane: reverse UV pairs in each row
    for (int y = 0; y < uv_height; y++) {
        const uint8_t *sr = src_uv + y * stride;
        uint8_t *dr = dst_uv + y * stride;
        for (int x = 0; x < width; x += 2) {
            dr[x]     = sr[width - 2 - x];
            dr[x + 1] = sr[width - 1 - x];
        }
        // Clear padding area if stride > width
        if (stride > width)
            memset(dr + width, 128, stride - width);
    }
}

// ---------------------------------------------------------------

static void camera_thread(int index, const CameraConfig &cam_cfg,
                          const StreamConfig &stream_cfg,
                          RuntimeCameraParams &runtime_params,
                          EurocRecorder &recorder)
{
    const char *name = cam_cfg.name.c_str();
    int port = stream_cfg.base_port + index;

    fprintf(stderr, "[cam%d] Starting %s on %s -> tcp://*:%d\n",
            index, name, cam_cfg.device.c_str(), port);

    V4L2Camera camera;
    if (!camera.open(cam_cfg.device, cam_cfg.width, cam_cfg.height, cam_cfg.fps)) {
        fprintf(stderr, "[cam%d] Failed to open camera\n", index);
        return;
    }

    camera.open_subdev(cam_cfg.subdev);
    camera.list_controls();

    // Initialize rkaiq if enabled
    RkaiqController *rkaiq = nullptr;
    if (cam_cfg.use_rkaiq && !cam_cfg.sensor_entity_name.empty() && !cam_cfg.iq_file_dir.empty()) {
        fprintf(stderr, "[cam%d] Initializing rkaiq for %s\n", index, cam_cfg.sensor_entity_name.c_str());
        rkaiq = new RkaiqController();
        if (rkaiq->init(cam_cfg.sensor_entity_name, cam_cfg.iq_file_dir, cam_cfg.width, cam_cfg.height)) {
            if (!rkaiq->start()) {
                fprintf(stderr, "[cam%d] Failed to start rkaiq\n", index);
                delete rkaiq;
                rkaiq = nullptr;
            } else {
                fprintf(stderr, "[cam%d] rkaiq started successfully\n", index);
                // Re-query format after rkaiq changes ISP configuration
                if (camera.refresh_format()) {
                    fprintf(stderr, "[cam%d] Format refreshed after rkaiq, stride=%d\n", index, camera.stride());
                }
            }
        } else {
            fprintf(stderr, "[cam%d] Failed to init rkaiq\n", index);
            delete rkaiq;
            rkaiq = nullptr;
        }
    }

    // Initialize encoder AFTER rkaiq to use correct stride
    MppJpegEncoder encoder;
    if (!encoder.init(cam_cfg.width, cam_cfg.height, camera.stride(),
                      runtime_params.jpeg_quality)) {
        fprintf(stderr, "[cam%d] Failed to init encoder\n", index);
        return;
    }

    // Initialize runtime params from config
    runtime_params.auto_exposure = cam_cfg.auto_exposure;
    runtime_params.exposure_us = cam_cfg.exposure_us;
    runtime_params.analogue_gain = cam_cfg.analogue_gain;
    runtime_params.jpeg_quality = stream_cfg.jpeg_quality;

    ZmqStreamer streamer;
    if (!streamer.bind(port)) {
        fprintf(stderr, "[cam%d] Failed to bind ZMQ port %d\n", index, port);
        return;
    }

    // ---- STREAMON barrier: wait for all cameras to be initialized ----
    {
        std::unique_lock<std::mutex> lock(g_streamon_mtx);
        g_cameras_initialized++;
        fprintf(stderr, "[cam%d] %s initialized, waiting for barrier (%d/%d)\n",
                index, name, g_cameras_initialized, g_total_enabled_cams);
        if (g_cameras_initialized >= g_total_enabled_cams) {
            g_all_cams_ready = true;
            g_streamon_cv.notify_all();
        } else {
            g_streamon_cv.wait(lock, [] {
                return g_all_cams_ready || !g_running.load();
            });
        }
    }

    if (!g_running) return;

    // Start streaming
    if (!camera.start()) {
        fprintf(stderr, "[cam%d] Failed to start streaming\n", index);
        return;
    }

    // Set initial exposure and enable auto exposure
    AutoExposure *auto_exp = nullptr;
    if (cam_cfg.auto_exposure) {
        camera.set_exposure(false, cam_cfg.exposure_us, cam_cfg.analogue_gain);
        auto_exp = new AutoExposure(128, 100, 30000);
        fprintf(stderr, "[cam%d] Auto exposure enabled\n", index);
    } else {
        camera.set_exposure(false, cam_cfg.exposure_us, cam_cfg.analogue_gain);
        fprintf(stderr, "[cam%d] Manual exposure: %dus gain=%d\n",
                index, cam_cfg.exposure_us, cam_cfg.analogue_gain);
    }

    bool need_transform = cam_cfg.rotate_180 || cam_cfg.mirror;

    fprintf(stderr, "[cam%d] %s running: %dx%d@%dfps q=%d rot180=%d mirror=%d\n",
            index, name, cam_cfg.width, cam_cfg.height,
            cam_cfg.fps, runtime_params.jpeg_quality.load(),
            cam_cfg.rotate_180, cam_cfg.mirror);

    // Allocate transform buffers if needed (two buffers for rotate then mirror)
    size_t buf_size = camera.stride() * cam_cfg.height * 3 / 2;
    std::vector<uint8_t> xform_a, xform_b;
    if (need_transform) {
        xform_a.resize(buf_size);
        if (cam_cfg.rotate_180 && cam_cfg.mirror)
            xform_b.resize(buf_size);
    }

    std::vector<uint8_t> jpeg_buf;
    std::vector<uint8_t> msg_buf;
    int frame_count = 0;
    int ae_frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();
    uint64_t last_param_version = runtime_params.version.load();

    while (g_running) {
        // Check for parameter updates
        uint64_t current_version = runtime_params.version.load();
        if (current_version != last_param_version) {
            if (!auto_exp) {
                camera.set_exposure(false,
                                  runtime_params.exposure_us,
                                  runtime_params.analogue_gain);
                fprintf(stderr, "[cam%d] Manual params updated: exp=%dus gain=%d\n",
                        index,
                        runtime_params.exposure_us.load(),
                        runtime_params.analogue_gain.load());
            }
            encoder.set_quality(runtime_params.jpeg_quality);
            last_param_version = current_version;
        }

        size_t frame_size = 0;
        uint64_t ts = 0;
        const uint8_t *frame_data = camera.capture(frame_size, ts);
        if (!frame_data) continue;

        const uint8_t *enc_input = frame_data;

        if (need_transform) {
            int w = camera.width();
            int h = camera.height();
            int s = camera.stride();

            if (cam_cfg.rotate_180 && cam_cfg.mirror) {
                // Rotate 180 first, then mirror
                nv12_rotate_180(frame_data, xform_a.data(), w, h, s);
                nv12_mirror(xform_a.data(), xform_b.data(), w, h, s);
                enc_input = xform_b.data();
            } else if (cam_cfg.rotate_180) {
                nv12_rotate_180(frame_data, xform_a.data(), w, h, s);
                enc_input = xform_a.data();
            } else {
                nv12_mirror(frame_data, xform_a.data(), w, h, s);
                enc_input = xform_a.data();
            }
        }

        // Auto exposure: adjust every 3 frames
        if (auto_exp && ++ae_frame_count >= 3) {
            ae_frame_count = 0;
            float brightness = camera.get_brightness(frame_data);
            int exp = runtime_params.exposure_us.load();
            int gain = runtime_params.analogue_gain.load();
            auto_exp->adjust_exposure(brightness, exp, gain);
            if (exp != runtime_params.exposure_us.load()) {
                runtime_params.exposure_us = exp;
                runtime_params.analogue_gain = gain;
                camera.set_exposure(false, exp, gain);
                fprintf(stderr, "[cam%d] AE adjust: brightness=%.1f exp=%dus gain=%d\n",
                        index, brightness, exp, gain);
            }
        }

        if (encoder.encode(enc_input, buf_size, jpeg_buf)) {
            // Build message: [timestamp_ns (8 bytes LE)][JPEG data]
            msg_buf.resize(8 + jpeg_buf.size());
            memcpy(msg_buf.data(), &ts, 8);
            memcpy(msg_buf.data() + 8, jpeg_buf.data(), jpeg_buf.size());
            streamer.send(msg_buf.data(), msg_buf.size());
            frame_count++;

            if (recorder.is_recording())
                recorder.push_frame(index, ts, jpeg_buf.data(), jpeg_buf.size());
        }

        camera.release();

        // Log FPS every 5 seconds
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - fps_start);
        if (elapsed.count() >= 5) {
            float fps = frame_count / (float)elapsed.count();
            int current_exp = runtime_params.exposure_us.load();
            int current_gain = runtime_params.analogue_gain.load();
            fprintf(stderr, "[cam%d] %s: %.1f fps | exp=%dus gain=%d | jpeg=%zu KB\n",
                    index, name, fps, current_exp, current_gain, jpeg_buf.size() / 1024);
            frame_count = 0;
            fps_start = now;
        }
    }

    fprintf(stderr, "[cam%d] %s stopping\n", index, name);
    camera.stop();
    delete auto_exp;
    delete rkaiq;
}

static void imu_thread(const ImuConfig &imu_cfg, EurocRecorder &recorder)
{
    fprintf(stderr, "[imu] Starting IMU reader -> tcp://*:%d\n", imu_cfg.zmq_port);

    ImuReader reader;
    if (!reader.init(imu_cfg)) {
        fprintf(stderr, "[imu] Failed to init IMU reader\n");
        return;
    }

    ZmqStreamer streamer;
    if (!streamer.bind(imu_cfg.zmq_port, 200)) {
        fprintf(stderr, "[imu] Failed to bind ZMQ port %d\n", imu_cfg.zmq_port);
        return;
    }

    fprintf(stderr, "[imu] Running at %d Hz\n", imu_cfg.sampling_frequency);

    int sample_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (g_running) {
        ImuSample sample;
        if (reader.read(sample)) {
            streamer.send((const uint8_t *)&sample, sizeof(sample));
            sample_count++;

            if (recorder.is_recording())
                recorder.push_imu(sample);
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - fps_start);
        if (elapsed.count() >= 5) {
            float rate = sample_count / (float)elapsed.count();
            fprintf(stderr, "[imu] %.1f Hz, accel=[%.2f,%.2f,%.2f] gyro=[%.3f,%.3f,%.3f]\n",
                    rate,
                    sample.accel[0], sample.accel[1], sample.accel[2],
                    sample.gyro[0], sample.gyro[1], sample.gyro[2]);
            sample_count = 0;
            fps_start = now;
        }
    }

    fprintf(stderr, "[imu] Stopping\n");
    reader.close();
}

int main(int argc, char *argv[])
{
    const char *config_path = "/etc/quad_cam_streamer/config.json";
    if (argc > 1)
        config_path = argv[1];

    fprintf(stderr, "quad_cam_streamer - loading config: %s\n", config_path);

    AppConfig cfg;
    try {
        cfg = load_config(config_path);
    } catch (const std::exception &e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    // Install signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Count enabled cameras
    int enabled = 0;
    for (auto &c : cfg.cameras)
        if (c.enabled) enabled++;

    fprintf(stderr, "Found %zu cameras configured, %d enabled\n",
            cfg.cameras.size(), enabled);

    // Debug: print camera configs
    for (size_t i = 0; i < cfg.cameras.size(); i++) {
        if (cfg.cameras[i].enabled) {
            fprintf(stderr, "[cfg] cam%zu: auto_exp=%d exp=%dus gain=%d\n",
                    i, cfg.cameras[i].auto_exposure,
                    cfg.cameras[i].exposure_us, cfg.cameras[i].analogue_gain);
        }
    }

    g_total_enabled_cams = enabled;

    if (enabled == 0) {
        fprintf(stderr, "Error: no cameras enabled\n");
        return 1;
    }

    // Initialize audio player (non-fatal if ALSA unavailable)
    AudioPlayer audio;
    audio.init("plughw:0,0");

    // Initialize EuRoC recorder
    EurocRecorder recorder(cfg, &audio);

    // Initialize runtime camera parameters
    std::vector<RuntimeCameraParams> runtime_params(cfg.cameras.size());

    // Initialize and start PWM sync BEFORE cameras.
    // Sensors in FSIN slave mode need pulses immediately at STREAMON,
    // so PWM must already be running when camera threads start.
    PwmSync pwm_sync;
    bool pwm_ok = false;
    if (cfg.hw_sync.enabled) {
        pwm_ok = pwm_sync.init(cfg.hw_sync);
        if (pwm_ok) {
            pwm_sync.start();
        } else {
            fprintf(stderr, "Warning: hardware sync init failed, continuing without sync\n");
        }
    }

    // Launch threads
    std::vector<std::thread> threads;

    // Parameter controller thread
    threads.emplace_back([&cfg, &runtime_params]() {
        ParamController ctrl(cfg, runtime_params.data(), runtime_params.size());
        ctrl.run(g_running);
    });

    // Key monitor thread
    if (cfg.recording.enabled) {
        threads.emplace_back([&cfg, &recorder]() {
            KeyMonitor km(cfg.recording.record_key_code,
                          cfg.recording.input_device,
                          [&recorder]() {
                              std::thread([&recorder]() {
                                  recorder.toggle_recording();
                              }).detach();
                          });
            km.run(g_running);
        });
    }

    if (cfg.imu.enabled) {
        fprintf(stderr, "IMU enabled: %s:0x%02x @ %d Hz -> port %d\n",
                cfg.imu.i2c_device.c_str(), cfg.imu.i2c_addr,
                cfg.imu.sampling_frequency, cfg.imu.zmq_port);
        threads.emplace_back(imu_thread, std::cref(cfg.imu), std::ref(recorder));
    }

    for (size_t i = 0; i < cfg.cameras.size(); i++) {
        if (!cfg.cameras[i].enabled) continue;
        threads.emplace_back(camera_thread, (int)i, std::cref(cfg.cameras[i]),
                             std::cref(cfg.stream), std::ref(runtime_params[i]),
                             std::ref(recorder));
    }

    // Wait for threads
    for (auto &t : threads)
        t.join();

    // Cleanup
    if (pwm_ok)
        pwm_sync.stop();
    recorder.stop();

    fprintf(stderr, "quad_cam_streamer exiting\n");
    return 0;
}
