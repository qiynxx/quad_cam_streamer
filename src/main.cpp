#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <csignal>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <algorithm>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>

#include "audio_player.h"
#include "ble_imu_manager.h"
#include "config.h"
#include "v4l2_camera.h"
#include "mpp_encoder.h"
#include "zmq_streamer.h"
#include "imu_reader.h"
#include "serial_imu_reader.h"
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

static std::string lower_copy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return (char)std::tolower(c);
    });
    return s;
}

static BleHandRole infer_ble_hand_role(const SerialImuConfig &cfg)
{
    std::string key = cfg.role.empty() ? cfg.name : cfg.role;
    key = lower_copy(key);
    if (key.find("left") != std::string::npos)
        return BleHandRole::LEFT;
    if (key.find("right") != std::string::npos)
        return BleHandRole::RIGHT;
    if (key.find("waist") != std::string::npos)
        return BleHandRole::WAIST;
    return BleHandRole::UNKNOWN;
}

static std::vector<BleImuOutputConfig> build_ble_outputs(const AppConfig &cfg)
{
    std::vector<BleImuOutputConfig> outputs;
    int serial_index = 0;
    bool have_left = false;
    bool have_right = false;
    bool have_waist = false;

    for (const auto &sc : cfg.serial_imus) {
        if (!sc.enabled)
            continue;

        BleHandRole hand = infer_ble_hand_role(sc);
        if (hand == BleHandRole::UNKNOWN) {
            fprintf(stderr, "[ble_imu] Warning: cannot infer hand role for serial output '%s'\n",
                    sc.name.c_str());
            serial_index++;
            continue;
        }
        if ((hand == BleHandRole::LEFT && have_left) ||
            (hand == BleHandRole::RIGHT && have_right) ||
            (hand == BleHandRole::WAIST && have_waist)) {
            fprintf(stderr, "[ble_imu] Warning: duplicate BLE output role for '%s', skipping\n",
                    sc.name.c_str());
            serial_index++;
            continue;
        }

        BleImuOutputConfig out;
        out.hand = hand;
        out.name = sc.name;
        out.serial_index = serial_index;
        out.zmq_port = sc.zmq_port;
        outputs.push_back(out);

        if (hand == BleHandRole::LEFT) have_left = true;
        if (hand == BleHandRole::RIGHT) have_right = true;
        if (hand == BleHandRole::WAIST) have_waist = true;
        serial_index++;
    }

    return outputs;
}

// ---- NV12 image transforms (applied before JPEG encoding) ----
// Uses ARM NEON to reverse 16 bytes at a time for ~8x speedup over scalar C.

#ifdef __aarch64__
#include <arm_neon.h>

// Reverse 'n' bytes from src into dst using NEON (n must be >= 0, handles tail)
static inline void neon_reverse_row(const uint8_t *src, uint8_t *dst, int n)
{
    const uint8_t *end = src + n;
    int i = 0;
    // Process 16 bytes at a time from the end
    for (; i + 16 <= n; i += 16) {
        uint8x16_t v = vld1q_u8(end - i - 16);
        v = vrev64q_u8(v);
        v = vcombine_u8(vget_high_u8(v), vget_low_u8(v));
        vst1q_u8(dst + i, v);
    }
    // Scalar tail
    for (; i < n; i++)
        dst[i] = src[n - 1 - i];
}

// Reverse UV pairs: swap (U0,V0)(U1,V1)... → ...(U1,V1)(U0,V0)
// Width bytes total, processed as width/2 UV pairs
static inline void neon_reverse_uv_row(const uint8_t *src, uint8_t *dst, int width)
{
    const uint8_t *end = src + width;
    int i = 0;
    // 16 bytes = 8 UV pairs at a time
    for (; i + 16 <= width; i += 16) {
        // Reverse UV pairs while preserving byte order inside each pair.
        // NV12 stores chroma as [U0 V0][U1 V1]...; swapping bytes inside a
        // pair turns NV12 into NV21 and causes blue/orange color casts.
        uint16x8_t v16 = vreinterpretq_u16_u8(vld1q_u8(end - i - 16));
        v16 = vrev64q_u16(v16);
        v16 = vcombine_u16(vget_high_u16(v16), vget_low_u16(v16));
        vst1q_u8(dst + i, vreinterpretq_u8_u16(v16));
    }
    // Scalar tail
    for (; i < width; i += 2) {
        dst[i]     = src[width - 2 - i];
        dst[i + 1] = src[width - 1 - i];
    }
}
#endif // __aarch64__

// Rotate NV12 180 degrees: src -> dst (must not overlap)
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
#ifdef __aarch64__
        neon_reverse_row(sr, dr, width);
#else
        for (int x = 0; x < width; x++)
            dr[x] = sr[width - 1 - x];
#endif
        if (stride > width)
            memset(dr + width, 0, stride - width);
    }

    // UV plane: UV pair(i,y) -> UV pair(W/2-1-i, H/2-1-y)
    for (int y = 0; y < uv_height; y++) {
        const uint8_t *sr = src_uv + (uv_height - 1 - y) * stride;
        uint8_t *dr = dst_uv + y * stride;
#ifdef __aarch64__
        neon_reverse_uv_row(sr, dr, width);
#else
        for (int x = 0; x < width; x += 2) {
            dr[x]     = sr[width - 2 - x];
            dr[x + 1] = sr[width - 1 - x];
        }
#endif
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
#ifdef __aarch64__
        neon_reverse_row(sr, dr, width);
#else
        for (int x = 0; x < width; x++)
            dr[x] = sr[width - 1 - x];
#endif
        if (stride > width)
            memset(dr + width, 0, stride - width);
    }

    // UV plane: reverse UV pairs in each row
    for (int y = 0; y < uv_height; y++) {
        const uint8_t *sr = src_uv + y * stride;
        uint8_t *dr = dst_uv + y * stride;
#ifdef __aarch64__
        neon_reverse_uv_row(sr, dr, width);
#else
        for (int x = 0; x < width; x += 2) {
            dr[x]     = sr[width - 2 - x];
            dr[x + 1] = sr[width - 1 - x];
        }
#endif
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

    if (cam_cfg.sensor_entity_name.find("ov9281") != std::string::npos ||
        cam_cfg.sensor_entity_name.find("ov9282") != std::string::npos) {
        camera.set_line_time_us(9.1f);   // OV9281: HTS=728 @80MHz pixel clock
        // Select 30fps or 120fps sensor mode via subdev frame interval
        camera.set_frame_interval(cam_cfg.fps);
    } else {
        camera.set_line_time_us(14.8f);  // IMX334: HMAX=1100, INCK=74.25MHz
    }
    fprintf(stderr, "[cam%d] Line time: %.1f us/line\n", index, camera.line_time_us());

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

    // Initialize runtime params from config BEFORE encoder init
    runtime_params.auto_exposure = cam_cfg.auto_exposure;
    runtime_params.exposure_us = cam_cfg.exposure_us;
    runtime_params.analogue_gain = cam_cfg.analogue_gain;
    runtime_params.jpeg_quality = stream_cfg.jpeg_quality;

    // Initialize encoder AFTER rkaiq to use correct stride
    MppJpegEncoder encoder;
    if (!encoder.init(cam_cfg.width, cam_cfg.height, camera.stride(),
                      runtime_params.jpeg_quality)) {
        fprintf(stderr, "[cam%d] Failed to init encoder\n", index);
        return;
    }

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
        // Use the tighter of the frame-period limit and the driver's actual
        // exposure control range to avoid pushing the sensor past its accepted
        // coarse-integration setting.
        int min_exp_us = camera.exposure_min_us();
        if (min_exp_us <= 0) min_exp_us = 100;

        int frame_max_exp_us = (int)(1000000.0f / cam_cfg.fps - 2 * camera.line_time_us());
        if (frame_max_exp_us < min_exp_us)
            frame_max_exp_us = min_exp_us;

        int sensor_max_exp_us = camera.exposure_max_us();
        int max_exp_us = frame_max_exp_us;
        if (sensor_max_exp_us > 0)
            max_exp_us = std::min(max_exp_us, sensor_max_exp_us);
        if (max_exp_us < min_exp_us)
            max_exp_us = min_exp_us;

        // Keep software AE well away from the sensor's hard frame-length edge.
        // IMX334 becomes unstable and visibly flickers when exposure is pushed
        // deep into the last part of the frame period.
        int ae_max_exp_us = max_exp_us;
        if (cam_cfg.sensor_entity_name.find("imx334") != std::string::npos) {
            ae_max_exp_us = (max_exp_us * 65) / 100;
            if (ae_max_exp_us < min_exp_us)
                ae_max_exp_us = min_exp_us;
        }

        int min_gain = camera.gain_min();
        if (min_gain <= 0)
            min_gain = cam_cfg.analogue_gain;
        min_gain = std::max(min_gain, cam_cfg.analogue_gain);

        int max_gain = cam_cfg.ae_max_analogue_gain;
        int sensor_max_gain = camera.gain_max();
        if (sensor_max_gain > 0)
            max_gain = std::min(max_gain, sensor_max_gain);
        if (cam_cfg.sensor_entity_name.find("imx334") != std::string::npos)
            max_gain = std::min(max_gain, 48);
        if (max_gain < min_gain)
            max_gain = min_gain;

        int min_exp_lines = camera.exposure_min_lines();
        int max_exp_lines = camera.exposure_max_lines();
        int ae_max_exp_lines = camera.exposure_us_to_lines(ae_max_exp_us);

        auto_exp = new AutoExposure(cam_cfg.ae_target_brightness,
                                    min_exp_us, ae_max_exp_us,
                                    min_gain, max_gain);
        fprintf(stderr,
                "[cam%d] Auto exposure enabled, target=%d"
                " min_exp=%dus(%d lines) ae_max_exp=%dus(%d lines)"
                " ctrl_max_exp=%dus(%d lines)"
                " min_gain=%d max_gain=%d"
                " (frame_limit=%d sensor_limit=%d sensor_gain_limit=%d)\n",
                index, cam_cfg.ae_target_brightness,
                min_exp_us, min_exp_lines, ae_max_exp_us, ae_max_exp_lines,
                max_exp_us, max_exp_lines,
                min_gain, max_gain,
                frame_max_exp_us, sensor_max_exp_us, sensor_max_gain);
    } else {
        camera.set_exposure(false, cam_cfg.exposure_us, cam_cfg.analogue_gain);
        fprintf(stderr, "[cam%d] Manual exposure: %dus (%d lines) gain=%d\n",
                index, cam_cfg.exposure_us,
                camera.exposure_us_to_lines(cam_cfg.exposure_us),
                cam_cfg.analogue_gain);
    }

    bool need_transform = cam_cfg.rotate_180 || cam_cfg.mirror;
    int ae_adjust_interval = 3;
    if (cam_cfg.sensor_entity_name.find("imx334") != std::string::npos)
        ae_adjust_interval = 6;

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

        // Auto exposure: IMX334 uses a slower update cadence to avoid flicker.
        if (auto_exp && ++ae_frame_count >= ae_adjust_interval) {
            ae_frame_count = 0;
            float brightness = camera.get_brightness(frame_data);
            int old_exp = runtime_params.exposure_us.load();
            int old_gain = runtime_params.analogue_gain.load();
            int exp = old_exp;
            int gain = old_gain;
            auto_exp->adjust_exposure(brightness, exp, gain);
            if (exp != old_exp || gain != old_gain) {
                runtime_params.exposure_us = exp;
                runtime_params.analogue_gain = gain;
                camera.set_exposure(false, exp, gain);
                fprintf(stderr,
                        "[cam%d] AE adjust: brightness=%.1f exp=%dus (%d lines) gain=%d\n",
                        index, brightness, exp, camera.exposure_us_to_lines(exp), gain);
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
            int current_exp_lines = camera.exposure_us_to_lines(current_exp);
            // IMX334 exposure control is in raw sensor lines; the us value is
            // an approximate conversion kept for readability and config I/O.
            fprintf(stderr, "[cam%d] %s: %.1f fps | exp=%dus (%d lines) gain=%d | jpeg=%zu KB\n",
                    index, name, fps, current_exp, current_exp_lines,
                    current_gain, jpeg_buf.size() / 1024);
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

static void serial_imu_thread(int serial_index, const SerialImuConfig &sc,
                              EurocRecorder &recorder, AudioPlayer *audio)
{
    fprintf(stderr, "[serial_imu:%s] Starting on %s @ %d baud -> tcp://*:%d\n",
            sc.name.c_str(), sc.uart_device.c_str(), sc.baudrate, sc.zmq_port);

    SerialImuReader reader;
    if (!reader.init(sc)) {
        fprintf(stderr, "[serial_imu:%s] FAILED to open %s\n",
                sc.name.c_str(), sc.uart_device.c_str());
        if (audio) audio->play(AudioPlayer::Sound::IMU_DISCONNECT);
        return;
    }
    fprintf(stderr, "[serial_imu:%s] Port opened, waiting for data...\n", sc.name.c_str());

    ZmqStreamer streamer;
    if (!streamer.bind(sc.zmq_port, 200)) {
        fprintf(stderr, "[serial_imu:%s] Failed to bind ZMQ port %d\n",
                sc.name.c_str(), sc.zmq_port);
        reader.close();
        return;
    }

    enum class State { WAITING, CONNECTED, DISCONNECTED };
    State state = State::WAITING;
    int sample_count = 0;
    int period_count = 0;
    ImuSample last_sample{};
    auto fps_start = std::chrono::steady_clock::now();
    auto last_data_time = fps_start;
    auto last_alarm_time = fps_start;
    constexpr int TIMEOUT_SEC = 3;
    constexpr int ALARM_INTERVAL_SEC = 5;

    while (g_running) {
        ImuSample sample;
        if (reader.read(sample)) {
            streamer.send((const uint8_t *)&sample, sizeof(sample));
            sample_count++;
            period_count++;
            last_sample = sample;
            last_data_time = std::chrono::steady_clock::now();

            if (state != State::CONNECTED) {
                fprintf(stderr, "[serial_imu:%s] CONNECTED - receiving data\n",
                        sc.name.c_str());
                state = State::CONNECTED;
            }

            if (recorder.is_recording())
                recorder.push_serial_imu(serial_index, sample);
        }

        auto now = std::chrono::steady_clock::now();

        if (state == State::CONNECTED) {
            auto silent = std::chrono::duration_cast<std::chrono::seconds>(
                              now - last_data_time).count();
            if (silent >= TIMEOUT_SEC) {
                fprintf(stderr, "[serial_imu:%s] DISCONNECTED - no data for %ds\n",
                        sc.name.c_str(), TIMEOUT_SEC);
                state = State::DISCONNECTED;
                if (audio) audio->play(AudioPlayer::Sound::IMU_DISCONNECT);
                last_alarm_time = now;
            }
        }

        if (state == State::DISCONNECTED) {
            auto since_alarm = std::chrono::duration_cast<std::chrono::seconds>(
                                   now - last_alarm_time).count();
            if (since_alarm >= ALARM_INTERVAL_SEC) {
                if (audio) audio->play(AudioPlayer::Sound::IMU_DISCONNECT);
                last_alarm_time = now;
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - fps_start);
        if (elapsed.count() >= 5) {
            SerialImuParseStats pst{};
            reader.get_parse_stats(pst, true);
            float rate = period_count / (float)elapsed.count();
            if (state == State::CONNECTED) {
                fprintf(stderr, "[serial_imu:%s] %.1f Hz (total=%d) "
                        "accel=[%.2f,%.2f,%.2f] gyro=[%.3f,%.3f,%.3f]\n",
                        sc.name.c_str(), rate, sample_count,
                        last_sample.accel[0], last_sample.accel[1], last_sample.accel[2],
                        last_sample.gyro[0], last_sample.gyro[1], last_sample.gyro[2]);
                if (pst.bad_type_or_len || pst.bad_checksum || pst.bad_tail) {
                    fprintf(stderr, "[serial_imu:%s] parse_5s: ok=%llu type=%llu chk=%llu tail=%llu rx_bytes=%llu\n",
                            sc.name.c_str(),
                            (unsigned long long)pst.frames_ok,
                            (unsigned long long)pst.bad_type_or_len,
                            (unsigned long long)pst.bad_checksum,
                            (unsigned long long)pst.bad_tail,
                            (unsigned long long)pst.bytes_read);
                }
            } else {
                const char *status = (state == State::WAITING) ? "WAITING" : "DISCONNECTED";
                fprintf(stderr, "[serial_imu:%s] %s - no valid frames (parsed=%d) "
                        "parse_5s: ok=%llu type=%llu chk=%llu tail=%llu rx_bytes=%llu\n",
                        sc.name.c_str(), status, sample_count,
                        (unsigned long long)pst.frames_ok,
                        (unsigned long long)pst.bad_type_or_len,
                        (unsigned long long)pst.bad_checksum,
                        (unsigned long long)pst.bad_tail,
                        (unsigned long long)pst.bytes_read);
            }
            period_count = 0;
            fps_start = now;
        }
    }

    fprintf(stderr, "[serial_imu:%s] Stopping (total samples=%d)\n",
            sc.name.c_str(), sample_count);
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
    signal(SIGPIPE, SIG_IGN);

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

    std::unique_ptr<BleImuManager> ble_imu;
    if (cfg.ble_imus.enabled) {
        auto ble_outputs = build_ble_outputs(cfg);
        if (ble_outputs.empty()) {
            fprintf(stderr, "Error: BLE IMU enabled but no valid left/right output mapping was found in serial_imus\n");
            return 1;
        }

        fprintf(stderr, "[ble_imu] BLE IMU mode enabled, reusing %zu serial IMU output slots\n",
                ble_outputs.size());
        ble_imu = std::make_unique<BleImuManager>(cfg.ble_imus, cfg.config_path,
                                                  ble_outputs, recorder, &audio);
        if (!ble_imu->start()) {
            fprintf(stderr, "Error: failed to start BLE IMU manager\n");
            return 1;
        }
    }

    // Initialize runtime camera parameters
    std::vector<RuntimeCameraParams> runtime_params(cfg.cameras.size());

    // Initialize and start PWM sync BEFORE cameras.
    // Sensors in FSIN slave mode need pulses immediately at STREAMON,
    // so PWM must already be running when camera threads start.
    //
    // Auto-detect OV9281 fps from camera config so PWM always matches
    // the actual sensor frame rate — no manual ov9281_fps needed.
    PwmSync pwm_sync;
    bool pwm_ok = false;
    if (cfg.hw_sync.enabled) {
        // Scan cameras: pick OV9281 fps from first enabled OV9281
        for (auto &c : cfg.cameras) {
            if (!c.enabled) continue;
            if (c.sensor_entity_name.find("ov9281") != std::string::npos ||
                c.sensor_entity_name.find("ov9282") != std::string::npos) {
                cfg.hw_sync.ov9281_fps = c.fps;
                fprintf(stderr, "[pwm_sync] OV9281 fps auto-detected from config: %d\n", c.fps);
                break;
            }
        }
        // Pick IMX334 fps from first enabled IMX334
        for (auto &c : cfg.cameras) {
            if (!c.enabled) continue;
            if (c.sensor_entity_name.find("imx334") != std::string::npos) {
                cfg.hw_sync.fps = c.fps;
                fprintf(stderr, "[pwm_sync] IMX334 fps auto-detected from config: %d\n", c.fps);
                break;
            }
        }
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

    // Key monitor + handler (short press = record toggle, long press = BLE unbind + rescan).
    // Uses condition variable for instant key response instead of polling.
    std::atomic<bool> short_press_requested{false};
    std::atomic<bool> long_press_requested{false};
    std::mutex key_cv_mtx;
    std::condition_variable key_cv;

    if (cfg.recording.enabled || cfg.ble_imus.enabled) {
        threads.emplace_back([&cfg, &short_press_requested, &long_press_requested,
                              &key_cv]() {
            KeyMonitor km(
                cfg.recording.record_key_code,
                cfg.recording.input_device,
                [&short_press_requested, &key_cv]() {
                    short_press_requested.store(true);
                    key_cv.notify_one();
                },
                [&long_press_requested, &key_cv]() {
                    long_press_requested.store(true);
                    key_cv.notify_one();
                },
                cfg.ble_imus.enabled ? cfg.ble_imus.pair_long_press_ms : 1200);
            km.run(g_running);
        });

        threads.emplace_back([&cfg, &recorder, &audio, &short_press_requested,
                              &long_press_requested, &key_cv, &key_cv_mtx,
                              ble = ble_imu.get()]() {
            fprintf(stderr, "[keyctl] Handler thread started\n");

            while (g_running.load()) {
                {
                    std::unique_lock<std::mutex> lk(key_cv_mtx);
                    key_cv.wait_for(lk, std::chrono::milliseconds(200), [&]() {
                        return short_press_requested.load() || long_press_requested.load()
                               || !g_running.load();
                    });
                }

                if (!g_running.load()) break;

                if (long_press_requested.exchange(false)) {
                    if (ble) {
                        fprintf(stderr, "[keyctl] Long press -> BLE unbind + enter pairing\n");
                        ble->reset_pairing_and_rescan();
                        ble->enter_pairing_mode();
                        // Audio is handled by enter_pairing_mode -> update_pairing_audio
                    } else {
                        audio.play(AudioPlayer::Sound::REC_ERROR);
                    }
                }

                if (short_press_requested.exchange(false)) {
                    if (ble && ble->is_pairing_mode()) {
                        fprintf(stderr, "[keyctl] Short press -> exit pairing mode\n");
                        ble->finalize_pairing();
                        continue;  // skip recording and all other actions
                    }

                    bool was_recording = recorder.is_recording();
                    bool now_recording = was_recording;

                    if (cfg.recording.enabled) {
                        now_recording = recorder.toggle_recording();
                    }

                    if (ble) {
                        // BLE command is determined by actual local result:
                        //  - was recording -> always STOP
                        //  - was not recording -> START only if local succeeded, else STOP
                        RecordCommand ble_cmd;
                        if (was_recording) {
                            ble_cmd = RecordCommand::STOP;
                        } else {
                            ble_cmd = now_recording ? RecordCommand::START : RecordCommand::STOP;
                        }

                        const char *action = (ble_cmd == RecordCommand::START) ? "START" : "STOP";
                        fprintf(stderr,
                                "[keyctl] Short press -> local %s->%s, send BLE %s\n",
                                was_recording ? "REC" : "IDLE",
                                now_recording ? "REC" : "IDLE",
                                action);
                        ble->send_record_command(ble_cmd);
                    }
                }
            }
            fprintf(stderr, "[keyctl] Handler thread stopped\n");
        });
    }

    if (cfg.imu.enabled) {
        fprintf(stderr, "IMU enabled: %s:0x%02x @ %d Hz -> port %d\n",
                cfg.imu.i2c_device.c_str(), cfg.imu.i2c_addr,
                cfg.imu.sampling_frequency, cfg.imu.zmq_port);
        threads.emplace_back(imu_thread, std::cref(cfg.imu), std::ref(recorder));
    }

    if (!cfg.ble_imus.enabled) {
        int si = 0;
        for (size_t i = 0; i < cfg.serial_imus.size(); i++) {
            if (!cfg.serial_imus[i].enabled) continue;
            fprintf(stderr, "Serial IMU '%s': %s @ %d baud -> port %d\n",
                    cfg.serial_imus[i].name.c_str(),
                    cfg.serial_imus[i].uart_device.c_str(),
                    cfg.serial_imus[i].baudrate,
                    cfg.serial_imus[i].zmq_port);
            threads.emplace_back(serial_imu_thread, si,
                                 std::cref(cfg.serial_imus[i]),
                                 std::ref(recorder), &audio);
            si++;
        }
    } else {
        for (size_t i = 0; i < cfg.serial_imus.size(); i++) {
            if (!cfg.serial_imus[i].enabled) continue;
            fprintf(stderr, "BLE IMU output '%s': hand=%s -> port %d\n",
                    cfg.serial_imus[i].name.c_str(),
                    cfg.serial_imus[i].role.c_str(),
                    cfg.serial_imus[i].zmq_port);
        }
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
    if (ble_imu)
        ble_imu->stop();
    if (pwm_ok)
        pwm_sync.stop();
    recorder.stop();

    fprintf(stderr, "quad_cam_streamer exiting\n");
    return 0;
}
