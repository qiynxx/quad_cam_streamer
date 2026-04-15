#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gio/gio.h>

#include "config.h"

struct ImuSample;

class AudioPlayer;
class EurocRecorder;
class ZmqStreamer;

enum class BleHandRole {
    UNKNOWN,
    RIGHT,
    LEFT,
};

// Record control commands forwarded to the BLE peer (RK3588-W).
enum class RecordCommand {
    START,
    STOP,
};

struct BleImuOutputConfig {
    BleHandRole hand = BleHandRole::UNKNOWN;
    std::string name;
    int serial_index = -1;
    int zmq_port = 0;
};

class BleImuManager {
public:
    BleImuManager(const BleImuConfig &cfg,
                  const std::string &config_path,
                  const std::vector<BleImuOutputConfig> &outputs,
                  EurocRecorder &recorder,
                  AudioPlayer *audio);
    ~BleImuManager();

    bool start();
    void stop();

    void enter_pairing_mode();
    bool finalize_pairing();

    bool is_pairing_mode() const { return pairing_mode_.load(); }
    bool is_active() const { return active_mode_.load(); }
    bool can_finalize_pairing() const;
    int ready_hand_count() const;
    int expected_hand_count() const;

    void dispatch_device_found(const std::string &path,
                               const std::string &name,
                               const std::string &addr);
    void dispatch_timesync(const std::string &addr, GVariant *params);
    void dispatch_imu(const std::string &addr, GVariant *params);

    // Reset stored BLE pairing (MAC addresses) and restart scan.
    // Intended to be called from the key handler on triple-press.
    void reset_pairing_and_rescan();

    // Forward local recording state changes to the BLE peer. No-op if there is
    // no active STREAMING device or BLE is not running.
    void send_record_command(RecordCommand cmd);

private:
    struct CharHandles {
        std::string imu_path;
        std::string control_path;
        std::string config_path;
        std::string timesync_path;

        bool ready() const {
            return !imu_path.empty() && !control_path.empty() &&
                   !config_path.empty() && !timesync_path.empty();
        }
    };

    struct PongWaiter {
        bool got_pong = false;
        uint32_t esp_ts = 0;
        uint32_t rk_echo = 0;
        std::mutex mu;
        std::condition_variable cv;
    };

    enum class DevState {
        WAIT_ADV,
        CONNECTING,
        DISCOVERING,
        SYNCING,
        READY,
        STREAMING,
        RECONNECTING,
    };

    struct BleDevice {
        std::string name;
        std::string addr;
        std::string obj_path;
        BleHandRole role = BleHandRole::UNKNOWN;
        DevState state = DevState::WAIT_ADV;
        CharHandles chars;
        int32_t sync_offset = 0;
        int64_t last_sync_ms = 0;
        int64_t last_imu_ms = 0;
        PongWaiter pong_waiter;
        guint timesync_sub = 0;
        guint imu_sub = 0;
        std::atomic<bool> retry_pending{false};
    };

    struct OutputChannel {
        BleImuOutputConfig cfg;
        std::unique_ptr<ZmqStreamer> streamer;
        bool ready = false;
        bool alarm_active = false;
        int64_t last_alarm_ms = 0;
        uint32_t last_ts32 = 0;
        uint64_t ts_wraps = 0;
        bool has_ts = false;
    };

    bool init_dbus();
    bool reset_adapter();
    void start_scan();
    void stop_scan();
    void enumerate_known_devices();

    void on_device_found(const std::string &obj_path,
                         const std::string &name,
                         const std::string &addr);
    void connect_device(const std::shared_ptr<BleDevice> &dev);
    void disconnect_device(BleDevice &dev);
    void unsubscribe_device(BleDevice &dev);
    void discover_gatt(BleDevice &dev);
    bool discover_chars(BleDevice &dev);
    void run_device_init(BleDevice &dev);
    void schedule_retry(const std::shared_ptr<BleDevice> &dev, int delay_ms);

    bool do_stop_pattern(BleDevice &dev);
    bool do_time_sync(BleDevice &dev, int32_t &offset);
    bool do_set_offset(BleDevice &dev, int32_t offset);
    bool do_start_imu(BleDevice &dev);
    bool send_ping(BleDevice &dev, int64_t t1_ms);
    bool wait_pong(BleDevice &dev, int64_t t1_ms,
                   uint32_t &out_t2_ms, int64_t &out_t4_ms,
                   unsigned timeout_ms = 300);
    bool run_sync_round(BleDevice &dev, int32_t &out_offset);

    void on_timesync_notify(BleDevice &dev, GVariant *params);
    void on_imu_notify(BleDevice &dev, GVariant *params);

    bool write_char(const std::string &path,
                    const uint8_t *data, gsize len,
                    bool with_response = true);
    bool enable_notify(const std::string &path);

    // Best-effort request to tighten LE connection parameters for a given
    // device by calling `hcitool lecup` on RK3576. This operates at the HCI
    // level and is intended only for latency tuning; failures are non-fatal.
    void request_fast_conn_params(const BleDevice &dev);

    void watchdog_loop();
    void resync_loop();

    void update_pairing_audio();
    void update_connection_audio();
    void try_connect_next();
    void set_output_ready(BleHandRole hand, bool ready, bool play_alarm);
    OutputChannel *find_output(BleHandRole hand);
    const OutputChannel *find_output(BleHandRole hand) const;
    uint64_t expand_timestamp_ms(OutputChannel &out, uint32_t timestamp_ms32);
    void launch_worker(std::function<void()> fn);
    bool has_saved_pairing() const;
    std::string configured_addr_for(BleHandRole hand) const;
    BleHandRole role_from_configured_addr(const std::string &addr) const;
    bool persist_current_pairing();
    bool collect_paired_addresses(std::string &left_addr, std::string &right_addr) const;

    BleImuConfig cfg_;
    std::string config_path_;
    std::vector<OutputChannel> outputs_;
    EurocRecorder &recorder_;
    AudioPlayer *audio_ = nullptr;
    // Waist mode: only a single BLE device (RK3588-W) is expected. In this
    // mode we do not forward IMU samples over ZMQ / EuRoC, and treat IMU
    // notifications mainly as a heartbeat for disconnect detection.
    bool waist_mode_ = false;

    GDBusConnection *dbus_ = nullptr;
    GMainLoop *loop_ = nullptr;
    std::thread glib_thread_;
    std::thread watchdog_thread_;
    std::thread resync_thread_;
    std::mutex workers_mu_;
    std::vector<std::thread> worker_threads_;

    mutable std::mutex devs_mu_;
    std::map<std::string, std::shared_ptr<BleDevice>> devs_;

    mutable std::mutex outputs_mu_;
    std::atomic<bool> running_{false};
    std::atomic<bool> pairing_mode_{false};
    std::atomic<bool> active_mode_{false};
    bool scan_active_ = false;
    std::atomic<int> pairing_audio_state_{-2};
    std::atomic<int> connection_beep_state_{-1};
    bool pairing_persisted_ = false;
    guint scan_watch_ = 0;
};
