#include "ble_imu_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

#include "audio_player.h"
#include "euroc_recorder.h"
#include "imu_reader.h"
#include "zmq_streamer.h"

#define LOG(fmt, ...) fprintf(stderr, "[ble_imu] " fmt "\n", ##__VA_ARGS__)
#define WARN(fmt, ...) fprintf(stderr, "[ble_imu][warn] " fmt "\n", ##__VA_ARGS__)
#define ERR(fmt, ...) fprintf(stderr, "[ble_imu][err] " fmt "\n", ##__VA_ARGS__)

namespace {

constexpr const char *kTargetRightName = "ESP32S3-R";
constexpr const char *kTargetLeftName = "ESP32S3-L";
constexpr const char *kTargetWaistName = "RK3588-W";

constexpr const char *kUuidService = "12345678-1234-5678-1234-56789abcdeff";

constexpr const char *kUuidImu = "12345678-1234-5678-1234-56789abcdef1";
constexpr const char *kUuidControl = "12345678-1234-5678-1234-56789abcdef2";
constexpr const char *kUuidConfig = "12345678-1234-5678-1234-56789abcdef3";
constexpr const char *kUuidTimeSync = "12345678-1234-5678-1234-56789abcdef4";

constexpr uint8_t kCmdPing = 0x01;
constexpr uint8_t kCmdSetOffset = 0x02;
constexpr uint8_t kCmdStopPattern = 0x04;
constexpr uint8_t kCmdRecordStart = 0x10;
constexpr uint8_t kCmdRecordStop = 0x11;
constexpr uint8_t kRespPong = 0x81;

constexpr double kDegToRad = M_PI / 180.0;

int64_t monotonic_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void pack_u32_le(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value);
    buf[1] = (uint8_t)(value >> 8);
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
}

void pack_i32_le(uint8_t *buf, int32_t value)
{
    pack_u32_le(buf, (uint32_t)value);
}

uint32_t unpack_u32_le(const uint8_t *buf)
{
    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

float unpack_float_le(const uint8_t *buf)
{
    uint32_t raw = unpack_u32_le(buf);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

bool is_target_name(const std::string &name)
{
    return name == kTargetLeftName || name == kTargetRightName || name == kTargetWaistName;
}

BleHandRole role_from_name(const std::string &name)
{
    if (name == kTargetLeftName) return BleHandRole::LEFT;
    if (name == kTargetRightName) return BleHandRole::RIGHT;
    if (name == kTargetWaistName) return BleHandRole::WAIST;
    return BleHandRole::UNKNOWN;
}

const char *role_str(BleHandRole role)
{
    switch (role) {
    case BleHandRole::LEFT: return "left";
    case BleHandRole::RIGHT: return "right";
    case BleHandRole::WAIST: return "waist";
    default: return "unknown";
    }
}

bool method_requires_addr_type_arg(const GError *err, const char *method_name)
{
    if (!err || !err->message) return false;
    const std::string msg = err->message;

    // 如果错误本身就是 UnknownObject，说明 Device1 对象路径已经无效，
    // 这种情况下无论是否带 addr_type 参数，调用都会失败，没有必要再重试。
    // 这里直接返回 false，避免在已被 BlueZ 移除的对象上再次发起
    // Connect("auto") / Disconnect("auto")，导致类似：
    //   GDBus.Error:org.freedesktop.DBus.Error.UnknownObject:
    //     Method "Connect" with signature "s" on interface "org.bluez.Device1" doesn't exist
    if (msg.find("org.freedesktop.DBus.Error.UnknownObject") != std::string::npos)
        return false;

    const std::string sig = std::string("Method \"") + method_name +
                            "\" with signature \"\"";
    // 在不同版本的 BlueZ / GLib 下，这类错误可能表现为 UnknownMethod 或 UnknownObject，
    // 但真正关键的是提示 "Method \"X\" with signature \"\" ... doesn't exist"。
    // 这里主要根据消息内容匹配，但上面已排除了 UnknownObject 场景，避免
    // BlueZ 把设备对象删除后仍错误地走到带 addr_type 参数的兼容路径。
    return msg.find(sig) != std::string::npos &&
           msg.find("doesn't exist") != std::string::npos;
}

GVariant *call_device1_method_compat(GDBusConnection *dbus,
                                     const std::string &obj_path,
                                     const char *method_name,
                                     gint timeout_ms,
                                     GError **err_out)
{
    GError *err = nullptr;
    GVariant *res = g_dbus_connection_call_sync(
        dbus, "org.bluez", obj_path.c_str(),
        "org.bluez.Device1", method_name,
        nullptr, nullptr,
        G_DBUS_CALL_FLAGS_NONE, timeout_ms, nullptr, &err);
    if (res || !method_requires_addr_type_arg(err, method_name)) {
        if (err_out) *err_out = err;
        else if (err) g_error_free(err);
        return res;
    }

    g_error_free(err);
    err = nullptr;
    res = g_dbus_connection_call_sync(
        dbus, "org.bluez", obj_path.c_str(),
        "org.bluez.Device1", method_name,
        g_variant_new("(s)", "auto"), nullptr,
        G_DBUS_CALL_FLAGS_NONE, timeout_ms, nullptr, &err);
    if (err_out) *err_out = err;
    else if (err) g_error_free(err);
    return res;
}

GVariant *bytes_variant(const uint8_t *data, gsize len)
{
    GVariantBuilder builder;
    g_variant_builder_init(&builder, G_VARIANT_TYPE("ay"));
    for (gsize i = 0; i < len; ++i)
        g_variant_builder_add(&builder, "y", data[i]);
    return g_variant_builder_end(&builder);
}

GVariant *empty_options_dict()
{
    GVariantBuilder builder;
    g_variant_builder_init(&builder, G_VARIANT_TYPE("a{sv}"));
    return g_variant_builder_end(&builder);
}

struct IfaceAddedCtx {
    BleImuManager *mgr = nullptr;
};

struct TimeSyncCtx {
    BleImuManager *mgr = nullptr;
    std::string addr;
};

struct ImuCtx {
    BleImuManager *mgr = nullptr;
    std::string addr;
};

void on_interfaces_added(GDBusConnection *, const gchar *,
                         const gchar *, const gchar *,
                         const gchar *, GVariant *params,
                         gpointer user_data)
{
    auto *ctx = static_cast<IfaceAddedCtx *>(user_data);
    const gchar *obj_path = nullptr;
    GVariant *ifaces_props = nullptr;
    g_variant_get(params, "(&o@a{sa{sv}})", &obj_path, &ifaces_props);

    GVariantIter iter;
    g_variant_iter_init(&iter, ifaces_props);
    const gchar *iface_name = nullptr;
    GVariant *iface_props = nullptr;
    while (g_variant_iter_loop(&iter, "{&s@a{sv}}", &iface_name, &iface_props)) {
        if (strcmp(iface_name, "org.bluez.Device1") != 0)
            continue;

        GVariant *name_v = g_variant_lookup_value(iface_props, "Name", G_VARIANT_TYPE_STRING);
        GVariant *addr_v = g_variant_lookup_value(iface_props, "Address", G_VARIANT_TYPE_STRING);
        if (name_v && addr_v) {
            std::string name = g_variant_get_string(name_v, nullptr);
            std::string addr = g_variant_get_string(addr_v, nullptr);
            if (is_target_name(name))
                ctx->mgr->dispatch_device_found(obj_path, name, addr);
        }
        if (name_v) g_variant_unref(name_v);
        if (addr_v) g_variant_unref(addr_v);
    }

    g_variant_unref(ifaces_props);
}

void timesync_notify_cb(GDBusConnection *, const gchar *,
                        const gchar *, const gchar *,
                        const gchar *, GVariant *params,
                        gpointer user_data)
{
    auto *ctx = static_cast<TimeSyncCtx *>(user_data);
    ctx->mgr->dispatch_timesync(ctx->addr, params);
}

void imu_notify_cb(GDBusConnection *, const gchar *,
                   const gchar *, const gchar *,
                   const gchar *, GVariant *params,
                   gpointer user_data)
{
    auto *ctx = static_cast<ImuCtx *>(user_data);
    ctx->mgr->dispatch_imu(ctx->addr, params);
}

guint subscribe_props(GDBusConnection *dbus,
                      const std::string &char_path,
                      GDBusSignalCallback cb,
                      gpointer user_data,
                      GDestroyNotify notify = nullptr)
{
    return g_dbus_connection_signal_subscribe(
        dbus, "org.bluez",
        "org.freedesktop.DBus.Properties", "PropertiesChanged",
        char_path.c_str(), "org.bluez.GattCharacteristic1",
        G_DBUS_SIGNAL_FLAGS_NONE,
        cb, user_data, notify);
}

// 尝试通过 Device1.CancelConnect 取消当前正在进行的连接尝试。
// 某些 BlueZ 版本在自动重连或上一次 Connect 尚未收尾时，会对新的
// Connect 调用返回 "Operation already in progress"。在这种情况下先发
// 一次 CancelConnect 再稍候重试，可以避免长时间卡在“连接进行中”的状态。
static void cancel_connect_attempt(GDBusConnection *dbus,
                                   const std::string &obj_path,
                                   const char *name_for_log)
{
    if (!dbus || obj_path.empty())
        return;

    GError *err = nullptr;
    GVariant *res = g_dbus_connection_call_sync(
        dbus, "org.bluez", obj_path.c_str(),
        "org.bluez.Device1", "CancelConnect",
        nullptr, nullptr,
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (err) {
        WARN("[%s] CancelConnect(%s): %s",
             name_for_log ? name_for_log : "BLE",
             obj_path.c_str(), err->message);
        g_error_free(err);
    } else {
        LOG("[%s] CancelConnect(%s) succeeded",
            name_for_log ? name_for_log : "BLE",
            obj_path.c_str());
    }
    if (res) g_variant_unref(res);
}

// 尝试通过 Adapter1.RemoveDevice 把指定 Device1 对象从 BlueZ 中移除。
// 主要用于 Connect 长时间超时后的“强制清理”，让后续重新发现得到新的 obj_path。
bool remove_bluez_device(GDBusConnection *dbus, const std::string &obj_path)
{
    if (!dbus || obj_path.empty())
        return false;

    GError *err = nullptr;
    GVariant *res = g_dbus_connection_call_sync(
        dbus, "org.bluez", "/org/bluez/hci0",
        "org.bluez.Adapter1", "RemoveDevice",
        g_variant_new("(o)", obj_path.c_str()),
        nullptr,
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (err) {
        WARN("RemoveDevice(%s): %s", obj_path.c_str(), err->message);
        g_error_free(err);
    }
    if (res) g_variant_unref(res);
    return err == nullptr;
}

}  // namespace

BleImuManager::BleImuManager(const BleImuConfig &cfg,
                             const std::string &config_path,
                             const std::vector<BleImuOutputConfig> &outputs,
                             EurocRecorder &recorder,
                             AudioPlayer *audio)
    : cfg_(cfg), config_path_(config_path), recorder_(recorder), audio_(audio)
{
    outputs_.reserve(outputs.size());
    for (const auto &cfg_out : outputs) {
        OutputChannel out;
        out.cfg = cfg_out;
        outputs_.push_back(std::move(out));
    }
}

BleImuManager::~BleImuManager()
{
    stop();
}

bool BleImuManager::start()
{
    if (running_.load())
        return true;

    for (auto &out : outputs_) {
        out.streamer = std::make_unique<ZmqStreamer>();
        if (!out.streamer->bind(out.cfg.zmq_port, 200)) {
            ERR("Failed to bind BLE IMU ZMQ output '%s' on port %d",
                out.cfg.name.c_str(), out.cfg.zmq_port);
            return false;
        }
        LOG("BLE IMU output '%s' -> tcp://*:%d (%s)",
            out.cfg.name.c_str(), out.cfg.zmq_port, role_str(out.cfg.hand));
    }

    if (!init_dbus())
        return false;

    // 尝试在每次启动时软重置一次 BLE 适配器，避免上一次进程异常退出
    // 留下的连接状态干扰本次 Connect 流程。
    reset_adapter();

    running_ = true;
    active_mode_ = true;  // Always allow streaming once devices are ready
    watchdog_thread_ = std::thread(&BleImuManager::watchdog_loop, this);
    resync_thread_ = std::thread(&BleImuManager::resync_loop, this);
    glib_thread_ = std::thread([this]() {
        g_main_loop_run(loop_);
    });

    if (has_saved_pairing()) {
        LOG("Auto-resume BLE pairing: left=%s right=%s waist=%s",
            cfg_.paired_left_addr.c_str(), cfg_.paired_right_addr.c_str(),
            cfg_.paired_waist_addr.c_str());

        // Only scan and connect when we have saved pairing info to auto-resume.
        enumerate_known_devices();
        start_scan();
        update_connection_audio();
    } else {
        LOG("No saved BLE pairing, waiting for long-press to enter pairing mode");
    }

    LOG("BLE IMU manager started");
    return true;
}

void BleImuManager::stop()
{
    if (!running_.exchange(false)) {
        if (audio_)
            audio_->stop_status_ticker();
        return;
    }

    pairing_mode_ = false;
    active_mode_ = false;
    update_pairing_audio();
    stop_scan();

    std::vector<std::shared_ptr<BleDevice>> devices;
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        for (auto &kv : devs_)
            devices.push_back(kv.second);
    }
    for (auto &dev : devices) {
        unsubscribe_device(*dev);
        disconnect_device(*dev);
    }

    if (loop_)
        g_main_loop_quit(loop_);

    if (glib_thread_.joinable())
        glib_thread_.join();
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    if (resync_thread_.joinable())
        resync_thread_.join();

    while (true) {
        std::vector<std::thread> workers;
        {
            std::lock_guard<std::mutex> lk(workers_mu_);
            if (worker_threads_.empty())
                break;
            workers.swap(worker_threads_);
        }
        for (auto &worker : workers) {
            if (worker.joinable())
                worker.join();
        }
    }

    if (dbus_ && scan_watch_) {
        g_dbus_connection_signal_unsubscribe(dbus_, scan_watch_);
        scan_watch_ = 0;
    }
    if (loop_) {
        g_main_loop_unref(loop_);
        loop_ = nullptr;
    }
    if (dbus_) {
        g_object_unref(dbus_);
        dbus_ = nullptr;
    }

    LOG("BLE IMU manager stopped");
}

bool BleImuManager::init_dbus()
{
    GError *err = nullptr;
    dbus_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &err);
    if (!dbus_) {
        ERR("g_bus_get_sync failed: %s", err ? err->message : "?");
        if (err) g_error_free(err);
        return false;
    }

    loop_ = g_main_loop_new(nullptr, FALSE);
    auto *ctx = new IfaceAddedCtx{this};
    scan_watch_ = g_dbus_connection_signal_subscribe(
        dbus_, "org.bluez", "org.freedesktop.DBus.ObjectManager",
        "InterfacesAdded", "/",
        nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
        on_interfaces_added, ctx,
        [](gpointer p) { delete static_cast<IfaceAddedCtx *>(p); });
    return true;
}

bool BleImuManager::reset_adapter()
{
    if (!dbus_)
        return false;

    // 通过 BlueZ Properties.Set("Powered") OFF/ON 来软重启适配器，
    // 尽量把上一轮遗留的连接状态清干净，避免下一次 Connect
    // 撞到蓝牙内核 / BlueZ 的“半连接”状态。
    auto power_set = [&](bool on) -> bool {
        GError *err = nullptr;
        GVariant *res = g_dbus_connection_call_sync(
            dbus_, "org.bluez", "/org/bluez/hci0",
            "org.freedesktop.DBus.Properties", "Set",
            g_variant_new("(ssv)", "org.bluez.Adapter1", "Powered",
                          g_variant_new_boolean(on ? TRUE : FALSE)),
            nullptr,
            G_DBUS_CALL_FLAGS_NONE, 8000, nullptr, &err);
        if (err) {
            WARN("Adapter Powered=%d failed: %s", on ? 1 : 0, err->message);
            g_error_free(err);
            if (res) g_variant_unref(res);
            return false;
        }
        if (res) g_variant_unref(res);
        return true;
    };

    LOG("Resetting BLE adapter (Powered off/on)...");
    power_set(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool ok = power_set(true);
    if (ok)
        LOG("BLE adapter reset complete");
    else
        WARN("BLE adapter reset may be incomplete");
    return ok;
}

void BleImuManager::start_scan()
{
    if (!dbus_ || scan_active_)
        return;

    GError *err = nullptr;
    g_dbus_connection_call_sync(
        dbus_, "org.bluez", "/org/bluez/hci0",
        "org.bluez.Adapter1", "StartDiscovery",
        nullptr, nullptr,
        G_DBUS_CALL_FLAGS_NONE, 10000, nullptr, &err);
    if (err) {
        // 如果 BlueZ 报 InProgress，说明扫描已经在进行中，可以视为成功，
        // 避免重复调用 StartDiscovery 导致日志刷屏。
        std::string emsg = err->message ? err->message : "";
        if (emsg.find("org.bluez.Error.InProgress") != std::string::npos) {
            LOG("BLE discovery already in progress");
            scan_active_ = true;
        } else {
            WARN("StartDiscovery: %s", err->message);
        }
        g_error_free(err);
        if (!scan_active_)
            return;
    }
    scan_active_ = true;
    LOG("BLE discovery started");
}

void BleImuManager::stop_scan()
{
    if (!dbus_ || !scan_active_)
        return;

    GError *err = nullptr;
    g_dbus_connection_call_sync(
        dbus_, "org.bluez", "/org/bluez/hci0",
        "org.bluez.Adapter1", "StopDiscovery",
        nullptr, nullptr,
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (err) {
        WARN("StopDiscovery: %s", err->message);
        g_error_free(err);
    }
    scan_active_ = false;
}

void BleImuManager::enumerate_known_devices()
{
    if (!dbus_)
        return;

    GError *err = nullptr;
    GVariant *managed = g_dbus_connection_call_sync(
        dbus_, "org.bluez", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects",
        nullptr, G_VARIANT_TYPE("(a{oa{sa{sv}}})"),
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (!managed) {
        WARN("GetManagedObjects: %s", err ? err->message : "?");
        if (err) g_error_free(err);
        return;
    }

    GVariant *objects = g_variant_get_child_value(managed, 0);
    GVariantIter obj_iter;
    g_variant_iter_init(&obj_iter, objects);

    const gchar *obj_path = nullptr;
    GVariant *ifaces = nullptr;
    while (g_variant_iter_loop(&obj_iter, "{&o@a{sa{sv}}}", &obj_path, &ifaces)) {
        GVariant *dev_props = g_variant_lookup_value(ifaces,
            "org.bluez.Device1", G_VARIANT_TYPE("a{sv}"));
        if (!dev_props)
            continue;

        GVariant *name_v = g_variant_lookup_value(dev_props, "Name", G_VARIANT_TYPE_STRING);
        GVariant *addr_v = g_variant_lookup_value(dev_props, "Address", G_VARIANT_TYPE_STRING);
        if (name_v && addr_v) {
            std::string name = g_variant_get_string(name_v, nullptr);
            std::string addr = g_variant_get_string(addr_v, nullptr);
            if (is_target_name(name))
                dispatch_device_found(obj_path, name, addr);
        }
        if (name_v) g_variant_unref(name_v);
        if (addr_v) g_variant_unref(addr_v);
        g_variant_unref(dev_props);
    }

    g_variant_unref(objects);
    g_variant_unref(managed);
}

void BleImuManager::enter_pairing_mode()
{
    if (!running_.load())
        return;

    // Allow re-entry: reset flags first so we always start clean.
    pairing_mode_ = false;
    active_mode_ = true;
    pairing_audio_state_ = -2;
    connection_beep_state_ = -1;

    // Suspend output
    {
        std::lock_guard<std::mutex> lk(outputs_mu_);
        for (auto &out : outputs_) {
            out.ready = false;
            out.alarm_active = false;
        }
    }

    // Stop any existing ticker before entering pairing mode
    if (audio_)
        audio_->stop_status_ticker();

    pairing_mode_ = true;
    active_mode_ = false;
    LOG("Entering BLE pairing mode");

    // Start scan and enumerate
    enumerate_known_devices();
    start_scan();

    // Start pairing audio ticker (will play based on missing device count)
    update_pairing_audio();
}

bool BleImuManager::finalize_pairing()
{
    if (!pairing_mode_.load())
        return active_mode_.load();

    // 1. Stop pairing audio immediately
    if (audio_)
        audio_->stop_status_ticker();

    const int ready = ready_hand_count();

    // 2. Save whatever is connected
    if (ready > 0) {
        if (!persist_current_pairing()) {
            ERR("Failed to save pairing info");
        } else {
            LOG("BLE pairing saved (%d/%d devices) to %s",
                ready, expected_hand_count(), config_path_.c_str());
        }
    } else {
        LOG("Exiting pairing mode with no devices connected");
    }

    // 3. Exit pairing mode
    pairing_mode_ = false;
    active_mode_ = true;
    pairing_audio_state_ = -2;
    connection_beep_state_ = -1;

    // 4. Purge devices that are not in saved pairing, to stop stale reconnects.
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        for (auto it = devs_.begin(); it != devs_.end(); ) {
            auto &dev = it->second;
            if (!dev) { it = devs_.erase(it); continue; }
            std::string saved = configured_addr_for(dev->role);
            if (saved.empty() || saved != dev->addr) {
                LOG("[%s] Removing unpaired device from devs_", dev->name.c_str());
                unsubscribe_device(*dev);
                disconnect_device(*dev);
                it = devs_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 5. Reset outputs for purged devices
    {
        std::lock_guard<std::mutex> lk(outputs_mu_);
        for (auto &out : outputs_) {
            std::string saved = configured_addr_for(out.cfg.hand);
            if (saved.empty()) {
                out.ready = false;
                out.alarm_active = false;
            }
        }
    }

    // 6. Play exit confirmation
    if (audio_)
        audio_->play(AudioPlayer::Sound::BLE_PAIR_EXIT);

    // 7. Start connection audio based on saved devices only
    update_connection_audio();

    LOG("Pairing mode exited");
    return true;
}

bool BleImuManager::can_finalize_pairing() const
{
    // 至少有一个设备 ready 就可以确认保存
    return ready_hand_count() > 0;
}

int BleImuManager::ready_hand_count() const
{
    std::lock_guard<std::mutex> lk(outputs_mu_);
    int ready = 0;
    for (const auto &out : outputs_) {
        if (out.ready)
            ready++;
    }
    return ready;
}

int BleImuManager::expected_hand_count() const
{
    return (int)outputs_.size();
}

void BleImuManager::dispatch_device_found(const std::string &path,
                                          const std::string &name,
                                          const std::string &addr)
{
    if (!running_.load())
        return;

    // Always use advertised name to infer hand role.
    BleHandRole expected_role = role_from_name(name);
    if (expected_role == BleHandRole::UNKNOWN)
        return;

    // If we have saved pairing info, for each role that has a saved MAC,
    // only accept devices whose MAC matches. Roles without saved addresses
    // are ignored in auto-resume (they were not paired last time).
    if (has_saved_pairing()) {
        std::string configured = configured_addr_for(expected_role);
        if (configured.empty() || configured != addr)
            return;
    }

    std::shared_ptr<BleDevice> dev;
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        auto it = devs_.find(addr);
        if (it != devs_.end()) {
            dev = it->second;
            dev->name = name;
            dev->obj_path = path;
            dev->role = expected_role;
        } else {
            dev = std::make_shared<BleDevice>();
            dev->name = name;
            dev->addr = addr;
            dev->obj_path = path;
            dev->role = expected_role;
            devs_[addr] = dev;
        }
    }

    if (dev->state == DevState::WAIT_ADV || dev->state == DevState::RECONNECTING) {
        LOG("Found BLE IMU %s [%s] @ %s", name.c_str(), addr.c_str(), path.c_str());
        try_connect_next();
    }
}

void BleImuManager::connect_device(const std::shared_ptr<BleDevice> &dev)
{
    if (!running_.load() || !dev)
        return;

    // 某些控制器在扫描与连接并发时稳定性较差，这里在发起 Connect 之前
    // 临时停止扫描，待本轮 Connect 返回后再恢复 StartDiscovery。
    stop_scan();

    dev->retry_pending = false;
    if (dev->state == DevState::CONNECTING ||
        dev->state == DevState::DISCOVERING ||
        dev->state == DevState::SYNCING ||
        dev->state == DevState::STREAMING) {
        start_scan();
        return;
    }

    dev->state = DevState::CONNECTING;
    GError *err = nullptr;
    LOG("[%s] Connecting...", dev->name.c_str());
    GVariant *res = call_device1_method_compat(
        dbus_, dev->obj_path, "Connect", 2000, &err);
    if (err) {
        const char *msg = err->message ? err->message : "";
        WARN("[%s] Connect failed: %s", dev->name.c_str(), msg);
        g_error_free(err);

        std::string emsg = msg ? msg : "";
        // 如果 Device1 对象已经不存在，说明 BlueZ 认为这个设备已经被移除。
        // 这种情况下不再对旧的 obj_path 重试，等待 on_interfaces_added /
        // enumerate_known_devices 发现新的对象即可。
        if (emsg.find("org.freedesktop.DBus.Error.UnknownObject") != std::string::npos) {
            WARN("[%s] Device object gone, waiting for rediscovery", dev->name.c_str());
            std::lock_guard<std::mutex> lk(devs_mu_);
            auto it = devs_.find(dev->addr);
            if (it != devs_.end() && it->second == dev) {
                devs_.erase(it);
            }
            start_scan();
            return;
        }

        // 根据错误类型调整重试节奏，避免在 BlueZ 仍在处理上一次连接时
        // 过于频繁地调用 Connect。
        if (emsg.find("Operation already in progress") != std::string::npos) {
            // BlueZ 认为当前设备仍有 Connect 在进行中：先尝试显式取消，
            // 避免长时间卡在“连接进行中”的状态，然后短暂等待后重试。
            cancel_connect_attempt(dbus_, dev->obj_path, dev->name.c_str());
            dev->state = DevState::WAIT_ADV;
            schedule_retry(dev, 1000);  // 1s 再试
        } else if (emsg.find("Timeout was reached") != std::string::npos) {
            // 超时：说明这轮完全没连上，且 BlueZ 可能在内部保持一个“半连接”状态。
            // 这里通过 Adapter1.RemoveDevice 强制清理一次该设备的内部状态，
            // 并从本地 devs_ 表中删除，让后续通过重新发现拿到一个干净的对象。
            WARN("[%s] Connect timeout, forcing RemoveDevice and rediscovery",
                 dev->name.c_str());
            remove_bluez_device(dbus_, dev->obj_path);
            {
                std::lock_guard<std::mutex> lk(devs_mu_);
                auto it = devs_.find(dev->addr);
                if (it != devs_.end() && it->second == dev) {
                    devs_.erase(it);
                }
            }
        } else {
            // 其他错误：按原来的节奏重试。
            dev->state = DevState::WAIT_ADV;
            schedule_retry(dev, 1500);
        }
        start_scan();
        return;
    }
    if (res) g_variant_unref(res);

    dev->state = DevState::DISCOVERING;
    LOG("[%s] Connected", dev->name.c_str());
    request_fast_conn_params(*dev);
    discover_gatt(*dev);
    // 当前设备的 Connect 流程结束，重新开启扫描以便发现其他设备或后续重连。
    start_scan();
}

void BleImuManager::disconnect_device(BleDevice &dev)
{
    if (!dbus_)
        return;
    GError *err = nullptr;
    GVariant *res = call_device1_method_compat(
        dbus_, dev.obj_path, "Disconnect", 5000, &err);
    if (err) {
        WARN("[%s] Disconnect: %s", dev.name.c_str(), err->message);
        g_error_free(err);
    }
    if (res) g_variant_unref(res);
}

void BleImuManager::unsubscribe_device(BleDevice &dev)
{
    if (!dbus_)
        return;
    if (dev.timesync_sub) {
        g_dbus_connection_signal_unsubscribe(dbus_, dev.timesync_sub);
        dev.timesync_sub = 0;
    }
    if (dev.imu_sub) {
        g_dbus_connection_signal_unsubscribe(dbus_, dev.imu_sub);
        dev.imu_sub = 0;
    }
}

void BleImuManager::discover_gatt(BleDevice &dev)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    if (!running_.load())
        return;

    if (!discover_chars(dev)) {
        WARN("[%s] GATT discovery failed", dev.name.c_str());
        dev.state = DevState::WAIT_ADV;
        auto retry_dev = [&]() -> std::shared_ptr<BleDevice> {
            std::lock_guard<std::mutex> lk(devs_mu_);
            auto it = devs_.find(dev.addr);
            return (it != devs_.end()) ? it->second : nullptr;
        }();
        schedule_retry(retry_dev, 1500);
        return;
    }

    LOG("[%s] GATT discovered", dev.name.c_str());
    run_device_init(dev);
}

bool BleImuManager::discover_chars(BleDevice &dev)
{
    GError *err = nullptr;
    GVariant *managed = g_dbus_connection_call_sync(
        dbus_, "org.bluez", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects",
        nullptr, G_VARIANT_TYPE("(a{oa{sa{sv}}})"),
        G_DBUS_CALL_FLAGS_NONE, 10000, nullptr, &err);
    if (!managed) {
        WARN("GetManagedObjects: %s", err ? err->message : "?");
        if (err) g_error_free(err);
        return false;
    }

    dev.chars = {};

    GVariant *objects = g_variant_get_child_value(managed, 0);
    GVariantIter obj_iter;
    const gchar *obj_path = nullptr;
    GVariant *ifaces = nullptr;

    // First pass: collect all GattService1 UUIDs so we can later filter
    // characteristics by their parent service. Important on RK3588-W where
    // BlueZ may expose multiple services with overlapping characteristic UUIDs.
    std::map<std::string, std::string> service_uuids;
    g_variant_iter_init(&obj_iter, objects);
    while (g_variant_iter_loop(&obj_iter, "{&o@a{sa{sv}}}", &obj_path, &ifaces)) {
        GVariant *svc_props = g_variant_lookup_value(
            ifaces, "org.bluez.GattService1", G_VARIANT_TYPE("a{sv}"));
        if (!svc_props)
            continue;

        GVariant *uuid_v = g_variant_lookup_value(
            svc_props, "UUID", G_VARIANT_TYPE_STRING);
        if (uuid_v) {
            std::string uuid = g_variant_get_string(uuid_v, nullptr);
            service_uuids[std::string(obj_path)] = uuid;
            g_variant_unref(uuid_v);
        }
        g_variant_unref(svc_props);
    }

    // Second pass: find characteristics that belong to the target device AND
    // whose parent service UUID matches kUuidService.
    g_variant_iter_init(&obj_iter, objects);
    while (g_variant_iter_loop(&obj_iter, "{&o@a{sa{sv}}}", &obj_path, &ifaces)) {
        if (strncmp(obj_path, dev.obj_path.c_str(), dev.obj_path.size()) != 0)
            continue;

        GVariant *char_props = g_variant_lookup_value(ifaces,
            "org.bluez.GattCharacteristic1", G_VARIANT_TYPE("a{sv}"));
        if (!char_props)
            continue;

        // For WAIST (RK3588-W) devices, filter characteristics by parent
        // service UUID to avoid BlueZ services with overlapping char UUIDs.
        // ESP32 devices don't need this filter.
        if (dev.role == BleHandRole::WAIST) {
            std::string path_str = obj_path;
            auto last_slash = path_str.rfind('/');
            if (last_slash != std::string::npos) {
                std::string service_path = path_str.substr(0, last_slash);
                auto svc_it = service_uuids.find(service_path);
                if (svc_it != service_uuids.end() && svc_it->second != kUuidService) {
                    g_variant_unref(char_props);
                    continue;
                }
            }
        }

        GVariant *uuid_v = g_variant_lookup_value(char_props, "UUID", G_VARIANT_TYPE_STRING);
        if (uuid_v) {
            std::string uuid = g_variant_get_string(uuid_v, nullptr);
            if (uuid == kUuidImu) dev.chars.imu_path = obj_path;
            else if (uuid == kUuidControl) dev.chars.control_path = obj_path;
            else if (uuid == kUuidConfig) dev.chars.config_path = obj_path;
            else if (uuid == kUuidTimeSync) dev.chars.timesync_path = obj_path;
            g_variant_unref(uuid_v);
        }
        g_variant_unref(char_props);
    }

    g_variant_unref(objects);
    g_variant_unref(managed);
    return dev.chars.ready();
}

void BleImuManager::run_device_init(BleDevice &dev)
{
    dev.state = DevState::SYNCING;

    if (!enable_notify(dev.chars.timesync_path)) {
        WARN("[%s] Failed to enable TimeSync notify", dev.name.c_str());
    }
    auto *ts_ctx = new TimeSyncCtx{this, dev.addr};
    dev.timesync_sub = subscribe_props(
        dbus_, dev.chars.timesync_path, timesync_notify_cb, ts_ctx,
        [](gpointer p) { delete static_cast<TimeSyncCtx *>(p); });

    GError *err = nullptr;
    GVariant *cfg_val = g_dbus_connection_call_sync(
        dbus_, "org.bluez", dev.chars.config_path.c_str(),
        "org.bluez.GattCharacteristic1", "ReadValue",
        g_variant_new("(@a{sv})", empty_options_dict()), G_VARIANT_TYPE("(ay)"),
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (cfg_val) {
        GVariant *ay = g_variant_get_child_value(cfg_val, 0);
        gsize n = 0;
        const uint8_t *bytes = (const uint8_t *)g_variant_get_fixed_array(ay, &n, 1);
        if (n >= 1) {
            // WAIST role is determined by device name (RK3588-W), not config byte.
            // Only override for ESP32 devices where config byte determines L/R.
            if (dev.role != BleHandRole::WAIST) {
                dev.role = (bytes[0] == 0x01) ? BleHandRole::LEFT : BleHandRole::RIGHT;
            }
            LOG("[%s] Config read: hand=%s", dev.name.c_str(), role_str(dev.role));
        }
        g_variant_unref(ay);
        g_variant_unref(cfg_val);
    } else if (err) {
        WARN("[%s] Config read: %s", dev.name.c_str(), err->message);
        g_error_free(err);
    }

    if (!do_stop_pattern(dev))
        WARN("[%s] StopPattern failed, continuing", dev.name.c_str());

    // Time sync: best-effort for all devices. RK3588-W may not respond to
    // pings (no pong), in which case sync fails gracefully with offset=0.
    int32_t offset = 0;
    if (do_time_sync(dev, offset)) {
        if (!do_set_offset(dev, offset))
            WARN("[%s] SetOffset failed, continuing with offset=0", dev.name.c_str());
    } else {
        WARN("[%s] Time sync failed, continuing with offset=0", dev.name.c_str());
    }

    // IMU notify: only for devices that transmit IMU data (ESP32).
    // WAIST (RK3588-W) only receives recording commands, no IMU output.
    if (dev.role != BleHandRole::WAIST) {
        if (!do_start_imu(dev))
            WARN("[%s] Failed to start IMU notify, continuing", dev.name.c_str());
    }

    dev.state = DevState::STREAMING;
    dev.last_imu_ms = monotonic_ms();
    LOG("[%s] BLE device ready (sync_offset=%dms)", dev.name.c_str(), offset);

    // 主动标记 output ready，驱动连接状态提示音。
    // 对于 WAIST 等可能没有 IMU notify 的设备也需要在此置 ready。
    set_output_ready(dev.role, true, /*play_alarm=*/false);

    if (pairing_mode_.load())
        update_pairing_audio();

    // 当前设备已完成初始化，尝试连接下一个设备。
    try_connect_next();
}

void BleImuManager::schedule_retry(const std::shared_ptr<BleDevice> &dev, int delay_ms)
{
    if (!dev || !running_.load())
        return;
    if (dev->retry_pending.exchange(true))
        return;

    launch_worker([this, dev, delay_ms]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        dev->retry_pending = false;
        if (!running_.load())
            return;
        if (!pairing_mode_.load() && !active_mode_.load())
            return;
        // 交给统一调度逻辑，根据优先级（先 R 后 L）选择下一个要连接的设备，
        // 并确保全局只有一条 Connect 在进行。
        try_connect_next();
    });
}

bool BleImuManager::do_stop_pattern(BleDevice &dev)
{
    uint8_t cmd = kCmdStopPattern;
    return write_char(dev.chars.timesync_path, &cmd, 1);
}

bool BleImuManager::do_time_sync(BleDevice &dev, int32_t &offset)
{
    if (run_sync_round(dev, offset))
        return true;

    WARN("[%s] Initial sync failed, retrying once", dev.name.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return run_sync_round(dev, offset);
}

bool BleImuManager::do_set_offset(BleDevice &dev, int32_t offset)
{
    uint8_t buf[5];
    buf[0] = kCmdSetOffset;
    pack_i32_le(buf + 1, offset);
    if (!write_char(dev.chars.timesync_path, buf, sizeof(buf)))
        return false;

    dev.sync_offset = offset;
    dev.last_sync_ms = monotonic_ms();
    return true;
}

bool BleImuManager::do_start_imu(BleDevice &dev)
{
    if (!enable_notify(dev.chars.imu_path))
        return false;

    auto *imu_ctx = new ImuCtx{this, dev.addr};
    dev.imu_sub = subscribe_props(
        dbus_, dev.chars.imu_path, imu_notify_cb, imu_ctx,
        [](gpointer p) { delete static_cast<ImuCtx *>(p); });

    uint8_t enable = 0x01;
    return write_char(dev.chars.control_path, &enable, 1);
}

void BleImuManager::request_fast_conn_params(const BleDevice &dev)
{
    // Use hcitool to issue an LE Connection Update. We first need to discover
    // the HCI handle corresponding to this peer address.
    FILE *fp = popen("hcitool con", "r");
    if (!fp) {
        WARN("[%s] hcitool con failed (popen)", dev.name.c_str());
        return;
    }

    char line[256];
    int handle = -1;
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, " LE ") && strstr(line, dev.addr.c_str())) {
            const char *p = strstr(line, "handle");
            if (p) {
                unsigned int h = 0;
                if (sscanf(p, "handle %u", &h) == 1) {
                    handle = (int)h;
                    break;
                }
            }
        }
    }
    pclose(fp);

    if (handle < 0) {
        WARN("[%s] Could not find HCI handle for %s",
             dev.name.c_str(), dev.addr.c_str());
        return;
    }

    char cmd[128];
    // 6,6 -> 7.5ms interval; latency=0; timeout=100*10ms=1000ms
    snprintf(cmd, sizeof(cmd),
             "hcitool lecup %u 6 6 0 100 >/dev/null 2>&1",
             (unsigned int)handle);
    int ret = system(cmd);
    if (ret != 0) {
        WARN("[%s] hcitool lecup(handle=%u) failed, ret=%d",
             dev.name.c_str(), (unsigned int)handle, ret);
    } else {
        LOG("[%s] Requested fast LE params via hcitool lecup(handle=%u, interval=7.5ms)",
            dev.name.c_str(), (unsigned int)handle);
    }
}

void BleImuManager::send_record_command(RecordCommand cmd)
{
    if (!running_.load())
        return;

    uint8_t opcode = 0;
    switch (cmd) {
    case RecordCommand::START:
        opcode = kCmdRecordStart;
        break;
    case RecordCommand::STOP:
        opcode = kCmdRecordStop;
        break;
    default:
        return;
    }

    uint8_t buf[5];
    buf[0] = opcode;
    pack_u32_le(buf + 1, (uint32_t)(monotonic_ms() & 0xffffffffu));

    std::shared_ptr<BleDevice> target;
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        for (auto &kv : devs_) {
            auto &dev = kv.second;
            if (dev && dev->state == DevState::STREAMING &&
                dev->role == BleHandRole::WAIST) {
                target = dev;
                break;
            }
        }
    }

    if (!target) {
        WARN("Record command %s ignored: no STREAMING waist device",
             cmd == RecordCommand::START ? "START" : "STOP");
        return;
    }

    if (!write_char(target->chars.timesync_path, buf, sizeof(buf))) {
        WARN("Failed to send record command %s to %s",
             cmd == RecordCommand::START ? "START" : "STOP",
             target->name.c_str());
    } else {
        LOG("Sent record command %s to %s",
            cmd == RecordCommand::START ? "START" : "STOP",
            target->name.c_str());
    }
}

bool BleImuManager::send_ping(BleDevice &dev, int64_t t1_ms)
{
    uint8_t buf[5];
    buf[0] = kCmdPing;
    pack_u32_le(buf + 1, (uint32_t)(t1_ms & 0xffffffffu));
    return write_char(dev.chars.timesync_path, buf, sizeof(buf));
}

bool BleImuManager::wait_pong(BleDevice &dev, int64_t t1_ms,
                              uint32_t &out_t2_ms, int64_t &out_t4_ms,
                              unsigned timeout_ms)
{
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    std::unique_lock<std::mutex> lk(dev.pong_waiter.mu);
    bool ok = dev.pong_waiter.cv.wait_until(lk, deadline, [&]() {
        return dev.pong_waiter.got_pong;
    });
    if (!ok)
        return false;

    out_t4_ms = monotonic_ms();
    out_t2_ms = dev.pong_waiter.esp_ts;
    const uint32_t t1_32 = (uint32_t)(t1_ms & 0xffffffffu);
    if (dev.pong_waiter.rk_echo != t1_32) {
        WARN("[%s] Ping echo mismatch: expected %u got %u",
             dev.name.c_str(), t1_32, dev.pong_waiter.rk_echo);
        return false;
    }
    return true;
}

bool BleImuManager::run_sync_round(BleDevice &dev, int32_t &out_offset)
{
    struct Sample {
        int64_t rtt_ms = 0;
        int32_t offset_ms = 0;
    };

    constexpr int kPingCount = 7;
    constexpr int kPingIntervalMs = 50;
    constexpr int kMaxRttMs = 100;
    constexpr int kMinSamples = 5;
    constexpr int kBestCount = 3;

    std::vector<Sample> samples;
    LOG("[%s] Starting BLE sync round", dev.name.c_str());

    for (int i = 0; i < kPingCount; ++i) {
        const int64_t t1 = monotonic_ms();
        {
            std::lock_guard<std::mutex> lk(dev.pong_waiter.mu);
            dev.pong_waiter.got_pong = false;
        }
        if (!send_ping(dev, t1)) {
            WARN("[%s] Ping %d send failed", dev.name.c_str(), i);
            std::this_thread::sleep_for(std::chrono::milliseconds(kPingIntervalMs));
            continue;
        }

        uint32_t t2 = 0;
        int64_t t4 = 0;
        if (!wait_pong(dev, t1, t2, t4, 300)) {
            WARN("[%s] Ping %d timeout", dev.name.c_str(), i);
            std::this_thread::sleep_for(std::chrono::milliseconds(kPingIntervalMs));
            continue;
        }

        const int64_t rtt = t4 - t1;
        if (rtt > kMaxRttMs) {
            WARN("[%s] Ping %d RTT=%lldms discarded", dev.name.c_str(), i, (long long)rtt);
            std::this_thread::sleep_for(std::chrono::milliseconds(kPingIntervalMs));
            continue;
        }

        const int64_t mid = t1 + rtt / 2;
        const int32_t offset = (int32_t)(mid - (int64_t)t2);
        samples.push_back({rtt, offset});
        std::this_thread::sleep_for(std::chrono::milliseconds(kPingIntervalMs));
    }

    if ((int)samples.size() < kMinSamples) {
        WARN("[%s] Too few sync samples: %zu", dev.name.c_str(), samples.size());
        return false;
    }

    std::sort(samples.begin(), samples.end(),
              [](const Sample &a, const Sample &b) { return a.rtt_ms < b.rtt_ms; });

    std::vector<int32_t> offsets;
    for (int i = 0; i < std::min(kBestCount, (int)samples.size()); ++i)
        offsets.push_back(samples[i].offset_ms);
    std::sort(offsets.begin(), offsets.end());
    out_offset = offsets[offsets.size() / 2];

    LOG("[%s] Sync complete: offset=%dms best_rtt=%lldms",
        dev.name.c_str(), out_offset, (long long)samples.front().rtt_ms);
    return true;
}

void BleImuManager::dispatch_timesync(const std::string &addr, GVariant *params)
{
    std::shared_ptr<BleDevice> dev;
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        auto it = devs_.find(addr);
        if (it != devs_.end())
            dev = it->second;
    }
    if (dev)
        on_timesync_notify(*dev, params);
}

void BleImuManager::dispatch_imu(const std::string &addr, GVariant *params)
{
    std::shared_ptr<BleDevice> dev;
    {
        std::lock_guard<std::mutex> lk(devs_mu_);
        auto it = devs_.find(addr);
        if (it != devs_.end())
            dev = it->second;
    }
    if (dev)
        on_imu_notify(*dev, params);
}

void BleImuManager::on_timesync_notify(BleDevice &dev, GVariant *params)
{
    GVariant *changed = nullptr;
    const gchar *iface = nullptr;
    g_variant_get(params, "(&s@a{sv}@as)", &iface, &changed, nullptr);
    if (!changed)
        return;

    GVariant *val = g_variant_lookup_value(changed, "Value", G_VARIANT_TYPE("ay"));
    if (!val) {
        g_variant_unref(changed);
        return;
    }

    gsize n = 0;
    const uint8_t *buf = (const uint8_t *)g_variant_get_fixed_array(val, &n, 1);
    if (n >= 9 && buf[0] == kRespPong) {
        std::lock_guard<std::mutex> lk(dev.pong_waiter.mu);
        dev.pong_waiter.esp_ts = unpack_u32_le(buf + 1);
        dev.pong_waiter.rk_echo = unpack_u32_le(buf + 5);
        dev.pong_waiter.got_pong = true;
        dev.pong_waiter.cv.notify_one();
    }

    g_variant_unref(val);
    g_variant_unref(changed);
}

void BleImuManager::on_imu_notify(BleDevice &dev, GVariant *params)
{
    GVariant *changed = nullptr;
    const gchar *iface = nullptr;
    g_variant_get(params, "(&s@a{sv}@as)", &iface, &changed, nullptr);
    if (!changed)
        return;

    GVariant *val = g_variant_lookup_value(changed, "Value", G_VARIANT_TYPE("ay"));
    if (!val) {
        g_variant_unref(changed);
        return;
    }

    gsize n = 0;
    const uint8_t *buf = (const uint8_t *)g_variant_get_fixed_array(val, &n, 1);
    if (n != 32) {
        WARN("[%s] Unexpected IMU packet length %zu", dev.name.c_str(), n);
        g_variant_unref(val);
        g_variant_unref(changed);
        return;
    }

    dev.last_imu_ms = monotonic_ms();

    ImuSample sample{};
    bool became_ready = false;
    int serial_index = -1;
    ZmqStreamer *streamer = nullptr;

    {
        std::lock_guard<std::mutex> lk(outputs_mu_);
        OutputChannel *out = find_output(dev.role);
        if (out) {
            sample.timestamp_ns = expand_timestamp_ms(*out, unpack_u32_le(buf + 28)) * 1000000ULL;
            sample.accel[0] = unpack_float_le(buf + 0);
            sample.accel[1] = unpack_float_le(buf + 4);
            sample.accel[2] = unpack_float_le(buf + 8);
            sample.gyro[0] = unpack_float_le(buf + 12) * kDegToRad;
            sample.gyro[1] = unpack_float_le(buf + 16) * kDegToRad;
            sample.gyro[2] = unpack_float_le(buf + 20) * kDegToRad;
            became_ready = !out->ready;
            out->ready = true;
            out->alarm_active = false;
            serial_index = out->cfg.serial_index;
            streamer = out->streamer.get();
        }
    }

    if (became_ready)
        update_connection_audio();

    if (active_mode_.load() && streamer) {
        streamer->send((const uint8_t *)&sample, sizeof(sample));
        if (recorder_.is_recording())
            recorder_.push_serial_imu(serial_index, sample);
    }

    g_variant_unref(val);
    g_variant_unref(changed);
}

bool BleImuManager::write_char(const std::string &path,
                               const uint8_t *data, gsize len,
                               bool with_response)
{
    GVariantBuilder opts;
    g_variant_builder_init(&opts, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(&opts, "{sv}", "type",
        g_variant_new_string(with_response ? "request" : "command"));

    GError *err = nullptr;
    GVariant *res = g_dbus_connection_call_sync(
        dbus_, "org.bluez", path.c_str(),
        "org.bluez.GattCharacteristic1", "WriteValue",
        g_variant_new("(@ay@a{sv})", bytes_variant(data, len),
                      g_variant_builder_end(&opts)),
        nullptr,
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (err) {
        WARN("WriteValue(%s): %s", path.c_str(), err->message);
        g_error_free(err);
        return false;
    }
    if (res) g_variant_unref(res);
    return true;
}

bool BleImuManager::enable_notify(const std::string &path)
{
    GError *err = nullptr;
    GVariant *res = g_dbus_connection_call_sync(
        dbus_, "org.bluez", path.c_str(),
        "org.bluez.GattCharacteristic1", "StartNotify",
        nullptr, nullptr,
        G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
    if (err) {
        WARN("StartNotify(%s): %s", path.c_str(), err->message);
        g_error_free(err);
        return false;
    }
    if (res) g_variant_unref(res);
    return true;
}

void BleImuManager::watchdog_loop()
{
    int64_t last_enum_ms = 0;

    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const int64_t now_ms = monotonic_ms();

        // 若在启动阶段 BlueZ 尚未就绪导致 StartDiscovery 失败，scan_active_
        // 会一直是 false，从而无法发现任何 BLE 设备。这里在 watchdog 中做
        // 兜底重试：只要扫描未开启，就周期性尝试调用 start_scan()，一旦
        // BlueZ 准备好，StartDiscovery 成功后 scan_active_ 会被置为 true。
        if (!scan_active_) {
            start_scan();
        }

        // 若启动早于 BlueZ / 设备广播，初始的 GetManagedObjects 可能为空，
        // devs_ 中长期看不到任何 ESP32 设备。实际使用中长按按键触发
        // reset_pairing_and_rescan() + enumerate_known_devices() 能立即恢复，
        // 说明周期性补一次 GetManagedObjects 即可修复。
        //
        // 这里做一个轻量级兜底：当 devs_ 为空时，每隔 5 秒调用一次
        // enumerate_known_devices()，直到至少发现一台设备为止。
        {
            bool devs_empty = false;
            {
                std::lock_guard<std::mutex> lk(devs_mu_);
                devs_empty = devs_.empty();
            }
            if (devs_empty && (now_ms - last_enum_ms >= 5000)) {
                enumerate_known_devices();
                last_enum_ms = now_ms;
            }
        }

        std::vector<std::shared_ptr<BleDevice>> devices;
        {
            std::lock_guard<std::mutex> lk(devs_mu_);
            for (auto &kv : devs_)
                devices.push_back(kv.second);
        }

        for (const auto &dev : devices) {
            if (!dev || dev->state != DevState::STREAMING)
                continue;

            // WAIST device does not send IMU data; skip timeout check.
            if (dev->role == BleHandRole::WAIST)
                continue;

            const int64_t age_ms = now_ms - dev->last_imu_ms;
            if (age_ms < cfg_.disconnect_timeout_ms)
                continue;

            ERR("[%s] IMU silent for %lldms, reconnecting",
                dev->name.c_str(), (long long)age_ms);
            dev->state = DevState::RECONNECTING;
            set_output_ready(dev->role, false, true);
            unsubscribe_device(*dev);
            disconnect_device(*dev);
            schedule_retry(dev, 1200);
        }

        if (!active_mode_.load())
            continue;

        // 当前至少有多少个输出是 ready 状态（用于区分“部分掉线”的告警场景）
        const int ready_count = ready_hand_count();

        std::vector<AudioPlayer::Sound> alarms;
        {
            std::lock_guard<std::mutex> lk(outputs_mu_);
            for (auto &out : outputs_) {
                if (out.ready || !out.alarm_active)
                    continue;
                if (now_ms - out.last_alarm_ms >= cfg_.disconnect_alarm_interval_ms) {
                    out.last_alarm_ms = now_ms;
                    alarms.push_back(AudioPlayer::Sound::IMU_DISCONNECT);
                }
            }
        }
        // 如果所有输出都已经不 ready（完全断开），只保留连接数量的节奏提示，
        // 不再周期性播放 IMU_DISCONNECT 告警，避免两只手都关机时喇叭“乱响”。
        if (ready_count > 0) {
            for (auto sound : alarms) {
                if (audio_)
                    audio_->play(sound);
            }
        }
    }
}

void BleImuManager::resync_loop()
{
    // Stagger re-syncs: one device at a time, with a gap between each device
    // and between rounds, to avoid BLE controller congestion (ATT 0x11).
    constexpr int kRoundIntervalSec = 30;   // interval between full rounds
    constexpr int kDeviceGapMs = 3000;      // gap between syncing two devices

    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(kRoundIntervalSec));
        if (!running_.load())
            break;

        std::vector<std::shared_ptr<BleDevice>> devices;
        {
            std::lock_guard<std::mutex> lk(devs_mu_);
            for (auto &kv : devs_) {
                if (kv.second && kv.second->state == DevState::STREAMING)
                    devices.push_back(kv.second);
            }
        }

        for (const auto &dev : devices) {
            if (!running_.load())
                break;
            if (!dev || dev->state != DevState::STREAMING)
                continue;

            int32_t offset = 0;
            if (!run_sync_round(*dev, offset)) {
                WARN("[%s] Periodic re-sync failed", dev->name.c_str());
            } else if (!do_set_offset(*dev, offset)) {
                WARN("[%s] Periodic SetOffset failed", dev->name.c_str());
            } else {
                LOG("[%s] Re-sync offset=%dms", dev->name.c_str(), offset);
            }

            // Wait before syncing the next device
            std::this_thread::sleep_for(std::chrono::milliseconds(kDeviceGapMs));
        }
    }
}

void BleImuManager::update_pairing_audio()
{
    if (!audio_)
        return;

    if (!pairing_mode_.load()) {
        // pairing mode 已退出，停止 ticker
        audio_->stop_status_ticker();
        return;
    }

    const int ready = ready_hand_count();
    const int expected = expected_hand_count();
    int missing = expected - ready;
    if (missing < 0) missing = 0;

    // missing 数量决定提示音：
    //  >=3 → 三声滴滴滴 (BLE_PAIR_BOTH)
    //    2 → 两声滴滴   (BLE_PAIR_ONE)
    //    1 → 一声滴     (BLE_CONN_ONE)
    //    0 → 全部连上，长响一次 (BLE_CONN_BOTH)
    int desired_state = missing;
    if (desired_state > 3) desired_state = 3;

    const int previous = pairing_audio_state_.load();
    if (desired_state == previous)
        return;
    pairing_audio_state_ = desired_state;

    if (desired_state >= 3) {
        audio_->start_status_ticker(AudioPlayer::Sound::BLE_PAIR_BOTH,
                                    cfg_.pairing_status_interval_ms);
    } else if (desired_state == 2) {
        audio_->start_status_ticker(AudioPlayer::Sound::BLE_PAIR_ONE,
                                    cfg_.pairing_status_interval_ms);
    } else if (desired_state == 1) {
        audio_->start_status_ticker(AudioPlayer::Sound::BLE_CONN_ONE,
                                    cfg_.pairing_status_interval_ms);
    } else {
        // 全部连上
        audio_->stop_status_ticker();
        audio_->play(AudioPlayer::Sound::BLE_CONN_BOTH);
    }
}

void BleImuManager::update_connection_audio()
{
    if (!audio_)
        return;

    // 正常连接模式下，expected 基于已保存配对的设备数量（而非 output 槽位总数）。
    // 这样只配对了 1 个设备时，连上那 1 个就不再告警。
    const int ready = ready_hand_count();
    const int expected = saved_device_count();

    // 没有保存任何配对 → 静音（等用户长按进入配对模式）
    if (expected <= 0) {
        audio_->stop_status_ticker();
        return;
    }

    int beep_count = expected - ready;
    if (beep_count < 0)
        beep_count = 0;

    int desired_state = -1;
    if (beep_count == 0) {
        desired_state = 2;  // FULL: BLE_CONN_BOTH (one-shot long beep)
    } else if (beep_count == 1) {
        desired_state = 1;  // SINGLE: BLE_CONN_ONE
    } else {
        desired_state = 0;  // DOUBLE: BLE_CONN_ZERO
    }

    const int previous = connection_beep_state_.load();
    if (desired_state == previous)
        return;
    connection_beep_state_ = desired_state;

    if (desired_state == 0) {
        audio_->start_status_ticker(AudioPlayer::Sound::BLE_CONN_ZERO, 2000);
    } else if (desired_state == 1) {
        audio_->start_status_ticker(AudioPlayer::Sound::BLE_CONN_ONE, 2000);
    } else {
        audio_->stop_status_ticker();
        audio_->play(AudioPlayer::Sound::BLE_CONN_BOTH);

        // 当所有已保存设备都 ready 时，如果尚未保存配对信息，则自动持久化。
        // 配对模式下不自动保存（由 finalize_pairing 显式触发）。
        if (!pairing_mode_.load() &&
            ready >= expected && expected > 0 &&
            !pairing_persisted_) {
            if (persist_current_pairing()) {
                pairing_persisted_ = true;
            }
        }
    }
}

void BleImuManager::try_connect_next()
{
    if (!running_.load())
        return;

    std::shared_ptr<BleDevice> candidate;

    {
        std::lock_guard<std::mutex> lk(devs_mu_);

        // 如果当前已经有设备在执行 Connect / Discover / Sync，则先不再发起新的连接，
        // 等待当前流程结束，避免 BlueZ 报 "Operation already in progress"。
        for (auto &kv : devs_) {
            const auto &d = kv.second;
            if (!d) continue;
            if (d->state == DevState::CONNECTING ||
                d->state == DevState::DISCOVERING ||
                d->state == DevState::SYNCING) {
                return;
            }
        }

        // 按优先级选择下一个要连接的设备：RIGHT → LEFT → WAIST
        const BleHandRole priority[] = {BleHandRole::RIGHT, BleHandRole::LEFT, BleHandRole::WAIST};
        for (auto role : priority) {
            if (candidate) break;
            for (auto &kv : devs_) {
                const auto &d = kv.second;
                if (!d) continue;
                if ((d->state == DevState::WAIT_ADV ||
                     d->state == DevState::RECONNECTING) &&
                    d->role == role) {
                    candidate = d;
                    break;
                }
            }
        }
    }

    if (candidate) {
        launch_worker([this, candidate]() {
            connect_device(candidate);
        });
    }
}

void BleImuManager::set_output_ready(BleHandRole hand, bool ready, bool play_alarm)
{
    bool changed = false;
    bool should_alarm = false;
    {
        std::lock_guard<std::mutex> lk(outputs_mu_);
        OutputChannel *out = find_output(hand);
        if (!out)
            return;
        changed = (out->ready != ready);
        out->ready = ready;
        if (ready) {
            out->alarm_active = false;
        } else if (play_alarm) {
            const int64_t now_ms = monotonic_ms();
            should_alarm = !out->alarm_active ||
                           now_ms - out->last_alarm_ms >= cfg_.disconnect_alarm_interval_ms;
            out->alarm_active = true;
            out->last_alarm_ms = now_ms;
        }
    }

    if (changed)
        update_connection_audio();
    if (should_alarm && active_mode_.load() && audio_)
        audio_->play(AudioPlayer::Sound::IMU_DISCONNECT);
}

BleImuManager::OutputChannel *BleImuManager::find_output(BleHandRole hand)
{
    for (auto &out : outputs_) {
        if (out.cfg.hand == hand)
            return &out;
    }
    return nullptr;
}

const BleImuManager::OutputChannel *BleImuManager::find_output(BleHandRole hand) const
{
    for (const auto &out : outputs_) {
        if (out.cfg.hand == hand)
            return &out;
    }
    return nullptr;
}

uint64_t BleImuManager::expand_timestamp_ms(OutputChannel &out, uint32_t timestamp_ms32)
{
    if (!out.has_ts) {
        out.has_ts = true;
    } else if (timestamp_ms32 < out.last_ts32 &&
               (out.last_ts32 - timestamp_ms32) > 1000) {
        out.ts_wraps++;
    }
    out.last_ts32 = timestamp_ms32;
    return (out.ts_wraps << 32) | timestamp_ms32;
}

void BleImuManager::launch_worker(std::function<void()> fn)
{
    std::lock_guard<std::mutex> lk(workers_mu_);
    worker_threads_.emplace_back(std::move(fn));
}

bool BleImuManager::has_saved_pairing() const
{
    return saved_device_count() > 0;
}

int BleImuManager::saved_device_count() const
{
    int count = 0;
    if (!cfg_.paired_left_addr.empty()) count++;
    if (!cfg_.paired_right_addr.empty()) count++;
    if (!cfg_.paired_waist_addr.empty()) count++;
    return count;
}

std::string BleImuManager::configured_addr_for(BleHandRole hand) const
{
    switch (hand) {
    case BleHandRole::LEFT:
        return cfg_.paired_left_addr;
    case BleHandRole::RIGHT:
        return cfg_.paired_right_addr;
    case BleHandRole::WAIST:
        return cfg_.paired_waist_addr;
    default:
        return "";
    }
}

BleHandRole BleImuManager::role_from_configured_addr(const std::string &addr) const
{
    if (!cfg_.paired_left_addr.empty() && addr == cfg_.paired_left_addr)
        return BleHandRole::LEFT;
    if (!cfg_.paired_right_addr.empty() && addr == cfg_.paired_right_addr)
        return BleHandRole::RIGHT;
    if (!cfg_.paired_waist_addr.empty() && addr == cfg_.paired_waist_addr)
        return BleHandRole::WAIST;
    return BleHandRole::UNKNOWN;
}

bool BleImuManager::collect_paired_addresses(std::string &left_addr, std::string &right_addr,
                                              std::string &waist_addr) const
{
    std::lock_guard<std::mutex> lk(devs_mu_);
    for (const auto &kv : devs_) {
        const auto &dev = kv.second;
        if (!dev || dev->state != DevState::STREAMING)
            continue;
        if (dev->role == BleHandRole::LEFT)
            left_addr = dev->addr;
        else if (dev->role == BleHandRole::RIGHT)
            right_addr = dev->addr;
        else if (dev->role == BleHandRole::WAIST)
            waist_addr = dev->addr;
    }
    // 只要至少有一个设备 STREAMING 就认为收集成功
    return !left_addr.empty() || !right_addr.empty() || !waist_addr.empty();
}

bool BleImuManager::persist_current_pairing()
{
    std::string left_addr;
    std::string right_addr;
    std::string waist_addr;
    if (!collect_paired_addresses(left_addr, right_addr, waist_addr)) {
        ERR("Cannot persist BLE pairing: no devices are streaming yet");
        return false;
    }

    cfg_.paired_left_addr = left_addr;
    cfg_.paired_right_addr = right_addr;
    cfg_.paired_waist_addr = waist_addr;

    std::string error;
    if (!persist_ble_pairing_config(config_path_, cfg_, &error)) {
        ERR("Failed to persist BLE pairing to %s: %s",
            config_path_.c_str(), error.empty() ? "unknown error" : error.c_str());
        return false;
    }

    LOG("Saved BLE pairing: left=%s right=%s waist=%s",
        cfg_.paired_left_addr.c_str(), cfg_.paired_right_addr.c_str(),
        cfg_.paired_waist_addr.c_str());
    return true;
}

void BleImuManager::reset_pairing_and_rescan()
{
    // Reset mode flags so that enter_pairing_mode() can re-enter cleanly.
    pairing_mode_ = false;
    active_mode_ = true;
    pairing_audio_state_ = -2;

    // Clear persisted MAC addresses for all 3 device roles
    cfg_.paired_left_addr.clear();
    cfg_.paired_right_addr.clear();
    cfg_.paired_waist_addr.clear();
    pairing_persisted_ = false;

    std::string error;
    if (!persist_ble_pairing_config(config_path_, cfg_, &error)) {
        ERR("Failed to clear BLE pairing in %s: %s",
            config_path_.c_str(),
            error.empty() ? "unknown error" : error.c_str());
    } else {
        LOG("Cleared BLE pairing in %s", config_path_.c_str());
    }

    // Disconnect and reset all known devices
    {
        std::vector<std::shared_ptr<BleDevice>> devices;
        {
            std::lock_guard<std::mutex> lk(devs_mu_);
            for (auto &kv : devs_)
                devices.push_back(kv.second);
            // 清空本地表，后续通过 enumerate_known_devices 重新发现
            devs_.clear();
        }
        for (auto &dev : devices) {
            if (!dev) continue;
            unsubscribe_device(*dev);
            disconnect_device(*dev);
        }
    }

    // Reset output channels
    {
        std::lock_guard<std::mutex> lk(outputs_mu_);
        for (auto &out : outputs_) {
            out.ready = false;
            out.alarm_active = false;
            out.last_alarm_ms = 0;
            out.has_ts = false;
            out.last_ts32 = 0;
            out.ts_wraps = 0;
        }
    }

    if (audio_) {
        audio_->stop_status_ticker();
    }
    connection_beep_state_ = -1;

    // Stop scan so enter_pairing_mode can restart it cleanly
    stop_scan();

    LOG("BLE pairing reset: cleared stored MACs, ready for enter_pairing_mode()");
}
