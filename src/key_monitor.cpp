#include "key_monitor.h"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

KeyMonitor::KeyMonitor(int key_code, const std::string &input_device,
                       Callback short_press_cb, Callback long_press_cb,
                       int long_press_ms, int debounce_ms)
    : key_code_(key_code),
      input_device_(input_device),
      short_press_cb_(std::move(short_press_cb)),
      long_press_cb_(std::move(long_press_cb)),
      long_press_ms_(long_press_ms),
      debounce_ms_(debounce_ms)
{
}

KeyMonitor::~KeyMonitor() = default;

std::string KeyMonitor::find_adc_keys_device() const
{
    DIR *dir = opendir("/dev/input");
    if (!dir) return "";

    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        std::string path = std::string("/dev/input/") + ent->d_name;
        int fd = open(path.c_str(), O_RDONLY);
        if (fd < 0) continue;

        char name[256] = {};
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
            if (strstr(name, "adc-keys") != nullptr) {
                close(fd);
                closedir(dir);
                fprintf(stderr, "[keymon] Found adc-keys at %s\n", path.c_str());
                return path;
            }
        }
        close(fd);
    }
    closedir(dir);
    return "";
}

void KeyMonitor::run(const std::atomic<bool> &running)
{
    std::string dev_path = input_device_;
    if (dev_path.empty())
        dev_path = find_adc_keys_device();

    if (dev_path.empty()) {
        fprintf(stderr, "[keymon] No adc-keys input device found, key monitor disabled\n");
        while (running.load())
            std::this_thread::sleep_for(std::chrono::seconds(1));
        return;
    }

    int fd = open(dev_path.c_str(), O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "[keymon] Cannot open %s: %s\n",
                dev_path.c_str(), strerror(errno));
        return;
    }

    fprintf(stderr, "[keymon] Monitoring key %d on %s (long_press=%dms)\n",
            key_code_, dev_path.c_str(), long_press_ms_);

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    auto last_action = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    bool pressed = false;
    bool long_fired = false;
    auto press_time = std::chrono::steady_clock::now();

    while (running.load()) {
        int ret = poll(&pfd, 1, 100);
        auto now = std::chrono::steady_clock::now();
        if (pressed && !long_fired) {
            auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - press_time).count();
            auto since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_action).count();
            if (held_ms >= long_press_ms_ && since_last >= debounce_ms_) {
                long_fired = true;
                last_action = now;
                fprintf(stderr, "[keymon] Key %d long pressed (%ldms)\n",
                        key_code_, held_ms);
                if (long_press_cb_) long_press_cb_();
            }
        }

        if (ret <= 0)
            continue;

        struct input_event ev;
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n != sizeof(ev)) continue;

        if (ev.type != EV_KEY || ev.code != key_code_)
            continue;

        if (ev.value == 1 && !pressed) {
            pressed = true;
            long_fired = false;
            press_time = std::chrono::steady_clock::now();
        } else if (ev.value == 0 && pressed) {
            auto release_time = std::chrono::steady_clock::now();
            auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                release_time - press_time).count();
            auto since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                release_time - last_action).count();
            if (!long_fired && since_last >= debounce_ms_) {
                fprintf(stderr, "[keymon] Key %d short pressed (%ldms)\n",
                        key_code_, held_ms);
                if (short_press_cb_) short_press_cb_();
                last_action = release_time;
            } else if (!long_fired && since_last < debounce_ms_) {
                fprintf(stderr, "[keymon] Key %d debounced (%ldms since last action)\n",
                        key_code_, since_last);
            }
            pressed = false;
            long_fired = false;
        }
    }

    close(fd);
    fprintf(stderr, "[keymon] Key monitor stopped\n");
}
