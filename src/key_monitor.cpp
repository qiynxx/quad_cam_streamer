#include "key_monitor.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

KeyMonitor::KeyMonitor(int key_code, const std::string &input_device, Callback cb)
    : key_code_(key_code), input_device_(input_device), callback_(std::move(cb))
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

    fprintf(stderr, "[keymon] Monitoring key %d on %s\n", key_code_, dev_path.c_str());

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    while (running.load()) {
        int ret = poll(&pfd, 1, 500);  // 500ms timeout
        if (ret <= 0) continue;

        struct input_event ev;
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n != sizeof(ev)) continue;

        if (ev.type == EV_KEY && ev.code == key_code_ && ev.value == 1) {
            fprintf(stderr, "[keymon] Key %d pressed, toggling recording\n", key_code_);
            if (callback_) callback_();
        }
    }

    close(fd);
    fprintf(stderr, "[keymon] Key monitor stopped\n");
}
