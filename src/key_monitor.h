#pragma once

#include <atomic>
#include <functional>
#include <string>

class KeyMonitor {
public:
    using Callback = std::function<void()>;

    KeyMonitor(int key_code, const std::string &input_device, Callback cb);
    ~KeyMonitor();

    // Run the monitor loop (blocking, respects g_running)
    void run(const std::atomic<bool> &running);

private:
    std::string find_adc_keys_device() const;

    int key_code_;
    std::string input_device_;
    Callback callback_;
};
