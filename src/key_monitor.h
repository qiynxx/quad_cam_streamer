#pragma once

#include <atomic>
#include <functional>
#include <string>

class KeyMonitor {
public:
    using Callback = std::function<void()>;

    KeyMonitor(int key_code, const std::string &input_device,
               Callback short_press_cb, Callback long_press_cb = {},
               int long_press_ms = 1200, int debounce_ms = 500);
    ~KeyMonitor();

    // Run the monitor loop (blocking, respects g_running)
    void run(const std::atomic<bool> &running);

private:
    std::string find_adc_keys_device() const;

    int key_code_;
    std::string input_device_;
    Callback short_press_cb_;
    Callback long_press_cb_;
    int long_press_ms_ = 1200;
    int debounce_ms_ = 500;
};
