#pragma once

#include <cstdint>
#include <string>

struct HwSyncConfig;

// Controls 4 PWM FSIN channels on the PWM1 controller via sysfs.
// Sequential sysfs enable has ~10us offset between channels,
// which is 0.03% of the 33.3ms period at 30Hz — negligible.
class PwmSync {
public:
    static constexpr int NUM_CHANNELS = 4;

    bool init(const HwSyncConfig &cfg);
    void start();
    void stop();

private:
    // PWM1 channel physical base addresses (ch1-ch4)
    static constexpr uint32_t PWM1_CH_BASE[NUM_CHANNELS] = {
        0x2add1000,  // pwm1_6ch_1 → IMX334 #0
        0x2add2000,  // pwm1_6ch_2 → IMX334 #1
        0x2add3000,  // pwm1_6ch_3 → OV9281 #0
        0x2add4000,  // pwm1_6ch_4 → OV9281 #1
    };

    struct SysfsChannel {
        std::string chip_path;
        std::string pwm_path;  // chip_path + "/pwm0"
        bool exported = false;
    };

    bool find_sysfs_channel(uint32_t phys_addr, SysfsChannel &out);
    bool configure_sysfs(SysfsChannel &ch, uint32_t period_ns, uint32_t duty_ns);
    void sysfs_write(const std::string &path, const std::string &value);
    std::string sysfs_read(const std::string &path);

    SysfsChannel sysfs_ch_[NUM_CHANNELS];

    uint32_t period_ns_ = 33333333;
    uint32_t duty_ns_ = 100000;
    bool enabled_ = false;
    bool initialized_ = false;
};
