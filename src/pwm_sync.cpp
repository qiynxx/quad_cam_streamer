#include "pwm_sync.h"
#include "config.h"

#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

// Static member definitions
constexpr uint32_t PwmSync::PWM1_CH_BASE[NUM_CHANNELS];

bool PwmSync::init(const HwSyncConfig &cfg)
{
    if (!cfg.enabled) return false;

    period_ns_ = 1000000000 / cfg.fps;
    duty_ns_ = 100000;  // 100us pulse width

    // Find and configure all 4 sysfs channels
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (!find_sysfs_channel(PWM1_CH_BASE[i], sysfs_ch_[i])) {
            fprintf(stderr, "[pwm_sync] Cannot find sysfs for PWM1 ch%d (0x%08x)\n",
                    i + 1, PWM1_CH_BASE[i]);
            return false;
        }
        if (!configure_sysfs(sysfs_ch_[i], period_ns_, duty_ns_))
            return false;
    }

    initialized_ = true;
    fprintf(stderr, "[pwm_sync] Initialized %d channels: period=%uns duty=%uns\n",
            NUM_CHANNELS, period_ns_, duty_ns_);
    return true;
}

bool PwmSync::find_sysfs_channel(uint32_t phys_addr, SysfsChannel &out)
{
    char addr_str[16];
    snprintf(addr_str, sizeof(addr_str), "%x", phys_addr);

    DIR *dir = opendir("/sys/class/pwm");
    if (!dir) {
        fprintf(stderr, "[pwm_sync] Cannot open /sys/class/pwm\n");
        return false;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (strncmp(ent->d_name, "pwmchip", 7) != 0) continue;

        std::string chip = std::string("/sys/class/pwm/") + ent->d_name;

        // Read device symlink to match base address
        char link_buf[512] = {};
        std::string device_path = chip + "/device";
        ssize_t len = readlink(device_path.c_str(), link_buf, sizeof(link_buf) - 1);
        if (len > 0) {
            std::string link(link_buf, len);
            if (link.find(addr_str) != std::string::npos) {
                out.chip_path = chip;
                out.pwm_path = chip + "/pwm0";  // each node has npwm=1
                closedir(dir);
                fprintf(stderr, "[pwm_sync] Found %s for address 0x%08x\n",
                        chip.c_str(), phys_addr);
                return true;
            }
        }
    }
    closedir(dir);
    return false;
}

bool PwmSync::configure_sysfs(SysfsChannel &ch, uint32_t period_ns, uint32_t duty_ns)
{
    // Export channel if not already
    struct stat st;
    if (stat(ch.pwm_path.c_str(), &st) != 0) {
        sysfs_write(ch.chip_path + "/export", "0");
        usleep(100000);
        if (stat(ch.pwm_path.c_str(), &st) != 0) {
            fprintf(stderr, "[pwm_sync] Failed to export %s\n", ch.chip_path.c_str());
            return false;
        }
        ch.exported = true;
    }

    // Configure period and duty (do NOT enable yet)
    sysfs_write(ch.pwm_path + "/polarity", "normal");
    sysfs_write(ch.pwm_path + "/period", std::to_string(period_ns));
    sysfs_write(ch.pwm_path + "/duty_cycle", std::to_string(duty_ns));

    fprintf(stderr, "[pwm_sync] Configured %s: period=%u duty=%u\n",
            ch.pwm_path.c_str(), period_ns, duty_ns);
    return true;
}

void PwmSync::start()
{
    if (!initialized_ || enabled_) return;

    // Enable all channels via sysfs.
    // The kernel PWM driver handles pinctrl muxing (GPIO → PWM function)
    // and clock gating (clk_prepare_enable) on each enable call.
    // Sequential enable has ~10us offset — 0.03% of 33.3ms, negligible.
    for (int i = 0; i < NUM_CHANNELS; i++) {
        sysfs_write(sysfs_ch_[i].pwm_path + "/enable", "1");

        // Verify enable
        std::string val = sysfs_read(sysfs_ch_[i].pwm_path + "/enable");
        fprintf(stderr, "[pwm_sync] Enabled %s → %s\n",
                sysfs_ch_[i].pwm_path.c_str(), val.c_str());
    }

    enabled_ = true;
    fprintf(stderr, "[pwm_sync] All %d PWM FSIN channels started\n", NUM_CHANNELS);
}

void PwmSync::stop()
{
    if (!initialized_ || !enabled_) return;

    // Disable all channels via sysfs
    for (int i = 0; i < NUM_CHANNELS; i++)
        sysfs_write(sysfs_ch_[i].pwm_path + "/enable", "0");

    // Unexport all channels
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (sysfs_ch_[i].exported)
            sysfs_write(sysfs_ch_[i].chip_path + "/unexport", "0");
    }

    enabled_ = false;
    fprintf(stderr, "[pwm_sync] All PWM FSIN channels stopped\n");
}

void PwmSync::sysfs_write(const std::string &path, const std::string &value)
{
    std::ofstream f(path);
    if (f.is_open()) {
        f << value;
    } else {
        fprintf(stderr, "[pwm_sync] Warning: cannot write '%s' to %s\n",
                value.c_str(), path.c_str());
    }
}

std::string PwmSync::sysfs_read(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open()) return "";
    std::string val;
    std::getline(f, val);
    while (!val.empty() && (val.back() == '\n' || val.back() == '\r' || val.back() == ' '))
        val.pop_back();
    return val;
}
