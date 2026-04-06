#pragma once

#include <cstdint>
#include <atomic>

class AutoExposure {
public:
    AutoExposure(int target_brightness = 100, int min_exp = 100, int max_exp = 30000,
                 int min_gain = 16, int max_gain = 96);

    // 计算 NV12 图像的平均亮度（采样方式，低CPU占用）
    float calc_brightness(const uint8_t *nv12_data, int width, int height, int stride);

    // 根据当前亮度调整曝光参数
    void adjust_exposure(float current_brightness, int &exposure_us, int &gain);

    void set_target_brightness(int target) { target_brightness_ = target; }
    void set_exposure_range(int min_exp, int max_exp) {
        min_exposure_ = min_exp;
        max_exposure_ = max_exp;
    }
    void set_gain_range(int min_gain, int max_gain) {
        min_gain_ = min_gain;
        max_gain_ = max_gain;
    }

private:
    int target_brightness_;
    int min_exposure_;
    int max_exposure_;
    int min_gain_;
    int max_gain_;
    float filtered_brightness_ = -1.0f;
    int under_target_count_ = 0;
    int over_target_count_ = 0;
};
