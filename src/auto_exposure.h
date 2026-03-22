#pragma once

#include <cstdint>
#include <atomic>

class AutoExposure {
public:
    AutoExposure(int target_brightness = 128, int min_exp = 100, int max_exp = 30000);

    // 计算 NV12 图像的平均亮度（采样方式，低CPU占用）
    float calc_brightness(const uint8_t *nv12_data, int width, int height, int stride);

    // 根据当前亮度调整曝光参数
    void adjust_exposure(float current_brightness, int &exposure_us, int &gain);

    void set_target_brightness(int target) { target_brightness_ = target; }
    void set_exposure_range(int min_exp, int max_exp) {
        min_exposure_ = min_exp;
        max_exposure_ = max_exp;
    }

private:
    int target_brightness_;
    int min_exposure_;
    int max_exposure_;
    float last_error_ = 0.0f;
};
