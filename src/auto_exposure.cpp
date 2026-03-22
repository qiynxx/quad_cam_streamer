#include "auto_exposure.h"
#include <algorithm>
#include <cmath>

AutoExposure::AutoExposure(int target_brightness, int min_exp, int max_exp)
    : target_brightness_(target_brightness)
    , min_exposure_(min_exp)
    , max_exposure_(max_exp)
{
}

float AutoExposure::calc_brightness(const uint8_t *nv12_data, int width, int height, int stride)
{
    // 采样策略：每16个像素采样1个，降低计算量
    const int sample_step = 16;
    uint64_t sum = 0;
    int count = 0;

    for (int y = 0; y < height; y += sample_step) {
        const uint8_t *row = nv12_data + y * stride;
        for (int x = 0; x < width; x += sample_step) {
            sum += row[x];
            count++;
        }
    }

    return count > 0 ? (float)sum / count : 0.0f;
}

void AutoExposure::adjust_exposure(float current_brightness, int &exposure_us, int &gain)
{
    float error = target_brightness_ - current_brightness;
    float error_ratio = error / target_brightness_;

    // 如果误差小于10%，不调整
    if (std::abs(error_ratio) < 0.1f) {
        return;
    }

    // 计算需要的总增益调整
    float adjust_factor = 1.0f + error_ratio * 0.3f;  // 最大30%调整
    adjust_factor = std::max(0.8f, std::min(1.2f, adjust_factor));

    // 优先调整曝光时间
    int new_exposure = (int)(exposure_us * adjust_factor);
    new_exposure = std::max(min_exposure_, std::min(max_exposure_, new_exposure));

    // 如果曝光达到上限且还需要更亮，增加增益
    if (new_exposure >= max_exposure_ && error > 0) {
        int new_gain = (int)(gain * adjust_factor);
        new_gain = std::max(1, std::min(255, new_gain));  // 增益范围 1-255
        gain = new_gain;
    }
    // 如果曝光达到下限且还需要更暗，减少增益
    else if (new_exposure <= min_exposure_ && error < 0) {
        int new_gain = (int)(gain * adjust_factor);
        new_gain = std::max(1, std::min(255, new_gain));
        gain = new_gain;
    }

    exposure_us = new_exposure;
}
