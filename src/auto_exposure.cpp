#include "auto_exposure.h"
#include <algorithm>
#include <cmath>

AutoExposure::AutoExposure(int target_brightness, int min_exp, int max_exp,
                           int min_gain, int max_gain)
    : target_brightness_(target_brightness)
    , min_exposure_(min_exp)
    , max_exposure_(max_exp)
    , min_gain_(min_gain)
    , max_gain_(max_gain)
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
    if (target_brightness_ <= 0) return;

    constexpr float brightness_alpha = 0.18f;
    constexpr float deadband = 0.14f;
    constexpr float deep_dark_ratio = 0.22f;
    constexpr float gain_enable_ratio = 0.30f;
    constexpr int brighten_confirm_cycles = 2;

    exposure_us = std::max(min_exposure_, std::min(max_exposure_, exposure_us));
    gain = std::max(min_gain_, std::min(max_gain_, gain));

    if (filtered_brightness_ < 0.0f)
        filtered_brightness_ = current_brightness;
    else
        filtered_brightness_ =
            filtered_brightness_ * (1.0f - brightness_alpha) +
            current_brightness * brightness_alpha;

    float brightness = filtered_brightness_;
    float error = target_brightness_ - brightness;
    float error_ratio = error / target_brightness_;

    if (std::abs(error_ratio) < deadband) {
        under_target_count_ = 0;
        over_target_count_ = 0;
        return;
    }

    if (error_ratio > 0.0f) {
        under_target_count_++;
        over_target_count_ = 0;
    } else {
        over_target_count_++;
        under_target_count_ = 0;
    }

    const int exposure_span = std::max(1, max_exposure_ - min_exposure_);
    const float deep_dark_threshold =
        std::max(18.0f, target_brightness_ * deep_dark_ratio);
    const float gain_enable_threshold =
        std::max(26.0f, target_brightness_ * gain_enable_ratio);
    const bool deep_dark = brightness <= deep_dark_threshold;
    const bool allow_gain_up = brightness >= gain_enable_threshold;

    if (error > 0) {
        if (under_target_count_ < brighten_confirm_cycles)
            return;

        if (exposure_us < max_exposure_) {
            float severity =
                std::min(1.0f, std::max(0.0f, (error_ratio - deadband) / 0.55f));
            int step = 160 + (int)std::lround(severity * 900.0f);
            step = std::min(step, std::max(220, exposure_span / 9));
            exposure_us = std::min(max_exposure_, exposure_us + step);
        } else if (gain < max_gain_ && !deep_dark && allow_gain_up) {
            float severity =
                std::min(1.0f, std::max(0.0f, (error_ratio - deadband) / 0.60f));
            int step = 1 + (severity >= 0.45f ? 1 : 0);
            gain = std::min(max_gain_, gain + step);
        }
    } else {
        float severity =
            std::min(1.0f, std::max(0.0f, ((-error_ratio) - deadband) / 0.60f));
        if (gain > min_gain_) {
            int step = 2 + (int)std::lround(severity * 4.0f);
            gain = std::max(min_gain_, gain - step);
        } else if (exposure_us > min_exposure_) {
            int step = 240 + (int)std::lround(severity * 1800.0f);
            step = std::min(step, std::max(320, exposure_span / 5));
            exposure_us = std::max(min_exposure_, exposure_us - step);
        }
    }
}
