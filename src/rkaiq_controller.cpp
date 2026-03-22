#include "rkaiq_controller.h"
#include <cstdio>
#include <rk_aiq_user_api2_sysctl.h>
#include <common/rk_aiq_comm.h>

RkaiqController::RkaiqController() {}

RkaiqController::~RkaiqController()
{
    stop();
}

bool RkaiqController::init(const std::string &sensor_entity, const std::string &iq_dir, int width, int height)
{
    rk_aiq_working_mode_t mode = RK_AIQ_WORKING_MODE_NORMAL;
    ctx_ = rk_aiq_uapi2_sysctl_init(sensor_entity.c_str(), iq_dir.c_str(), nullptr, nullptr);
    if (!ctx_) {
        fprintf(stderr, "[rkaiq] Failed to init for %s\n", sensor_entity.c_str());
        return false;
    }

    if (rk_aiq_uapi2_sysctl_prepare(ctx_, width, height, mode) != 0) {
        fprintf(stderr, "[rkaiq] Failed to prepare\n");
        rk_aiq_uapi2_sysctl_deinit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    fprintf(stderr, "[rkaiq] Initialized for %s with IQ dir %s, resolution %dx%d\n",
            sensor_entity.c_str(), iq_dir.c_str(), width, height);
    return true;
}

bool RkaiqController::start()
{
    if (!ctx_ || running_) return false;

    if (rk_aiq_uapi2_sysctl_start(ctx_) != 0) {
        fprintf(stderr, "[rkaiq] Failed to start\n");
        return false;
    }

    fprintf(stderr, "[rkaiq] Started (modules disabled in init)\n");

    running_ = true;
    return true;
}

void RkaiqController::stop()
{
    if (ctx_ && running_) {
        rk_aiq_uapi2_sysctl_stop(ctx_, false);
        running_ = false;
    }
    if (ctx_) {
        rk_aiq_uapi2_sysctl_deinit(ctx_);
        ctx_ = nullptr;
    }
}
