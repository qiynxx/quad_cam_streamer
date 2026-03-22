#pragma once

#include <string>

struct rk_aiq_sys_ctx_s;
typedef struct rk_aiq_sys_ctx_s rk_aiq_sys_ctx_t;

class RkaiqController {
public:
    RkaiqController();
    ~RkaiqController();

    bool init(const std::string &sensor_entity, const std::string &iq_dir, int width, int height);
    bool start();
    void stop();

private:
    rk_aiq_sys_ctx_t *ctx_ = nullptr;
    bool running_ = false;
};
