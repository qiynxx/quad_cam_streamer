#pragma once

#include <cstdint>
#include <string>

struct ImuConfig;

struct ImuSample {
    uint64_t timestamp_ns;
    double accel[3];  // m/s², body frame
    double gyro[3];   // rad/s, body frame
};

class ImuReader {
public:
    bool init(const ImuConfig &config);
    void close();
    bool read(ImuSample &sample);

private:
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t *buf, int len);

    int fd_ = -1;
    double accel_scale_ = 0;  // raw -> m/s²
    double gyro_scale_ = 0;   // raw -> rad/s
    double rotation_[3][3] = {};
    int interval_us_ = 5000;
    bool initialized_ = false;
};
