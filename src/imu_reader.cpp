#include "imu_reader.h"
#include "config.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define LOG(fmt, ...) fprintf(stderr, "[IMU] " fmt "\n", ##__VA_ARGS__)

// LSM6DS3 registers
#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10  // Accel: ODR[7:4], FS[3:2], BW[1:0]
#define CTRL2_G       0x11  // Gyro:  ODR[7:4], FS[3:1]
#define CTRL3_C       0x12  // BDU, IF_INC, etc.
#define OUTX_L_G      0x22  // Gyro X low byte (12 bytes: gyro XYZ + accel XYZ)

// ODR bits (CTRL1_XL[7:4] and CTRL2_G[7:4])
#define ODR_POWER_DOWN 0x00
#define ODR_12_5HZ     0x10
#define ODR_26HZ       0x20
#define ODR_52HZ       0x30
#define ODR_104HZ      0x40
#define ODR_208HZ      0x50
#define ODR_416HZ      0x60
#define ODR_833HZ      0x70

// Accel full-scale (CTRL1_XL[3:2])
#define FS_XL_2G       0x00
#define FS_XL_4G       0x08
#define FS_XL_8G       0x0C
#define FS_XL_16G      0x04

// Gyro full-scale (CTRL2_G[3:2])
#define FS_G_245DPS    0x00
#define FS_G_500DPS    0x04
#define FS_G_1000DPS   0x08
#define FS_G_2000DPS   0x0C

static uint8_t odr_bits(int freq)
{
    if (freq <= 12)  return ODR_12_5HZ;
    if (freq <= 26)  return ODR_26HZ;
    if (freq <= 52)  return ODR_52HZ;
    if (freq <= 104) return ODR_104HZ;
    if (freq <= 208) return ODR_208HZ;
    if (freq <= 416) return ODR_416HZ;
    return ODR_833HZ;
}

static uint8_t accel_fs_bits(int range_g, double &scale)
{
    // scale = m/s² per LSB
    if (range_g <= 2)       { scale = 0.000598; return FS_XL_2G; }
    else if (range_g <= 4)  { scale = 0.001197; return FS_XL_4G; }
    else if (range_g <= 8)  { scale = 0.002394; return FS_XL_8G; }
    else                    { scale = 0.004788; return FS_XL_16G; }
}

static uint8_t gyro_fs_bits(int range_dps, double &scale)
{
    // scale = rad/s per LSB
    if (range_dps <= 245)       { scale = 0.0001527; return FS_G_245DPS; }
    else if (range_dps <= 500)  { scale = 0.0003054; return FS_G_500DPS; }
    else if (range_dps <= 1000) { scale = 0.0006109; return FS_G_1000DPS; }
    else                        { scale = 0.0012217; return FS_G_2000DPS; }
}

bool ImuReader::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return ::write(fd_, buf, 2) == 2;
}

bool ImuReader::read_regs(uint8_t reg, uint8_t *buf, int len)
{
    if (::write(fd_, &reg, 1) != 1) return false;
    return ::read(fd_, buf, len) == len;
}

bool ImuReader::init(const ImuConfig &config)
{
    fd_ = open(config.i2c_device.c_str(), O_RDWR);
    if (fd_ < 0) {
        LOG("Failed to open %s", config.i2c_device.c_str());
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, config.i2c_addr) < 0) {
        LOG("Failed to set I2C address 0x%02x", config.i2c_addr);
        ::close(fd_); fd_ = -1;
        return false;
    }

    // Verify WHO_AM_I
    uint8_t wai = 0;
    if (!read_regs(WHO_AM_I, &wai, 1) || wai != 0x69) {
        LOG("WHO_AM_I mismatch: expected 0x69, got 0x%02x", wai);
        ::close(fd_); fd_ = -1;
        return false;
    }
    LOG("LSM6DS3 detected (WHO_AM_I=0x%02x) at %s:0x%02x",
        wai, config.i2c_device.c_str(), config.i2c_addr);

    // CTRL3_C: enable BDU (block data update) + IF_INC (auto-increment address)
    write_reg(CTRL3_C, 0x44);

    // Configure accel
    uint8_t odr = odr_bits(config.sampling_frequency);
    uint8_t accel_fs = accel_fs_bits(config.accel_range, accel_scale_);
    write_reg(CTRL1_XL, odr | accel_fs);

    // Configure gyro
    uint8_t gyro_fs = gyro_fs_bits(config.gyro_range, gyro_scale_);
    write_reg(CTRL2_G, odr | gyro_fs);

    interval_us_ = 1000000 / config.sampling_frequency;
    memcpy(rotation_, config.rotation_matrix, sizeof(rotation_));

    initialized_ = true;
    LOG("Initialized: ODR=%d Hz, accel=+/-%dg (scale=%.6f), gyro=+/-%ddps (scale=%.6f)",
        config.sampling_frequency, config.accel_range, accel_scale_,
        config.gyro_range, gyro_scale_);
    return true;
}

void ImuReader::close()
{
    if (fd_ >= 0) {
        // Power down: set ODR to 0 for both accel and gyro
        write_reg(CTRL1_XL, ODR_POWER_DOWN);
        write_reg(CTRL2_G, ODR_POWER_DOWN);
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

bool ImuReader::read(ImuSample &sample)
{
    if (!initialized_)
        return false;

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    // Burst-read 12 bytes: gyro XYZ (6B) + accel XYZ (6B) starting from OUTX_L_G
    uint8_t raw[12];
    if (!read_regs(OUTX_L_G, raw, 12))
        return false;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    sample.timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    // Sleep remaining time to hit target rate (subtract I2C read duration)
    int64_t elapsed_us = (ts.tv_sec - t0.tv_sec) * 1000000LL +
                         (ts.tv_nsec - t0.tv_nsec) / 1000LL;
    int64_t remaining = interval_us_ - elapsed_us;
    if (remaining > 0)
        usleep(remaining);

    // Parse gyro (first 6 bytes): X_L, X_H, Y_L, Y_H, Z_L, Z_H
    double gx = (int16_t)(raw[0] | (raw[1] << 8)) * gyro_scale_;
    double gy = (int16_t)(raw[2] | (raw[3] << 8)) * gyro_scale_;
    double gz = (int16_t)(raw[4] | (raw[5] << 8)) * gyro_scale_;

    // Parse accel (next 6 bytes)
    double ax = (int16_t)(raw[6]  | (raw[7]  << 8)) * accel_scale_;
    double ay = (int16_t)(raw[8]  | (raw[9]  << 8)) * accel_scale_;
    double az = (int16_t)(raw[10] | (raw[11] << 8)) * accel_scale_;

    // Apply rotation matrix: body = R * sensor
    sample.accel[0] = rotation_[0][0]*ax + rotation_[0][1]*ay + rotation_[0][2]*az;
    sample.accel[1] = rotation_[1][0]*ax + rotation_[1][1]*ay + rotation_[1][2]*az;
    sample.accel[2] = rotation_[2][0]*ax + rotation_[2][1]*ay + rotation_[2][2]*az;

    sample.gyro[0] = rotation_[0][0]*gx + rotation_[0][1]*gy + rotation_[0][2]*gz;
    sample.gyro[1] = rotation_[1][0]*gx + rotation_[1][1]*gy + rotation_[1][2]*gz;
    sample.gyro[2] = rotation_[2][0]*gx + rotation_[2][1]*gy + rotation_[2][2]*gz;

    return true;
}
