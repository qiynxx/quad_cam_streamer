#include "serial_imu_reader.h"
#include "config.h"
#include "imu_reader.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define LOG(fmt, ...) fprintf(stderr, "[SerialIMU] " fmt "\n", ##__VA_ARGS__)

static constexpr int FRAME_TOTAL_LEN = 31;
static constexpr int IMU_PAYLOAD_LEN = 24;
static constexpr size_t MAX_BUFFER_SIZE = 4096;

bool SerialImuReader::init(const SerialImuConfig &config)
{
    stats_ = {};

    /* O_RDONLY: we only need RX; avoids driving TX / modem control on some boards */
    fd_ = open(config.uart_device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        LOG("Failed to open %s", config.uart_device.c_str());
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        LOG("tcgetattr failed for %s", config.uart_device.c_str());
        ::close(fd_); fd_ = -1;
        return false;
    }

    speed_t baud;
    switch (config.baudrate) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default:     baud = B115200; break;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        LOG("tcsetattr failed for %s", config.uart_device.c_str());
        ::close(fd_); fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    buffer_.reserve(MAX_BUFFER_SIZE);

    initialized_ = true;
    LOG("Opened %s @ %d baud (%s) O_RDONLY", config.uart_device.c_str(),
        config.baudrate, config.name.c_str());
    return true;
}

void SerialImuReader::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    buffer_.clear();
    initialized_ = false;
}

void SerialImuReader::get_parse_stats(SerialImuParseStats &out, bool reset_after)
{
    out = stats_;
    if (reset_after)
        stats_ = {};
}

// Binary frame format (31 bytes):
//   [0xAA 0x55] [0x01] [0x18] [24B payload] [XOR checksum] [0x0A 0x0A]
//   Payload: 6 x float32 LE = ax,ay,az (g), gx,gy,gz (deg/s)
bool SerialImuReader::read(ImuSample &sample)
{
    if (!initialized_) return false;

    uint8_t temp[256];
    ssize_t n = ::read(fd_, temp, sizeof(temp));
    if (n > 0) {
        stats_.bytes_read += (uint64_t)n;
        buffer_.insert(buffer_.end(), temp, temp + n);
        if (buffer_.size() > MAX_BUFFER_SIZE)
            buffer_.erase(buffer_.begin(),
                          buffer_.begin() + (buffer_.size() - MAX_BUFFER_SIZE));
    }

    while (buffer_.size() >= static_cast<size_t>(FRAME_TOTAL_LEN)) {
        // Find header 0xAA 0x55
        auto it = buffer_.begin();
        bool found = false;
        for (; it + 1 < buffer_.end(); ++it) {
            if (*it == 0xAA && *(it + 1) == 0x55) {
                found = true;
                break;
            }
        }

        if (!found) {
            if (buffer_.size() > 1)
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
            return false;
        }

        if (it != buffer_.begin())
            buffer_.erase(buffer_.begin(), it);

        if (buffer_.size() < static_cast<size_t>(FRAME_TOTAL_LEN))
            return false;

        if (buffer_[2] != 0x01 || buffer_[3] != IMU_PAYLOAD_LEN) {
            stats_.bad_type_or_len++;
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
            continue;
        }

        uint8_t checksum = 0;
        for (int i = 0; i < IMU_PAYLOAD_LEN; ++i)
            checksum ^= buffer_[4 + i];
        if (checksum != buffer_[4 + IMU_PAYLOAD_LEN]) {
            stats_.bad_checksum++;
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
            continue;
        }

        {
            uint8_t t0 = buffer_[FRAME_TOTAL_LEN - 2];
            uint8_t t1 = buffer_[FRAME_TOTAL_LEN - 1];
            bool tail_ok = (t0 == 0x0A && t1 == 0x0A) || (t0 == 0x0D && t1 == 0x0A);
            if (!tail_ok) {
                stats_.bad_tail++;
                buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
                continue;
            }
        }

        float data[6];
        std::memcpy(data, &buffer_[4], sizeof(data));

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        sample.timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

        sample.accel[0] = (double)data[0] * 9.81;
        sample.accel[1] = (double)data[1] * 9.81;
        sample.accel[2] = (double)data[2] * 9.81;
        sample.gyro[0] = (double)data[3] * M_PI / 180.0;
        sample.gyro[1] = (double)data[4] * M_PI / 180.0;
        sample.gyro[2] = (double)data[5] * M_PI / 180.0;

        stats_.frames_ok++;
        buffer_.erase(buffer_.begin(), buffer_.begin() + FRAME_TOTAL_LEN);
        return true;
    }

    return false;
}
