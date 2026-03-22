#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <linux/videodev2.h>

class V4L2Camera {
public:
    V4L2Camera();
    ~V4L2Camera();

    V4L2Camera(const V4L2Camera &) = delete;
    V4L2Camera &operator=(const V4L2Camera &) = delete;

    bool open(const std::string &device, int width, int height, int fps);
    void close();

    // Re-query format after external ISP configuration (e.g., rkaiq)
    bool refresh_format();

    void set_exposure(bool auto_exp, int exposure_us, int gain);
    void open_subdev(const std::string &subdev_path);
    float get_brightness(const uint8_t *nv12_data);  // Calculate brightness from NV12 frame
    void list_controls();  // List all available V4L2 controls

    bool start();
    void stop();

    // Returns pointer to frame data, sets size and timestamp_ns (kernel monotonic).
    // Caller must call release() after use.
    const uint8_t *capture(size_t &size, uint64_t &timestamp_ns, int timeout_ms = 2000);
    void release();

    int width() const { return width_; }
    int height() const { return height_; }
    int stride() const { return stride_; }

private:
    static constexpr int NUM_BUFFERS = 4;
    static constexpr int MAX_PLANES = 3;

    int fd_ = -1;
    int subdev_fd_ = -1;  // sensor subdev for exposure/gain control
    int width_ = 0;
    int height_ = 0;
    int stride_ = 0;
    bool multiplanar_ = false;

    struct Buffer {
        void *start = nullptr;
        size_t length = 0;
    };
    Buffer buffers_[NUM_BUFFERS] = {};
    int buf_count_ = 0;
    int current_buf_index_ = -1;
    bool streaming_ = false;

    // For multi-planar DQBUF/QBUF
    struct v4l2_plane planes_[MAX_PLANES] = {};

    uint32_t buf_type() const;
};
