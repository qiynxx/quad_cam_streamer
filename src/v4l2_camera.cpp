#include "v4l2_camera.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define LOG(fmt, ...) fprintf(stderr, "[V4L2] " fmt "\n", ##__VA_ARGS__)

static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

V4L2Camera::V4L2Camera() {}

V4L2Camera::~V4L2Camera()
{
    close();
}

uint32_t V4L2Camera::buf_type() const
{
    return multiplanar_ ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                        : V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

bool V4L2Camera::open(const std::string &device, int width, int height, int fps)
{
    fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        LOG("Failed to open %s: %s", device.c_str(), strerror(errno));
        return false;
    }

    // Query device capabilities to determine single-planar vs multi-planar
    struct v4l2_capability cap = {};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        LOG("Failed to query capabilities: %s", strerror(errno));
        close();
        return false;
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        multiplanar_ = true;
        LOG("Using multi-planar API for %s", device.c_str());
    } else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        multiplanar_ = false;
        LOG("Using single-planar API for %s", device.c_str());
    } else {
        LOG("Device %s does not support video capture", device.c_str());
        close();
        return false;
    }

    // Set format
    struct v4l2_format fmt = {};
    fmt.type = buf_type();

    if (multiplanar_) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;  // NV12 as single contiguous plane
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        LOG("Failed to set format on %s: %s", device.c_str(), strerror(errno));
        close();
        return false;
    }

    if (multiplanar_) {
        width_ = fmt.fmt.pix_mp.width;
        height_ = fmt.fmt.pix_mp.height;
        stride_ = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        LOG("Format: %dx%d stride=%d planes=%d",
            width_, height_, stride_, fmt.fmt.pix_mp.num_planes);
    } else {
        width_ = fmt.fmt.pix.width;
        height_ = fmt.fmt.pix.height;
        stride_ = fmt.fmt.pix.bytesperline;
        LOG("Format: %dx%d stride=%d", width_, height_, stride_);
    }

    // Set frame rate
    struct v4l2_streamparm parm = {};
    parm.type = buf_type();
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0)
        LOG("Warning: Failed to set frame rate: %s", strerror(errno));

    // Request buffers
    struct v4l2_requestbuffers req = {};
    req.count = NUM_BUFFERS;
    req.type = buf_type();
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        LOG("Failed to request buffers: %s", strerror(errno));
        close();
        return false;
    }

    buf_count_ = req.count;

    // Map buffers
    for (int i = 0; i < buf_count_; i++) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane planes[MAX_PLANES] = {};
        buf.type = buf_type();
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (multiplanar_) {
            buf.m.planes = planes;
            buf.length = MAX_PLANES;
        }

        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG("Failed to query buffer %d: %s", i, strerror(errno));
            close();
            return false;
        }

        size_t buf_length;
        off_t buf_offset;
        if (multiplanar_) {
            buf_length = planes[0].length;
            buf_offset = planes[0].m.mem_offset;
        } else {
            buf_length = buf.length;
            buf_offset = buf.m.offset;
        }

        buffers_[i].length = buf_length;
        buffers_[i].start = mmap(nullptr, buf_length,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 fd_, buf_offset);
        if (buffers_[i].start == MAP_FAILED) {
            LOG("Failed to mmap buffer %d: %s", i, strerror(errno));
            buffers_[i].start = nullptr;
            close();
            return false;
        }
    }

    return true;
}

bool V4L2Camera::refresh_format()
{
    if (fd_ < 0) return false;

    struct v4l2_format fmt = {};
    fmt.type = buf_type();

    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
        LOG("Failed to get format: %s", strerror(errno));
        return false;
    }

    if (multiplanar_) {
        stride_ = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        LOG("Format refreshed: stride=%d", stride_);
    } else {
        stride_ = fmt.fmt.pix.bytesperline;
        LOG("Format refreshed: stride=%d", stride_);
    }

    return true;
}

void V4L2Camera::close()
{
    stop();

    for (int i = 0; i < buf_count_; i++) {
        if (buffers_[i].start && buffers_[i].start != MAP_FAILED) {
            munmap(buffers_[i].start, buffers_[i].length);
            buffers_[i].start = nullptr;
        }
    }
    buf_count_ = 0;

    if (subdev_fd_ >= 0) {
        ::close(subdev_fd_);
        subdev_fd_ = -1;
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void V4L2Camera::open_subdev(const std::string &subdev_path)
{
    if (subdev_path.empty()) return;

    subdev_fd_ = ::open(subdev_path.c_str(), O_RDWR);
    if (subdev_fd_ < 0)
        LOG("Warning: Failed to open subdev %s: %s", subdev_path.c_str(), strerror(errno));
    else
        LOG("Opened subdev %s for controls", subdev_path.c_str());
}

void V4L2Camera::set_exposure(bool auto_exp, int exposure_us, int gain)
{
    int ctrl_fd = (subdev_fd_ >= 0) ? subdev_fd_ : fd_;
    if (ctrl_fd < 0) return;

    struct v4l2_control ctrl = {};

    if (!auto_exp) {
        int exposure_lines = (int)(exposure_us / line_time_us_ + 0.5f);
        if (exposure_lines < 1) exposure_lines = 1;

        ctrl.id = V4L2_CID_EXPOSURE;
        ctrl.value = exposure_lines;
        if (xioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0)
            LOG("Warning: Failed to set exposure: %s", strerror(errno));
    }

    ctrl.id = V4L2_CID_ANALOGUE_GAIN;
    ctrl.value = gain;
    if (xioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0)
        LOG("Warning: Failed to set gain: %s", strerror(errno));
}

float V4L2Camera::get_brightness(const uint8_t *nv12_data)
{
    if (!nv12_data) return 128.0f;

    const int sample_step = 16;
    uint64_t sum = 0;
    int count = 0;

    for (int y = 0; y < height_; y += sample_step) {
        const uint8_t *row = nv12_data + y * stride_;
        for (int x = 0; x < width_; x += sample_step) {
            sum += row[x];
            count++;
        }
    }

    return count > 0 ? (float)sum / count : 128.0f;
}

void V4L2Camera::list_controls()
{
    int ctrl_fd = (subdev_fd_ >= 0) ? subdev_fd_ : fd_;
    if (ctrl_fd < 0) return;

    LOG("Listing V4L2 controls:");

    struct v4l2_queryctrl qctrl = {};
    qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

    while (xioctl(ctrl_fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
        if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
            LOG("  0x%08x: %s (min=%d max=%d step=%d default=%d)",
                qctrl.id, qctrl.name, qctrl.minimum, qctrl.maximum,
                qctrl.step, qctrl.default_value);
        }
        qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
}

bool V4L2Camera::start()
{
    if (fd_ < 0 || streaming_) return false;

    // Queue all buffers
    for (int i = 0; i < buf_count_; i++) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane planes[MAX_PLANES] = {};
        buf.type = buf_type();
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (multiplanar_) {
            buf.m.planes = planes;
            buf.length = 1;
        }

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            LOG("Failed to queue buffer %d: %s", i, strerror(errno));
            return false;
        }
    }

    int type = buf_type();
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        LOG("Failed to start streaming: %s", strerror(errno));
        return false;
    }

    streaming_ = true;
    return true;
}

void V4L2Camera::stop()
{
    if (fd_ < 0 || !streaming_) return;

    int type = buf_type();
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
    current_buf_index_ = -1;
}

const uint8_t *V4L2Camera::capture(size_t &size, uint64_t &timestamp_ns, int timeout_ms)
{
    if (!streaming_) return nullptr;

    struct pollfd pfd = {};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
        if (ret == 0)
            LOG("Capture timeout");
        else
            LOG("Poll error: %s", strerror(errno));
        return nullptr;
    }

    struct v4l2_buffer buf = {};
    buf.type = buf_type();
    buf.memory = V4L2_MEMORY_MMAP;

    if (multiplanar_) {
        memset(planes_, 0, sizeof(planes_));
        buf.m.planes = planes_;
        buf.length = MAX_PLANES;
    }

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        LOG("Failed to dequeue buffer: %s", strerror(errno));
        return nullptr;
    }

    current_buf_index_ = buf.index;

    // Use kernel buffer timestamp (set at frame start by CIF/ISP driver)
    timestamp_ns = (uint64_t)buf.timestamp.tv_sec * 1000000000ULL
                 + (uint64_t)buf.timestamp.tv_usec * 1000ULL;

    // Return actual NV12 size: stride * height * 3/2
    // Don't trust bytesused - it may only report Y plane size
    size = stride_ * height_ * 3 / 2;

    return static_cast<const uint8_t *>(buffers_[buf.index].start);
}

void V4L2Camera::release()
{
    if (current_buf_index_ < 0 || !streaming_) return;

    struct v4l2_buffer buf = {};
    struct v4l2_plane planes[MAX_PLANES] = {};
    buf.type = buf_type();
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buf_index_;

    if (multiplanar_) {
        buf.m.planes = planes;
        buf.length = 1;
    }

    xioctl(fd_, VIDIOC_QBUF, &buf);
    current_buf_index_ = -1;
}
