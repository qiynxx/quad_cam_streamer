#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#include "rockchip/rk_type.h"
#include "rockchip/rk_mpi.h"

class MppJpegEncoder {
public:
    MppJpegEncoder();
    ~MppJpegEncoder();

    MppJpegEncoder(const MppJpegEncoder &) = delete;
    MppJpegEncoder &operator=(const MppJpegEncoder &) = delete;

    bool init(int width, int height, int stride, int quality);
    void deinit();

    // Update JPEG quality dynamically (1-100)
    bool set_quality(int quality);

    // Encode NV12 frame to JPEG. Returns JPEG data in output vector.
    bool encode(const uint8_t *nv12_data, size_t nv12_size,
                std::vector<uint8_t> &jpeg_out);

private:
    MppCtx ctx_ = nullptr;
    MppApi *mpi_ = nullptr;
    MppBufferGroup buf_grp_ = nullptr;
    MppBuffer frame_buf_ = nullptr;
    MppFrame frame_ = nullptr;

    int width_ = 0;
    int height_ = 0;
    int stride_ = 0;
    int ver_stride_ = 0;
    size_t frame_size_ = 0;
    bool initialized_ = false;
};
