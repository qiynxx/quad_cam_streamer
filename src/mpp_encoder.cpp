#include "mpp_encoder.h"

#include <cstdio>
#include <cstring>

#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/mpp_packet.h"
#include "rockchip/rk_venc_cfg.h"

#define LOG(fmt, ...) fprintf(stderr, "[MPP] " fmt "\n", ##__VA_ARGS__)

// Align to 16 bytes as required by MPP
#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))

MppJpegEncoder::MppJpegEncoder() {}

MppJpegEncoder::~MppJpegEncoder()
{
    deinit();
}

bool MppJpegEncoder::init(int width, int height, int stride, int quality)
{
    MPP_RET ret;

    width_ = width;
    height_ = height;
    stride_ = stride > 0 ? stride : MPP_ALIGN(width, 16);
    ver_stride_ = MPP_ALIGN(height, 16);

    // NV12: Y plane = stride * ver_stride, UV plane = stride * ver_stride / 2
    frame_size_ = stride_ * ver_stride_ * 3 / 2;

    // Create MPP context
    ret = mpp_create(&ctx_, &mpi_);
    if (ret != MPP_OK) {
        LOG("mpp_create failed: %d", ret);
        return false;
    }

    // Disable internal thread for synchronous encoding
    RK_U32 need_disable = 1;
    ret = mpi_->control(ctx_, MPP_SET_DISABLE_THREAD, &need_disable);
    if (ret != MPP_OK)
        LOG("Warning: MPP_SET_DISABLE_THREAD failed: %d", ret);

    ret = mpp_init(ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingMJPEG);
    if (ret != MPP_OK) {
        LOG("mpp_init MJPEG encoder failed: %d", ret);
        deinit();
        return false;
    }

    // Configure encoder using MppEncCfg
    MppEncCfg enc_cfg = nullptr;
    ret = mpp_enc_cfg_init(&enc_cfg);
    if (ret != MPP_OK) {
        LOG("mpp_enc_cfg_init failed: %d", ret);
        deinit();
        return false;
    }

    // Get default config
    ret = mpi_->control(ctx_, MPP_ENC_GET_CFG, enc_cfg);
    if (ret != MPP_OK) {
        LOG("MPP_ENC_GET_CFG failed: %d", ret);
        mpp_enc_cfg_deinit(enc_cfg);
        deinit();
        return false;
    }

    // Prep config: input format
    mpp_enc_cfg_set_s32(enc_cfg, "prep:width", width_);
    mpp_enc_cfg_set_s32(enc_cfg, "prep:height", height_);
    mpp_enc_cfg_set_s32(enc_cfg, "prep:hor_stride", stride_);
    mpp_enc_cfg_set_s32(enc_cfg, "prep:ver_stride", ver_stride_);
    mpp_enc_cfg_set_s32(enc_cfg, "prep:format", MPP_FMT_YUV420SP);

    // RC config: for MJPEG use fixqp mode
    mpp_enc_cfg_set_s32(enc_cfg, "rc:mode", MPP_ENC_RC_MODE_FIXQP);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_in_num", 30);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_in_denom", 1);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_out_num", 30);
    mpp_enc_cfg_set_s32(enc_cfg, "rc:fps_out_denom", 1);

    // JPEG quality factor (1-99, higher = better quality)
    mpp_enc_cfg_set_s32(enc_cfg, "jpeg:q_factor", quality);

    ret = mpi_->control(ctx_, MPP_ENC_SET_CFG, enc_cfg);
    mpp_enc_cfg_deinit(enc_cfg);

    if (ret != MPP_OK) {
        LOG("MPP_ENC_SET_CFG failed: %d", ret);
        deinit();
        return false;
    }

    // Allocate frame buffer group and buffer
    ret = mpp_buffer_group_get_internal(&buf_grp_, MPP_BUFFER_TYPE_DRM);
    if (ret != MPP_OK) {
        LOG("mpp_buffer_group_get_internal failed: %d", ret);
        deinit();
        return false;
    }

    ret = mpp_buffer_get(buf_grp_, &frame_buf_, frame_size_);
    if (ret != MPP_OK) {
        LOG("mpp_buffer_get failed: %d", ret);
        deinit();
        return false;
    }

    // Init frame
    ret = mpp_frame_init(&frame_);
    if (ret != MPP_OK) {
        LOG("mpp_frame_init failed: %d", ret);
        deinit();
        return false;
    }

    mpp_frame_set_width(frame_, width_);
    mpp_frame_set_height(frame_, height_);
    mpp_frame_set_hor_stride(frame_, stride_);
    mpp_frame_set_ver_stride(frame_, ver_stride_);
    mpp_frame_set_fmt(frame_, MPP_FMT_YUV420SP);
    mpp_frame_set_buffer(frame_, frame_buf_);
    mpp_frame_set_eos(frame_, 0);

    initialized_ = true;
    LOG("JPEG encoder initialized: %dx%d stride=%d ver_stride=%d q=%d",
        width_, height_, stride_, ver_stride_, quality);
    return true;
}

bool MppJpegEncoder::set_quality(int quality)
{
    if (!initialized_) {
        LOG("set_quality: encoder not initialized");
        return false;
    }

    if (quality < 1 || quality > 100) {
        LOG("set_quality: invalid quality %d (must be 1-100)", quality);
        return false;
    }

    MppEncCfg enc_cfg = nullptr;
    MPP_RET ret = mpp_enc_cfg_init(&enc_cfg);
    if (ret != MPP_OK) {
        LOG("set_quality: mpp_enc_cfg_init failed: %d", ret);
        return false;
    }

    ret = mpi_->control(ctx_, MPP_ENC_GET_CFG, enc_cfg);
    if (ret != MPP_OK) {
        LOG("set_quality: MPP_ENC_GET_CFG failed: %d", ret);
        mpp_enc_cfg_deinit(enc_cfg);
        return false;
    }

    mpp_enc_cfg_set_s32(enc_cfg, "jpeg:q_factor", quality);

    ret = mpi_->control(ctx_, MPP_ENC_SET_CFG, enc_cfg);
    mpp_enc_cfg_deinit(enc_cfg);

    if (ret != MPP_OK) {
        LOG("set_quality: MPP_ENC_SET_CFG failed: %d", ret);
        return false;
    }

    return true;
}

void MppJpegEncoder::deinit()
{
    if (frame_) {
        mpp_frame_deinit(&frame_);
        frame_ = nullptr;
    }
    if (frame_buf_) {
        mpp_buffer_put(frame_buf_);
        frame_buf_ = nullptr;
    }
    if (buf_grp_) {
        mpp_buffer_group_put(buf_grp_);
        buf_grp_ = nullptr;
    }
    if (ctx_) {
        mpi_->reset(ctx_);
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        mpi_ = nullptr;
    }
    initialized_ = false;
}

bool MppJpegEncoder::encode(const uint8_t *nv12_data, size_t nv12_size,
                            std::vector<uint8_t> &jpeg_out)
{
    if (!initialized_) return false;

    void *buf_ptr = mpp_buffer_get_ptr(frame_buf_);
    uint8_t *dst = (uint8_t *)buf_ptr;
    const uint8_t *src = nv12_data;

    // Copy Y plane (height rows)
    for (int y = 0; y < height_; y++) {
        memcpy(dst + y * stride_, src + y * stride_, stride_);
    }

    // Clear padding rows in Y plane if ver_stride > height
    if (ver_stride_ > height_) {
        memset(dst + height_ * stride_, 0, (ver_stride_ - height_) * stride_);
    }

    // Copy UV plane (height/2 rows)
    int uv_height = height_ / 2;
    const uint8_t *src_uv = src + stride_ * height_;
    uint8_t *dst_uv = dst + stride_ * ver_stride_;

    for (int y = 0; y < uv_height; y++) {
        memcpy(dst_uv + y * stride_, src_uv + y * stride_, stride_);
    }

    // Clear padding rows in UV plane if ver_stride > height
    if (ver_stride_ > height_) {
        int uv_ver_stride = ver_stride_ / 2;
        memset(dst_uv + uv_height * stride_, 128, (uv_ver_stride - uv_height) * stride_);
    }

    // Encode
    MppPacket packet = nullptr;
    MPP_RET ret = mpi_->encode_put_frame(ctx_, frame_);
    if (ret != MPP_OK) {
        LOG("encode_put_frame failed: %d", ret);
        return false;
    }

    ret = mpi_->encode_get_packet(ctx_, &packet);
    if (ret != MPP_OK || !packet) {
        LOG("encode_get_packet failed: %d", ret);
        return false;
    }

    // Extract JPEG data
    void *pkt_ptr = mpp_packet_get_pos(packet);
    size_t pkt_len = mpp_packet_get_length(packet);

    jpeg_out.resize(pkt_len);
    memcpy(jpeg_out.data(), pkt_ptr, pkt_len);

    mpp_packet_deinit(&packet);
    return true;
}
