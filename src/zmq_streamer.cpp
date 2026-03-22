#include "zmq_streamer.h"

#include <cstdio>
#include <zmq.hpp>

#define LOG(fmt, ...) fprintf(stderr, "[ZMQ] " fmt "\n", ##__VA_ARGS__)

ZmqStreamer::ZmqStreamer() {}

ZmqStreamer::~ZmqStreamer()
{
    close();
}

bool ZmqStreamer::bind(int port, int sndhwm)
{
    try {
        ctx_ = new zmq::context_t(1);
        socket_ = new zmq::socket_t(*ctx_, zmq::socket_type::pub);

        socket_->set(zmq::sockopt::sndhwm, sndhwm);

        // Set linger to 0 for immediate close
        int linger = 0;
        socket_->set(zmq::sockopt::linger, linger);

        char addr[64];
        snprintf(addr, sizeof(addr), "tcp://*:%d", port);
        socket_->bind(addr);

        LOG("Bound to %s", addr);
        return true;
    } catch (const zmq::error_t &e) {
        LOG("Failed to bind port %d: %s", port, e.what());
        close();
        return false;
    }
}

void ZmqStreamer::close()
{
    delete socket_;
    socket_ = nullptr;
    delete ctx_;
    ctx_ = nullptr;
}

bool ZmqStreamer::send(const uint8_t *data, size_t size)
{
    if (!socket_) return false;

    try {
        zmq::message_t msg(data, size);
        auto result = socket_->send(msg, zmq::send_flags::dontwait);
        return result.has_value();
    } catch (const zmq::error_t &e) {
        LOG("Send failed: %s", e.what());
        return false;
    }
}
