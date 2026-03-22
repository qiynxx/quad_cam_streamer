#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

// Forward declarations to avoid including zmq headers here
namespace zmq {
class context_t;
class socket_t;
}

class ZmqStreamer {
public:
    ZmqStreamer();
    ~ZmqStreamer();

    ZmqStreamer(const ZmqStreamer &) = delete;
    ZmqStreamer &operator=(const ZmqStreamer &) = delete;

    bool bind(int port, int sndhwm = 2);
    void close();

    bool send(const uint8_t *data, size_t size);

private:
    zmq::context_t *ctx_ = nullptr;
    zmq::socket_t *socket_ = nullptr;
};
