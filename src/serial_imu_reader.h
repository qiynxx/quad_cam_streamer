#pragma once

#include <cstdint>
#include <vector>

struct SerialImuConfig;
struct ImuSample;

struct SerialImuParseStats {
    uint64_t frames_ok = 0;
    uint64_t bad_type_or_len = 0;
    uint64_t bad_checksum = 0;
    uint64_t bad_tail = 0;
    uint64_t bytes_read = 0;
};

class SerialImuReader {
public:
    bool init(const SerialImuConfig &config);
    void close();
    bool read(ImuSample &sample);
    /** Snapshot counters; if reset_after, clears stats after copy. */
    void get_parse_stats(SerialImuParseStats &out, bool reset_after);

private:
    int fd_ = -1;
    std::vector<uint8_t> buffer_;
    bool initialized_ = false;
    SerialImuParseStats stats_{};
};
