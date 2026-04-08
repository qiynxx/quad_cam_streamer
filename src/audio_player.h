#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

// Async audio player using ALSA PCM.
// All play_* calls are non-blocking; audio is rendered in a background thread.
// Generates tones via PCM synthesis — no audio files needed.
class AudioPlayer {
public:
    enum class Sound {
        BOOT,            // startup chime
        REC_START,       // ascending beep: recording started
        REC_BEEP,        // short beep: recording in progress tick
        REC_STOP,        // descending beep: recording stopped
        REC_ERROR,       // error: failed to start recording (e.g. no SD card)
        BLE_PAIR_ENTER,  // rapid double beep: pairing mode entered
        BLE_PAIR_ONE,    // double beep: one hand ready
        BLE_PAIR_BOTH,   // triple beep: both hands ready
        IMU_DISCONNECT,  // urgent warning: serial IMU lost / open failed
    };

    AudioPlayer();
    ~AudioPlayer();

    // Open ALSA device and play boot sound. Call once at startup.
    bool init(const char *device = "plughw:0,0");

    // Queue a sound for async playback (non-blocking).
    void play(Sound s);

    // Start/stop the 1-second periodic beep during recording.
    void start_rec_ticker();
    void stop_rec_ticker();
    void start_status_ticker(Sound s, int interval_ms);
    void stop_status_ticker();

private:
    void worker();
    void ticker();
    void status_ticker();
    void render_sound(Sound s);
    bool write_pcm(const std::vector<int16_t> &samples);

    // PCM synthesis helpers
    std::vector<int16_t> make_tone(float freq_hz, float duration_s, float amplitude = 0.6f);
    std::vector<int16_t> make_sweep(float f0, float f1, float duration_s, float amplitude = 0.6f);
    std::vector<int16_t> make_silence(float duration_s);

    void *pcm_ = nullptr;  // snd_pcm_t*
    int sample_rate_ = 44100;

    std::queue<Sound> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
    std::thread worker_thread_;

    std::atomic<bool> ticker_running_{false};
    std::thread ticker_thread_;

    std::atomic<bool> status_ticker_running_{false};
    std::thread status_ticker_thread_;
    std::mutex status_ticker_mtx_;
    Sound status_ticker_sound_ = Sound::REC_BEEP;
    int status_ticker_interval_ms_ = 1000;
};
