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
        BOOT,        // startup chime
        REC_START,   // ascending beep: recording started
        REC_BEEP,    // short beep: recording in progress tick
        REC_STOP,    // descending beep: recording stopped
        REC_ERROR,   // error: failed to start recording (e.g. no SD card)
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

private:
    void worker();
    void ticker();
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
};
