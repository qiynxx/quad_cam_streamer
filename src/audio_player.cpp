#include "audio_player.h"

#include <alsa/asoundlib.h>
#include <cmath>
#include <cstdio>
#include <vector>

AudioPlayer::AudioPlayer() = default;

AudioPlayer::~AudioPlayer()
{
    stop_rec_ticker();
    stop_status_ticker();

    running_ = false;
    cv_.notify_all();
    if (worker_thread_.joinable())
        worker_thread_.join();

    if (pcm_) {
        snd_pcm_drain(static_cast<snd_pcm_t *>(pcm_));
        snd_pcm_close(static_cast<snd_pcm_t *>(pcm_));
        pcm_ = nullptr;
    }
}

bool AudioPlayer::init(const char *device)
{
    snd_pcm_t *pcm = nullptr;
    int err = snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        fprintf(stderr, "[audio] snd_pcm_open(%s): %s\n", device, snd_strerror(err));
        return false;
    }

    err = snd_pcm_set_params(pcm,
                             SND_PCM_FORMAT_S16_LE,
                             SND_PCM_ACCESS_RW_INTERLEAVED,
                             1,            // mono
                             sample_rate_,
                             1,            // allow resampling
                             50000);       // 50ms latency
    if (err < 0) {
        fprintf(stderr, "[audio] snd_pcm_set_params: %s\n", snd_strerror(err));
        snd_pcm_close(pcm);
        return false;
    }

    pcm_ = pcm;
    running_ = true;
    worker_thread_ = std::thread(&AudioPlayer::worker, this);

    fprintf(stderr, "[audio] ALSA init OK (%s, %d Hz mono)\n", device, sample_rate_);
    play(Sound::BOOT);
    return true;
}

void AudioPlayer::play(Sound s)
{
    if (!pcm_) return;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        queue_.push(s);
    }
    cv_.notify_one();
}

void AudioPlayer::start_rec_ticker()
{
    if (ticker_running_.exchange(true)) return;  // already running
    ticker_thread_ = std::thread(&AudioPlayer::ticker, this);
}

void AudioPlayer::stop_rec_ticker()
{
    ticker_running_ = false;
    if (ticker_thread_.joinable())
        ticker_thread_.join();
}

void AudioPlayer::start_status_ticker(Sound s, int interval_ms)
{
    stop_status_ticker();
    {
        std::lock_guard<std::mutex> lk(status_ticker_mtx_);
        status_ticker_sound_ = s;
        status_ticker_interval_ms_ = interval_ms;
    }
    status_ticker_running_ = true;
    status_ticker_thread_ = std::thread(&AudioPlayer::status_ticker, this);
}

void AudioPlayer::stop_status_ticker()
{
    status_ticker_running_ = false;
    if (status_ticker_thread_.joinable())
        status_ticker_thread_.join();
}

// ---- private ----

void AudioPlayer::worker()
{
    while (running_) {
        Sound s;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
            if (!running_ && queue_.empty()) break;
            s = queue_.front();
            queue_.pop();
        }
        render_sound(s);
    }
}

void AudioPlayer::ticker()
{
    while (ticker_running_) {
        play(Sound::REC_BEEP);
        // Sleep 1 second in small increments so we can exit quickly
        for (int i = 0; i < 20 && ticker_running_; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void AudioPlayer::status_ticker()
{
    while (status_ticker_running_) {
        Sound sound = Sound::REC_BEEP;
        int interval_ms = 1000;
        {
            std::lock_guard<std::mutex> lk(status_ticker_mtx_);
            sound = status_ticker_sound_;
            interval_ms = status_ticker_interval_ms_;
        }
        play(sound);
        for (int elapsed = 0; elapsed < interval_ms && status_ticker_running_; elapsed += 50)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void AudioPlayer::render_sound(Sound s)
{
    std::vector<int16_t> samples;

    switch (s) {
    case Sound::BOOT:
        // Three ascending short tones: 440 → 660 → 880 Hz
        {
            auto a = make_tone(440.0f, 0.12f, 0.5f);
            auto b = make_tone(660.0f, 0.12f, 0.5f);
            auto c = make_tone(880.0f, 0.18f, 0.5f);
            auto sil = make_silence(0.05f);
            samples.insert(samples.end(), a.begin(), a.end());
            samples.insert(samples.end(), sil.begin(), sil.end());
            samples.insert(samples.end(), b.begin(), b.end());
            samples.insert(samples.end(), sil.begin(), sil.end());
            samples.insert(samples.end(), c.begin(), c.end());
        }
        break;

    case Sound::REC_START:
        // Ascending sweep 400 → 1200 Hz, 0.3s
        samples = make_sweep(400.0f, 1200.0f, 0.3f, 0.6f);
        break;

    case Sound::REC_BEEP:
        // Short 880 Hz beep, 0.08s
        samples = make_tone(880.0f, 0.08f, 0.4f);
        break;

    case Sound::REC_STOP:
        // Descending sweep 1200 → 400 Hz, 0.3s
        samples = make_sweep(1200.0f, 400.0f, 0.3f, 0.6f);
        break;

    case Sound::REC_ERROR:
        // Three short low-pitched beeps: 300 Hz x3, signals failure
        {
            auto beep = make_tone(300.0f, 0.12f, 0.7f);
            auto sil  = make_silence(0.08f);
            for (int i = 0; i < 3; i++) {
                samples.insert(samples.end(), beep.begin(), beep.end());
                if (i < 2)
                    samples.insert(samples.end(), sil.begin(), sil.end());
            }
        }
        break;

    case Sound::BLE_PAIR_ENTER:
        {
            auto beep = make_tone(1400.0f, 0.05f, 0.55f);
            auto sil  = make_silence(0.04f);
            samples.insert(samples.end(), beep.begin(), beep.end());
            samples.insert(samples.end(), sil.begin(), sil.end());
            samples.insert(samples.end(), beep.begin(), beep.end());
        }
        break;

    case Sound::BLE_PAIR_ONE:
        {
            auto beep = make_tone(1100.0f, 0.07f, 0.45f);
            auto sil  = make_silence(0.06f);
            samples.insert(samples.end(), beep.begin(), beep.end());
            samples.insert(samples.end(), sil.begin(), sil.end());
            samples.insert(samples.end(), beep.begin(), beep.end());
        }
        break;

    case Sound::BLE_PAIR_BOTH:
        {
            // 更强调“连响”感：高音短 beep，多一些间隔，方便与长响区分。
            auto beep = make_tone(1400.0f, 0.05f, 0.55f);
            auto sil  = make_silence(0.12f);
            for (int i = 0; i < 3; i++) {
                samples.insert(samples.end(), beep.begin(), beep.end());
                if (i < 2)
                    samples.insert(samples.end(), sil.begin(), sil.end());
            }
        }
        break;

    case Sound::BLE_CONN_ZERO:
        // 0 连接：较温和的双响，提示“还未连接任何 BLE IMU”
        {
            // 稍微拉长单次 beep 和两次之间的间隔，让双响更明显
            auto beep = make_tone(900.0f, 0.10f, 0.5f);
            auto sil  = make_silence(0.15f);
            samples.insert(samples.end(), beep.begin(), beep.end());
            samples.insert(samples.end(), sil.begin(), sil.end());
            samples.insert(samples.end(), beep.begin(), beep.end());
        }
        break;

    case Sound::BLE_CONN_ONE:
        // 1 连接：单个短 beep，时间稍微拉长一点
        samples = make_tone(1100.0f, 0.12f, 0.55f);
        break;

    case Sound::BLE_CONN_BOTH:
        // 2 连接：使用与 0 连接双响相同的频率，改为单次明显更长的确认 beep。
        samples = make_tone(900.0f, 0.8f, 0.7f);
        break;

    case Sound::IMU_DISCONNECT:
        // Two-tone urgent alarm: 800 Hz → 400 Hz, repeated twice
        {
            auto hi  = make_tone(800.0f, 0.10f, 0.8f);
            auto lo  = make_tone(400.0f, 0.15f, 0.8f);
            auto sil = make_silence(0.06f);
            for (int i = 0; i < 2; i++) {
                samples.insert(samples.end(), hi.begin(), hi.end());
                samples.insert(samples.end(), lo.begin(), lo.end());
                if (i < 1)
                    samples.insert(samples.end(), sil.begin(), sil.end());
            }
        }
        break;
    }

    write_pcm(samples);
}

bool AudioPlayer::write_pcm(const std::vector<int16_t> &samples)
{
    if (!pcm_ || samples.empty()) return false;
    snd_pcm_t *pcm = static_cast<snd_pcm_t *>(pcm_);

    const int16_t *ptr = samples.data();
    snd_pcm_sframes_t remaining = (snd_pcm_sframes_t)samples.size();

    while (remaining > 0) {
        snd_pcm_sframes_t written = snd_pcm_writei(pcm, ptr, remaining);
        if (written == -EPIPE) {
            snd_pcm_prepare(pcm);
            continue;
        }
        if (written < 0) {
            int err = snd_pcm_recover(pcm, (int)written, 0);
            if (err < 0) {
                fprintf(stderr, "[audio] write error: %s\n", snd_strerror(err));
                return false;
            }
            continue;
        }
        ptr += written;
        remaining -= written;
    }
    return true;
}

std::vector<int16_t> AudioPlayer::make_tone(float freq_hz, float duration_s, float amplitude)
{
    int n = (int)(sample_rate_ * duration_s);
    std::vector<int16_t> buf(n);
    float scale = amplitude * 32767.0f;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sample_rate_;
        // Apply simple fade-in/out envelope (10% each end)
        float env = 1.0f;
        float fade = 0.1f * duration_s;
        if (t < fade) env = t / fade;
        else if (t > duration_s - fade) env = (duration_s - t) / fade;
        buf[i] = (int16_t)(scale * env * sinf(2.0f * M_PI * freq_hz * t));
    }
    return buf;
}

std::vector<int16_t> AudioPlayer::make_sweep(float f0, float f1, float duration_s, float amplitude)
{
    int n = (int)(sample_rate_ * duration_s);
    std::vector<int16_t> buf(n);
    float scale = amplitude * 32767.0f;
    float fade = 0.05f * duration_s;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sample_rate_;
        float freq = f0 + (f1 - f0) * (t / duration_s);
        float env = 1.0f;
        if (t < fade) env = t / fade;
        else if (t > duration_s - fade) env = (duration_s - t) / fade;
        buf[i] = (int16_t)(scale * env * sinf(2.0f * M_PI * freq * t));
    }
    return buf;
}

std::vector<int16_t> AudioPlayer::make_silence(float duration_s)
{
    int n = (int)(sample_rate_ * duration_s);
    return std::vector<int16_t>(n, 0);
}
