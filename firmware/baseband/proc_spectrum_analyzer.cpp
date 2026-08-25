/*
 * Copyright (C) 2026 zxkmm
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_spectrum_analyzer.hpp"

#include "dsp_fft.hpp"  // for the fma-friendly std::complex<float> operator*
#include "event_m4.hpp"
#include "portapack_shared_memory.hpp"
#include "utility.hpp"

#include <algorithm>

namespace {

/* Radix-2 DIT butterflies over bit-reversed input, runtime length.
 * Same trigonometric recurrence as dsp_fft.hpp's fft_c_preswapped(), but the
 * size is a run-time parameter here: RBW is chosen by picking the FFT length,
 * so 256/512/1024 all have to be reachable without three code copies. */
constexpr std::complex<float> wp_table[10] = {
    {-2.0f, 0.0f},                                             // 2
    {-1.0f, -1.0f},                                            // 4
    {-0.2928932188134524756f, -0.7071067811865475244f},        // 8
    {-0.076120467488713243872f, -0.38268343236508977173f},     // 16
    {-0.019214719596769550874f, -0.19509032201612826785f},     // 32
    {-0.0048152733278031137552f, -0.098017140329560601994f},   // 64
    {-0.0012045437948276072852f, -0.049067674327418014255f},   // 128
    {-0.00030118130379577988423f, -0.024541228522912288032f},  // 256
    {-0.000075298160855458470f, -0.012271538285719926079f},    // 512
    {-0.0000188247173988573380f, -0.0061358846491544753597f},  // 1024
};

void fft_dit_preswapped(std::complex<float>* const data, const size_t log2_n) {
    const size_t n = 1u << log2_n;

    for (size_t k = 0; k < log2_n; k++) {
        const size_t mmax = 1u << k;
        const auto wp = wp_table[k];
        std::complex<float> w{1.0f, 0.0f};
        for (size_t m = 0; m < mmax; ++m) {
            for (size_t i = m; i < n; i += mmax * 2) {
                const size_t j = i + mmax;
                const auto temp = w * data[j];
                data[j] = data[i] - temp;
                data[i] += temp;
            }
            w += w * wp;
        }
    }
}

/* Five-term flat-top (MATLAB flattopwin). Frequency-domain taps are a0 for the
 * bin itself and -/+ a_k/2 for the k-th neighbours. */
constexpr float ft_a0 = 0.21557895f;
constexpr float ft_a1 = 0.41663158f / 2.0f;
constexpr float ft_a2 = 0.277263158f / 2.0f;
constexpr float ft_a3 = 0.083578947f / 2.0f;
constexpr float ft_a4 = 0.006947368f / 2.0f;

/* DC gain of the equivalent time-domain window: a tone's peak bin is scaled by
 * this, so it has to be divided back out for the level readout to be right. */
float coherent_gain(const spec_analyzer::Window window) {
    switch (window) {
        case spec_analyzer::Window::Hamming:
            return 0.54f;
        case spec_analyzer::Window::Blackman:
            return 0.42f;
        case spec_analyzer::Window::FlatTop:
            return ft_a0;
        default:
            return 1.0f;
    }
}

}  // namespace

void SpectrumAnalyzerProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured || !acquiring)
        return;

    /* Let the PLL settle: these buffers were sampled while (or just after) the
     * M0 was writing the synthesizer, so they belong to no frequency at all. */
    if (settle_remaining) {
        settle_remaining--;
        return;
    }

    /* Idle thread hasn't consumed the previous block yet. */
    if (capture_full)
        return;

    const size_t n = std::min(fft_size - fill, buffer.count);
    for (size_t i = 0; i < n; i++) {
        capture[fill + i] = {
            static_cast<int16_t>(buffer.p[i].real()),
            static_cast<int16_t>(buffer.p[i].imag())};
    }
    fill += n;

    if (fill >= fft_size) {
        fill = 0;
        capture_full = true;
        EventDispatcher::events_flag(EVT_MASK_SPECTRUM);
    }
}

void SpectrumAnalyzerProcessor::accumulate_power() {
    const size_t n = fft_size;
    const size_t mask = n - 1;
    const size_t shift = 32 - fft_log2;

    for (size_t i = 0; i < n; i++) {
        const size_t i_rev = __RBIT(i) >> shift;
        transform[i_rev] = {
            static_cast<float>(capture[i].real()),
            static_cast<float>(capture[i].imag())};
    }

    fft_dit_preswapped(transform.data(), fft_log2);

    /* Windowing as a convolution of the (unwindowed) spectrum with the window's
     * few non-zero frequency-domain taps. Costs a handful of flops per bin
     * instead of a full-length time-domain multiply plus its coefficient table. */
    switch (window) {
        case spec_analyzer::Window::None:
            for (size_t i = 0; i < n; i++)
                power[i] += magnitude_squared(transform[i]);
            break;

        case spec_analyzer::Window::Blackman:
            for (size_t i = 0; i < n; i++) {
                const auto c = transform[i] * 0.42f -
                               (transform[(i - 1) & mask] + transform[(i + 1) & mask]) * 0.25f +
                               (transform[(i - 2) & mask] + transform[(i + 2) & mask]) * 0.04f;
                power[i] += magnitude_squared(c);
            }
            break;

        case spec_analyzer::Window::FlatTop:
            for (size_t i = 0; i < n; i++) {
                const auto c = transform[i] * ft_a0 -
                               (transform[(i - 1) & mask] + transform[(i + 1) & mask]) * ft_a1 +
                               (transform[(i - 2) & mask] + transform[(i + 2) & mask]) * ft_a2 -
                               (transform[(i - 3) & mask] + transform[(i + 3) & mask]) * ft_a3 +
                               (transform[(i - 4) & mask] + transform[(i + 4) & mask]) * ft_a4;
                power[i] += magnitude_squared(c);
            }
            break;

        case spec_analyzer::Window::Hamming:
        default:
            for (size_t i = 0; i < n; i++) {
                const auto c = transform[i] * 0.54f -
                               (transform[(i - 1) & mask] + transform[(i + 1) & mask]) * 0.23f;
                power[i] += magnitude_squared(c);
            }
            break;
    }
}

void SpectrumAnalyzerProcessor::emit_frame() {
    const size_t n = fft_size;
    const size_t mask = n - 1;
    const size_t ratio = out_ratio;

    /* Full-scale (|I|=|Q|=127) complex tone must land at 0 dBFS whatever the
     * FFT length, averaging depth or window is, so the marker readout stays
     * comparable across settings. */
    const float amplitude_scale = 1.0f / (static_cast<float>(n) * 128.0f * coherent_gain(window));
    const float k = amplitude_scale * amplitude_scale / static_cast<float>(averages);
    const float inv_ratio = 1.0f / static_cast<float>(ratio);

    SpecAnalyzerFrameMessage message;
    message.seq = pending_seq;

    for (size_t j = 0; j < out_bins; j++) {
        /* Upper sideband only. In natural FFT order bin 0 is DC and bins
         * 1..N/2-1 are the positive offsets, so reported bin j starts at
         * natural bin j*ratio, i.e. offset j*ratio*fs/N above the LO. Nothing
         * below the LO (bins N/2..N-1) is ever reported, which is what keeps
         * the DC offset / LO feedthrough off the display entirely.
         * See spec_analyzer::bin_count. */
        const size_t base = (j * ratio) & mask;

        float v = power[base];
        if (ratio > 1) {
            switch (detector) {
                case spec_analyzer::Detector::Sample:
                    break;

                case spec_analyzer::Detector::Average:
                    for (size_t r = 1; r < ratio; r++)
                        v += power[(base + r) & mask];
                    v *= inv_ratio;
                    break;

                case spec_analyzer::Detector::NegPeak:
                    for (size_t r = 1; r < ratio; r++)
                        v = std::min(v, power[(base + r) & mask]);
                    break;

                case spec_analyzer::Detector::Peak:
                default:
                    for (size_t r = 1; r < ratio; r++)
                        v = std::max(v, power[(base + r) & mask]);
                    break;
            }
        }

        float mag2 = v * k;
        if (mag2 < 1.0e-14f)
            mag2 = 1.0e-14f;  // keeps fast_log2() away from denormals

        const float dbfs = mag2_to_dbv_norm(mag2);
        const int value = static_cast<int>((dbfs - spec_analyzer::db_offset) / spec_analyzer::db_per_lsb + 0.5f);
        message.db[j] = static_cast<uint8_t>(clip<int>(value, 0, 255));
    }

    shared_memory.application_queue.push(message);
}

void SpectrumAnalyzerProcessor::on_update() {
    if (!capture_full)
        return;

    accumulate_power();
    averages_done++;

    if (averages_done >= averages) {
        acquiring = false;
        capture_full = false;
        emit_frame();
    } else {
        capture_full = false;
    }
}

void SpectrumAnalyzerProcessor::on_config(const SpecAnalyzerConfigMessage& message) {
    acquiring = false;
    capture_full = false;

    baseband_fs = message.sampling_rate;
    fft_log2 = clip<size_t>(message.fft_size_log2, 9, 10);
    fft_size = 1u << fft_log2;

    /* Only the upper sideband is reported, so the reported window has to fit
     * inside the positive half of the transform. */
    out_ratio = clip<size_t>(message.out_ratio, 1, (fft_size / 2) / out_bins);

    averages = std::max<size_t>(1, message.averages);
    settle_blocks = message.settle_blocks;
    window = message.window;
    detector = message.detector;

    baseband_thread.set_sampling_rate(baseband_fs);

    fill = 0;
    averages_done = 0;
    configured = true;
}

void SpectrumAnalyzerProcessor::on_request(const SpecAnalyzerRequestMessage& message) {
    if (!configured)
        return;

    acquiring = false;  // abandon anything still in flight (e.g. a retried request)
    pending_seq = message.seq;
    fill = 0;
    averages_done = 0;
    std::fill(power.begin(), power.begin() + fft_size, 0.0f);
    settle_remaining = settle_blocks;
    capture_full = false;
    acquiring = true;  // must be last: arms execute()
}

void SpectrumAnalyzerProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::UpdateSpectrum:
            on_update();
            break;

        case Message::ID::SpecAnalyzerConfig:
            on_config(*reinterpret_cast<const SpecAnalyzerConfigMessage*>(message));
            break;

        case Message::ID::SpecAnalyzerRequest:
            on_request(*reinterpret_cast<const SpecAnalyzerRequestMessage*>(message));
            break;

        default:
            break;
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<SpectrumAnalyzerProcessor>()};
    event_dispatcher.run();
    return 0;
}
