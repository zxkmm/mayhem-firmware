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

#ifndef __PROC_SPECTRUM_ANALYZER_H__
#define __PROC_SPECTRUM_ANALYZER_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "message.hpp"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>

/* Swept-FFT spectrum analyzer front end.
 *
 * One measurement == settle_blocks discarded DMA buffers, then 'averages'
 * FFTs of 'fft_size' points accumulated in the power domain, reduced to
 * spec_analyzer::bin_count DC-centered bins with the selected detector.
 *
 * Sample capture happens in execute() (baseband thread, must be quick); the
 * transform runs in the idle thread via EVT_MASK_SPECTRUM, same as the stock
 * SpectrumCollector. Nothing is captured unless the M0 asked for it, so the
 * M4 never spends cycles on data that will be thrown away after a retune. */
class SpectrumAnalyzerProcessor : public BasebandProcessor {
   public:
    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr size_t max_fft_size = 1024;
    static constexpr size_t out_bins = spec_analyzer::bin_count;

    bool configured{false};
    uint32_t baseband_fs{20'000'000};

    size_t fft_size{512};
    size_t fft_log2{9};
    size_t out_ratio{1};
    size_t averages{1};
    size_t settle_blocks{1};
    spec_analyzer::Window window{spec_analyzer::Window::Hamming};
    spec_analyzer::Detector detector{spec_analyzer::Detector::Peak};

    /* Acquisition state. 'capture_full' hands the buffer between the baseband
     * thread (writer) and the idle thread (reader). */
    volatile bool acquiring{false};
    volatile bool capture_full{false};
    size_t settle_remaining{0};
    size_t fill{0};
    size_t averages_done{0};
    uint32_t pending_seq{0};

    std::array<complex16_t, max_fft_size> capture{};
    std::array<std::complex<float>, max_fft_size> transform{};
    std::array<float, max_fft_size> power{};

    void on_config(const SpecAnalyzerConfigMessage& message);
    void on_request(const SpecAnalyzerRequestMessage& message);
    void on_update();

    void accumulate_power();
    void emit_frame();

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_SPECTRUM_ANALYZER_H__*/
