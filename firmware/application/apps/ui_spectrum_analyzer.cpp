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

#include "ui_spectrum_analyzer.hpp"

#include "audio.hpp"
#include "max283x.hpp"
#include "portapack.hpp"
#include "radio.hpp"
#include "string_format.hpp"
#include "theme.hpp"
#include "utility.hpp"

#include <algorithm>

extern ui::SystemView* system_view_ptr;

using namespace portapack;

namespace ui {

namespace {

constexpr Color color_bg = Color::black();
constexpr Color color_grid = Color::dark_grey();
constexpr Color color_trace = Color::yellow();
constexpr Color color_marker = Color::red();
constexpr Color color_text = Color::grey();
constexpr Color color_text_hi = Color::white();

/* Equivalent-noise-bandwidth of each window, in FFT bins, scaled by 1000. Used
 * to turn the transform's bin spacing into a resolution bandwidth the way a
 * real analyzer quotes it. Order matches spec_analyzer::Window. */
constexpr uint32_t enbw_x1000[4] = {1000, 1363, 1727, 3770};

/* Samples per DMA buffer handed to the baseband processor. Settle time is
 * quantised to this, since that is the granularity at which the M4 can drop
 * data captured while the synthesiser was still moving. */
constexpr uint32_t samples_per_block = 2048;

/* Manual RBW choices, as (sample rate, FFT length, FFT bins per reported bin).
 *
 * Only the positive half of the transform is reported, so bin_count*ratio must
 * fit in N/2. A narrower RBW means either a longer transform or a narrower
 * slice, and a narrower slice means more retunes for the same span: that is
 * the RBW / sweep-time trade of a swept analyzer, made explicit.
 * Index 0 is "auto" and is never read from here. */
struct RbwOption {
    uint32_t fs;
    uint8_t log2n;
    uint8_t ratio;
};

constexpr RbwOption rbw_table[7] = {
    {20'000'000, 10, 2},  // auto placeholder
    {20'000'000, 9, 1},   // ~53 kHz, cheapest transform
    {20'000'000, 10, 2},  // ~27 kHz
    {10'000'000, 10, 2},  // ~13 kHz
    {5'000'000, 10, 2},   // ~6.7 kHz
    {2'000'000, 10, 2},   // ~2.7 kHz
    {1'000'000, 10, 2},   // ~1.3 kHz
};

const Style style_small{
    .font = font::fixed_5x8,
    .background = color_bg,
    .foreground = color_text,
};

const Style style_small_hi{
    .font = font::fixed_5x8,
    .background = color_bg,
    .foreground = color_trace,
};

/* The whole screen is an instrument display: force a black canvas so the
 * control fields at the bottom match the graticule instead of the theme. */
const Style style_view{
    .font = font::fixed_8x16,
    .background = color_bg,
    .foreground = color_text_hi,
};

/* Highest reported bin we trust, for a given transform length and reduction.
 * 13/32 of the sample rate keeps the top of the band clear of the anti-alias
 * filter's corner, where both the roll-off and folded energy live. */
size_t usable_hi_for(uint8_t log2n, uint8_t ratio) {
    const size_t limit = (13u * (1u << log2n)) / (32u * ratio);
    return std::min<size_t>(spec_analyzer::bin_count, limit);
}

/* Smallest MAX283x LPF that still passes the whole slice. */
uint32_t filter_bandwidth_for(uint32_t fs) {
    for (const auto bw : max283x::filter::bandwidths) {
        if (bw >= fs)
            return bw;
    }
    return max283x::filter::bandwidth_maximum;
}

std::string format_hz(uint64_t hz) {
    if (hz >= 1'000'000'000)
        return to_string_decimal(hz / 1.0e9f, 3) + "G";
    if (hz >= 1'000'000)
        return to_string_decimal(hz / 1.0e6f, 3) + "M";
    if (hz >= 1'000)
        return to_string_decimal(hz / 1.0e3f, 1) + "k";
    return to_string_dec_uint(hz);
}

std::string format_dbm_x10(int32_t v) {
    const bool neg = v < 0;
    const int32_t a = neg ? -v : v;
    return std::string(neg ? "-" : "") + to_string_dec_uint(a / 10) + "." + to_string_dec_uint(a % 10);
}

}  // namespace

/* ------------------------------------------------------------------------ */
/* Construction                                                             */
/* ------------------------------------------------------------------------ */

SpectrumAnalyzerView::SpectrumAnalyzerView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_image(portapack::spi_flash::image_tag_spec_analyzer);

    set_style(&style_view);

    /* Settings come back from the SD card unvalidated, and several of them
     * index tables directly. */
    rbw_index_ = clip<int32_t>(rbw_index_, 0, 6);
    detector_index_ = clip<int32_t>(detector_index_, 0, 3);
    window_index_ = clip<int32_t>(window_index_, 0, 3);
    trace_mode_ = clip<int32_t>(trace_mode_, TraceClearWrite, TraceAverage);
    ref_level_ = clip<int32_t>(ref_level_, -120, 30);
    cal_offset_ = clip<int32_t>(cal_offset_, -60, 60);
    if (db_per_div_ != 1 && db_per_div_ != 2 && db_per_div_ != 5 && db_per_div_ != 10)
        db_per_div_ = 10;
    if (span_ < 100'000)
        span_ = 100'000;
    if (settle_us_ != 100 && settle_us_ != 300 && settle_us_ != 1000 && settle_us_ != 3000)
        settle_us_ = 300;

    add_children({&field_center,
                  &field_span,
                  &button_run,
                  &field_rbw,
                  &field_vbw,
                  &field_detector,
                  &field_window,
                  &label_ref,
                  &field_ref,
                  &field_db_div,
                  &field_trace,
                  &field_cal,
                  &field_settle,
                  &label_lna,
                  &field_lna,
                  &label_vga,
                  &field_vga,
                  &label_amp,
                  &field_amp,
                  &button_marker,
                  &button_auto});

    field_span.set_options({
        {"SP 100k", 100},
        {"SP 200k", 200},
        {"SP 500k", 500},
        {"SP   1M", 1'000},
        {"SP   2M", 2'000},
        {"SP   5M", 5'000},
        {"SP  10M", 10'000},
        {"SP  20M", 20'000},
        {"SP  50M", 50'000},
        {"SP 100M", 100'000},
        {"SP 200M", 200'000},
        {"SP 500M", 500'000},
        {"SP   1G", 1'000'000},
        {"SP   2G", 2'000'000},
        {"SP  MAX", 7'000'000},
    });

    field_center.set_value(center_);
    field_center.on_change = [this](rf::Frequency f) {
        center_ = f;
        reconfigure();
    };
    field_center.on_edit = [this]() {
        auto keypad = nav_.push<FrequencyKeypadView>(center_);
        keypad->on_changed = [this](rf::Frequency f) {
            field_center.set_value(f);  // fires on_change, which replans the sweep
        };
    };

    field_span.set_by_nearest_value(static_cast<int32_t>(span_ / 1000));
    field_span.on_change = [this](size_t, OptionsField::value_t v) {
        span_ = static_cast<uint64_t>(v) * 1000u;
        reconfigure();
    };

    field_rbw.set_by_value(rbw_index_);
    field_rbw.on_change = [this](size_t, OptionsField::value_t v) {
        rbw_index_ = v;
        reconfigure();
    };

    field_vbw.set_by_value(std::max<int32_t>(1, vbw_index_));
    field_vbw.on_change = [this](size_t, OptionsField::value_t v) {
        vbw_index_ = v;
        reconfigure();
    };

    field_detector.set_by_value(detector_index_);
    field_detector.on_change = [this](size_t, OptionsField::value_t v) {
        detector_index_ = v;
        reconfigure();
    };

    field_window.set_by_value(window_index_);
    field_window.on_change = [this](size_t, OptionsField::value_t v) {
        window_index_ = v;
        reconfigure();
    };

    field_settle.set_by_nearest_value(settle_us_);
    field_settle.on_change = [this](size_t, OptionsField::value_t v) {
        settle_us_ = v;
        reconfigure();
    };

    field_ref.set_value(ref_level_);
    field_ref.on_change = [this](int32_t v) {
        ref_level_ = v;
        rebuild_code_to_y();
        invalidate_trace();
        set_dirty();
    };

    field_db_div.set_by_value(db_per_div_);
    field_db_div.on_change = [this](size_t, OptionsField::value_t v) {
        db_per_div_ = v;
        rebuild_code_to_y();
        invalidate_trace();
        set_dirty();
    };

    field_trace.set_by_value(trace_mode_);
    field_trace.on_change = [this](size_t, OptionsField::value_t v) {
        trace_mode_ = v;
        trace_valid_ = false;
    };

    field_cal.set_value(cal_offset_);
    field_cal.on_change = [this](int32_t v) {
        cal_offset_ = v;
        rebuild_code_to_y();
        invalidate_trace();
        set_dirty();
    };

    /* These override the handlers the gain fields install for themselves, so
     * the receiver_model call has to be repeated here. Amplitude is referred to
     * the antenna port, so the whole dB axis moves when the gain does. */
    field_lna.on_change = [this](int32_t v) {
        receiver_model.set_lna(v);
        on_gain_changed();
    };
    field_vga.on_change = [this](int32_t v) {
        receiver_model.set_vga(v);
        on_gain_changed();
    };
    field_amp.on_change = [this](int32_t v) {
        receiver_model.set_rf_amp(v);
        on_gain_changed();
    };

    button_run.on_select = [this](Button&) {
        set_running(!running_);
    };

    button_auto.on_select = [this](Button&) {
        autoscale();
    };

    /* The marker rides the encoder while this button holds focus; pressing it
     * steps through the peak table, like a "next peak" softkey. */
    button_marker.on_change = [this]() {
        const int32_t delta = button_marker.get_encoder_delta();
        button_marker.set_encoder_delta(0);
        if (delta == 0)
            return;
        const int32_t x = static_cast<int32_t>(marker_x_) + delta;
        marker_x_ = static_cast<size_t>(clip<int32_t>(x, 0, scr_w - 1));
        update_marker_label();
        draw_marker(false);
        Painter painter;
        draw_readout(painter, false);
    };

    button_marker.on_select = [this](ButtonWithEncoder&) {
        if (peak_count_ == 0)
            return;
        marker_peak_ = (marker_peak_ + 1) % peak_count_;
        marker_x_ = static_cast<size_t>(peaks_[marker_peak_].x);
        update_marker_label();
        draw_marker(false);
        Painter painter;
        draw_readout(painter, false);
        draw_peak_table(painter, false);
    };

    system_view_ptr->set_app_fullscreen(true);

    plan_sweep();
    rebuild_code_to_y();
    invalidate_trace();
    update_marker_label();

    audio::output::stop();  // nothing here makes sound; don't inherit the last app's
    receiver_model.set_target_frequency(center_);
    receiver_model.set_squelch_level(0);
    receiver_model.enable();
    apply_radio_config();
    set_running(true);  // actually starts once on_show() runs
}

SpectrumAnalyzerView::~SpectrumAnalyzerView() {
    running_ = false;
    receiver_model.set_sampling_rate(3'072'000);  // don't leave a 20 MHz rate behind
    receiver_model.disable();
    baseband::shutdown();
    system_view_ptr->set_app_fullscreen(false);
}

void SpectrumAnalyzerView::focus() {
    field_center.focus();
}

/* on_show()/on_hide() fire when the view is attached to / detached from the
 * widget tree, which is what happens when a modal (the frequency keypad, say)
 * is pushed over us. Both the sweep and the direct-to-LCD drawing have to stop
 * for that, otherwise we scribble over whatever is on top. */
void SpectrumAnalyzerView::on_show() {
    active_ = true;
    invalidate_trace();
    marker_drawn_x_ = -1;
    if (running_ && !awaiting_)
        restart_sweep();
}

void SpectrumAnalyzerView::on_hide() {
    active_ = false;
    awaiting_ = false;
}

/* ------------------------------------------------------------------------ */
/* Sweep planning                                                           */
/* ------------------------------------------------------------------------ */

void SpectrumAnalyzerView::plan_sweep() {
    const rf::Frequency f_lo = rf::tuning_range.minimum;
    const rf::Frequency f_hi = rf::tuning_range.maximum;

    int64_t span = static_cast<int64_t>(span_);
    if (span > (f_hi - f_lo))
        span = f_hi - f_lo;

    rf::Frequency start = center_ - span / 2;
    if (start < f_lo)
        start = f_lo;
    if (start + span > f_hi)
        start = f_hi - span;

    f_start_ = start;
    span_eff_ = span;

    /* Pick the transform geometry FIRST and derive everything from it once:
     * bin_bw_, coverage_ and slice0_ must describe exactly the configuration
     * the M4 is given, or the frequency axis silently lies. */
    if (rbw_index_ == 0) {
        /* Auto. Prefer the un-reduced transform: it puts 240 bins across the
         * slice at the finest spacing this scheme can produce. Fall back to the
         * 2:1 reduction when the span is too wide for that to fit one tune,
         * since it doubles the slice width for the same sample rate. */
        fft_log2_ = 10;
        out_ratio_ = 1;
        size_t usable_bins = usable_hi_for(fft_log2_, out_ratio_) - usable_lo;
        int64_t want_fs = (span << fft_log2_) / (static_cast<int64_t>(out_ratio_) * usable_bins) + 1024;

        if (want_fs > fs_max) {
            out_ratio_ = 2;
            usable_bins = usable_hi_for(fft_log2_, out_ratio_) - usable_lo;
            want_fs = (span << fft_log2_) / (static_cast<int64_t>(out_ratio_) * usable_bins) + 1024;
        }

        fs_ = static_cast<uint32_t>(clip<int64_t>(want_fs, fs_min, fs_max));

        /* A wide sweep spends most of its time in the transform, so drop to
         * the cheap one once the slice count makes that the dominant cost.
         * 512/x1 and 1024/x2 cover the same fraction of the band, so this
         * costs resolution bandwidth and nothing else. */
        if (out_ratio_ == 2) {
            const int64_t est_cov =
                static_cast<int64_t>(usable_bins) * fs_ * out_ratio_ / (1 << fft_log2_);
            if (est_cov > 0 && (span + est_cov - 1) / est_cov > 24) {
                fft_log2_ = 9;
                out_ratio_ = 1;
            }
        }
    } else {
        fs_ = rbw_table[rbw_index_].fs;
        fft_log2_ = rbw_table[rbw_index_].log2n;
        out_ratio_ = rbw_table[rbw_index_].ratio;
    }

    usable_hi_ = usable_hi_for(fft_log2_, out_ratio_);

    const int64_t n = static_cast<int64_t>(1u << fft_log2_);
    bin_bw_ = (static_cast<int64_t>(fs_) * out_ratio_ + n / 2) / n;
    if (bin_bw_ < 1)
        bin_bw_ = 1;

    coverage_ = static_cast<int64_t>(usable_hi_ - usable_lo) * bin_bw_;
    n_slices_ = static_cast<size_t>((span + coverage_ - 1) / coverage_);
    if (n_slices_ < 1)
        n_slices_ = 1;
    if (n_slices_ > max_slices)
        n_slices_ = max_slices;

    /* The LO sits a guard band below the first bin we use, so DC never lands
     * inside the displayed span. */
    slice0_ = f_start_ - static_cast<int64_t>(usable_lo) * bin_bw_;

    averages_ = static_cast<uint8_t>(clip<int32_t>(vbw_index_ == 0 ? 1 : vbw_index_, 1, 100));

    const uint32_t blocks = static_cast<uint32_t>(
        (static_cast<uint64_t>(settle_us_) * fs_ + (samples_per_block * 1'000'000ull - 1)) /
        (samples_per_block * 1'000'000ull));
    settle_blocks_ = static_cast<uint8_t>(clip<uint32_t>(blocks, 1, 255));

    const uint32_t enbw = enbw_x1000[clip<int32_t>(window_index_, 0, 3)];
    rbw_hz_ = static_cast<uint32_t>((static_cast<int64_t>(fs_) * enbw) / (n * 1000));

    if (slice_index_ >= n_slices_)
        slice_index_ = 0;

    field_center.set_step(std::max<rf::Frequency>(1000, span_eff_ / 10));
    update_marker_label();
}

void SpectrumAnalyzerView::apply_radio_config() {
    receiver_model.set_sampling_rate(fs_);
    receiver_model.set_baseband_bandwidth(filter_bandwidth_for(fs_));

    baseband::set_spec_analyzer(
        fs_,
        fft_log2_,
        out_ratio_,
        averages_,
        static_cast<spec_analyzer::Window>(clip<int32_t>(window_index_, 0, 3)),
        static_cast<spec_analyzer::Detector>(clip<int32_t>(detector_index_, 0, 3)),
        settle_blocks_);
}

void SpectrumAnalyzerView::reconfigure() {
    plan_sweep();
    apply_radio_config();
    trace_valid_ = false;
    invalidate_trace();
    set_dirty();
    restart_sweep();
}

void SpectrumAnalyzerView::restart_sweep() {
    slice_index_ = 0;
    acc_val_.fill(0);
    acc_cnt_.fill(0);
    acc_sum_.fill(0);

    if (running_)
        tune_and_request();
}

void SpectrumAnalyzerView::tune_and_request() {
    if (!active_)
        return;  // a modal is up; on_show() will kick the sweep off again

    const rf::Frequency f = slice0_ + static_cast<int64_t>(slice_index_) * coverage_;

    /* Tune the hardware directly: receiver_model would also push the frequency
     * to persistent memory, which is far too slow to do once per slice. */
    radio::set_tuning_frequency(f);

    seq_++;
    awaiting_ = true;
    watchdog_ = 0;
    baseband::request_spec_analyzer_frame(seq_);
}

void SpectrumAnalyzerView::on_frame(const SpecAnalyzerFrameMessage& message) {
    if (message.seq != seq_)
        return;  // stale: belongs to a tuning we have already abandoned

    awaiting_ = false;

    if (!running_ || !active_)
        return;

    accumulate_slice(message);

    slice_index_++;
    if (slice_index_ >= n_slices_) {
        finish_sweep();
        restart_sweep();
    } else {
        tune_and_request();
    }
}

void SpectrumAnalyzerView::accumulate_slice(const SpecAnalyzerFrameMessage& message) {
    /* Reported bin j covers [LO + j*bin_bw, LO + (j+1)*bin_bw): the whole
     * window is above the LO, so bin 0 is DC and the guard band below
     * usable_lo is what keeps the DC artefact off screen.
     *
     * Reporting only the upper sideband also rejects the quadrature image for
     * free: a signal at LO+d mirrors to LO-d, which is in the half of the
     * transform we never look at. */
    const rf::Frequency f_lo = slice0_ + static_cast<int64_t>(slice_index_) * coverage_;
    const int64_t span = span_eff_;
    if (span <= 0)
        return;

    for (size_t j = usable_lo; j < usable_hi_; j++) {
        const int64_t fl = f_lo + static_cast<int64_t>(j) * bin_bw_;
        int64_t xl = ((fl - f_start_) * scr_w) / span;
        int64_t xr = ((fl + bin_bw_ - f_start_) * scr_w) / span;

        if (xr < 0 || xl >= scr_w)
            continue;
        if (xl < 0)
            xl = 0;
        if (xr > scr_w - 1)
            xr = scr_w - 1;

        const uint8_t v = message.db[j];

        for (int64_t x = xl; x <= xr; x++) {
            const size_t xi = static_cast<size_t>(x);
            if (acc_cnt_[xi] == 0) {
                acc_val_[xi] = v;
                acc_sum_[xi] = v;
                acc_cnt_[xi] = 1;
                continue;
            }
            acc_cnt_[xi]++;
            acc_sum_[xi] += v;
            switch (detector_index_) {
                case 0:  // peak
                    if (v > acc_val_[xi])
                        acc_val_[xi] = v;
                    break;
                case 3:  // negative peak
                    if (v < acc_val_[xi])
                        acc_val_[xi] = v;
                    break;
                default:  // sample keeps the first hit, average is resolved later
                    break;
            }
        }
    }
}

void SpectrumAnalyzerView::finish_sweep() {
    /* A narrow-span sweep can finish in well under a millisecond, so time a
     * batch of them instead of quoting a permanent "0ms". */
    const uint32_t now = chTimeNow();
    rate_sweeps_++;
    if (static_cast<uint32_t>(now - rate_ref_ms_) >= 500) {
        sweep_time_ms_ = (now - rate_ref_ms_) / rate_sweeps_;
        rate_ref_ms_ = now;
        rate_sweeps_ = 0;
    }

    if (detector_index_ == 1) {
        for (size_t x = 0; x < scr_w; x++) {
            if (acc_cnt_[x])
                acc_val_[x] = static_cast<uint8_t>(acc_sum_[x] / acc_cnt_[x]);
        }
    }

    /* Columns that no bin landed on (rounding at the sweep edges) hold the
     * value of their nearest populated neighbour rather than reading as -120. */
    size_t first = 0;
    while (first < scr_w && acc_cnt_[first] == 0)
        first++;

    if (first < scr_w) {
        uint8_t last = acc_val_[first];
        for (size_t x = first; x < scr_w; x++) {
            if (acc_cnt_[x])
                last = acc_val_[x];
            else
                acc_val_[x] = last;
        }
        for (size_t x = 0; x < first; x++)
            acc_val_[x] = acc_val_[first];
    }

    if (!trace_valid_) {
        trace_ = acc_val_;
        for (size_t x = 0; x < scr_w; x++)
            avg_acc_[x] = static_cast<uint16_t>(acc_val_[x] * 16);
    } else {
        switch (trace_mode_) {
            case TraceMaxHold:
                for (size_t x = 0; x < scr_w; x++)
                    trace_[x] = std::max(trace_[x], acc_val_[x]);
                break;

            case TraceMinHold:
                for (size_t x = 0; x < scr_w; x++)
                    trace_[x] = std::min(trace_[x], acc_val_[x]);
                break;

            case TraceAverage:
                for (size_t x = 0; x < scr_w; x++) {
                    const int32_t target = static_cast<int32_t>(acc_val_[x]) * 16;
                    const int32_t acc = static_cast<int32_t>(avg_acc_[x]);
                    avg_acc_[x] = static_cast<uint16_t>(acc + (target - acc) / 4);
                    trace_[x] = static_cast<uint8_t>(avg_acc_[x] / 16);
                }
                break;

            case TraceClearWrite:
            default:
                trace_ = acc_val_;
                break;
        }
    }

    trace_valid_ = true;
    find_peaks();
    pending_render_ = true;
}

/* True if the trace falls by at least the peak excursion on the given side of x
 * before it climbs back above trace_[x]. This is what stops an un-averaged
 * noise floor - which routinely wanders 6-10 dB - from filling the peak table.
 * dir is -1 or +1. */
bool SpectrumAnalyzerView::peak_is_prominent(size_t x, int dir) const {
    const uint8_t v = trace_[x];
    uint8_t lowest = v;

    for (int i = static_cast<int>(x) + dir; i >= 0 && i < static_cast<int>(scr_w); i += dir) {
        const uint8_t s = trace_[i];
        if (s > v)
            return false;  // a taller peak owns this slope
        if (s < lowest)
            lowest = s;
        if (v - lowest >= peak_excursion)
            return true;
    }

    // Ran off the end of the trace: accept whatever drop we found.
    return (v - lowest) >= peak_excursion;
}

void SpectrumAnalyzerView::find_peaks() {
    /* The noise floor has to come from a median, not a mean: a strong carrier
     * drags the mean up and the threshold with it. */
    uint16_t histogram[256] = {0};
    for (size_t x = 0; x < scr_w; x++)
        histogram[trace_[x]]++;

    uint32_t seen = 0;
    uint8_t floor_code = 0;
    for (int c = 0; c < 256; c++) {
        seen += histogram[c];
        if (seen >= scr_w / 2) {
            floor_code = static_cast<uint8_t>(c);
            break;
        }
    }

    const uint8_t threshold = static_cast<uint8_t>(
        std::min<uint32_t>(255, static_cast<uint32_t>(floor_code) + peak_excursion));

    peak_count_ = 0;

    for (size_t x = 1; x + 1 < scr_w; x++) {
        const uint8_t v = trace_[x];
        if (v < threshold)
            continue;

        /* Zoomed in, one reported bin is painted across several columns, so a
         * carrier's top is a flat plateau rather than a single column. Accept
         * only the far end of a plateau and then report its midpoint,
         * otherwise a single carrier fills the whole peak table with copies of
         * itself a few pixels apart. */
        if (trace_[x - 1] > v || trace_[x + 1] >= v)
            continue;

        size_t left = x;
        while (left > 0 && trace_[left - 1] == v)
            left--;

        if (!peak_is_prominent(left, -1) || !peak_is_prominent(x, +1))
            continue;

        const int16_t px = static_cast<int16_t>((left + x) / 2);

        if (peak_count_ < peak_rows) {
            peaks_[peak_count_++] = {px, v};
        } else {
            size_t weakest = 0;
            for (size_t i = 1; i < peak_count_; i++) {
                if (peaks_[i].code < peaks_[weakest].code)
                    weakest = i;
            }
            if (v > peaks_[weakest].code)
                peaks_[weakest] = {px, v};
        }
    }

    std::sort(peaks_.begin(), peaks_.begin() + peak_count_,
              [](const Peak& a, const Peak& b) { return a.code > b.code; });

    if (marker_peak_ >= peak_count_)
        marker_peak_ = 0;
}

/* ------------------------------------------------------------------------ */
/* Amplitude helpers                                                        */
/* ------------------------------------------------------------------------ */

int32_t SpectrumAnalyzerView::gain_db() const {
    return receiver_model.lna() + receiver_model.vga() + (receiver_model.rf_amp() ? 14 : 0);
}

int32_t SpectrumAnalyzerView::code_to_dbm_x10(uint8_t code) const {
    /* code is dBFS at 0.5 dB/LSB with a -120 dBFS origin. Referring that back
     * to the antenna port is only ever an estimate: it assumes full scale sits
     * at 0 dBm with no gain, which is what field_cal is there to trim. */
    const int32_t dbfs_x10 = static_cast<int32_t>(code) * 5 - 1200;
    return dbfs_x10 - gain_db() * 10 + cal_offset_ * 10;
}

void SpectrumAnalyzerView::rebuild_code_to_y() {
    const int32_t range_x10 = db_per_div_ * grid_div_y * 10;
    const int32_t bottom_x10 = ref_level_ * 10 - range_x10;

    for (int c = 0; c < 256; c++) {
        const int32_t dbm_x10 = code_to_dbm_x10(static_cast<uint8_t>(c));
        const int32_t y = grid_bottom - ((dbm_x10 - bottom_x10) * static_cast<int32_t>(grid_h)) / range_x10;
        code_to_y_[c] = static_cast<uint8_t>(clip<int32_t>(y, grid_y, grid_bottom));
    }

    last_gain_db_ = gain_db();
}

/* Front-end gain shifts the whole dBm axis. Slide the reference level by the
 * same amount so turning the gain up doesn't walk the trace off the screen. */
void SpectrumAnalyzerView::on_gain_changed() {
    const int32_t gain = gain_db();

    if (last_gain_db_ > -1000 && gain != last_gain_db_) {
        ref_level_ = clip<int32_t>(ref_level_ - (gain - last_gain_db_), -120, 30);
        field_ref.set_value(ref_level_);  // fires on_change: rebuild + repaint
    }

    rebuild_code_to_y();
    invalidate_trace();
    set_dirty();
}

/* Park the strongest point of the trace just under the top graticule line. */
void SpectrumAnalyzerView::autoscale() {
    if (!trace_valid_)
        return;

    const uint8_t peak = *std::max_element(trace_.begin(), trace_.end());
    const int32_t top = code_to_dbm_x10(peak) / 10 + 5;

    ref_level_ = clip<int32_t>(((top + 4) / 5) * 5, -120, 30);
    field_ref.set_value(ref_level_);
}

/* ------------------------------------------------------------------------ */
/* Rendering                                                                */
/* ------------------------------------------------------------------------ */

void SpectrumAnalyzerView::invalidate_trace() {
    drawn_top_.fill(-1);
    drawn_bot_.fill(-1);
}

void SpectrumAnalyzerView::draw_grid(Painter& painter) {
    painter.fill_rectangle({0, grid_y, scr_w, grid_h}, color_bg);

    for (int k = 0; k <= grid_div_y; k++) {
        const Coord y = (k == grid_div_y) ? grid_bottom : (grid_y + k * div_h);
        for (Coord x = 0; x < scr_w; x += 4)
            display.draw_pixel({x, y}, color_grid);
    }

    for (int k = 0; k <= grid_div_x; k++) {
        const Coord x = (k == grid_div_x) ? (scr_w - 1) : (k * div_w);
        for (Coord y = grid_y; y <= grid_bottom; y += 4)
            display.draw_pixel({x, y}, color_grid);
    }
}

void SpectrumAnalyzerView::erase_run(Coord x, Coord y0, Coord y1) {
    if (y1 < y0)
        return;

    display.fill_rectangle({x, y0, 1, y1 - y0 + 1}, color_bg);

    const bool on_vline = ((x % div_w) == 0) || (x == scr_w - 1);
    if (on_vline) {
        for (Coord y = y0; y <= y1; y++) {
            if ((y % 4) == 0)
                display.draw_pixel({x, y}, color_grid);
        }
    }

    if ((x % 4) == 0) {
        for (Coord y = y0; y <= y1; y++) {
            if (((y - grid_y) % div_h) == 0 || y == grid_bottom)
                display.draw_pixel({x, y}, color_grid);
        }
    }
}

void SpectrumAnalyzerView::render_trace() {
    if (!trace_valid_)
        return;

    if (gain_db() != last_gain_db_) {
        /* dBFS -> dBm depends on the front-end gain, so the whole vertical
         * mapping moved. Let paint() clear the graticule and start over rather
         * than differentially erasing against stale y-extents. */
        rebuild_code_to_y();
        set_dirty();
        return;
    }

    int16_t prev_y = code_to_y_[trace_[0]];

    for (Coord x = 0; x < scr_w; x++) {
        const int16_t y = code_to_y_[trace_[x]];
        const int16_t nt = std::min(y, prev_y);
        const int16_t nb = std::max(y, prev_y);
        prev_y = y;

        const int16_t ot = drawn_top_[x];
        const int16_t ob = drawn_bot_[x];

        if (ot == nt && ob == nb)
            continue;

        if (ot < 0 || ob < nt || nb < ot) {
            if (ot >= 0)
                erase_run(x, ot, ob);
            display.fill_rectangle({x, nt, 1, nb - nt + 1}, color_trace);
        } else {
            if (ot < nt)
                erase_run(x, ot, nt - 1);
            if (ob > nb)
                erase_run(x, nb + 1, ob);
            if (nt < ot)
                display.fill_rectangle({x, nt, 1, ot - nt}, color_trace);
            if (nb > ob)
                display.fill_rectangle({x, ob + 1, 1, nb - ob}, color_trace);
        }

        drawn_top_[x] = nt;
        drawn_bot_[x] = nb;
    }
}

void SpectrumAnalyzerView::draw_marker(bool force) {
    const Coord x = static_cast<Coord>(marker_x_);

    if (!force && marker_drawn_x_ == x)
        return;

    if (marker_drawn_x_ >= 0) {
        const Coord ex = std::max<Coord>(0, marker_drawn_x_ - 3);
        display.fill_rectangle({ex, mkr_strip_y, 7, mkr_strip_h}, color_bg);
    }

    for (Dim i = 0; i < mkr_strip_h - 1; i++) {
        const Dim w = 7 - 2 * i;
        if (w < 1)
            break;
        const Coord sx = std::max<Coord>(0, x - 3 + i);
        display.fill_rectangle({sx, mkr_strip_y + i, w, 1}, color_marker);
    }

    marker_drawn_x_ = x;
}

rf::Frequency SpectrumAnalyzerView::freq_at_x(size_t x) const {
    return f_start_ + (static_cast<int64_t>(x) * span_eff_) / scr_w + (span_eff_ / scr_w) / 2;
}

void SpectrumAnalyzerView::update_marker_label() {
    auto text = "M " + to_string_rounded_freq(freq_at_x(marker_x_), 4) + "M";
    if (text == marker_label_)
        return;  // set_text() always repaints, so only call it on a real change
    marker_label_ = text;
    button_marker.set_text(marker_label_);
}

void SpectrumAnalyzerView::draw_header(Painter& painter, bool force) {
    std::string l0 = "CTR " + to_string_rounded_freq(f_start_ + span_eff_ / 2, 4) +
                     "M   SPAN " + format_hz(span_eff_);

    std::string l1 = "RBW " + format_hz(rbw_hz_) +
                     "  VBW " + format_hz(rbw_hz_ / averages_) +
                     "  SWP " + to_string_dec_uint(sweep_time_ms_) + "ms" +
                     "  N" + to_string_dec_uint(n_slices_);

    std::string l2 = "REF " + to_string_dec_int(ref_level_) + "dBm  " +
                     to_string_dec_uint(db_per_div_) + "dB/DIV  " +
                     field_detector.selected_index_name() + "  " +
                     field_trace.selected_index_name() +
                     (running_ ? "" : "  *HOLD*");

    const std::string* lines[3] = {&l0, &l1, &l2};
    for (int i = 0; i < 3; i++) {
        if (!force && hdr_text_[i] == *lines[i])
            continue;
        hdr_text_[i] = *lines[i];
        painter.fill_rectangle({0, hdr_y + i * 8, scr_w, 8}, color_bg);
        painter.draw_string({0, hdr_y + i * 8}, style_small, hdr_text_[i]);
    }
}

void SpectrumAnalyzerView::draw_readout(Painter& painter, bool force) {
    std::string text;
    if (trace_valid_) {
        text = "M " + to_string_rounded_freq(freq_at_x(marker_x_), 4) + "M " +
               format_dbm_x10(code_to_dbm_x10(trace_[marker_x_])) + "dBm";
    } else {
        text = "M --.----M  ---.- dBm";
    }

    if (!force && readout_text_ == text)
        return;

    readout_text_ = text;
    painter.fill_rectangle({0, readout_y, scr_w, 16}, color_bg);
    painter.draw_string({0, readout_y}, font::fixed_8x16, color_text_hi, color_bg, readout_text_);
}

void SpectrumAnalyzerView::draw_peak_table(Painter& painter, bool force) {
    for (size_t i = 0; i < peak_rows; i++) {
        std::string text;
        if (i < peak_count_) {
            text = to_string_dec_uint(i + 1) + " " +
                   to_string_rounded_freq(freq_at_x(peaks_[i].x), 4) + "MHz  " +
                   format_dbm_x10(code_to_dbm_x10(peaks_[i].code)) + "dBm";
        }

        if (!force && peak_text_[i] == text)
            continue;

        peak_text_[i] = text;
        const Coord y = peak_y + static_cast<Coord>(i) * 8;
        painter.fill_rectangle({0, y, scr_w, 8}, color_bg);
        if (!text.empty())
            painter.draw_string({0, y}, (i == marker_peak_) ? style_small_hi : style_small, text);
    }
}

void SpectrumAnalyzerView::paint(Painter& painter) {
    painter.fill_rectangle({0, 0, scr_w, scr_h}, color_bg);

    draw_grid(painter);
    invalidate_trace();
    marker_drawn_x_ = -1;

    draw_header(painter, true);
    draw_readout(painter, true);
    draw_peak_table(painter, true);
    draw_marker(true);
    render_trace();
}

/* ------------------------------------------------------------------------ */
/* Event loop hooks                                                         */
/* ------------------------------------------------------------------------ */

void SpectrumAnalyzerView::on_frame_sync() {
    if (!active_)
        return;

    if (pending_render_) {
        pending_render_ = false;
        Painter painter;
        render_trace();
        draw_header(painter, false);
        draw_readout(painter, false);
        draw_peak_table(painter, false);
        draw_marker(false);
    }

    if (!running_)
        return;

    /* Self-heal: a request can be dropped, and a settings change made while a
     * modal was up leaves the sweep with nothing in flight. */
    watchdog_++;
    if (awaiting_) {
        if (watchdog_ > 30)
            tune_and_request();
    } else if (watchdog_ > 2) {
        restart_sweep();
    }
}

void SpectrumAnalyzerView::set_running(bool run) {
    running_ = run;
    button_run.set_text(run ? "RUN" : "HOLD");

    if (run && !awaiting_)
        restart_sweep();
}

}  // namespace ui
