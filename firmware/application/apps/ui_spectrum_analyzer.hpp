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

#ifndef __UI_SPECTRUM_ANALYZER_H__
#define __UI_SPECTRUM_ANALYZER_H__

#include "app_settings.hpp"
#include "baseband_api.hpp"
#include "message.hpp"
#include "radio_state.hpp"
#include "receiver_model.hpp"
#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_widget.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace ui {

/* Swept spectrum analyzer.
 *
 * Deliberately does NOT use the shared spectrum/waterfall widget: that one
 * repaints whole columns every update and is both slow and visibly flickery.
 * The trace here is stored as one y-extent per screen column and redrawn
 * differentially, so a steady signal costs almost no LCD traffic at all.
 *
 * Every slice is tuned so that the LO sits a guard band BELOW the piece of
 * spectrum it contributes (see spec_analyzer::bin_count). The zero-IF DC
 * offset and LO feedthrough therefore never appear on screen, which matters
 * most when the span is small enough to need a single tune: a centred LO would
 * put that artefact exactly on the frequency the user dialled in.
 *
 * Layout is portrait for now (the fonts and the widget set can't rotate);
 * geometry is hard-wired to the 240x320 panel. */
class SpectrumAnalyzerView : public View {
   public:
    SpectrumAnalyzerView(NavigationView& nav);
    ~SpectrumAnalyzerView();

    SpectrumAnalyzerView(const SpectrumAnalyzerView&) = delete;
    SpectrumAnalyzerView& operator=(const SpectrumAnalyzerView&) = delete;

    std::string title() const override { return "Spectrum"; }

    void focus() override;
    void paint(Painter& painter) override;
    void on_show() override;
    void on_hide() override;

   private:
    static constexpr Dim scr_w = 240;
    static constexpr Dim scr_h = 320;

    static constexpr Coord hdr_y = 0;  // 3 rows of 5x8 text
    static constexpr Coord mkr_strip_y = 26;
    static constexpr Dim mkr_strip_h = 6;
    static constexpr Coord grid_y = 32;
    static constexpr Dim grid_h = 160;  // 8 divisions of 20 px
    static constexpr int grid_div_y = 8;
    static constexpr int grid_div_x = 10;
    static constexpr Dim div_h = grid_h / grid_div_y;
    static constexpr Dim div_w = scr_w / grid_div_x;
    static constexpr Coord grid_bottom = grid_y + grid_h - 1;
    static constexpr Coord readout_y = grid_y + grid_h + 2;  // one 8x16 line
    static constexpr Coord peak_y = readout_y + 18;
    static constexpr size_t peak_rows = 5;
    static constexpr Coord ctl_y = peak_y + peak_rows * 8 + 4;

    static constexpr size_t bins = spec_analyzer::bin_count;

    /* Guard between the LO and the lowest reported bin we trust. The DC offset
     * spike only spreads a bin or two, but 1/f noise near DC needs more room. */
    static constexpr size_t usable_lo = 16;

    static constexpr uint32_t fs_min = 1'000'000;
    static constexpr uint32_t fs_max = 20'000'000;
    static constexpr size_t max_slices = 4096;

    /* One column of the trace is worth 0.5 dB, so 12 dB of peak excursion is
     * 24 codes. Anything less and an un-averaged noise floor is all "peaks". */
    static constexpr uint8_t peak_excursion = 24;

    enum TraceMode : uint8_t {
        TraceClearWrite = 0,
        TraceMaxHold = 1,
        TraceMinHold = 2,
        TraceAverage = 3,
    };

    struct Peak {
        int16_t x;
        uint8_t code;
    };

    NavigationView& nav_;
    RxRadioState radio_state_{ReceiverModel::Mode::SpectrumAnalysis};

    /* ---- persisted settings ---- */
    rf::Frequency center_{433'920'000};
    uint64_t span_{20'000'000};
    int32_t rbw_index_{0};  // 0 == auto
    int32_t vbw_index_{0};
    int32_t detector_index_{0};
    int32_t window_index_{1};
    int32_t ref_level_{-60};
    int32_t db_per_div_{10};
    int32_t trace_mode_{TraceClearWrite};
    int32_t cal_offset_{0};
    int32_t settle_us_{300};

    app_settings::SettingsManager settings_{
        "rx_specan"sv,
        app_settings::Mode::RX,
        {
            {"center"sv, &center_},
            {"span"sv, &span_},
            {"rbw"sv, &rbw_index_},
            {"vbw"sv, &vbw_index_},
            {"det"sv, &detector_index_},
            {"win"sv, &window_index_},
            {"ref"sv, &ref_level_},
            {"dbdiv"sv, &db_per_div_},
            {"trace"sv, &trace_mode_},
            {"cal"sv, &cal_offset_},
            {"settle"sv, &settle_us_},
        }};

    /* ---- derived sweep plan ---- */
    uint32_t fs_{20'000'000};
    uint8_t fft_log2_{10};
    uint8_t out_ratio_{2};
    uint8_t averages_{1};
    uint8_t settle_blocks_{3};
    size_t usable_hi_{bins};
    size_t n_slices_{1};
    rf::Frequency f_start_{0};
    int64_t span_eff_{20'000'000};  // span after clamping to the tuning range
    int64_t bin_bw_{0};             // Hz per reported bin (fs * ratio / N)
    int64_t coverage_{0};           // Hz contributed by one slice
    rf::Frequency slice0_{0};       // LO of the first slice
    uint32_t rbw_hz_{0};
    int32_t last_gain_db_{-1000};

    /* ---- sweep state ---- */
    bool running_{false};  // started at the end of the ctor, once the radio is up
    bool active_{false};   // false while another view sits on top of us
    bool awaiting_{false};
    size_t slice_index_{0};
    uint32_t seq_{0};
    uint32_t watchdog_{0};
    uint32_t rate_ref_ms_{0};
    uint32_t rate_sweeps_{0};
    uint32_t sweep_time_ms_{0};
    bool pending_render_{false};
    bool trace_valid_{false};

    /* ---- per-column accumulation for the sweep in progress ---- */
    std::array<uint8_t, scr_w> acc_val_{};
    std::array<uint32_t, scr_w> acc_sum_{};
    std::array<uint16_t, scr_w> acc_cnt_{};

    /* ---- displayed trace, and what is currently on the LCD ---- */
    std::array<uint8_t, scr_w> trace_{};
    std::array<uint16_t, scr_w> avg_acc_{};  // trace_ * 16, so slow drift isn't lost to truncation
    std::array<int16_t, scr_w> drawn_top_{};
    std::array<int16_t, scr_w> drawn_bot_{};
    std::array<uint8_t, 256> code_to_y_{};

    std::array<Peak, peak_rows> peaks_{};
    size_t peak_count_{0};
    size_t marker_x_{scr_w / 2};
    size_t marker_peak_{0};
    int16_t marker_drawn_x_{-1};

    std::string hdr_text_[3]{};
    std::string readout_text_{};
    std::string peak_text_[peak_rows]{};
    std::string marker_label_{};

    /* ---- sweep plumbing ---- */
    void plan_sweep();
    void apply_radio_config();
    void reconfigure();
    void restart_sweep();
    void tune_and_request();
    void on_frame(const SpecAnalyzerFrameMessage& message);
    void accumulate_slice(const SpecAnalyzerFrameMessage& message);
    void finish_sweep();
    void find_peaks();
    bool peak_is_prominent(size_t x, int dir) const;

    /* ---- amplitude helpers (integer only: no FPU on the M0) ---- */
    int32_t gain_db() const;
    int32_t code_to_dbm_x10(uint8_t code) const;
    void rebuild_code_to_y();
    void on_gain_changed();
    void autoscale();

    /* ---- rendering ---- */
    void draw_grid(Painter& painter);
    void draw_header(Painter& painter, bool force);
    void draw_readout(Painter& painter, bool force);
    void draw_peak_table(Painter& painter, bool force);
    void draw_marker(bool force);
    void render_trace();
    void erase_run(Coord x, Coord y0, Coord y1);
    void invalidate_trace();

    rf::Frequency freq_at_x(size_t x) const;
    void set_running(bool run);
    void update_marker_label();
    void on_frame_sync();

    /* ---- widgets ---- */
    FrequencyField field_center{
        {0, ctl_y}};

    OptionsField field_span{
        {88, ctl_y},
        8,
        {}};

    Button button_run{
        {168, ctl_y, 72, 16},
        "RUN"};

    OptionsField field_rbw{
        {0, ctl_y + 16},
        8,
        {
            {"RBW AUTO", 0},
            {"RBW  53k", 1},
            {"RBW  27k", 2},
            {"RBW  13k", 3},
            {"RBW 6.7k", 4},
            {"RBW 2.7k", 5},
            {"RBW 1.3k", 6},
        }};

    OptionsField field_vbw{
        {72, ctl_y + 16},
        7,
        {
            {"VBW=RBW", 1},
            {"VBW/3  ", 3},
            {"VBW/10 ", 10},
            {"VBW/30 ", 30},
            {"VBW/100", 100},
        }};

    OptionsField field_detector{
        {136, ctl_y + 16},
        6,
        {
            {"D:PEAK", 0},
            {"D:AVG ", 1},
            {"D:SMPL", 2},
            {"D:NEG ", 3},
        }};

    OptionsField field_window{
        {192, ctl_y + 16},
        6,
        {
            {"W:RECT", 0},
            {"W:HAMM", 1},
            {"W:BLCK", 2},
            {"W:FLAT", 3},
        }};

    Text label_ref{
        {0, ctl_y + 32, 3 * 8, 16},
        "REF"};

    NumberField field_ref{
        {24, ctl_y + 32},
        4,
        {-120, 30},
        5,
        ' '};

    OptionsField field_db_div{
        {64, ctl_y + 32},
        5,
        {
            {" 1dB/", 1},
            {" 2dB/", 2},
            {" 5dB/", 5},
            {"10dB/", 10},
        }};

    OptionsField field_trace{
        {112, ctl_y + 32},
        6,
        {
            {"T:CLRW", TraceClearWrite},
            {"T:MAXH", TraceMaxHold},
            {"T:MINH", TraceMinHold},
            {"T:AVG ", TraceAverage},
        }};

    NumberField field_cal{
        {168, ctl_y + 32},
        4,
        {-60, 60},
        1,
        ' '};

    OptionsField field_settle{
        {208, ctl_y + 32},
        4,
        {
            {"S.1m", 100},
            {"S.3m", 300},
            {"S1ms", 1000},
            {"S3ms", 3000},
        }};

    Text label_lna{
        {0, ctl_y + 48, 8, 16},
        "L"};

    LNAGainField field_lna{
        {8, ctl_y + 48}};

    Text label_vga{
        {32, ctl_y + 48, 8, 16},
        "V"};

    VGAGainField field_vga{
        {40, ctl_y + 48}};

    Text label_amp{
        {64, ctl_y + 48, 8, 16},
        "A"};

    RFAmpField field_amp{
        {72, ctl_y + 48}};

    ButtonWithEncoder button_marker{
        {88, ctl_y + 48, 104, 16},
        "MKR"};

    Button button_auto{
        {192, ctl_y + 48, 48, 16},
        "AUTO"};

    MessageHandlerRegistration message_handler_frame{
        Message::ID::SpecAnalyzerFrame,
        [this](const Message* const p) {
            this->on_frame(*reinterpret_cast<const SpecAnalyzerFrameMessage*>(p));
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_frame_sync();
        }};
};

}  // namespace ui

#endif /*__UI_SPECTRUM_ANALYZER_H__*/
