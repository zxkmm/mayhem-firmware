/*
 * Copyright (C) 2017 Jared Boone, ShareBrained Technology, Inc.
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

#include "backlight.hpp"

#include "portapack_io.hpp"

namespace portapack {

namespace {

BacklightOnOffPWM* pwm_instance = nullptr;

}  // namespace

void BacklightOnOff::on() {
    if (!is_on()) {
        io.lcd_backlight(true);
        on_ = true;
    }
}

void BacklightOnOff::off() {
    if (is_on()) {
        io.lcd_backlight(false);
        on_ = false;
    }
}

BacklightOnOffPWM::BacklightOnOffPWM() {
    pwm_instance = this;
}

void backlight_pwm_tick() {
    BacklightOnOffPWM::tick_isr();
}

void BacklightOnOffPWM::tick_isr() {
    if (pwm_instance == nullptr || !pwm_instance->pwm_active_) return;
    auto& self = *pwm_instance;

    self.phase_ = (self.phase_ + 1) & 7;
    const bool high = self.phase_ < duty_phases[self.level_];
    if (high != self.output_high_) {
        self.output_high_ = high;
        self.drive(high);
    }
}

void BacklightOnOffPWM::set_level(const value_t value) {
    auto target = value;
    if (target < 0) target = 0;
    if (target > maximum_level) target = maximum_level;

    level_ = target;
    update_output();
}

void BacklightOnOffPWM::on() {
    if (!is_on()) {
        on_ = true;
        update_output();
    }
}

void BacklightOnOffPWM::off() {
    if (is_on()) {
        on_ = false;
        update_output();
    }
}

void BacklightOnOffPWM::set_fake_brightness(const uint8_t level) {
    /* level: 0 = disabled (100%), 1 = 50%, 2 = 25%, 3 = 12.5%.
     * Our brightness levels run the opposite way: higher = brighter. */
    set_level(level == 0 ? maximum_level
                         : static_cast<value_t>(maximum_level - level));
}

void BacklightOnOffPWM::update_output() {
    if (!on_) {
        pwm_active_ = false;
        output_high_ = false;
        drive(false);
        return;
    }

    if (level_ >= maximum_level) {
        // Full brightness: solid drive, no cycling.
        pwm_active_ = false;
        output_high_ = true;
        drive(true);
        return;
    }

    // Dimming: start a cycle with the line driven high; the tick ISR
    // alternates it per phase from here on.
    pwm_active_ = true;
    phase_ = 0;
    output_high_ = true;
    drive(true);
}

void BacklightOnOffPWM::drive(const bool high) {
    io.lcd_backlight_isr(high);
}

void BacklightCAT4004::set_level(const value_t value) {
    auto target = value;

    // Clip target value to valid range.
    if (target < 0) {
        target = 0;
    }
    if (target > maximum_level) {
        target = maximum_level;
    }

    if (is_on()) {
        pulses(target);
    } else {
        level_ = target;
    }
}

void BacklightCAT4004::on() {
    if (!is_on()) {
        io.lcd_backlight(true);
        halPolledDelay(ticks_setup);
        on_ = true;

        // Just enabled driver, initial value is maximum.
        const auto target_level = level();
        level_ = maximum_level;

        pulses(target_level);
    }
}

void BacklightCAT4004::off() {
    if (is_on()) {
        io.lcd_backlight(false);
        chThdSleepMilliseconds(ms_pwrdwn);
        on_ = false;
    }
}

void BacklightCAT4004::set_fake_brightness(const uint8_t level) {
    /* Real dimming via the CAT4004 pulse-count interface.
     * level: 0 = disabled (100%), 1 = 50%, 2 = 25%, 3 = 12.5%. */
    switch (level) {
    case 1:
        set_level(maximum_level / 2);
        break;
    case 2:
        set_level(maximum_level / 4);
        break;
    case 3:
        set_level(maximum_level / 8);
        break;
    default:
        set_level(maximum_level);
        break;
    }
}

void BacklightCAT4004::pulses(value_t target) {
    while (level() != target) {
        pulse();
    }
}

void BacklightCAT4004::pulse() {
    io.lcd_backlight(false);
    halPolledDelay(ticks_lo);
    io.lcd_backlight(true);
    halPolledDelay(ticks_hi);

    level_ -= 1;
    if (level_ < 0) {
        level_ = levels() - 1;
    }
}

} /* namespace portapack */
