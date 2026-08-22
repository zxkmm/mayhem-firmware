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

#ifndef __BACKLIGHT_H
#define __BACKLIGHT_H

#include <cstdint>

namespace portapack {

class Backlight {
   public:
    using value_t = int_fast8_t;

    virtual ~Backlight() = default;

    virtual value_t levels() const = 0;

    virtual void set_level(const value_t value) = 0;
    virtual value_t level() const = 0;

    virtual void increase() = 0;
    virtual void decrease() = 0;

    virtual void on() = 0;
    virtual void off() = 0;

    virtual bool is_on() const = 0;

    /* Fake brightness dimming, for drivers that support it.
     * level: 0 = disabled (100%), 1 = 50%, 2 = 25%, 3 = 12.5%.
     * Default is a no-op for drivers without dimming support. */
    virtual void set_fake_brightness(const uint8_t level) {
        (void)level;
    }
};

class BacklightBase : public Backlight {
   public:
    void increase() override {
        set_level(level() + 1);
    }

    void decrease() override {
        set_level(level() - 1);
    }
};

class BacklightOnOff : public BacklightBase {
   public:
    value_t levels() const override {
        return 1;
    }

    void set_level(const value_t) override {
    }

    value_t level() const override {
        return levels() - 1;
    }

    void on() override;
    void off() override;

    bool is_on() const override {
        return on_;
    }

   private:
    static constexpr value_t maximum_level = 1;

    bool on_{false};
};

/* Software-PWM dimming for boards whose backlight line is the plain CPLD
 * register bit (no CAT4004 LED driver). The backlight bit is toggled from the
 * 1 ms controls/touch timer ISR: 8 phases per cycle (125 Hz PWM), giving the
 * fixed 100 / 50 / 25 / 12.5% options with steady, even cycles. */
class BacklightOnOffPWM : public BacklightBase {
   public:
    BacklightOnOffPWM();

    /* Levels are brightness, higher is brighter: 0=12.5%, 1=25%, 2=50%, 3=100%. */
    value_t levels() const override {
        return maximum_level + 1;
    }

    void set_level(const value_t value) override;

    value_t level() const override {
        return level_;
    }

    void on() override;
    void off() override;

    bool is_on() const override {
        return on_;
    }

    void set_fake_brightness(const uint8_t level) override;

    /* Called every 1ms from the controls/touch timer ISR. */
    static void tick_isr();

   private:
    static constexpr value_t maximum_level = 3;

    /* "On" phases of the 8-phase PWM cycle per dim level. */
    static constexpr uint8_t duty_phases[maximum_level] = {1, 2, 4};

    value_t level_{maximum_level};
    bool on_{false};
    bool output_high_{false};
    bool pwm_active_{false};
    uint8_t phase_{0};

    void update_output();
    void drive(const bool high);
};

/* PWM phase tick, called every 1ms from the controls/touch timer ISR. */
void backlight_pwm_tick();

class BacklightCAT4004 : public BacklightBase {
   public:
    value_t levels() const override {
        return maximum_level + 1;
    }

    void set_level(const value_t value) override;

    value_t level() const override {
        return level_;
    }

    void on() override;
    void off() override;

    bool is_on() const override {
        return on_;
    }

    /* Real hardware dimming via pulse-count protocol. */
    void set_fake_brightness(const uint8_t level) override;

   private:
    static constexpr value_t initial_brightness = 25;
    static constexpr value_t maximum_level = 31;

    static constexpr uint32_t ticks_setup = 204e6 * 10e-6;
    static constexpr uint32_t ms_pwrdwn = 5;
    static constexpr uint32_t ticks_lo = 204e6 * 1e-6;
    static constexpr uint32_t ticks_hi = 204e6 * 1e-6;

    value_t level_{initial_brightness};
    bool on_{false};

    void pulses(value_t target);
    void pulse();
};

} /* namespace portapack */

#endif
