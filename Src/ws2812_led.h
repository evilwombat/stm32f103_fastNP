/*
 * Copyright (c) 2020, evilwombat
 *
 * Based on principles described by Martin Hubáček in:
 *  http://www.martinhubacek.cz/arm/improved-stm32-ws2812b-library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * Structure definition for one channel (one LED strip).
 * Provide a pointer to your pixel data, and a length (in bytes, not in pixels)
 * Setting length=0 disables the channel.
 */
struct led_strip_info {
    const uint8_t *framebuffer;
    int length;
};

/*
 * Timer selection. STM32F1 has three general-purpose timers that we can use for driving
 * WS2812 LEDs. Select the timer that will be used for this purpse, to de-conflict it with
 * other timers that may be more desireable for other uses.
 *
 * Supported values: 2, 3, 4
 */
#define WS2812_TIMER_INSTANCE 2

/* We assume our Timer runs at 72MHz (the maximum). If yours runs at a different rate, set it here. */
#define WS2812_TIMER_FREQ_HZ          72000000

/* Calculate timer period, CH A pulse width, and CH B pulse width to be 1.25uS, 0.4uS, and 0.8uS */
#define WS2812_TIMER_PERIOD        (((WS2812_TIMER_FREQ_HZ / 1000) * 125) / 100000)
#define WS2812_TIMER_PWM_CH_A_TIME (((WS2812_TIMER_FREQ_HZ / 1000) *  40) / 100000)
#define WS2812_TIMER_PWM_CH_B_TIME (((WS2812_TIMER_FREQ_HZ / 1000) *  80) / 100000)

/* How many channels (strips) of LEDs we want to control. This must not exceed 16. */
#define WS2812_NUM_STRIPS     16

/*
 * We support up to 16 LED channels (that is, up to 16 distinct strips of LEDs.
 * Channels must be used sequentially, but their GPIOs do not have to be sequential.
 * Although all the LED strips must be on the same GPIO bank, the channel number does not
 * necessarily have to match its GPIO number. This is useful if we want fewer than 16 channels,
 * but we don't want them to start from GPIO 0.
 *
 * Change these assignments based on which strip you are wiring to which GPIO.
 *
 * To change the GPIO associated with each channel, modify the code below:
 */
#define WS2812_CH0_GPIO      0
#define WS2812_CH1_GPIO      1
#define WS2812_CH2_GPIO      2
#define WS2812_CH3_GPIO      3
#define WS2812_CH4_GPIO      4
#define WS2812_CH5_GPIO      5
#define WS2812_CH6_GPIO      6
#define WS2812_CH7_GPIO      7
#define WS2812_CH8_GPIO      8
#define WS2812_CH9_GPIO      9
#define WS2812_CH10_GPIO    10
#define WS2812_CH11_GPIO    11
#define WS2812_CH12_GPIO    12
#define WS2812_CH13_GPIO    13
#define WS2812_CH14_GPIO    14
#define WS2812_CH15_GPIO    15
/* (These need to be constants at compile-time because they are used in inline ASM) */

void ws2812_init();
void ws2812_refresh(const struct led_strip_info *strips, GPIO_TypeDef *gpio_bank);
