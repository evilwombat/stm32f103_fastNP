/*
 * Copyright (c) 2020, evilwombat
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

#include <string.h>
#include "ws2812_led.h"

/* Framebuffers don't have to be of fixed length.
 * We're just using a fixed length for the demo animation, to keep the animation code simple.
 */
#define FRAMEBUFFER_SIZE        256

/* The order of R, G, B is important, because that's the order these get clocked out.
 * The LEDs expect the green channel first, then red, then blue.
 */
struct pixel {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};

void make_pretty_colors(struct pixel *framebuffer, int channel, int state)
{
    int red_offset, green_offset, blue_offset, i;

    red_offset = 0;
    green_offset = (FRAMEBUFFER_SIZE / 4) * ((channel) & 0x03);
    blue_offset = (FRAMEBUFFER_SIZE / 4) * ((channel >> 2) & 0x03);

    /* Just generate a different-looking psychedelic-looking pattern for each channel, to
     * test/prove that each channel is receiving a unique pattern
     */
    for (i = 0; i < FRAMEBUFFER_SIZE / 2; i++) {
        framebuffer[(i + red_offset + state) % FRAMEBUFFER_SIZE].r = i;
        framebuffer[(i + green_offset + state) % FRAMEBUFFER_SIZE].g = i;
        framebuffer[(i + blue_offset + state) % FRAMEBUFFER_SIZE].b = i;
    }
}

struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];

int demo_main(void)
{
    struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

    int ch, animation_state = 0;

    __enable_irq();
    HAL_Delay(200);

    /* Populate the led_channel_info structure. Set the framebuffer pointer for each channel,
     * and the length of the framebuffer for that channel. Lengths are in bytes, NOT in pixels.
     * We allow some framebuffers to be shorter than others (ie, if your LED strips are of
     * unequal lengths) without wasting memory. Removing the length check from ws2812_refresh()
     * will improve performance, but we are within spec as-is.
     */
    for (int i = 0; i < WS2812_NUM_CHANNELS; i++) {
        led_channels[i].framebuffer = channel_framebuffers[i];
        led_channels[i].length = FRAMEBUFFER_SIZE * sizeof(struct pixel);
    }

    ws2812_init();

    while(1) {
        /* Draw a pretty (but different) pattern for each channel.
         * We can trivially send the same framebuffer to each channel, but we won't do that, just
         * to prove that we can truly refresh 16 unique framebuffers at once.
         */
        for (ch = 0; ch < WS2812_NUM_CHANNELS; ch++)
            make_pretty_colors(channel_framebuffers[ch], ch, animation_state);

        animation_state++;

        __disable_irq();
        ws2812_refresh(led_channels, GPIOB);
        __enable_irq();
    }

    while(1);
}
