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

#include <string.h>
#include "ws2812_led.h"

TIM_HandleTypeDef htimer2;
DMA_HandleTypeDef hdma_tim2_update;
DMA_HandleTypeDef hdma_tim2_pwm_ch1;
DMA_HandleTypeDef hdma_tim2_pwm_ch2;

/* Generally you should not need to mess with these */
#define DMA_BUFFER_SIZE         16
#define DMA_BUFFER_FILL_SIZE    ((DMA_BUFFER_SIZE) / 2)

static uint16_t ws2812_gpio_set_bits = 0;
static uint16_t dma_buffer[DMA_BUFFER_SIZE];

/*
 * Convenience array for GPIO setup code.
 * DO NOT MODIFY THIS DIRECTLY - you will break things
 * Modify the definitions in the header file instead.
 */
const static uint8_t ws2812_channel_gpio_map[16] = {
    WS2812_CH0_GPIO,
    WS2812_CH1_GPIO,
    WS2812_CH2_GPIO,
    WS2812_CH3_GPIO,
    WS2812_CH4_GPIO,
    WS2812_CH5_GPIO,
    WS2812_CH6_GPIO,
    WS2812_CH7_GPIO,
    WS2812_CH8_GPIO,
    WS2812_CH9_GPIO,
    WS2812_CH10_GPIO,
    WS2812_CH11_GPIO,
    WS2812_CH12_GPIO,
    WS2812_CH13_GPIO,
    WS2812_CH14_GPIO,
    WS2812_CH15_GPIO
};

static void ws2812_timer2_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htimer2.Instance = TIM2;
    htimer2.Init.Prescaler = 0;
    htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimer2.Init.Period = WS2812_TIMER_PERIOD;

    htimer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htimer2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htimer2, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htimer2);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htimer2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = WS2812_TIMER_PWM_CH1_TIME;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htimer2, &sConfigOC, TIM_CHANNEL_1);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = WS2812_TIMER_PWM_CH2_TIME;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htimer2, &sConfigOC, TIM_CHANNEL_2);
}

static void ws2812_dma_start(GPIO_TypeDef *gpio_bank)
{
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 DMA Init */
    /* TIM2_UP Init */
    hdma_tim2_update.Instance = DMA1_Channel2;
    hdma_tim2_update.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_update.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_update.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim2_update.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_update.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_update.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_update.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* TIM2_CH1 Init */
    hdma_tim2_pwm_ch1.Instance = DMA1_Channel5;
    hdma_tim2_pwm_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_pwm_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_pwm_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_pwm_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_pwm_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_pwm_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_pwm_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* TIM2_CH2_CH4 Init */
    hdma_tim2_pwm_ch2.Instance = DMA1_Channel7;
    hdma_tim2_pwm_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_pwm_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_pwm_ch2.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim2_pwm_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_pwm_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_pwm_ch2.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_pwm_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* I don't know why, but making all DMAs run as long as the buffer size makes things more
     * efficient. Is it the extra full/half-done flags? Only the 2nd DMA needs to run for a given
     * size ...
     */
    HAL_DMA_Init(&hdma_tim2_update);
    HAL_DMA_Init(&hdma_tim2_pwm_ch1);
    HAL_DMA_Init(&hdma_tim2_pwm_ch2);

    HAL_DMA_Start(&hdma_tim2_update, (uint32_t)&ws2812_gpio_set_bits, (uint32_t)&gpio_bank->BSRR, DMA_BUFFER_SIZE);
	HAL_DMA_Start(&hdma_tim2_pwm_ch1, (uint32_t)dma_buffer, (uint32_t) &gpio_bank->BRR, DMA_BUFFER_SIZE);
    HAL_DMA_Start(&hdma_tim2_pwm_ch2, (uint32_t)&ws2812_gpio_set_bits, (uint32_t)&gpio_bank->BRR, DMA_BUFFER_SIZE);

	__HAL_TIM_ENABLE_DMA(&htimer2, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&htimer2, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htimer2, TIM_DMA_CC2);
}


/*
 * Unpack the bits of ch_val and pack them into the bit positions of cur0-cur7 that correspond to
 * the given GPIO number. Later, cur0-cur7 will be DMAed directly to a register within our GPIO
 * bank.
 */
#define UNPACK_CHANNEL(gpio_num)                    \
    asm volatile (                                  \
    "ubfx   r0, %[ch_val], #7, #1 \n"               \
    "bfi    %[cur0], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #6, #1 \n"               \
    "bfi    %[cur1], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #5, #1 \n"               \
    "bfi    %[cur2], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #4, #1 \n"               \
    "bfi    %[cur3], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #3, #1 \n"               \
    "bfi    %[cur4], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #2, #1 \n"               \
    "bfi    %[cur5], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #1, #1 \n"               \
    "bfi    %[cur6], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #0, #1 \n"               \
    "bfi    %[cur7], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    : [cur0]"+r" (cur0),                            \
        [cur1]"+r" (cur1),                          \
        [cur2]"+r" (cur2),                          \
        [cur3]"+r" (cur3),                          \
        [cur4]"+r" (cur4),                          \
        [cur5]"+r" (cur5),                          \
        [cur6]"+r" (cur6),                          \
        [cur7]"+r" (cur7)                           \
    : [ch_val]"r" (ch_val)                          \
    : "r0", "cc");  /* r0 is a temp variable */


/*
 * Unpack the bits for one byte of one channel, and pack them into the bit positions of
 * the cur0 - cur7 variables, corresponding to the GPIO number for that channel.
 * The 'if' statement will be optimized away by the compiler, depending on how many channels
 * are actually defined.
 */
#define HANDLE_CHANNEL(ch_num, gpio_num)                    \
    if (ch_num < WS2812_NUM_CHANNELS) {                     \
        ch_val = get_channel_byte(channels + ch_num, pos);  \
        UNPACK_CHANNEL(gpio_num);                           \
    }

static inline uint8_t get_channel_byte(const struct led_channel_info *channel, int pos)
{
    /* If all channels are the same length, we can skip the 'pos' range check, and speed up our
     * inner loop *substantially*
     */

    if (WS212_ALL_CHANNELS_SAME_LENGTH || (pos < channel->length))
        return channel->framebuffer[pos] ^ 0xff;

    return 0xff;
}

static void fill_dma_buffer(uint16_t *dest, int pos, const struct led_channel_info *channels)
{
    register uint16_t cur0 = 0, cur1 = 0, cur2 = 0, cur3 = 0, cur4 = 0, cur5 = 0, cur6 = 0, cur7 = 0;

    /* cur0 - cur7 represent eight successive words to be output to the DMA buffer. Each value
     * holds all the nth bits from across each channel. That is, when the unpacking is done,
     * cur0 holds all the bit0 of the current byte from every channel
     * cur1 holds all the bit1 of the current byte from every channel
     * cur2 holds all the bit2 of the current byte from every channel
     * etc
     * (in reality they're bit7 through bit0, depending on which direction you're looking)
     *
     * This allows us to be very very efficient in terms of the unpacking operation.
     * Processing each channel requires a load of the next byte from that channel (plus some
     * range checking), and each bit from this byte gets stuffed into the corresponding position
     * in eight different variables. We do our best to keep these variables in registers, which
     * GCC refuses to do, unless we force its hand with some inline ASM.
     * Finally, when we are done, we have to store the eight outputs to the DMA buffer itself.
     * That's a total of 16 + 8 memory operations, and no conditional instructions on the data
     * transfer path (again, excluding the range check at the start of each unpack).
     *
     * We cannot turn the below code into a loop, because UNPACK_CHANNEL generates inline ASM which
     * directly uses the channel number as a literal (translating it into an output bit position).
     * There is no bit-manipulation instruction that will do this operation in a single step, and
     * still allow the destination bit position to be a register argument. Yes, we could use an LSL
     * instruction on an intermediate value, but this will increase our overhead by 33%, and with
     * all the error checks, we don't have the headroom (at least, on an STM32F103 at 72MHz).
     *
     * If you want it to be fast, don't expect it to always be pretty.
     */
    uint8_t ch_val;
    HANDLE_CHANNEL( 0, WS2812_CH0_GPIO);
    HANDLE_CHANNEL( 1, WS2812_CH1_GPIO);
    HANDLE_CHANNEL( 2, WS2812_CH2_GPIO);
    HANDLE_CHANNEL( 3, WS2812_CH3_GPIO);
    HANDLE_CHANNEL( 4, WS2812_CH4_GPIO);
    HANDLE_CHANNEL( 5, WS2812_CH5_GPIO);
    HANDLE_CHANNEL( 6, WS2812_CH6_GPIO);
    HANDLE_CHANNEL( 7, WS2812_CH7_GPIO);
    HANDLE_CHANNEL( 8, WS2812_CH8_GPIO);
    HANDLE_CHANNEL( 9, WS2812_CH9_GPIO);
    HANDLE_CHANNEL(10, WS2812_CH10_GPIO);
    HANDLE_CHANNEL(11, WS2812_CH11_GPIO);
    HANDLE_CHANNEL(12, WS2812_CH12_GPIO);
    HANDLE_CHANNEL(13, WS2812_CH13_GPIO);
    HANDLE_CHANNEL(14, WS2812_CH14_GPIO);
    HANDLE_CHANNEL(15, WS2812_CH15_GPIO);

    /*
     * Store the repacked bits in our DMA buffer, ready to be sent to the GPIO bit-reset register.
     * cur0-cur7 represents bits0 - bits7 of all our channels. Each bit within curX is one channel.
     */
    dest[0] = cur0;
    dest[1] = cur1;
    dest[2] = cur2;
    dest[3] = cur3;
    dest[4] = cur4;
    dest[5] = cur5;
    dest[6] = cur6;
    dest[7] = cur7;
}

void ws2812_refresh(const struct led_channel_info *channels, GPIO_TypeDef *gpio_bank)
{
    int cycles = 0;
    int i;
    int pos = 0;
    int max_length = 0;

    /* This is what gets DMAed to the GPIO BSR / BSRR at the start/end of each bit cycle.
     * We will dynamically build this shortly
     */
    ws2812_gpio_set_bits = 0;

    /* Pre-fill the DMA buffer, because we won't start filling things on-the-fly until the first
     * half has already been transferred.
     */
    for (i = 0; i < DMA_BUFFER_SIZE; i+= 8) {
        fill_dma_buffer(dma_buffer + i, pos, channels);
        pos++;
    }

    /* Go through the channel list, figure out which channels are used, and set up the GPIO set/
     * reset bit masks. While we're at it, find the length of the longest framebuffer, in case
     * they're of unequal length. This determines how many total bits we will clock out.
     */
    for (i = 0; i < WS2812_NUM_CHANNELS; i++) {
        if (channels[i].length > max_length)
            max_length = channels[i].length;

        if (channels[i].length != 0)
            ws2812_gpio_set_bits |= (1 << ws2812_channel_gpio_map[i]);
    }

    /* Give DMA time to finish out the current buffer, before turning it off, plus an extra blank pixel (24 bits) */
    max_length += DMA_BUFFER_SIZE / 8;

    /* If per-channel range checks are enabled, add an extra "dummy" pixel to the end of our data stream.
     * This must only be done with range checks enabled, or we'll walk off the end of our framebuffers.
     */
#if !WS212_ALL_CHANNELS_SAME_LENGTH
    max_length += 3;
#endif

    /* We're going to use our standard timer to generate the RESET pulse, so for now just run the
     * timer without any DMA.
     */
	__HAL_TIM_DISABLE_DMA(&htimer2, TIM_DMA_UPDATE);
	__HAL_TIM_DISABLE_DMA(&htimer2, TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA(&htimer2, TIM_DMA_CC2);

    __HAL_TIM_DISABLE(&htimer2);

    /* Set all LED GPIOs to 0, to begin reset pulse */
    gpio_bank->BRR = ws2812_gpio_set_bits;

    __HAL_TIM_ENABLE(&htimer2);

    /* We know the timer overflows every 1.25uS (our bit-time interval). So rather than
     * reprogram the timer for 280uS (reset pulse duration) and back, we're gonna be lazy
     * and just count out ~225 update intervals
     */
    for (i = 0; i < 225; i++) {
        while (!__HAL_TIM_GET_FLAG(&htimer2, TIM_FLAG_UPDATE));
        __HAL_TIM_CLEAR_FLAG(&htimer2, TIM_FLAG_UPDATE);
    }

    /* Now that we're done with the RESET pulse, turn off the timer and prepare the DMA stuff */
    __HAL_TIM_DISABLE(&htimer2);
    ws2812_dma_start(gpio_bank);

    /* We set the timer to juuust before the overflow condition, so that the UPDATE event happens
     * before the CH1 / CH2 match events. We want this so that the UPDATE event gives us a clean
     * starting "high" level for the first edge of the first bit.
     */
    __HAL_TIM_SET_COUNTER(&htimer2, __HAL_TIM_GET_AUTORELOAD(&htimer2) - 10);

    /* Clear the DMA transfer status flags for the DMA we're using */
    DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5);

    /* Enable the timer.... and so it begins */
    __HAL_TIM_ENABLE(&htimer2);

    while(1) {
        /* Wait for DMA full-transfer or half-transfer event. This tells us when to fill the next buffer */
        if (!(DMA1->ISR & (DMA_ISR_TCIF5 | DMA_ISR_HTIF5))) {
            cycles++;
            continue;
        }

        uint16_t *dest = dma_buffer;

        /* Figure out if we're filling the first half of the DMA buffer, or the second half */
        if (DMA1->ISR & DMA_ISR_TCIF5)
            dest += DMA_BUFFER_FILL_SIZE;

        /* Clear DMA event flags */
        DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5);

        /* Unpack one new byte from each channel, into eight words in our DMA buffer
         * Each 16-bit word in the DMA buffer contains to one bit of the output byte (from each channel)
         */
        for (i = 0; i < DMA_BUFFER_FILL_SIZE; i+= 8) {
            fill_dma_buffer(dest + i, pos, channels);
            pos++;
        }

        if (pos > max_length)
            break;
    }

    __HAL_TIM_DISABLE(&htimer2);

    /* Set all LED GPIOs back to 0 */
    gpio_bank->BRR = ws2812_gpio_set_bits;

	__HAL_DMA_DISABLE(&hdma_tim2_update);
	__HAL_DMA_DISABLE(&hdma_tim2_pwm_ch1);
	__HAL_DMA_DISABLE(&hdma_tim2_pwm_ch2);
}

void ws2812_init()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init, not that we're using it... */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);

    ws2812_timer2_init();
}
