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

TIM_HandleTypeDef htimer;
DMA_HandleTypeDef hdma_tim_update;
DMA_HandleTypeDef hdma_tim_pwm_ch_a;
DMA_HandleTypeDef hdma_tim_pwm_ch_b;

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

/*
 * Timer selection logic. Do not change this.
 * To use a different timer, change WS2812_TIMER_INSTANCE in ws2812_led.h instead.
 */

#if WS2812_TIMER_INSTANCE == 2
    /* HAL stuff for our timer */
    #define TIMER_CLK_ENABLE        __HAL_RCC_TIM2_CLK_ENABLE
    #define HAL_TIMER_INSTANCE      TIM2

    /*
     * TIM3 doesn't have a DMA event for CH2, so we have to generalize CH1/CH2 to CH_A / CH_B,
     * then use CH1/CH2 for TIM2 and TIM4, and CH1/CH3 for TIM3 :/
    */
    #define TIMER_CHANNEL_A         TIM_CHANNEL_1
    #define TIMER_CHANNEL_B         TIM_CHANNEL_2
    #define TIM_DMA_CC_A            TIM_DMA_CC1
    #define TIM_DMA_CC_B            TIM_DMA_CC2


    /* DMA channel numbers for the timer update, Ch1, and Ch2 events */
    #define DMA_CHANNEL_TIM_UP      DMA1_Channel2
    #define DMA_CHANNEL_TIM_CH_A    DMA1_Channel5
    #define DMA_CHANNEL_TIM_CH_B    DMA1_Channel7

    /* DMA status flags for transfer complete and friends */
    #define DMA_TIM_CH_A_ICFR_FLAGS (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5)
    #define DMA_TIM_CH_A_TCIF       DMA_ISR_TCIF5
    #define DMA_TIM_CH_A_HTIF       DMA_ISR_HTIF5

    /* We don't use DMA interrupts, but be pedantic and define them*/
    #define DMA_IRQ_TIM_UP          DMA1_Channel2_IRQn
    #define DMA_IRQ_TIM_CH_A        DMA1_Channel5_IRQn
    #define DMA_IRQ_TIM_CH_B        DMA1_Channel7_IRQn
#elif WS2812_TIMER_INSTANCE == 3
    #define TIMER_CLK_ENABLE        __HAL_RCC_TIM3_CLK_ENABLE
    #define HAL_TIMER_INSTANCE      TIM3
    #define TIMER_CHANNEL_A         TIM_CHANNEL_1
    /* TIM3 doesn't have a DMA-capable CH2, so we use CH3 instead */
    #define TIMER_CHANNEL_B         TIM_CHANNEL_3
    #define TIM_DMA_CC_A            TIM_DMA_CC1
    #define TIM_DMA_CC_B            TIM_DMA_CC3
    #define DMA_CHANNEL_TIM_UP      DMA1_Channel3
    #define DMA_CHANNEL_TIM_CH_A    DMA1_Channel6
    #define DMA_CHANNEL_TIM_CH_B    DMA1_Channel2
    #define DMA_TIM_CH_A_ICFR_FLAGS (DMA_IFCR_CTCIF6 | DMA_IFCR_CHTIF6)
    #define DMA_TIM_CH_A_TCIF       DMA_ISR_TCIF6
    #define DMA_TIM_CH_A_HTIF       DMA_ISR_HTIF6
    #define DMA_IRQ_TIM_UP          DMA1_Channel3_IRQn
    #define DMA_IRQ_TIM_CH_A        DMA1_Channel6_IRQn
    #define DMA_IRQ_TIM_CH_B        DMA1_Channel2_IRQn
#elif WS2812_TIMER_INSTANCE == 4
    #define TIMER_CLK_ENABLE        __HAL_RCC_TIM4_CLK_ENABLE
    #define HAL_TIMER_INSTANCE      TIM4
    #define TIMER_CHANNEL_A         TIM_CHANNEL_1
    #define TIMER_CHANNEL_B         TIM_CHANNEL_2
    #define TIM_DMA_CC_A            TIM_DMA_CC1
    #define TIM_DMA_CC_B            TIM_DMA_CC2
    #define DMA_CHANNEL_TIM_UP      DMA1_Channel7
    #define DMA_CHANNEL_TIM_CH_A    DMA1_Channel1
    #define DMA_CHANNEL_TIM_CH_B    DMA1_Channel4
    #define DMA_TIM_CH_A_ICFR_FLAGS (DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1)
    #define DMA_TIM_CH_A_TCIF       DMA_ISR_TCIF1
    #define DMA_TIM_CH_A_HTIF       DMA_ISR_HTIF1
    #define DMA_IRQ_TIM_UP          DMA1_Channel7_IRQn
    #define DMA_IRQ_TIM_CH_A        DMA1_Channel1_IRQn
    #define DMA_IRQ_TIM_CH_B        DMA1_Channel4_IRQn
#else
    #error "Unsupported tiemr instance. Check that WS2812_TIMER_INSTANCE is set to a supported value"
#endif


static void ws2812_timer_init(void)
{
    TIMER_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htimer.Instance = HAL_TIMER_INSTANCE;
    htimer.Init.Prescaler = 0;
    htimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimer.Init.Period = WS2812_TIMER_PERIOD;

    htimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htimer);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htimer, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htimer);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htimer, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = WS2812_TIMER_PWM_CH_A_TIME;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htimer, &sConfigOC, TIMER_CHANNEL_A);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = WS2812_TIMER_PWM_CH_B_TIME;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htimer, &sConfigOC, TIMER_CHANNEL_B);
}

static void ws2812_dma_start(GPIO_TypeDef *gpio_bank)
{
    /* Peripheral clock enable */
    TIMER_CLK_ENABLE();

    /* Timer DMA Init */
    /* Timer Update Init */
    hdma_tim_update.Instance = DMA_CHANNEL_TIM_UP;
    hdma_tim_update.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim_update.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim_update.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim_update.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim_update.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim_update.Init.Mode = DMA_CIRCULAR;
    hdma_tim_update.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* Timer Ch A Init */
    hdma_tim_pwm_ch_a.Instance = DMA_CHANNEL_TIM_CH_A;
    hdma_tim_pwm_ch_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim_pwm_ch_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim_pwm_ch_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim_pwm_ch_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim_pwm_ch_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim_pwm_ch_a.Init.Mode = DMA_CIRCULAR;
    hdma_tim_pwm_ch_a.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* Timer Ch B Init */
    hdma_tim_pwm_ch_b.Instance = DMA_CHANNEL_TIM_CH_B;
    hdma_tim_pwm_ch_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim_pwm_ch_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim_pwm_ch_b.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim_pwm_ch_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim_pwm_ch_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim_pwm_ch_b.Init.Mode = DMA_CIRCULAR;
    hdma_tim_pwm_ch_b.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    /* I don't know why, but making all DMAs run as long as the buffer size makes things more
     * efficient. Is it the extra full/half-done flags? Only the 2nd DMA needs to run for a given
     * size ...
     */
    HAL_DMA_Init(&hdma_tim_update);
    HAL_DMA_Init(&hdma_tim_pwm_ch_a);
    HAL_DMA_Init(&hdma_tim_pwm_ch_b);

    HAL_DMA_Start(&hdma_tim_update, (uint32_t)&ws2812_gpio_set_bits, (uint32_t)&gpio_bank->BSRR, DMA_BUFFER_SIZE);
    HAL_DMA_Start(&hdma_tim_pwm_ch_a, (uint32_t)dma_buffer, (uint32_t) &gpio_bank->BRR, DMA_BUFFER_SIZE);
    HAL_DMA_Start(&hdma_tim_pwm_ch_b, (uint32_t)&ws2812_gpio_set_bits, (uint32_t)&gpio_bank->BRR, DMA_BUFFER_SIZE);

	__HAL_TIM_ENABLE_DMA(&htimer, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&htimer, TIM_DMA_CC_A);
	__HAL_TIM_ENABLE_DMA(&htimer, TIM_DMA_CC_B);
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
 * We only care about the lower 8 bits, but it's more efficient to flip all the bits rather
 * than just trying to XOR in 0xff.
 */
#define HANDLE_CHANNEL(ch_num, gpio_num)                                \
        ch_val = 0xffffffff;                                            \
        if (pos < strips[ch_num].length) {                              \
            ch_val = strips[ch_num].framebuffer[pos] ^ 0xffffffff;      \
        }                                                               \
        UNPACK_CHANNEL(gpio_num);

static void __always_inline fill_dma_buffer(uint16_t *dest, const int pos, const struct led_strip_info* strips)
{
    // Use 'int' (instead of int16) here, or GCC will waste a lot of instructions on silly sign
    // extension things, which we don't even need here.
    register unsigned int cur0 = 0, cur1 = 0, cur2 = 0, cur3 = 0, cur4 = 0, cur5 = 0, cur6 = 0, cur7 = 0;

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
    unsigned int ch_val;

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

void ws2812_refresh(const struct led_strip_info *strips, GPIO_TypeDef *gpio_bank)
{
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
        fill_dma_buffer(dma_buffer + i, pos, strips);
        pos++;
    }

    /* Go through the channel list, figure out which channels are used, and set up the GPIO set/
     * reset bit masks. While we're at it, find the length of the longest framebuffer, in case
     * they're of unequal length. This determines how many total bits we will clock out.
     */
    for (i = 0; i < WS2812_NUM_STRIPS; i++) {
        if (strips[i].length > max_length)
            max_length = strips[i].length;

        if (strips[i].length != 0)
            ws2812_gpio_set_bits |= (1 << ws2812_channel_gpio_map[i]);
    }

    /* Give DMA time to finish out the current buffer, before turning it off, plus an extra blank pixel (24 bits) */
    max_length += DMA_BUFFER_SIZE / 8;

    /* We're going to use our standard timer to generate the RESET pulse, so for now just run the
     * timer without any DMA.
     */
	__HAL_TIM_DISABLE_DMA(&htimer, TIM_DMA_UPDATE);
	__HAL_TIM_DISABLE_DMA(&htimer, TIM_DMA_CC_A);
	__HAL_TIM_DISABLE_DMA(&htimer, TIM_DMA_CC_B);

    __HAL_TIM_DISABLE(&htimer);

    ws2812_dma_start(gpio_bank);

    /* We set the timer to juuust before the overflow condition, so that the UPDATE event happens
     * before the CH1 / CH2 match events. We want this so that the UPDATE event gives us a clean
     * starting "high" level for the first edge of the first bit.
     */
    __HAL_TIM_SET_COUNTER(&htimer, __HAL_TIM_GET_AUTORELOAD(&htimer) - 10);

    /* Clear the DMA transfer status flags for the DMA we're using */
    DMA1->IFCR = DMA_TIM_CH_A_ICFR_FLAGS;

    /* Enable the timer.... and so it begins */
    __HAL_TIM_ENABLE(&htimer);

    while(1) {
        /* Wait for DMA full-transfer or half-transfer event. This tells us when to fill the next buffer */
        if (!(DMA1->ISR & (DMA_TIM_CH_A_TCIF | DMA_TIM_CH_A_HTIF))) {
            continue;
        }

        uint16_t *dest = dma_buffer;

        /* Figure out if we're filling the first half of the DMA buffer, or the second half */
        if (DMA1->ISR & DMA_TIM_CH_A_TCIF)
            dest += DMA_BUFFER_FILL_SIZE;

        /* Clear DMA event flags */
        DMA1->IFCR = DMA_TIM_CH_A_ICFR_FLAGS;

        /* Unpack one new byte from each channel, into eight words in our DMA buffer
         * Each 16-bit word in the DMA buffer contains to one bit of the output byte (from each channel)
         */
        for (i = 0; i < DMA_BUFFER_FILL_SIZE; i+= 8) {
            fill_dma_buffer(dest + i, pos, strips);
            pos++;
        }

        if (pos > max_length)
            break;
    }

    __HAL_TIM_DISABLE(&htimer);

    /* Set all LED GPIOs back to 0 */
    gpio_bank->BRR = ws2812_gpio_set_bits;

	__HAL_DMA_DISABLE(&hdma_tim_update);
	__HAL_DMA_DISABLE(&hdma_tim_pwm_ch_a);
	__HAL_DMA_DISABLE(&hdma_tim_pwm_ch_b);

    __HAL_TIM_ENABLE(&htimer);

    /* We know the timer overflows every 1.25uS (our bit-time interval). So rather than
     * reprogram the timer for 280uS (reset pulse duration) and back, we're gonna be lazy
     * and just count out ~225 update intervals
     */
    for (i = 0; i < 225; i++) {
        while (!__HAL_TIM_GET_FLAG(&htimer, TIM_FLAG_UPDATE));
        __HAL_TIM_CLEAR_FLAG(&htimer, TIM_FLAG_UPDATE);
    }

    /* Now that we're done with the RESET pulse, turn the timer back off */
    __HAL_TIM_DISABLE(&htimer);
}

void ws2812_init()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init, not that we're using it... */
    HAL_NVIC_SetPriority(DMA_IRQ_TIM_UP, 0, 0);
    HAL_NVIC_SetPriority(DMA_IRQ_TIM_CH_A, 0, 0);
    HAL_NVIC_SetPriority(DMA_IRQ_TIM_CH_B, 0, 0);

    ws2812_timer_init();
}
