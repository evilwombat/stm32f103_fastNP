# fastNP for STM32F103

## What is this?
fastNP is an STM32 library for refreshing multiple chains of addressable LEDs in parallel. It works with WS2812B LEDs and their clones like SK6812, etc (often called NeoPixels). Up to 16 chains of LEDs can be refreshed in parallel, without overclocking the CPU. I have developed and tested this on an STM32F103 microcontroller (on a so-called "Blue Pill" board) but it should be possible to port it to other STM32 micros as well.


This library builds on the concepts of [Martin Hubáček](http://www.martinhubacek.cz/)'s  [WS2812B DMA Library](http://www.martinhubacek.cz/arm/improved-stm32-ws2812b-library), who uses a *brilliant* technique for refreshing multiple LED chains using one Timer and three DMA channels. In fastNP, we try to take this a step further, by heavily optimizing the code that generates the intermediate DMA bit-buffers, which is basically the bottleneck when trying to refresh multiple channels. With a little bit of inline assembly, we can generate a bitbuffer for all sixteen channels, within the time it takes to perform one half of a DMA transfer.

## How do I run the example?
1. Grab an STM32F103 board (I'm using [Blue Pill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) but anything similar should work).

2. If your board uses anything other than an 8MHz HSC oscillator, modify the CubeMX project (or clock settings in main.c) so that the CPU and Timers still run at 72MHz.

3. Connect strips of WS2812B LEDs to any pin(s) on PORTB. By default, LED data will be sent to GPIOs B0-B15.

4. Grab the code and compile:
``make``
(assuming you're on Linux)

5. Flash the board
``make flash`` if you have the st-flash utility on Linux, or use whatever way you prefer

6. Watch the pretty lights scroll by. See demo_main.c for more info.

NOTE: If writing your own Makefile (or using some other build system), an **optimization level of at least -O1** is required. Otherwise, GCC generates code that is too slow.

## How do I use this in my project?

First, you'll want to **customize ws2812_led.h** for your project. If you are using fewer than 16 LED channels, set WS2812_NUM_CHANNELS to the appropriate number. In practice it's usually okay to leave this as-is (you can specify zero-length framebuffers for the unused channels) but this is more optimal.

All your LED channels must be connected to the **same GPIO bank** (ie, all LEDs must be on GPIOA, or on GPIOB, but you can't mix the two). If using fewer than 16 channels, you can also decide which channels map to which GPIO numbers within the GPIO bank. By default, channel 0 goes to GPIO0 (A0 or B0 or whatever bank you're using) but you can change this relationship in ws2812_led.h if necessary.

Next, you'll need to set up your **channel array**. This is an array of *struct led_channel_info* and its length **must** be WS2812_NUM_CHANNELS. This array contains the framebuffer pointer for each channel, as well as the length of the framebuffer for that channel. Unused channels should have their length set to 0. Note that framebuffer lengths are **in bytes**, rather than in terms of number of pixels. This is because some LEDs want 3 bytes (R, G, B) and other types of LEDs want 4 bytes (R, G, B, W). The library doesn't actually care what kind of LEDs you're talking to, as long as you've laid out your framebuffers in a way that your LEDs expect.

Next, you'll want to call **ws2812_init()**. This sets up Timer2 and all the DMA channels, but it doesn't send out any data yet.

Then, we **disable interrupts**, call **ws2812_refresh()**, passing in our channel array, and the GPIO bank we want to use. Finally, we may (optionally) re-enable interrupts.

See the following code snippet:


```
// Declare channel array
struct led_channel_info channels[WS2812_NUM_CHANNELS];

// Clear it out
memset(channels, 0, sizeof(channels));

channels[0].framebuffer = <pointer to some byte array>;
channels[0].length = <framebuffer length in bytes>

channels[1].framebuffer = <pointer to some other byte array>;
channels[1].length = <framebuffer length in bytes>
...

// Send out the image data!
__disable_irq();
ws2812_refresh(channels, GPIOB);  // If using some other GPIO bank, change it here
__enable_irq();
```

For more info, take a look at demo_main.c.




> Written with [StackEdit](https://stackedit.io/).
