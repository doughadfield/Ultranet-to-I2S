/*
* Behringer Ultranet decoder
* Reads Ultranet stream, decoding 8 x audio channels
* outputs pairs of channels as i2S stereo streams
*
* Ultranet bit rate = 12.288MHz (8x32bit samples at 48khz)
* Ultranet biphase clock rate = 2 x 12.288MHz = 24.576MHz
* We need pio module to sample at 7 x incoming clock rate
* Ideal system clock therefore = 7 x 24.576 = 172.032MHz
* We set cpu clock frequency as close to 172032KHz as possible
* but can only set whole numbers of MHz, so 172000khz
*
* Use PICO LED to indicate if we have a framing error
* ie we get out of sync with the 8 subframes of the ultranet stream
* if we don't detect start of frame sync in the right place, 
* toggle the LED - so frequency of frame errors can be seen
*
* Use multicore to compensate for difference in speed between
* Ultranet input stream (clocked from source) and I2S output
* stream (clocked from this pico). Array of samples is filled
* from the Ulranet stream by core0, then read out to I2S using
* core1, asynchronously from core0.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "ultranet.pio.h"

// Ultranet input and MCLK state machines use pio0
#define ULTRANET_PIN 0          // ultranet input pin
#define SM 0                    // state machine to use for Ultranet input
#define MCLK_PIN 2              // I2S Master Clock Pin
#define SM_MCLK 1               // state machine for I2S master clock
// I2S outputs use second pio (pio1), four I2S outputs, 3 pins each
#define I2S1_PINS 3             // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 6             // base for I2S output pins (3 pins starting point)
#define I2S3_PINS 9             // base for I2S output pins (3 pins starting point)
#define I2S4_PINS 12            // base for I2S output pins (3 pins starting point)
#define CLOCKSPEED  172000      // 172000 for 7 slots per bit incoming Ultranet stream
#define PICO_LED 25             // LED pin - to indicate ultranet framing error

volatile uint32_t samples[9];   // array of samples read from Ultranet stream

// state machine init functions (used to be defined in <prog>.pio file)

void ultranet_pio_init(PIO pio, uint sm, uint pin)
{
    uint offset = pio_add_program(pio, &ultranet_program);  // load code into pio mem
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);      // configure 8 depth input fifo
    sm_config_set_in_pins (&c, pin);                    // input pin range base
    pio_sm_init(pio, sm, offset, &c);                   // apply structure to state machine
    pio_sm_set_jmp_pin(pio, SM, pin);                   // specify pin for jmp instructions
    pio_sm_set_enabled(pio, sm, true);                  // start state machine running
}

void mclk_pio_init(PIO pio, uint sm, uint pin)
{
    uint offset = pio_add_program(pio, &mclk_program);  // load clock code
    pio_sm_config c = mclk_program_get_default_config(offset);  // get default structure
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    sm_config_set_clkdiv_int_frac(&c, 7, 0);
    sm_config_set_set_pins (&c, pin, 1);                // output pin range base and count
    pio_sm_init(pio, sm, offset, &c);                   // apply structure to state machine
    pio_sm_set_enabled(pio, sm, true);                  // start state machine running
}

void i2s_pio_init(PIO pio, uint sm, uint pin, uint offset)
{
    pio_sm_config c = i2s_program_get_default_config(offset);  // get default structure
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);  // set base+3 pins to output
    sm_config_set_clkdiv_int_frac(&c, 7, 0);            // set frequency of SM to fs x 256
    sm_config_set_out_pins (&c, pin, 3);                // out pin range base and count
    sm_config_set_sideset_pins (&c, pin+1);             // sideset pin range base
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);      // configure 8 depth output fifo
    sm_config_set_out_shift(&c, false, false, 32);      // set shift left, no autpull for out FIFO
    pio_sm_init(pio, sm, offset, &c);                   // apply structure to state machine
}

void core1_entry(void)                                  // Core1 starts executiing here
{
    uint i2s_offset;                                    // position for i2s code in pio (shared for all SMs)

    i2s_offset = pio_add_program(pio1, &i2s_program);   // load i2c output code once for all state machines
    i2s_pio_init(pio1, 0, I2S1_PINS, i2s_offset);       // all 4 state machines use the same code
    i2s_pio_init(pio1, 1, I2S2_PINS, i2s_offset);
    i2s_pio_init(pio1, 2, I2S3_PINS, i2s_offset);
    i2s_pio_init(pio1, 3, I2S4_PINS, i2s_offset);
    sleep_ms(200);                                      // wait for samples to start
    pio_sm_put_blocking(pio1, 0, samples[0]);           // subframe 1 goes to I2S0 channel 1
    pio_sm_put_blocking(pio1, 1, samples[2]);           // subframe 3 goes to I2S1 channel 1
    pio_sm_put_blocking(pio1, 2, samples[4]);           // subframe 5 goes to I2S2 channel 1
    pio_sm_put_blocking(pio1, 3, samples[6]);           // subframe 7 goes to I2S3 channel 1
    pio_sm_put_blocking(pio1, 0, samples[1]);           // subframe 2 goes to I2S0 channel 2
    pio_sm_put_blocking(pio1, 1, samples[3]);           // subframe 4 goes to I2S1 channel 2
    pio_sm_put_blocking(pio1, 2, samples[5]);           // subframe 6 goes to I2S2 channel 2
    pio_sm_put_blocking(pio1, 3, samples[7]);           // subframe 8 goes to I2S3 channel 2

    pio_set_sm_mask_enabled(pio1, 0xF, true);           // enable all I2S state machines at the same instant

    while(true)                                         // asynchronously output samples forever
    {
        pio_sm_put_blocking(pio1, 0, samples[0]);       // subframe 1 goes to I2S0 channel 1
        pio_sm_put_blocking(pio1, 1, samples[2]);       // subframe 3 goes to I2S1 channel 1
        pio_sm_put_blocking(pio1, 2, samples[4]);       // subframe 5 goes to I2S2 channel 1
        pio_sm_put_blocking(pio1, 3, samples[6]);       // subframe 7 goes to I2S3 channel 1
        pio_sm_put_blocking(pio1, 0, samples[1]);       // subframe 2 goes to I2S0 channel 2
        pio_sm_put_blocking(pio1, 1, samples[3]);       // subframe 4 goes to I2S1 channel 2
        pio_sm_put_blocking(pio1, 2, samples[5]);       // subframe 6 goes to I2S2 channel 2
        pio_sm_put_blocking(pio1, 3, samples[7]);       // subframe 8 goes to I2S3 channel 2
    }
}

int main()
{
    volatile uint32_t sample;                // temp store for sample read from Ultranet stream

    stdio_init_all();
    set_sys_clock_khz(CLOCKSPEED,false);                // set cpu clock frequency

    sleep_ms(200);                  // allow time for clocks etc. to settle


    mclk_pio_init(pio0, SM_MCLK, MCLK_PIN);             // uncomment to enable I2S MCLK

    gpio_set_dir(ULTRANET_PIN, false);                  // set ultranet pin as input
    gpio_set_pulls(ULTRANET_PIN, true, false);          // set pullup on ultranet pin
    ultranet_pio_init(pio0, SM, ULTRANET_PIN);          // initialise and start ultranet state machine

    gpio_init(PICO_LED);                                // set LED pin as GPIO 
    gpio_set_dir(PICO_LED, GPIO_OUT);                   // set LED pin as output

    multicore_launch_core1(core1_entry);                // start core 1

    sleep_ms(100);                                      // wait for core1 to start

    // sync with ultranet frames initially, so we don't turn LED on at start 
    sample = pio_sm_get_blocking(pio0, SM);             // get initial sample from Ultranet FIFO

    while((sample & 0x3F) != 0x0000000B && (sample & 0x3F) != 0x0000000F)
    {
        sample = pio_sm_get_blocking(pio0, SM);         // get next sample from Ultranet FIFO
    }

    while (true)
    {
        // synchronise with first subframe in Ultranet frame
        if((sample & 0x3F) == 0x0000000B || (sample & 0x3F) == 0x0000000F)
        {
            // if we get here, sample contains the first subframe in Ultranet frame
            samples[0] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[1] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[2] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[3] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[4] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[5] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[6] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs

            sample = pio_sm_get_blocking(pio0, SM);     // get next sample from Ultranet FIFO
            samples[7] = (sample << 4) & 0xFFFFFE00;    // move 24 bits of audio into MSBs
        }
        else
        {
            if(gpio_get(PICO_LED) == 0)
                gpio_put(PICO_LED, 1);                  // turn on LED to indicate frame error
            else
                gpio_put(PICO_LED, 0);
        }
        sample = pio_sm_get_blocking(pio0, SM);         // get next sample from Ultranet FIFO
    }
}
