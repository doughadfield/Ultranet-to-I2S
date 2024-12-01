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
* but can only set certain numbers of MHz, so 172000khz
*
* OR we need pio module to sample at 8 x incoming clock rate
* Ideal system clock therefore = 8 x 24.576 = 196.608MHz
* We set cpu clock frequency as close to 196608KHz as possible
* but can only set certain numbers of MHz, so 176500khz
*
* Ultranet audio sample depth is in fact 22 bits, not 24
* So we mask the 2 LSBs when reading words from Ultranet stream
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
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "ultranet.pio.h"

// conditional compilation switches for hardware options
// #define DEBUG                   // enable debug code
#define WS2812                  // Our board has a ws2812 programmable LED
#define MCLK                    // Enable MCLK clock for I2S devices

                                // 172000 for 7 slots per bit incoming Ultranet stream
#define CLOCKSPEED  196500      // 196500 for 8 slots per bit incoming Ultranet stream
#define AUDIV 8                 // Audio divider for pio timing (7 for 172MHz, 8 for 196.5MHz)
#ifndef WS2812                  // Use normal LED on "genuine" PICO boards
#define PICO_LED 25             // LED pin - to indicate ultranet framing error
#endif // WS2812
// Ultranet input and MCLK state machines use pio0
#define UNET_PIN 0              // ultranet input pin
#define UNET_PIO pio0           // PIO module to use for Ultranet input
#define UNET_SM 0               // state machine to use for Ultranet input
#ifdef MCLK                     // if we want an I2S MCLK clock
#define MCLK_PIN 1              // I2S Master Clock Pin
#define MCLK_PIO pio0           // state machine for I2S master clock
#define MCLK_SM 1               // state machine for I2S master clock
#endif // MCLK
// I2S outputs use second pio (pio1), four I2S outputs, 3 pins each
#define I2S_PIO pio1            // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 2             // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 5             // base for I2S output pins (3 pins starting point)
#define I2S3_PINS 8             // base for I2S output pins (3 pins starting point)
#define I2S4_PINS 11            // base for I2S output pins (3 pins starting point)
// ws2812 multicolour LED driving
#ifdef WS2812
#define WS2812_PIN 16           // chinese pico boards have ws2812 on pin 16
#define WS2812_PIO pio0         // same pio as ultranet & mclk
#define WS2812_SM 2             // state machine for LED output
#define GREEN 0x0F000000        // send this to ws2812 for GREEN LED
#define RED   0x00180000        // send this to ws2812 for RED LED
#define BLUE  0x00001F00        // send this to ws2812 for BLUE LED
#define WHITE 0x0F0F0F00        // send this to ws2812 for WHITE LED
#define CYAN (RED|BLUE)         // above primary colour intensities are tuned for
#define MAGENTA (GREEN|BLUE)    // best colour mix and even-ness
#define YELLOW (GREEN|RED)
#define put_pixel(pixel) pio_sm_put(WS2812_PIO, WS2812_SM, (pixel))
#endif // WS2812

// for PWM analog audio outputs
#define PIN_PWM_1A 14           // A channel of PWM slice (left audio)
#define PIN_PWM_1B 15           // B channel of PWM slice (right audio)
#define PIN_PWM_2A 18           // A channel of PWM slice (left audio)
#define PIN_PWM_2B 19           // B channel of PWM slice (right audio)
#define PIN_PWM_3A 26           // A channel of PWM slice (left audio)
#define PIN_PWM_3B 27           // B channel of PWM slice (right audio)
#define PIN_PWM_4A 28           // A channel of PWM slice (left audio)
#define PIN_PWM_4B 29           // B channel of PWM slice (right audio)

#define pwm_set_a(slice,num) pwm_set_chan_level((slice), PWM_CHAN_A, (uint16_t)(num))
#define pwm_set_b(slice,num) pwm_set_chan_level((slice), PWM_CHAN_B, (uint16_t)(num))

volatile uint32_t samples[8];   // array of samples read from Ultranet stream
volatile uint8_t slice[4];      // PWM slice numbers for specified pins

void pwm_setup(void)
{
    uint count;                                             // loop counter
    gpio_set_function(PIN_PWM_1A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_1B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_2A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_2B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_3A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_3B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_4A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_4B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    slice[0] = pwm_gpio_to_slice_num(PIN_PWM_1A);           // get PWM slice that uses this pin
    slice[1] = pwm_gpio_to_slice_num(PIN_PWM_2A);           // get PWM slice that uses this pin
    slice[2] = pwm_gpio_to_slice_num(PIN_PWM_3A);           // get PWM slice that uses this pin
    slice[3] = pwm_gpio_to_slice_num(PIN_PWM_4A);           // get PWM slice that uses this pin
    for(count=0;count<4;count++)
    {
        pwm_set_wrap(slice[count], 0xFFF);                  // set PWM period to 4096
        pwm_set_a(slice[count], 0x1000/2);                  // start output at 50%
        pwm_set_b(slice[count], 0x1000/2);                  // start output at 50%
        pwm_set_enabled(slice[count], true);                // start PWM running
    }
}

// state machine init functions (used to be defined in <prog>.pio file)

void ultranet_pio_init(PIO pio, uint sm, uint pin)
{
    gpio_set_dir(pin, false);                               // set ultranet pin as input
    gpio_set_pulls(pin, true, false);                       // set pullup on ultranet pin
    uint offset = pio_add_program(pio, &ultranet_program);  // load code into pio mem
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);          // configure 8 depth input fifo
    sm_config_set_in_pins (&c, pin);                        // input pin range base
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
    pio_sm_set_jmp_pin(pio, UNET_SM, pin);                  // specify pin for jmp instructions
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running
}

#ifdef MCLK
void mclk_pio_init(PIO pio, uint sm, uint pin)
{
    uint offset = pio_add_program(pio, &mclk_program);      // load clock code
    pio_sm_config c = mclk_program_get_default_config(offset);  // get default structure
    pio_gpio_init(pio, pin);                                // iniitialise pin as pio output
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // set pin direction to output
    sm_config_set_clkdiv_int_frac(&c, AUDIV, 0);
    sm_config_set_set_pins (&c, pin, 1);                    // output pin range base and count
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running
}
#endif // MCLK

void i2s_pio_init(PIO pio, uint sm, uint pin, uint offset)
{
    pio_sm_config c = i2s_program_get_default_config(offset);  // get default structure
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);  // set base+3 pins to output
    sm_config_set_clkdiv_int_frac(&c, AUDIV, 0);            // set frequency of UNET_SM to fs x 256
    sm_config_set_out_pins (&c, pin, 3);                    // out pin range base and count
    sm_config_set_sideset_pins (&c, pin+1);                 // sideset pin range base
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // configure 8 depth output fifo
    sm_config_set_out_shift(&c, false, false, 32);          // set shift left, no autpull for out FIFO
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine
}

#ifdef WS2812
void ws2812_pio_init(PIO pio, uint sm, uint pin)            // Set up PIO SM for ws2812 LED module
{
    uint offset = pio_add_program(pio, &ws2812_program);    // PIO program shares code space with UNET and MCLK
    pio_gpio_init(pio, pin);                                // Set up GPIO pin for PIO...
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // ...output

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);           // set shift direction RIGHT, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // use 8 deep TX FIFO

#define CYCLES_PER_BIT ((ws2812_T1)+(ws2812_T2)+(ws2812_T3)) // constants defined in .pio source file
    float div = clock_get_hz(clk_sys) / (800000 * CYCLES_PER_BIT);  // ws2812 needs precise 800KHz timing
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);                      // Set ws2812 PIO state machine running
}
#endif // WS2812

/*
* Core 1 continuously reads the samples array and outputs the values to the I2S and PWM streams
* according to the pico internal clock (output frequency 48KHz)
* The samples array is continually filled by Core 0, synchronised to the incoming Ultranet stream
* This avoids audio sample sync issues as when the output sequence is running ahead, the Core 1 processing
* simply outputs the previous sample a second time, as it's still in the array.
* Or it outputs the new sample, missing one, if it's running behind.
* Provided the pico clock is at a close enough frequency to the incoming Ultranet stream, these corrections
* are infrequent and the samples very similar in amplitude, so there's no audible click or pop.
*/
void core1_entry(void)                                      // Core1 starts executiing here
{
    uint i2s_offset;                                        // position for i2s code in pio (shared for all SMs)
    volatile uint32_t ssample;                              // signed version of audio sample

    i2s_offset = pio_add_program(I2S_PIO, &i2s_program);    // load i2c output code once for all state machines
    i2s_pio_init(I2S_PIO, 0, I2S1_PINS, i2s_offset);        // all 4 state machines use the same code
    i2s_pio_init(I2S_PIO, 1, I2S2_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 2, I2S3_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 3, I2S4_PINS, i2s_offset);
    sleep_ms(200);                                          // wait for incoming samples to start
    // ensure there is data in each output FIFO before starting the state machines
    pio_sm_put_blocking(I2S_PIO, 0, samples[0]);            // subframe 1 goes to I2S0 channel 1
    pio_sm_put_blocking(I2S_PIO, 1, samples[2]);            // subframe 3 goes to I2S1 channel 1
    pio_sm_put_blocking(I2S_PIO, 2, samples[4]);            // subframe 5 goes to I2S2 channel 1
    pio_sm_put_blocking(I2S_PIO, 3, samples[6]);            // subframe 7 goes to I2S3 channel 1
    pio_sm_put_blocking(I2S_PIO, 0, samples[1]);            // subframe 2 goes to I2S0 channel 2
    pio_sm_put_blocking(I2S_PIO, 1, samples[3]);            // subframe 4 goes to I2S1 channel 2
    pio_sm_put_blocking(I2S_PIO, 2, samples[5]);            // subframe 6 goes to I2S2 channel 2
    pio_sm_put_blocking(I2S_PIO, 3, samples[7]);            // subframe 8 goes to I2S3 channel 2

    pio_set_sm_mask_enabled(I2S_PIO, 0xF, true);            // enable all I2S state machines at the same instant

    while(true)                                             // output samples synchronised with I2S streams
    {
        // continually load pio FIFOs for I2S outputs, and PWM registers for PWM outputs
        pio_sm_put_blocking(I2S_PIO, 0, samples[0]);        // subframe 1 goes to I2S0 channel 1

        ssample = (0x80000000 + (signed)samples[0]);
        pwm_set_a(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 1, samples[2]);        // subframe 3 goes to I2S1 channel 1

        ssample = (0x80000000 + (signed)samples[2]);
        pwm_set_a(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[4]);        // subframe 5 goes to I2S2 channel 1

        ssample = (0x80000000 + (signed)samples[4]);
        pwm_set_a(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[6]);        // subframe 7 goes to I2S3 channel 1

        ssample = (0x80000000 + (signed)samples[6]);
        pwm_set_a(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 0, samples[1]);        // subframe 2 goes to I2S0 channel 2

        ssample = (0x80000000 + (signed)samples[1]);
        pwm_set_b(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 1, samples[3]);        // subframe 4 goes to I2S1 channel 2

        ssample = (0x80000000 + (signed)samples[3]);
        pwm_set_b(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[5]);        // subframe 6 goes to I2S2 channel 2

        ssample = (0x80000000 + (signed)samples[5]);
        pwm_set_b(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[7]);        // subframe 8 goes to I2S3 channel 2

        ssample = (0x80000000 + (signed)samples[7]);
        pwm_set_b(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio
    }
}

int main()
{
    volatile uint32_t sample;                               // temp store for sample read from Ultranet stream

    stdio_init_all();
    set_sys_clock_khz(CLOCKSPEED,false);                    // set cpu clock frequency

    sleep_ms(200);                                          // allow time for clocks etc. to settle

    pwm_setup();                                            // initialise PWM hardware and start outputs

#ifdef MCLK
    mclk_pio_init(MCLK_PIO, MCLK_SM, MCLK_PIN);             // uncomment to enable I2S MCLK
#endif // MCLK

    ultranet_pio_init(UNET_PIO, UNET_SM, UNET_PIN);         // initialise and start ultranet state machine

#ifdef WS2812
    ws2812_pio_init(WS2812_PIO, WS2812_SM, WS2812_PIN);     // ws2812 output pio state machine
#endif // WS2812

#ifdef PICO_LED
    gpio_init(PICO_LED);                                    // set LED pin as GPIO 
    gpio_set_dir(PICO_LED, GPIO_OUT);                       // set LED pin as output
#endif // PICO_LED

    multicore_launch_core1(core1_entry);                    // start core 1

    sleep_ms(100);                                          // wait for core1 to start
#ifdef DEBUG
    sleep_ms(5000);
    puts("FINISHED setting everything up\n");
#ifdef WS2812
    while(true)
    {
        int inc = getchar();
        switch(inc)
        {
            case 'r':
            case 'R':
                pio_sm_put(WS2812_PIO, WS2812_SM, RED);
                puts("RED");
                break;
            case 'g':
            case 'G':
                pio_sm_put(WS2812_PIO, WS2812_SM, GREEN);
                puts("GREEN");
                break;
            case 'b':
            case 'B':
                pio_sm_put(WS2812_PIO, WS2812_SM, BLUE);
                puts("BLUE");
                break;
            case 'm':
            case 'M':
                pio_sm_put(WS2812_PIO, WS2812_SM, MAGENTA);
                puts("MAGENTA");
                break;
            case 'c':
            case 'C':
                pio_sm_put(WS2812_PIO, WS2812_SM, CYAN);
                puts("CYAN");
                break;
            case 'y':
            case 'Y':
                pio_sm_put(WS2812_PIO, WS2812_SM, YELLOW);
                puts("YELLOW");
                break;
            case 'w':
            case 'W':
                pio_sm_put(WS2812_PIO, WS2812_SM, WHITE);
                puts("WHITE");
                break;
        }
#endif // WS2812
    }
#endif // DEBUG

    // sync with ultranet frames initially, so we don't turn LED on at start 
    sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);        // get initial sample from Ultranet FIFO

    while((sample & 0x3F) != 0x0000000B && (sample & 0x3F) != 0x0000000F)
    {
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);    // get next sample from Ultranet FIFO
    }                                                       // "sample" now contains start frame

    while (true)
    {
        // synchronise with first subframe in Ultranet frame
        if((sample & 0x3F) == 0x0000000B || (sample & 0x3F) == 0x0000000F)
        {
            // if we get here, sample contains the first subframe in Ultranet frame
            samples[0] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[1] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[2] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[3] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[4] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[5] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[6] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs

            sample = pio_sm_get_blocking(UNET_PIO,UNET_SM); // get next sample from Ultranet FIFO
            samples[7] = (sample << 4) & 0xFFFFFC00;        // move 22 bits of audio into MSBs
        }
        else    // if we get here, we looked for start frame in the right place, but didn't find it
        {
#ifdef PIO_LED
            if(gpio_get(PICO_LED) == 0)
                gpio_put(PICO_LED, 1);                      // turn on LED to indicate frame error
            else
                gpio_put(PICO_LED, 0);
#endif // PIO_LED
#ifdef WS2812
            pio_sm_put(WS2812_PIO, WS2812_SM, RED);         // indicate framing error
#endif // PIO_LED
#ifdef DEBUG
            puts("TURN ON LED");
#endif // DEBUG
        }
        sample = pio_sm_get_blocking(UNET_PIO,UNET_SM);     // get next sample from Ultranet FIFO
    }
}
