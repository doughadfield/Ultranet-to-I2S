
/*
* Header file for the Ultranet project
* contains all #includes and #defines for the 
* project C files
*/


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

#include "build/ultranet.pio.h"

// strings for inclusion in binary info (for query by picotool)
#define DESCRIPTION "Single Ultranet stream input, 4xI2S, 8xPWM"
#define VERSION "1.1"

// conditional compilation switches for hardware options
// #define DEBUG                    // enable debug code
#define WS2812                      // Our board has a ws2812 programmable LED
#define MCLK                        // Enable MCLK clock for I2S devices

                                    // 172000 for 7 slots per bit incoming Ultranet stream
#define CLOCKSPEED  196500          // 196500 for 8 slots per bit incoming Ultranet stream
#define AUDIV 8                     // Audio divider for pio timing (7 for 172MHz, 8 for 196.5MHz)
// Ultranet input and MCLK state machines use pio0
#define UNETL_PIN 0                 // ultranet low stream (1-8) input pin
// #define UNETH_PIN 1              // ultranet high stream (9-16) input pin
#define UNET_PIN UNETL_PIN          // ultranet default input pin
#define UNET_PIO pio0               // PIO module to use for Ultranet input
#define UNET_SM 0                   // state machine to use for Ultranet input
#ifdef MCLK                         // if we want an I2S MCLK clock
    #ifdef UNETH_PIN                // MCLK pin alternate to UNETH input on pin 1
        #define MCLK_PIN 24         // I2S Master Clock Pin
    #else
        #define MCLK_PIN 1          // I2S Master Clock Pin 
    #endif // UNETH_PIN
    #define MCLK_PIO pio0           // state machine for I2S master clock
    #define MCLK_SM 1               // state machine for I2S master clock
#endif // MCLK
// I2S outputs use second pio (pio1), four I2S outputs, 3 pins each
#define I2S_PIO pio1                // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 2                 // base for I2S output pins (3 pins starting point)
#define I2S2_PINS 5                 // base for I2S output pins (3 pins starting point)
#define I2S3_PINS 8                 // base for I2S output pins (3 pins starting point)
#define I2S4_PINS 11                // base for I2S output pins (3 pins starting point)
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
    #define BLACK 0                 // turn off all LEDs in module
    #define LED_ERR_COLOUR RED      // set colour for LED frame error indiication
    #define LED_STREAM_COLOUR BLUE  // set colour for LED stream indication
    #define LED_STREAM_MASK 0xFFFF00FF  // Mask blue bits, for stream detect LED
    #define put_pixel(pixel) pio_sm_put(WS2812_PIO, WS2812_SM, (pixel))
#endif // WS2812             
// #define PICO_LED 25                 // Uncomment to use normal LED on standard PICO boards
#define STREAM_LED_RESET 200000     // Period in us to reset stream indicator LED
// for PWM analog audio outputs
#define PIN_PWM_1A 14               // A channel of PWM slice (left audio)
#define PIN_PWM_1B 15               // B channel of PWM slice (right audio)
#define PIN_PWM_2A 18               // A channel of PWM slice (left audio)
#define PIN_PWM_2B 19               // B channel of PWM slice (right audio)
#define PIN_PWM_3A 26               // A channel of PWM slice (left audio)
#define PIN_PWM_3B 27               // B channel of PWM slice (right audio)
#define PIN_PWM_4A 28               // A channel of PWM slice (left audio)
#define PIN_PWM_4B 29               // B channel of PWM slice (right audio)

// these need to be "volatile" otherwise the compiler optimises them out!
extern volatile uint32_t samples[8]; // array of samples read from Ultranet stream
extern volatile uint32_t led_state; // current value last sent to WS2812 LED

extern void core1_entry(void);      // main process for core 1, defined in "core1.c"
extern void set_core1_info(void);   // set binary info for pins used by core1
