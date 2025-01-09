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
* but can only set certain numbers of MHz, so 196500khz
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

#include "ultranet.h"

volatile uint32_t samples[8];   // array of samples read from Ultranet stream

void ultranet_gpio_init(void)
{
    int count;
    for(count=SELECTOR_SW_BASE; count < (SELECTOR_SW_BASE+3);count++)
    {
        gpio_init(count);
#ifdef SW_COMM_LOW                                          // switch common can be 0v or +3.3v
        gpio_pull_up(count);                                // for switch common to +3.3v
#else
        gpio_pull_down(count);                              // for switch common to +3.3v
#endif // SW_COMM_LOW
    }
#ifdef PICO_LED
    gpio_init(PICO_LED);                                    // set LED pin as GPIO 
    gpio_set_dir(PICO_LED, GPIO_OUT);                       // set LED pin as output
#endif // PICO_LED
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


#ifdef WS2812
volatile uint32_t led_state;                                // current value last sent to WS2812 LED
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
    led_state = BLACK;                                      // initialise WS2812 LED to all off
    pio_sm_put(pio, sm, led_state);                         // Clear all LED colours to off
}
#endif // WS2812

/*
* Timer callback for periodically turning off Ultranet detected LED
*  Outputs current state (as turned on by Ultranet stream code) then clears flag
* If no Ultranet stream received, LED will turn off at next alarm tick
*/
int64_t alarm_callback(alarm_id_t id, __unused void *repeatptr)
{
#ifdef WS2812
    put_pixel(led_state);                                   // Output current sate of led_state flag to LED
#endif // WS2812
#ifdef PICO_LED
    if((led_state & LED_STREAM_MASK) > 0)                   // led_state has been set by Ultranet stream code
        gpio_put(PICO_LED, 1);                              // turn on LED when stream is detected
    else
        gpio_put(PICO_LED, 0);                              // or turn off if no stream detected
#endif // PICO_LED
    led_state = led_state & LED_STREAM_MASK;                // Zero out the stream LED colour bits
    return *(const uint32_t*)repeatptr;                     // return value is repeat time
}

// Embedded binary information (for picotool interrogation of programmed device)
void set_binary_info(void)
{
    bi_decl(bi_program_description(DESCRIPTION));           // Description field for embedded identification 
    bi_decl(bi_program_version_string(VERSION));            // Version field for embedded identification
#ifdef UNETH_PIN
    bi_decl(bi_2pins_with_names(UNETL_PIN, "Ultranet Low (1-8) Stream Input", UNETH_PIN, "Ultranet High (9-16) Input"));
#else
    bi_decl(bi_1pin_with_name(UNET_PIN, "Ultranet Stream Input"));
#endif // UNETH_PIN
#ifdef MCLK
    bi_decl(bi_1pin_with_name(MCLK_PIN, "I2S MCLK Output"));
#endif // MCLK
#ifdef WS2812
    bi_decl(bi_1pin_with_name(WS2812_PIN, "WS2812 NeoPixel LED"));
#endif // WS2812
#ifdef PICO_LED
    bi_decl(bi_1pin_with_name(PICO_LED, "PICO board normal LED"));
#endif // PICO_LED
    set_core1_info();                                       // info for pins used by core1
}


// read selector switch and return uint with switch positions in the 3 LSBs
uint get_selector(void)
{
    static uint sw_mask = 0b111 << SELECTOR_SW_BASE;        // Mask for selecting only switch bits from all GPIOs

#ifdef SW_COMM_LOW                                          // sw pulls gpio pins low, so invert sw result
    return ((~gpio_get_all()) & sw_mask) >> SELECTOR_SW_BASE;
#else                                                       // sw pulls gpio pins high, so non-inverted result
    return (gpio_get_all() & sw_mask) >> SELECTOR_SW_BASE;
#endif // SW_COMM_LOW
}

int main()
{
    volatile uint32_t sample;                               // temp store for sample read from Ultranet stream
    const uint64_t repeat_us = STREAM_LED_RESET;            // Repeat time period for alarm to clear Ultranet stream LED
    uint selector;                                          // Selector switch state
 
    set_binary_info();                                      // info for querying by picotool
    stdio_init_all();                                       // initialise SDK libraries and interfaces
    set_sys_clock_khz(CLOCKSPEED,false);                    // set cpu clock frequency

#ifdef DEBUG
    sleep_ms(5000);                                         // allow time for USB serial to connect
#else
    sleep_ms(500);                                          // allow time for clocks etc. to settle
#endif // DEBUG

#ifdef WS2812
    ws2812_pio_init(WS2812_PIO, WS2812_SM, WS2812_PIN);     // ws2812 output pio state machine
#endif // WS2812

    ultranet_gpio_init();                                   // initialise required GPIO pins

    add_alarm_in_us(repeat_us, alarm_callback, (void*)&repeat_us, false);  // start timer for stream LED blanking

    selector = get_selector();                              // read selector switch once at boot time
#ifdef DEBUG
    printf("Selector = %d\n", selector);
#endif // DEBUG

    if(selector & 0b100)                                    // Most significant switch bit selects Ultranet input stream pin
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETH_PIN);    // initialise and start ultranet state machine
    else
        ultranet_pio_init(UNET_PIO, UNET_SM, UNETL_PIN);    // initialise and start ultranet state machine

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
    for(int count=0; count<200; count++)                    // discard the first 200 Ultranet frames after startup
        sample = pio_sm_get_blocking(UNET_PIO, UNET_SM);    // get frame word from Ultranet FIFO

    // now sync to start frame (starting with last sample read from FIFO)
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

            led_state = led_state | LED_STREAM_COLOUR;      // set selected LED on, preserving other colours
        }
        else    // if we get here, we looked for start frame in the right place, but didn't find it
        {
            led_state = led_state | LED_ERR_COLOUR;         // turn on RED, preserving other colours
        }
        sample = pio_sm_get_blocking(UNET_PIO,UNET_SM);     // get next sample from Ultranet FIFO
    }
}
