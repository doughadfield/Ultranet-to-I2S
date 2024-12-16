
/*
* Core 1 continuously reads the samples array and outputs the values to the I2S and PWM streams
* according to the pico internal clock (output frequency 48KHz)
* The samples array is continually filled by Core 0, synchronised to the incoming Ultranet stream
* This avoids audio sample sync issues as when the output sequence is running ahead, the Core 1 processing
* simply outputs the previous sample a second time, as it's still in the array.
* Or it outputs the new sample, missing one, if it's running behind.
* Provided the pico clock is at a close enough frequency to the incoming Ultranet stream, these corrections
* are infrequent and the samples very similar in amplitude, so there's no audible click or pop.
* 
* An optional selector switch on 3 pins determines which channels will be presented on which I2S or PWM outputs.
* The most significant switch pin selects the Ultranet stream (1-8 or 9-16), whereas the LS two pins select
* which pair of channels (1-2, 3-4, 5-6 or 7-8) appear on a particular I2S and PWM output pair.
* Switch is read once at boot time, so reset required when switch position changed.
*/

#include "ultranet.h"

volatile uint8_t slice[4];                                  // PWM slice numbers for specified pins

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

#define pwm_set_a(slice,num) pwm_set_chan_level((slice), PWM_CHAN_A, (uint16_t)(num))
#define pwm_set_b(slice,num) pwm_set_chan_level((slice), PWM_CHAN_B, (uint16_t)(num))

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
void set_core1_info(void)
{
    bi_decl(bi_pin_mask_with_name((1<<I2S1_PINS|(1<<I2S1_PINS+1)|(1<<I2S1_PINS+2)), "I2S_1 DATA,BCLK,LRCLK"));
    bi_decl(bi_pin_mask_with_name((1<<I2S2_PINS|(1<<I2S2_PINS+1)|(1<<I2S2_PINS+2)), "I2S_2"));
    bi_decl(bi_pin_mask_with_name((1<<I2S3_PINS|(1<<I2S3_PINS+1)|(1<<I2S3_PINS+2)), "I2S_3"));
    bi_decl(bi_pin_mask_with_name((1<<I2S4_PINS|(1<<I2S4_PINS+1)|(1<<I2S4_PINS+2)), "I2S_4"));
    bi_decl(bi_pin_mask_with_name((1<<SELECTOR_SW_BASE|(1<<SELECTOR_SW_BASE+1)|(1<<SELECTOR_SW_BASE+2)), "SELECTOR_SWITCH"));
    bi_decl(bi_4pins_with_names(PIN_PWM_1A, "PWM_1 Left", PIN_PWM_1B, "PWM_1 Right", PIN_PWM_2A, "PWM_2 Left", PIN_PWM_2B, "PWM_2 Right"));
    bi_decl(bi_4pins_with_names(PIN_PWM_3A, "PWM_3 Left", PIN_PWM_3B, "PWM_3 Right", PIN_PWM_4A, "PWM_4 Left", PIN_PWM_4B, "PWM_4 Right"));
}


void core1_entry(void)                                      // Core1 starts executiing here
{
    uint i2s_offset;                                        // position for i2s code in pio (shared for all SMs)
    volatile uint32_t ssample;                              // signed version of audio sample
    uint selector = get_selector();                         // read selector switch
    int count;                                              // general purpose counter
    uint ch[8], sel_sw;                                     // channel index into samples array from selector switch

    sel_sw = get_selector();                                // read selector switch GPIO pins
    for(count=0;count<8;count++)
    {
        ch[count] = (count + (sel_sw<<1)) & 7;              // offset channel number by selector switch setting
    }                                                       // (Lower two switch bits determine channel selection)

    pwm_setup();                                            // initialise PWM hardware and start outputs

#ifdef MCLK
    mclk_pio_init(MCLK_PIO, MCLK_SM, MCLK_PIN);             // uncomment to enable I2S MCLK
#endif // MCLK

    i2s_offset = pio_add_program(I2S_PIO, &i2s_program);    // load i2c output code once for all state machines
    i2s_pio_init(I2S_PIO, 0, I2S1_PINS, i2s_offset);        // all 4 state machines use the same code
    i2s_pio_init(I2S_PIO, 1, I2S2_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 2, I2S3_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 3, I2S4_PINS, i2s_offset);
    sleep_ms(200);                                          // wait for incoming samples to start
    // ensure there is data in each output FIFO before starting the state machines
    pio_sm_put_blocking(I2S_PIO, 0, samples[ch[0]]);        // subframe 1 goes to I2S0 channel 1
    pio_sm_put_blocking(I2S_PIO, 1, samples[ch[2]]);        // subframe 3 goes to I2S1 channel 1
    pio_sm_put_blocking(I2S_PIO, 2, samples[ch[4]]);        // subframe 5 goes to I2S2 channel 1
    pio_sm_put_blocking(I2S_PIO, 3, samples[ch[6]]);        // subframe 7 goes to I2S3 channel 1
    pio_sm_put_blocking(I2S_PIO, 0, samples[ch[1]]);        // subframe 2 goes to I2S0 channel 2
    pio_sm_put_blocking(I2S_PIO, 1, samples[ch[3]]);        // subframe 4 goes to I2S1 channel 2
    pio_sm_put_blocking(I2S_PIO, 2, samples[ch[5]]);        // subframe 6 goes to I2S2 channel 2
    pio_sm_put_blocking(I2S_PIO, 3, samples[ch[7]]);        // subframe 8 goes to I2S3 channel 2

    pio_set_sm_mask_enabled(I2S_PIO, 0xF, true);            // enable all I2S state machines at the same instant

    while(true)                                             // output samples synchronised with I2S streams
    {
        // continually load pio FIFOs for I2S outputs, and PWM registers for PWM outputs
        pio_sm_put_blocking(I2S_PIO, 0, samples[ch[0]]);    // subframe 1 goes to I2S0 channel 1

        ssample = (0x80000000 + (signed)samples[ch[0]]);
        pwm_set_a(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 1, samples[ch[2]]);    // subframe 3 goes to I2S1 channel 1

        ssample = (0x80000000 + (signed)samples[ch[2]]);
        pwm_set_a(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[ch[4]]);    // subframe 5 goes to I2S2 channel 1

        ssample = (0x80000000 + (signed)samples[ch[4]]);
        pwm_set_a(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[ch[6]]);    // subframe 7 goes to I2S3 channel 1

        ssample = (0x80000000 + (signed)samples[ch[6]]);
        pwm_set_a(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 0, samples[ch[1]]);    // subframe 2 goes to I2S0 channel 2

        ssample = (0x80000000 + (signed)samples[ch[1]]);
        pwm_set_b(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 1, samples[ch[3]]);    // subframe 4 goes to I2S1 channel 2

        ssample = (0x80000000 + (signed)samples[ch[3]]);
        pwm_set_b(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[ch[5]]);    // subframe 6 goes to I2S2 channel 2

        ssample = (0x80000000 + (signed)samples[ch[5]]);
        pwm_set_b(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[ch[7]]);    // subframe 8 goes to I2S3 channel 2

        ssample = (0x80000000 + (signed)samples[ch[7]]);
        pwm_set_b(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio
    }
}
