#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/binary_info.h"
#include "audio_i2s_handler.h"

bi_decl(bi_3pins_with_names(PICO_AUDIO_I2S_IN_DATA_PIN, "I2S DIN", PICO_AUDIO_I2S_IN_CLOCK_PIN_BASE, "I2S BCK", PICO_AUDIO_I2S_IN_CLOCK_PIN_BASE+1, "I2S LRCK"));

#define NUM_BUFFERS 3   

#define SINE_WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 256

static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];

void init_audio_out() {

    audio_i2s_config_t audio_i2s_config = {
            .data_pin = PICO_AUDIO_I2S_IN_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_IN_CLOCK_PIN_BASE,
            .dma_channel_a = 0,     // DMA channel
            .dma_channel_b = 1,     // DMA channel
            .pio_sm = 0             // PIO state machine
    };

    audio_i2s_out_setup( &audio_i2s_config );

}

void init_audio_in() {

    audio_i2s_config_t audio_i2s_config = {
            .data_pin = PICO_AUDIO_I2S_OUT_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_OUT_CLOCK_PIN_BASE,
            .dma_channel_a = 2,     // DMA channel
            .dma_channel_b = 3,     // DMA channel
            .pio_sm = 1             // PIO state machine
    };

    audio_i2s_in_setup( &audio_i2s_config );

}


void init_gpio() {

    gpio_init(FLAG_PIN_A);
    gpio_init(FLAG_PIN_B);
    gpio_set_dir(FLAG_PIN_A, GPIO_OUT);
    gpio_set_dir(FLAG_PIN_B, GPIO_OUT);

}

/*
struct audio_buffer_pool *init_audio_in() {

    static audio_format_t audio_in_format = {
            .sample_freq = 24000,
//            .sample_freq = 312000,
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,      // This is the only format that is accepted!
            .channel_count = 1,
    };

    static struct audio_buffer_format consumer_format = {
            .format = &audio_in_format,
            .sample_stride = 2
    };

    struct audio_buffer_pool *consumer_pool = audio_new_consumer_pool(&consumer_format,
                                                                        NUM_BUFFERS,
                                                                        SAMPLES_PER_BUFFER); // todo correct size

    bool __unused ok;
    const struct audio_format *output_format;

    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_IN_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_IN_CLOCK_PIN_BASE,
            .dma_channel = 1,       // DMA channel
            .pio_sm = 1,            // PIO state machine
    };

    // Sets up the PIO program
    // Sets up the DMA
    output_format = audio_i2s_in_setup(&audio_in_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    // We have created the producer
    // The next call creates the consumer and connects our producer to the new consumer
//    ok = audio_i2s_connect(consumer_pool);
//    assert(ok);
    audio_i2s_set_enabled(true);

    return consumer_pool;

}
*/

/*
struct audio_buffer_pool *init_audio() {

    static audio_format_t audio_format = {
            .sample_freq = 24000,
//            .sample_freq = 312000,
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,      // This is the only format that is accepted!
            .channel_count = 1,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 2
    };
    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format,
                                                                        NUM_BUFFERS,
                                                                        SAMPLES_PER_BUFFER); // todo correct size

    bool __unused ok;
    const struct audio_format *output_format;

    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
            .dma_channel = 0,       // DMA channel
            .pio_sm = 0,            // PIO state machine
    };

    // Sets up the PIO program
    // Sets up the DMA
    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    // We have created the producer
    // The next call creates the consumer and connects our producer to the new consumer
    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);

    return producer_pool;
}
*/

int main()
{
    stdio_init_all();

    printf("Starting in Main\n");

    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
    }

    init_gpio();
    init_audio_out();
    init_audio_in();
    enable_pio_sms();

//
//    struct audio_buffer_pool *ap = init_audio();

//    struct audio_buffer_pool *api = init_audio_in();
    uint32_t step = 0x200000;
    uint32_t pos = 0;
    uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
    uint vol = 128;

    while (true) {
/*
        int c = getchar_timeout_us(0);
        if (c >= 0) {
            if (c == '-' && vol) vol -= 4;
            if ((c == '=' || c == '+') && vol < 255) vol += 4;
            if (c == '[' && step > 0x10000) step -= 0x10000;
            if (c == ']' && step < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step += 0x10000;
            if (c == 'q') break;

            printf("vol = %d, step = %d      \r", vol, step >> 16);

        }
*/
        if( samples_ready() ){

        }

//        gpio_put(FLAG_PIN, dma_channel_is_busy( 2 ) ? 1: 0 );

/*
        // Get an audio buffer - and block until one is free
        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        // Get number of samples
        int16_t *samples = (int16_t *) buffer->buffer->bytes;
        // Put the audio in the buffer
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i] = (vol * sine_wave_table[pos >> 16u]) >> 8u;
            pos += step;
            if (pos >= pos_max) pos -= pos_max;
        }
        buffer->sample_count = buffer->max_sample_count;
        // Now give the audio buffer so that it may be sent to the I2S device
        // this should may block
        give_audio_buffer(ap, buffer);
*/
    }
    puts("\n");

    puts("Hello, world!");

    return 0;
}
