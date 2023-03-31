
#include <stdio.h>

#include "audio_i2s_handler.h"

#include "audio_i2s.pio.h"

//#include "hardware/pio.h"
//#include "hardware/gpio.h"
//#include "hardware/dma.h"
//#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"


struct {
    uint32_t freq_out;
    uint32_t freq_in;
    uint8_t pio_sm_out;
    uint8_t pio_sm_in;
    uint8_t dma_channel;
    uint8_t dma_channel_in;
} shared_state;


#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_I2S_PIO)

static void update_pio_frequency(uint32_t sample_freq, uint sm ) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(audio_pio, sm, divider >> 8u, divider & 0xffu);
}
/*
*/
void audio_i2s_out_setup( audio_i2s_config_t *config){

    uint func_pio = GPIO_FUNC_PIOx;
    gpio_set_function(config->data_pin, func_pio);
    gpio_set_function(config->clock_pin_base, func_pio);
    gpio_set_function(config->clock_pin_base + 1, func_pio);

    uint8_t sm = shared_state.pio_sm_out = config->pio_sm;
    pio_sm_claim(audio_pio, sm);

    uint offset = pio_add_program(audio_pio, &audio_i2s_out_program);

    audio_i2s_out_program_init(audio_pio, sm, offset, config->data_pin, config->clock_pin_base);

    __mem_fence_release();
 //   uint8_t dma_channel = config->dma_channel;

    shared_state.freq_out = 70000;
    update_pio_frequency( shared_state.freq_out, shared_state.pio_sm_out );

    pio_sm_set_enabled(audio_pio, shared_state.pio_sm_out, true);

    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
    audio_pio->txf[sm]  = 0xaa00aa00;
}

void audio_i2s_in_setup( audio_i2s_config_t *config){

    uint func_pio = GPIO_FUNC_PIOx;
    gpio_set_function(config->data_pin, func_pio);
    gpio_set_function(config->clock_pin_base, func_pio);
    gpio_set_function(config->clock_pin_base + 1, func_pio);

    uint8_t sm = shared_state.pio_sm_in = config->pio_sm;
    pio_sm_claim(audio_pio, sm);

    uint offset = pio_add_program(audio_pio, &audio_i2s_in_program);

    audio_i2s_in_program_init(audio_pio, sm, offset, config->data_pin, config->clock_pin_base);

    __mem_fence_release();
 //   uint8_t dma_channel = config->dma_channel;

    shared_state.freq_in = 80000;
    update_pio_frequency( shared_state.freq_in, shared_state.pio_sm_in );

    pio_sm_set_enabled(audio_pio, shared_state.pio_sm_in, true);

    uint32_t v0;
    uint32_t v1;
    uint32_t v2;
    uint32_t v3;

    v0 = audio_pio->rxf[sm];
    v1 = audio_pio->rxf[sm];
    v2 = audio_pio->rxf[sm];
    v3 = audio_pio->rxf[sm];
    v0 = audio_pio->rxf[sm];
    v1 = audio_pio->rxf[sm];
    v2 = audio_pio->rxf[sm];
    v3 = audio_pio->rxf[sm];

}