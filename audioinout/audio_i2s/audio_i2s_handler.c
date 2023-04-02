
#include <stdio.h>

#include "pico/stdlib.h"

#include "audio_i2s_handler.h"
#include "audio_i2s.pio.h"

//#include "hardware/pio.h"
//#include "hardware/gpio.h"
#include "hardware/dma.h"
//#include "hardware/irq.h"

#include "hardware/clocks.h"
#include "hardware/sync.h"

uint8_t in_a_ready;
uint8_t in_b_ready;
uint8_t out_a_ready;
uint8_t out_b_ready;


struct {
    uint32_t freq_out;
    uint32_t freq_in;
    uint8_t pio_sm_out;
    uint8_t pio_sm_in;
    uint8_t dma_channel_out_a;
    uint8_t dma_channel_out_b;
    uint8_t dma_channel_in_a;
    uint8_t dma_channel_in_b;
} shared_state;


#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_I2S_PIO)

#define PICO_AUDIO_I2S_DMA_IRQ 0



static void __isr __time_critical_func(audio_i2s_dma_irq_handler)();


static void update_pio_frequency(uint32_t sample_freq, uint sm ) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(audio_pio, sm, divider >> 8u, divider & 0xffu);
}


volatile uint32_t jj;

void enable_pio_sms(){
    //    Enable the state machine
/*
    audio_pio->txf[0] = 0;
    audio_pio->txf[0] = 0;
    audio_pio->txf[0] = 0;
    audio_pio->txf[0] = 0;
    audio_pio->txf[0] = 0;

    jj = audio_pio->rxf[1];
    jj = audio_pio->rxf[1];
    jj = audio_pio->rxf[1];
    jj = audio_pio->rxf[1];
    jj = audio_pio->rxf[1];
*/

    dma_start_channel_mask( (1<<shared_state.dma_channel_out_a) );
    dma_start_channel_mask( (1<<shared_state.dma_channel_in_a) );

    pio_set_sm_mask_enabled ( audio_pio, (uint32_t) (1 << shared_state.pio_sm_out | 1 << shared_state.pio_sm_in ), true );
    irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, true);
}

buffers_t buffers;          // aligned buffers are instantiated here

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

    shared_state.freq_out = 40000;
    update_pio_frequency( shared_state.freq_out, sm );

    shared_state.dma_channel_out_a = config->dma_channel_a;
    shared_state.dma_channel_out_b = config->dma_channel_b;

    // Now we set up the DMA - channel A

    uint8_t dma_channel = shared_state.dma_channel_out_a;
    dma_channel_claim(dma_channel);

    // Set up the channel-A for the write
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq ( &dma_config, DREQ_PIO0_TX0 + sm );
    channel_config_set_read_increment( &dma_config, true );
    channel_config_set_write_increment( &dma_config, false );
    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_out_b);    // chain to channel b
    channel_config_set_transfer_data_size( &dma_config, DMA_SIZE_32 );
    dma_channel_set_config(dma_channel, &dma_config, false);

    dma_channel_set_write_addr( dma_channel, &audio_pio->txf[sm],  false );
    dma_channel_set_read_addr( dma_channel, buffers.out_a,  false );
    dma_channel_set_trans_count( dma_channel, SAMPLE_COUNT * 2 + 5, false);

    // Now we set up the DMA - channel B

    dma_channel = shared_state.dma_channel_out_b;
    dma_channel_claim(dma_channel);

    // Set up the channel-B for the read
    dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq ( &dma_config, DREQ_PIO0_TX0 + sm );
    channel_config_set_read_increment( &dma_config, true );
    channel_config_set_write_increment( &dma_config, false );
    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_out_a);    // chain to channel a
    channel_config_set_transfer_data_size( &dma_config, DMA_SIZE_32 );
    dma_channel_set_config(dma_channel, &dma_config, false);

    dma_channel_set_write_addr( dma_channel, &audio_pio->txf[sm],  false );
    dma_channel_set_read_addr( dma_channel, buffers.out_b,  false );
    dma_channel_set_trans_count( dma_channel, SAMPLE_COUNT * 2, false );

   // Enable the DMA channel A
    // - this will complete and then fire channel B
    // - which will complete and fire channel A again - forever
//    dma_start_channel_mask( (1<<shared_state.dma_channel_out_a) );

    irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, audio_i2s_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    dma_irqn_set_channel_enabled( PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_a, true );
    dma_irqn_set_channel_enabled( PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_b, true );

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
 
    shared_state.freq_in = 80000;
    update_pio_frequency( shared_state.freq_in, sm );

    shared_state.dma_channel_in_a = config->dma_channel_a;
    shared_state.dma_channel_in_b = config->dma_channel_b;
    static uint32_t zero;

    // Now we set up the DMA - channel A
    uint8_t dma_channel = shared_state.dma_channel_in_a;
    dma_channel_claim(dma_channel);

    // Set up the channel-A for the read
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq ( &dma_config, DREQ_PIO0_RX0 + sm );
    channel_config_set_read_increment( &dma_config, false );
    channel_config_set_write_increment( &dma_config, true );
//    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_in_a);    // chain to channel a (no chain)
    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_in_b);    // chain to channel b
    channel_config_set_transfer_data_size( &dma_config, DMA_SIZE_32 );
    dma_channel_set_config(dma_channel, &dma_config, false);

    //dma_channel_set_write_addr( dma_channel, &zero,  false );
    dma_channel_set_write_addr( dma_channel, buffers.in_a,  false );
    dma_channel_set_read_addr( dma_channel, &audio_pio->rxf[sm],  false );
    dma_channel_set_trans_count( dma_channel, SAMPLE_COUNT * 4, false);

    // Now we set up the DMA - channel B

    dma_channel = shared_state.dma_channel_in_b;
    dma_channel_claim(dma_channel);

    // Set up the channel-B for the read
    dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq ( &dma_config, DREQ_PIO0_RX0 + sm );
    channel_config_set_read_increment( &dma_config, false );
    channel_config_set_write_increment( &dma_config, true );
    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_in_a);    // chain to channel a 
//    channel_config_set_chain_to ( &dma_config, shared_state.dma_channel_in_b);    // chain to channel b (no chain)
    channel_config_set_transfer_data_size( &dma_config, DMA_SIZE_32 );
//    channel_config_set_ring	( &dma_config, true, SAMPLE_COUNT_BITS );     // true to limit the write to a ring

    dma_channel_set_config(dma_channel, &dma_config, false);

    dma_channel_set_write_addr( dma_channel, buffers.in_b,  false );
    dma_channel_set_read_addr( dma_channel, &audio_pio->rxf[sm],  false );
    dma_channel_set_trans_count( dma_channel, SAMPLE_COUNT * 4, false );

    // Enable the DMA channel A
    // - this will complete and then fire channel B
    // - which will complete and fire channel A again - forever
//    dma_start_channel_mask( (1<<shared_state.dma_channel_in_a) );

    irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, audio_i2s_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    dma_irqn_set_channel_enabled( PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_a, true );
    dma_irqn_set_channel_enabled( PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_b, true );
    
}

bool toggle = false;

bool samples_ready(){
    if ( in_b_ready && out_b_ready ){
        in_b_ready = false;
        out_b_ready = false;
//        toggle = !toggle;
//        gpio_put(FLAG_PIN, toggle ? 1: 0 );
        return true;
    }
    if ( in_a_ready && out_a_ready ){
        in_a_ready = false;
        out_a_ready = false;
//        toggle = !toggle;
//        gpio_put(FLAG_PIN, toggle ? 1: 0 );
        return true;
    }
    return false;
}

// 
/**
 * @brief irq handler for DMA
 * 
 */
void __isr __time_critical_func(audio_i2s_dma_irq_handler)() {

    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_a)) {
        in_b_ready = true;
        gpio_put(FLAG_PIN_B, 0 );
        dma_channel_set_write_addr( shared_state.dma_channel_in_a, buffers.in_a,  false );
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_a);
    }
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_b)) {
        dma_channel_set_write_addr( shared_state.dma_channel_in_b, buffers.in_b,  false );
        in_a_ready = true;
        gpio_put(FLAG_PIN_B, 1 );
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_in_b);
    }
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_a)) {
        out_b_ready = true;
        gpio_put(FLAG_PIN_A, 0 );
        dma_channel_set_trans_count( shared_state.dma_channel_out_a, SAMPLE_COUNT * 2, false);
        dma_channel_set_read_addr( shared_state.dma_channel_out_a, buffers.out_a,  false );
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_a);
    }
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_b)) {
        out_a_ready = true;
        gpio_put(FLAG_PIN_A, 1 );
        dma_channel_set_read_addr( shared_state.dma_channel_out_b, buffers.out_b,  false );
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, shared_state.dma_channel_out_b);
    }


    /*
//        DEBUG_PINS_SET(audio_timing, 4);
        // free the buffer we just finished
        if (shared_state.playing_buffer) {
            give_audio_buffer(audio_i2s_consumer, shared_state.playing_buffer);
#ifndef NDEBUG
            shared_state.playing_buffer = NULL;
#endif
        }
        audio_start_dma_transfer();
//        DEBUG_PINS_CLR(audio_timing, 4);
    }
    */

}