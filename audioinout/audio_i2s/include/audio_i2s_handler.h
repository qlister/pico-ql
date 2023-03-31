

#ifdef __cplusplus
extern "C" {
#endif

#define PICO_AUDIO_I2S_PIO 0

typedef struct audio_i2s_config {
    uint8_t data_pin;
    uint8_t clock_pin_base;
    uint8_t dma_channel;
    uint8_t pio_sm;
} audio_i2s_config_t;

void audio_i2s_out_setup( audio_i2s_config_t *config);
void audio_i2s_in_setup( audio_i2s_config_t *config);

static void update_pio_frequency(uint32_t sample_freq, uint sm );

#ifdef __cplusplus
}
#endif