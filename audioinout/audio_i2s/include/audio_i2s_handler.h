

#ifdef __cplusplus
extern "C" {
#endif

#define PICO_AUDIO_I2S_PIO 0

typedef struct audio_i2s_config {
    uint8_t data_pin;
    uint8_t clock_pin_base;
    uint8_t dma_channel_a;
    uint8_t dma_channel_b;
    uint8_t pio_sm;
} audio_i2s_config_t;

void audio_i2s_out_setup( audio_i2s_config_t *config);
void audio_i2s_in_setup( audio_i2s_config_t *config);

static void update_pio_frequency(uint32_t sample_freq, uint sm );
void enable_pio_sms();

#define SAMPLE_COUNT_BITS   4
#define SAMPLE_COUNT    (1<<4)
#define BUFFER_SIZE (SAMPLE_COUNT * 2)      // L/R channels per sample

typedef struct  {  uint32_t in_a[BUFFER_SIZE*2];   // we have 32 bit L&R samples on input
                    uint32_t in_b[BUFFER_SIZE*2];
                    uint32_t out_a[BUFFER_SIZE];    // we have 16 bit L&R samples on output
                    uint32_t out_b[BUFFER_SIZE];
                } buffers_t __attribute__ ((aligned (BUFFER_SIZE*4)));

typedef struct  {
                    uint32_t in[BUFFER_SIZE];
                    uint32_t out[BUFFER_SIZE];
                } samples_t;

bool samples_ready();

#define FLAG_PIN_A 16
#define FLAG_PIN_B 17

#ifdef __cplusplus
}
#endif