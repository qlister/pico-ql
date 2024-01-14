#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

const int PIN_TX = 2;

class rgbw_led{

public:
    uint8_t b;
    uint8_t r;
    uint8_t g;
    uint8_t w;

};

static inline void put_pixel(uint32_t pixel_grbw) {
    pio_sm_put_blocking(pio0, 0, pixel_grbw );
}

static inline uint32_t urgbw_u32(rgbw_led led) {
    return (  (uint32_t)(led.w) |
            ((uint32_t)(led.b) << 8) |
            ((uint32_t)(led.r) << 16) |
            ((uint32_t)(led.g) << 24) );
}

static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return (  (uint32_t)(w) |
            ((uint32_t)(b) << 8) |
            ((uint32_t)(r) << 16) |
            ((uint32_t)(g) << 24) );
}

int main() {
    stdio_init_all();

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    char str[12];

    ws2812_program_init(pio, sm, offset, PIN_TX, 800000, true);

    while (true) {

        puts("Hello RGB and other colors!");

        rgbw_led led;

        led.r = 0xff;
        led.g = 0;
        led.b = 0;
        led.w = 0;

        put_pixel(urgbw_u32( led ));  // Red
        sleep_ms(500);
        put_pixel(urgbw_u32(0, 0xff, 0, 0));  // Green
        sleep_ms(500);
        put_pixel(urgbw_u32(0, 0, 0xff, 0));  // Blue
        sleep_ms(500);
        put_pixel(urgbw_u32(0x0, 0x0, 0x0, 0xff));  // White
        sleep_ms(500);
        put_pixel(urgbw_u32(0, 0, 0, 0));  // Black or off
        sleep_ms(500);
    }

}
