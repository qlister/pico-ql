#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

const int PIN_TX = 2;

static const int LED_COUNT = 2;

class rgbw_led{

private:

    static constexpr int white_shift = 0;
    static constexpr int blue_shift = 8;
    static constexpr int red_shift = 16;
    static constexpr int green_shift = 24;

public:
    uint8_t b;
    uint8_t r;
    uint8_t g;
    uint8_t w;

    uint32_t rgbw;

    rgbw_led operator= (uint32_t obj ) {
        rgbw = obj;
//        return rgbw;
    }

    void set_colour( uint32_t col ){
        rgbw = col;
    }

    static constexpr uint32_t RED  = (uint32_t)( 0xff << red_shift );
    static constexpr uint32_t BLUE  = (uint32_t)( 0xff << blue_shift );
    static constexpr uint32_t GREEN  = (uint32_t)( 0xff << green_shift );
    static constexpr uint32_t WHITE  = (uint32_t)( 0xff << white_shift );
    static constexpr uint32_t YELLOW  = (uint32_t)( 0xff << green_shift | 0xff << red_shift );
    static constexpr uint32_t MAGENTA  = (uint32_t)( 0xff << blue_shift | 0xff << red_shift );
    static constexpr uint32_t CYAN  = (uint32_t)( 0xff << green_shift | 0xff  << blue_shift );
    static constexpr uint32_t OFF  = (uint32_t)( 0 );

};

class led_string{
private:
    rgbw_led leds[LED_COUNT];

    inline void put_pixel(uint32_t pixel_grbw) {
        pio_sm_put_blocking(pio0, 0, pixel_grbw );
    }

public:

    void update(){
        for( int i=0; i<LED_COUNT; i++){
            put_pixel( leds[i].rgbw );
        }
    }

    void set_colour( int led, uint32_t myrgbw ){
        leds[led] = myrgbw;
    }

    rgbw_led operator [](int idx){
        return leds[idx];
    }

    
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


static inline uint32_t urgbw_u32( uint32_t colour_32 ) {
    put_pixel( colour_32 );
    return 0;
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

    led_string myLEDs;

    while (true) {

        puts("Hello RGB and other colors!");

        //myLEDs[0] = rgbw_led::RED;
        //myLEDs[1] = rgbw_led::OFF;
        myLEDs.set_colour( 0, rgbw_led::RED );
        myLEDs.set_colour( 1, rgbw_led::OFF );
        myLEDs.update();
        sleep_ms(500);

        myLEDs.set_colour( 0, rgbw_led::OFF );
        myLEDs.set_colour( 1, rgbw_led::RED );
        myLEDs.update();
        sleep_ms(500);
/*
        rgbw_led led;

        led.r = 0xff;
        led.g = 0;
        led.b = 0;
        led.w = 0;

        put_pixel( rgbw_led::RED );  // Red
        sleep_ms(500);
        put_pixel( rgbw_led::GREEN );  // Green
        sleep_ms(500);
        put_pixel( rgbw_led::BLUE );  // Blue
        sleep_ms(500);
        put_pixel( rgbw_led::YELLOW );  // Yellow
        sleep_ms(500);
        put_pixel( rgbw_led::CYAN );  // Cyan
        sleep_ms(500);
        put_pixel( rgbw_led::MAGENTA );  // Magenta
        sleep_ms(500);
        put_pixel( rgbw_led::WHITE );  // White
        sleep_ms(500);
*/

    }

}
