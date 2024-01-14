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

    uint32_t rgbw;

    //rgbw_led operator= (uint32_t obj ) {
    //    rgbw = obj;
    //    return ;
    //}

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
        leds[led].rgbw = myrgbw;
    }

    rgbw_led& operator [](int idx){
        return leds[idx];
    }

    
};

int main() {
    stdio_init_all();

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    char str[12];

    ws2812_program_init(pio, sm, offset, PIN_TX, 800000, true);

    led_string myLEDs;

    while (true) {

        // puts("Hello RGB and other colors!");

        myLEDs[0].rgbw = rgbw_led::RED;
        myLEDs[1].rgbw = rgbw_led::BLUE;
        myLEDs.update();
        sleep_ms(500);

        myLEDs[0].rgbw = rgbw_led::BLUE;
        myLEDs[1].rgbw = rgbw_led::RED;
        myLEDs.update();
        sleep_ms(500);

    }

}
