#include "plasma.hpp"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

// TODO I count 30 inputs on the board- 12 per player + 6 util so we're probably OK with 32 buttons * 4 LEDs * 4 bytes?
uint8_t led_front_buffer[32 * 4 * 4] = {0};

static constexpr uint MAX_BRIGHTNESS = 31;
static constexpr size_t LED_COUNT = sizeof(led_front_buffer) / 4;
static PIO plasma_pio = pio0;
static uint plasma_sm = 0;

static inline uint8_t scale_color(uint8_t color, uint8_t brightness) {
    return (color * brightness) / MAX_BRIGHTNESS;
}

static inline uint32_t make_ws2812_pixel(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    uint8_t scaled_r = scale_color(r, brightness);
    uint8_t scaled_g = scale_color(g, brightness);
    uint8_t scaled_b = scale_color(b, brightness);
    return (static_cast<uint32_t>(scaled_g) << 16u) |
           (static_cast<uint32_t>(scaled_r) << 8u) |
           static_cast<uint32_t>(scaled_b);
}

static void plasma_write_pixels() {
    for(size_t i = 0; i < LED_COUNT; i++) {
        size_t base = i * 4;
        uint8_t b = led_front_buffer[base + 0];
        uint8_t g = led_front_buffer[base + 1];
        uint8_t r = led_front_buffer[base + 2];
        uint8_t brightness = led_front_buffer[base + 3];
        if(brightness > MAX_BRIGHTNESS) {
            brightness = MAX_BRIGHTNESS;
        }
        uint32_t pixel = make_ws2812_pixel(r, g, b, brightness);
        // The WS2812 PIO program expects the data left-aligned in the 32-bit word.
        pio_sm_put_blocking(plasma_pio, plasma_sm, pixel << 8u);
    }
}

static void clear_front_buffer() {
    for(auto &value : led_front_buffer) {
        value = 0;
    }
}

void plasma_init() {
    clear_front_buffer();

    uint offset = pio_add_program(plasma_pio, &ws2812_program);
    plasma_sm = pio_claim_unused_sm(plasma_pio, true);
    ws2812_program_init(plasma_pio, plasma_sm, offset, PLASMA_DATA, 800000, false);
    plasma_flip();
}

void plasma_flip() {
    plasma_write_pixels();
}

void plasma_set_all(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    for(auto x = 0u; x < sizeof(led_front_buffer); x += 4) {
        led_front_buffer[x + 0] = b;
        led_front_buffer[x + 1] = g;
        led_front_buffer[x + 2] = r;
        led_front_buffer[x + 3] = brightness <= MAX_BRIGHTNESS ? brightness : MAX_BRIGHTNESS;
    }
    plasma_flip();
}
