/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <string_view>
#include <ctype.h>  // isupper / islower
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "custom_gamepad.h"

#include "picade.hpp"
#include "plasma.hpp"
#include "rgbled.hpp"

#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/bootrom.h"
#include "hardware/structs/rosc.h"
#include "hardware/watchdog.h"
#include "pico/timeout_helper.h"

pimoroni::RGBLED led(17, 18, 19);

const size_t MAX_UART_PACKET = 64;

const size_t COMMAND_LEN = 4;
uint8_t command_buffer[COMMAND_LEN];
std::string_view command((const char *)command_buffer, COMMAND_LEN);


extern "C" {
void usb_serial_init(void);
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTES
//--------------------------------------------------------------------+

// Interface index depends on the order in configuration descriptor
enum {
  ITF_GAMEPAD_1,
  ITF_GAMEPAD_2,
  ITF_KEYBOARD,
  ITF_SERIAL,
  ITF_SERIAL_DATA,
};

void hid_task(void);
uint cdc_task(uint8_t *buf, size_t buf_len);

uint cdc_task(uint8_t *buf, size_t buf_len) {

    if (tud_cdc_connected()) {
        if (tud_cdc_available()) {
            return tud_cdc_read(buf, buf_len);
        }
    }

    return 0;
}

bool cdc_wait_for(std::string_view data, uint timeout_ms=50) {
    timeout_state ts;
    absolute_time_t until = delayed_by_ms(get_absolute_time(), timeout_ms);
    check_timeout_fn check_timeout = init_single_timeout_until(&ts, until);

    for(auto expected_char : data) {
        char got_char;
        while(1){
            tud_task();
            if (cdc_task((uint8_t *)&got_char, 1) == 1) break;
            if(check_timeout(&ts, false)) return false;
        }
        if (got_char != expected_char) return false;
    }
    return true;
}

size_t cdc_get_bytes(const uint8_t *buffer, const size_t len, const uint timeout_ms=1000) {
    memset((void *)buffer, len, 0);

    uint8_t *p = (uint8_t *)buffer;

    timeout_state ts;
    absolute_time_t until = delayed_by_ms(get_absolute_time(), timeout_ms);
    check_timeout_fn check_timeout = init_single_timeout_until(&ts, until);

    size_t bytes_remaining = len;
    while (bytes_remaining && !check_timeout(&ts, false)) {
        tud_task(); // tinyusb device task
        size_t bytes_read = cdc_task(p, std::min(bytes_remaining, MAX_UART_PACKET));
        bytes_remaining -= bytes_read;
        p += bytes_read;
    }
    return len - bytes_remaining;
}

// New multiverse bridge parsing
static const char MULTIVERSE_HEADER[] = "multiverse:data";
static const size_t MULTIVERSE_HEADER_LEN = sizeof(MULTIVERSE_HEADER) - 1;

// Apply frame: payload = N * (B,G,R,brightness(0..255))
static void apply_multiverse_frame(const uint8_t *payload, size_t len) {
    if (len == 0 || (len % 4) != 0) return;
    size_t n = len / 4;
    size_t max_pixels = sizeof(led_front_buffer) / 4; // number of pixels supported by firmware
    if (n > max_pixels) n = max_pixels;

    for (size_t i = 0; i < n; ++i) {
        size_t idx = i * 4;
        uint8_t b = payload[idx + 0];
        uint8_t g = payload[idx + 1];
        uint8_t r = payload[idx + 2];
        uint8_t br = payload[idx + 3];

        // Map brightness 0..255 to firmware's 0..MAX_BRIGHTNESS (defined in plasma.cpp)
        // MAX_BRIGHTNESS is 31 in plasma.cpp. We map proportionally.
        const uint8_t MAX_BRIGHT = 31;
        uint8_t mapped_br = (uint8_t)((((uint32_t)br) * MAX_BRIGHT) / 255);

        // Store as B,G,R,brightness (firmware expects this layout)
        size_t base = i * 4;
        led_front_buffer[base + 0] = b;
        led_front_buffer[base + 1] = g;
        led_front_buffer[base + 2] = r;
        led_front_buffer[base + 3] = mapped_br;
    }

    // If host sent fewer pixels than buffer, zero remaining pixels
    size_t total_pixels = sizeof(led_front_buffer) / 4;
    for (size_t i = n; i < total_pixels; ++i) {
        size_t base = i * 4;
        led_front_buffer[base + 0] = 0;
        led_front_buffer[base + 1] = 0;
        led_front_buffer[base + 2] = 0;
        led_front_buffer[base + 3] = 0;
    }

    plasma_flip();
}

// Startup animation similar to tools/code.py
static void startup_animation(size_t first_n = 7, uint32_t dwell_ms = 500, int fade_steps = 60, uint32_t fade_ms = 20, uint8_t white = 255) {
    size_t max_pixels = sizeof(led_front_buffer) / 4;
    if (first_n == 0) return;
    if (first_n > max_pixels) first_n = max_pixels;

    // all off
    for (size_t i = 0; i < first_n; ++i) {
        size_t base = i * 4;
        led_front_buffer[base + 0] = 0;
        led_front_buffer[base + 1] = 0;
        led_front_buffer[base + 2] = 0;
        led_front_buffer[base + 3] = 0;
    }
    plasma_flip();

    const uint8_t MAX_BRIGHT = 31; // matches plasma.cpp

    // per-LED: red -> green -> blue -> stay white
    for (size_t i = 0; i < first_n; ++i) {
        // red
        size_t base = i * 4;
        led_front_buffer[base + 0] = 0;         // b
        led_front_buffer[base + 1] = 0;         // g
        led_front_buffer[base + 2] = white;     // r
        led_front_buffer[base + 3] = MAX_BRIGHT;
        plasma_flip();
        sleep_ms(dwell_ms);

        // green
        led_front_buffer[base + 0] = 0;
        led_front_buffer[base + 1] = white;
        led_front_buffer[base + 2] = 0;
        led_front_buffer[base + 3] = MAX_BRIGHT;
        plasma_flip();
        sleep_ms(dwell_ms);

        // blue
        led_front_buffer[base + 0] = white;
        led_front_buffer[base + 1] = 0;
        led_front_buffer[base + 2] = 0;
        led_front_buffer[base + 3] = MAX_BRIGHT;
        plasma_flip();
        sleep_ms(dwell_ms);

        // white
        led_front_buffer[base + 0] = white;
        led_front_buffer[base + 1] = white;
        led_front_buffer[base + 2] = white;
        led_front_buffer[base + 3] = MAX_BRIGHT;
        plasma_flip();
        sleep_ms(dwell_ms);
    }

    // fade all to off by decreasing brightness
    for (int step = fade_steps; step >= 0; --step) {
        uint8_t mapped_b = (uint8_t)(((uint32_t)step * MAX_BRIGHT) / fade_steps);
        for (size_t i = 0; i < first_n; ++i) {
            size_t base = i * 4;
            // keep color white, change brightness
            led_front_buffer[base + 0] = white;
            led_front_buffer[base + 1] = white;
            led_front_buffer[base + 2] = white;
            led_front_buffer[base + 3] = mapped_b;
        }
        plasma_flip();
        sleep_ms(fade_ms);
    }

    // ensure all off
    for (size_t i = 0; i < first_n; ++i) {
        size_t base = i * 4;
        led_front_buffer[base + 0] = 0;
        led_front_buffer[base + 1] = 0;
        led_front_buffer[base + 2] = 0;
        led_front_buffer[base + 3] = 0;
    }
    plasma_flip();
}

/*------------- MAIN -------------*/
int main(void)
{
  // Apply a modest overvolt, default is 1.10v.
  // this is required for a stable 250MHz on some RP2040s
  //vreg_set_voltage(VREG_VOLTAGE_1_20);
  //sleep_ms(10);
  //set_sys_clock_khz(250000, true);

  led.set_rgb(255, 0, 0);
  board_init();
  // Fetch the Pico serial (actually the flash chip ID) into `usb_serial`
  usb_serial_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  led.set_rgb(0, 0, 255);

  picade_init();
  plasma_init();

  // run startup animation for first 7 LEDs (matches tools/code.py)
  startup_animation(7, 500, 60, 20, 255);

  led.set_rgb(0, 255, 0);

  // Buffer for incoming CDC bytes
  static uint8_t usb_buf[2048];
  static size_t usb_buf_len = 0;

  while (1)
  {
    tud_task();
    hid_task();

    // Read available CDC bytes into usb_buf
    if (tud_cdc_connected()) {
      int r = cdc_task(usb_buf + usb_buf_len, sizeof(usb_buf) - usb_buf_len);
      if (r > 0) {
          usb_buf_len += (size_t)r;
          // Parse for MULTIVERSE_HEADER ... payload ... [MULTIVERSE_HEADER ...]
          while (true) {
              // find header
              size_t i = SIZE_MAX;
              if (usb_buf_len >= MULTIVERSE_HEADER_LEN) {
                  for (size_t k = 0; k + MULTIVERSE_HEADER_LEN <= usb_buf_len; ++k) {
                      if (memcmp(usb_buf + k, MULTIVERSE_HEADER, MULTIVERSE_HEADER_LEN) == 0) { i = k; break; }
                  }
              }

              if (i == SIZE_MAX) {
                  // header not found
                  if (usb_buf_len > MULTIVERSE_HEADER_LEN) {
                      // keep a short tail in case header split
                      size_t tail = MULTIVERSE_HEADER_LEN;
                      memmove(usb_buf, usb_buf + usb_buf_len - tail, tail);
                      usb_buf_len = tail;
                  }
                  break;
              }

              // move header to start if not at 0
              if (i > 0) {
                  memmove(usb_buf, usb_buf + i, usb_buf_len - i);
                  usb_buf_len -= i;
              }

              // find next header after this one
              size_t j = SIZE_MAX;
              if (usb_buf_len >= MULTIVERSE_HEADER_LEN * 2) {
                  for (size_t k = MULTIVERSE_HEADER_LEN; k + MULTIVERSE_HEADER_LEN <= usb_buf_len; ++k) {
                      if (memcmp(usb_buf + k, MULTIVERSE_HEADER, MULTIVERSE_HEADER_LEN) == 0) { j = k; break; }
                  }
              }

              if (j == SIZE_MAX) {
                  // no next header — take remainder as payload candidate
                  size_t payload_len = (usb_buf_len > MULTIVERSE_HEADER_LEN) ? (usb_buf_len - MULTIVERSE_HEADER_LEN) : 0;
                  if (payload_len > 0 && (payload_len % 4) == 0) {
                      apply_multiverse_frame(usb_buf + MULTIVERSE_HEADER_LEN, payload_len);
                      // consume buffer
                      usb_buf_len = 0;
                  } else {
                      // keep data in buffer until more arrives (or trim to header tail)
                      if (usb_buf_len > MULTIVERSE_HEADER_LEN * 2) {
                          // prevent uncontrolled growth
                          size_t tail = MULTIVERSE_HEADER_LEN * 2;
                          memmove(usb_buf, usb_buf + usb_buf_len - tail, tail);
                          usb_buf_len = tail;
                      }
                  }
                  break;
              } else {
                  // payload between header and next header
                  size_t payload_len = j - MULTIVERSE_HEADER_LEN;
                  if (payload_len > 0 && (payload_len % 4) == 0) {
                      apply_multiverse_frame(usb_buf + MULTIVERSE_HEADER_LEN, payload_len);
                  }
                  // move remaining bytes (from next header) to front
                  size_t remain = usb_buf_len - j;
                  memmove(usb_buf, usb_buf + j, remain);
                  usb_buf_len = remain;
                  // continue parsing next packet(s)
              }
          }
      }

      // Also keep support for older command style: look for "multiverse:" then 4-byte command
      // (This keeps compatibility with existing hosts that send the fixed-size buffer.)
      if (usb_buf_len == 0 && tud_cdc_available()) {
          // quick non-blocking attempt to parse legacy commands
          if (cdc_wait_for("multiverse:", 10)) {
              if (cdc_get_bytes(command_buffer, COMMAND_LEN) == COMMAND_LEN) {
                  if (command == "data") {
                      // read full led_front_buffer if available
                      if (cdc_get_bytes(led_front_buffer, sizeof(led_front_buffer)) == sizeof(led_front_buffer)) {
                          plasma_flip();
                      }
                  } else if (command == "_rst") {
                      sleep_ms(500);
                      save_and_disable_interrupts();
                      rosc_hw->ctrl = ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB;
                      watchdog_reboot(0, 0, 0);
                  } else if (command == "_usb") {
                      sleep_ms(500);
                      save_and_disable_interrupts();
                      rosc_hw->ctrl = ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB;
                      reset_usb_boot(0, 0);
                  }
              }
          }
      }
    }

  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 1;
  static uint32_t start_ms = 0;
  static bool state = false;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  input_t in = picade_get_input();

  if(in.changed) {
    state = !state;
    led.set_rgb(255 * state, 0, 0);
  }

  // Remote wakeup
  if ( tud_suspended() && (in.changed) )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }

  /*------------- Keyboard -------------*/
  if ( tud_hid_n_ready(ITF_KEYBOARD) )
  {
    /*
    // use to avoid send multiple consecutive zero report for keyboard
    static uint8_t last_util = 0;

    if ( in.changed && in.util != last_util )
    {
      uint8_t keycode[6] = {
        (uint8_t)((in.util & (UTIL_P1_HOTKEY | UTIL_P2_HOTKEY)) ? HID_KEY_ESCAPE : 0u),
      };
    
      tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);

      last_util = in.util;
    }
    */
  }

  if ( tud_hid_n_ready(ITF_GAMEPAD_1) )
  {
    //tud_hid_n_gamepad_report(ITF_GAMEPAD_1, 0, in.p1_x, in.p1_y, 0, 0, 0, 0, 0, in.p1 & BUTTON_MASK);
    uint16_t extra = 0;
    extra |= (in.util & UTIL_P1_HOTKEY) ? (1 << 12) : 0;
    extra |= (in.util & UTIL_P1_X1) ? (1 << 13) : 0;
    extra |= (in.util & UTIL_P1_X2) ? (1 << 14) : 0;
    picade_gamepad_report(ITF_GAMEPAD_1, in.p1_x, in.p1_y, (in.p1 & BUTTON_MASK) | extra);
  }

  if ( tud_hid_n_ready(ITF_GAMEPAD_2) )
  {
    //tud_hid_n_gamepad_report(ITF_GAMEPAD_2, 0, in.p2_x, in.p2_y, 0, 0, 0, 0, 0, in.p2 & BUTTON_MASK);
    uint16_t extra = 0;
    extra |= (in.util & UTIL_P2_HOTKEY) ? (1 << 12) : 0;
    extra |= (in.util & UTIL_P2_X1) ? (1 << 13) : 0;
    extra |= (in.util & UTIL_P2_X2) ? (1 << 14) : 0;
    picade_gamepad_report(ITF_GAMEPAD_2, in.p2_x, in.p2_y, (in.p2 & BUTTON_MASK) | extra);
  }
}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // TODO set LED based on CAPLOCK, NUMLOCK etc...
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+

/*
void cdc_task(void)
{
  const uint32_t interval_ms = 500;
  static uint32_t start_ms = 0;
  static uint32_t ptr = 0;
  //uint8_t buf[64] = {0};

  if( tud_cdc_available() ) {
    //ptr = tud_cdc_read(led_buffer + ptr, sizeof(led_buffer));
    //if(ptr >= sizeof(led_buffer)){
    //  ptr = 0;
    //}
  }

  if(tud_cdc_connected()) {
#ifdef INPUT_DEBUG
    if ( board_millis() - start_ms < interval_ms) return; // not enough time
    start_ms += interval_ms;
    tud_cdc_write_str("input: ");
    for (auto i = 0u; i < 8; i++) {
      uint8_t in = input_debug[i];
      for (auto j = 0u; j < 8; j++) {
        if (in & (0b10000000 >>  j)) {
          tud_cdc_write_str("1");
        } else {
          tud_cdc_write_str("0");
        }
      }
      tud_cdc_write_str(" ");
    }
    tud_cdc_write_str("\r\n");
    tud_cdc_write_flush();
#endif
  }
}
*/
