/****************************************************************************
 http://retro.moe/unijoysticle2

 Copyright 2019 Ricardo Quesada

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 ****************************************************************************/

// ESP32 version

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "math.h"
#include "uni_config.h"
#include "uni_debug.h"
#include "uni_hid_device.h"
#include "uni_platform.h"

// Temporarily for this file only
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-const-variable="

// --- Consts

// GPIO for SPI interface

static const gpio_num_t GPIO_SPI_CSN = GPIO_NUM_5;
static const gpio_num_t GPIO_SPI_CLK = GPIO_NUM_16;
static const gpio_num_t GPIO_SPI_MOSI = GPIO_NUM_4;
static const gpio_num_t GPIO_SPI_MISO = GPIO_NUM_12;
static const gpio_num_t GPIO_SPI_WRITE_EN = GPIO_NUM_2;

static void spi_write_gamepad_state(const uni_joystick_t* joy);

// 20 milliseconds ~= 1 frame in PAL
// 17 milliseconds ~= 1 frame in NTSC
static const int AUTOFIRE_FREQ_MS = 20 * 4;  // change every ~4 frames

static const int MOUSE_DELAY_BETWEEN_EVENT_US = 1200;  // microseconds
// static const int MOUSE_MAX_DELTA = 32;

// --- Globals

static int64_t g_last_time_pressed_us = 0;  // in microseconds
static EventGroupHandle_t g_event_group;
static EventGroupHandle_t g_auto_fire_group;

// Mouse "shared data from main task to mouse task.
static int32_t g_delta_x = 0;
static int32_t g_delta_y = 0;

// Pot "shared data from main task to pot task.
static uint8_t g_pot_x = 0;
static uint8_t g_pot_y = 0;

// Autofire
static uint8_t g_autofire_a_enabled = 0;
static uint8_t g_autofire_b_enabled = 0;

// --- Code

// Mouse related
static void joy_update_port(uni_joystick_t* joy, const gpio_num_t* gpios);

#define MAX(a, b)           \
({                        \
__typeof__(a) _a = (a); \
__typeof__(b) _b = (b); \
_a > _b ? _a : _b;      \
})

#define MIN(a, b)           \
({                        \
__typeof__(a) _a = (a); \
__typeof__(b) _b = (b); \
_a < _b ? _a : _b;      \
})

void uni_platform_init(int argc, const char** argv) {
  UNUSED(argc);
  UNUSED(argv);
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  // GPIO configuration for SPI

  io_conf.pin_bit_mask = (
    1 << GPIO_SPI_CSN | 1 << GPIO_SPI_CLK | 1 << GPIO_SPI_MOSI | 1 << GPIO_SPI_WRITE_EN
  );

  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(GPIO_SPI_CSN, 1);

  // (set LED?)
}

void uni_platform_on_init_complete() {
  // (clear LED?)
}

void uni_platform_on_port_assign_changed(uni_joystick_port_t port) {
  bool port_status_a = ((port & JOYSTICK_PORT_A) != 0);
  bool port_status_b = ((port & JOYSTICK_PORT_B) != 0);
  // TODO: LED update based on (un)connected state
}

void uni_platform_on_mouse_data(int32_t delta_x, int32_t delta_y,
                                uint16_t buttons) {}

void uni_platform_on_joy_a_data(uni_joystick_t* joy) {
  // (restore if P2 is added)
}

void uni_platform_on_joy_b_data(uni_joystick_t* joy) {
  joy_update_port(joy, NULL);
}

uint8_t uni_platform_is_button_pressed() {
  return false;
}

static void joy_update_port(uni_joystick_t* joy, const gpio_num_t* gpios) {
  logd("up=%d, down=%d, left=%d, right=%d, fire=%d, potx=%d, poty=%d\n",
       joy->up, joy->down, joy->left, joy->right, joy->fire, joy->pot_x,
       joy->pot_y);

  // All updates are sent over SPI
  spi_write_gamepad_state(joy);
}

// Mouse handler
void handle_event_mouse() {}

// SPI write of gamepad state to FPGA

static void spi_write_gamepad_state(const uni_joystick_t* joy) {
  uint16_t pad_bits = joy->b;
  pad_bits = (pad_bits << 1) | joy->y;
  pad_bits = (pad_bits << 1) | joy->select;
  pad_bits = (pad_bits << 1) | joy->start;
  pad_bits = (pad_bits << 1) | joy->up;
  pad_bits = (pad_bits << 1) | joy->down;
  pad_bits = (pad_bits << 1) | joy->left;
  pad_bits = (pad_bits << 1) | joy->right;
  pad_bits = (pad_bits << 1) | joy->a;
  pad_bits = (pad_bits << 1) | joy->x;
  pad_bits = (pad_bits << 1) | joy->l;
  pad_bits = (pad_bits << 1) | joy->r;

  gpio_set_level(GPIO_SPI_CLK, 0);
  gpio_set_level(GPIO_SPI_WRITE_EN, 1);
  gpio_set_level(GPIO_SPI_CSN, 0);

  for (uint32_t i = 0; i < 8; i++) {
    gpio_set_level(GPIO_SPI_MOSI, pad_bits & 1);
    gpio_set_level(GPIO_SPI_CLK, 1);
    gpio_set_level(GPIO_SPI_CLK, 0);

    pad_bits >>= 1;
  }

  gpio_set_level(GPIO_SPI_CSN, 1);
}
