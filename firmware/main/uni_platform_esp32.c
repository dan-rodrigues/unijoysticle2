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

// --- Consts

// GPIO for SPI interface

static const gpio_num_t GPIO_LED = GPIO_NUM_5;

static const gpio_num_t GPIO_SPI_CSN = GPIO_NUM_2;
static const gpio_num_t GPIO_SPI_CLK = GPIO_NUM_16;
static const gpio_num_t GPIO_SPI_MOSI = GPIO_NUM_4;

static void spi_write_gamepad_state(const uni_joystick_t* joy);

// --- Globals

static EventGroupHandle_t g_led_event_group;
static void led_event_loop(void* arg);

enum {
  EVENT_BIT_CONNECTED = (1 << 0),
  EVENT_BIT_INITIALIZING = (1 << 1),
  EVENT_BIT_LED_TOGGLING = (1 << 2)
};

// --- Code

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
    1 << GPIO_SPI_CSN | 1 << GPIO_SPI_CLK | 1 << GPIO_SPI_MOSI |
    1 << GPIO_LED
  );

  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(GPIO_SPI_CSN, 1);

  // Event group for LED blinking when pad disconnected
  g_led_event_group = xEventGroupCreate();

  // LED set until config complete
  xEventGroupSetBits(g_led_event_group, EVENT_BIT_INITIALIZING);
  xTaskCreate(led_event_loop, "led_event_loop", 2048, NULL, 10, NULL);
}

void uni_platform_on_init_complete() {
  // LED clear when config complete
  xEventGroupClearBits(g_led_event_group, EVENT_BIT_INITIALIZING);
}

// Loop that is solely responsible for updating status LED

static void led_event_loop(void* arg) {
  const TickType_t delay_ticks = 250 / portTICK_PERIOD_MS;

  while (true) {
    EventBits_t event_bits = xEventGroupGetBits(g_led_event_group);

    bool connected = event_bits & EVENT_BIT_CONNECTED;
    bool initializing = event_bits & EVENT_BIT_INITIALIZING;

    if (connected || initializing) {
      gpio_set_level(GPIO_LED, 1);
    } else {
      // Flash while disconnected
      bool led_on = (event_bits & EVENT_BIT_LED_TOGGLING) != 0;
      gpio_set_level(GPIO_LED, led_on);

      if (led_on) {
        xEventGroupClearBits(g_led_event_group, EVENT_BIT_LED_TOGGLING);
      } else {
        xEventGroupSetBits(g_led_event_group, EVENT_BIT_LED_TOGGLING);
      }
    }

    vTaskDelay(delay_ticks);
  }
}

void uni_platform_on_port_assign_changed(uni_joystick_port_t port) {
  // Port A unused now
  // bool port_status_a = ((port & JOYSTICK_PORT_A) != 0);
  bool port_status_b = ((port & JOYSTICK_PORT_B) != 0);

  // LED update based on (un)connected state
  if (port_status_b) {
    xEventGroupSetBits(g_led_event_group, EVENT_BIT_CONNECTED);
  } else {
    xEventGroupClearBits(g_led_event_group, EVENT_BIT_CONNECTED);
  }
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
  pad_bits |= joy->y << 1;
  pad_bits |= joy->select << 2;
  pad_bits |= joy->start << 3;
  pad_bits |= joy->up << 4;
  pad_bits |= joy->down << 5;
  pad_bits |= joy->left << 6;
  pad_bits |= joy->right << 7;
  pad_bits |= joy->a << 8;
  pad_bits |= joy->x << 9;
  pad_bits |= joy->l << 10;
  pad_bits |= joy->r << 11;

  gpio_set_level(GPIO_SPI_CLK, 0);
  gpio_set_level(GPIO_SPI_CSN, 0);

  for (uint32_t i = 0; i < 12; i++) {
    gpio_set_level(GPIO_SPI_MOSI, pad_bits & 1);
    gpio_set_level(GPIO_SPI_CLK, 1);
    gpio_set_level(GPIO_SPI_CLK, 0);

    pad_bits >>= 1;
  }

  gpio_set_level(GPIO_SPI_CSN, 1);
}
