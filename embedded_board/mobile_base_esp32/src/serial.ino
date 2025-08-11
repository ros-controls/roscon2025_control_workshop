#include <Arduino.h>

extern "C" {
#include "zenoh-pico.h"
}

#define MODE "client"
#define LOCATOR "serial/20.19#baudrate=115200"
#define KEYEXPR "demo/example/zenoh-pico-pub"
#define VALUE "[ARDUINO]{ESP32} Publication from Zenoh-Pico via Serial!"

z_owned_session_t s;
z_owned_publisher_t pub;
static int idx = 0;

void setup() {
  digitalWrite(RGB_BUILTIN, HIGH); // Turn the RGB LED white
  delay(1000);

  Serial.begin(115200);
  while (!Serial) {
    delay(1000);
  }

  digitalWrite(RGB_BUILTIN, LOW); // Turn the RGB LED off
  delay(1000);

  // Initialize Zenoh Session and other parameters
  z_owned_config_t config;
  z_config_default(&config);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, LOCATOR);

  // Open Zenoh session

  if (z_open(&s, z_config_move(&config), NULL) < 0) {
    while (1) {
      neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // Red
      delay(500);
      neopixelWrite(RGB_BUILTIN, 0, 0, 0); // Red
      delay(500);
    }
  }

  // Start read and lease tasks for zenoh-pico
  if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 ||
      zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
    z_session_drop(z_session_move(&s));
    while (1) {
      neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // Red
      delay(500);
      neopixelWrite(RGB_BUILTIN, 0, 0, 0); // Red
      delay(500);
    }
  }

  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // Red
  delay(1000);

  // Declare Zenoh publisher
  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
  if (z_declare_publisher(z_session_loan(&s), &pub, z_view_keyexpr_loan(&ke),
                          NULL) < 0) {
    while (1) {
      neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Red
      delay(500);
      neopixelWrite(RGB_BUILTIN, 0, 0, 0); // Red
      delay(500);
    }
  }

  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // Green
  delay(1000);
}

void loop() {
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Blue
  delay(1000);

  char buf[256];
  sprintf(buf, "[%4d] %s", idx++, VALUE);

  // Create payload
  z_owned_bytes_t payload;
  z_bytes_copy_from_str(&payload, buf);
  if (z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&payload), NULL) <
      0) {
    // Error while publishing data
  }

  neopixelWrite(RGB_BUILTIN, 0, 0, 0); // Off / black
  delay(1000);
}