#include <Arduino.h>
#include <zenoh-pico.h>

#define BLINK_DURATION_MS 1000
#define MODE "client"
// #define CONNECT "serial/10.9#baudrate=115200"
#define CONNECT "serial/17.16#baudrate=115200"
// #define KEYEXPR "playground/hello/*"

z_owned_session_t s;
z_owned_subscriber_t sub;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // Red
  delay(BLINK_DURATION_MS);
  digitalWrite(RGB_BUILTIN, LOW); // Turn the RGB LED white

  Serial.begin(115200);
  while (!Serial) {
    delay(1000);
  }
  Serial.println("");
  Serial.println("Serial port initialized!");

  // Initialize Zenoh Session and other parameters
  z_owned_config_t config;
  z_config_default(&config);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, CONNECT);

  // Open Zenoh session
  Serial.print("Opening Zenoh Session... ");
  z_result_t res = z_open(&s, z_config_move(&config), NULL);
  Serial.print("Let's see how that session is doing... ");

  if (res < 0) {
    Serial.print("Unable to open session! Code: ");
    Serial.println(res);
    while (1) {
      neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // Red
      delay(BLINK_DURATION_MS);
      neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      delay(BLINK_DURATION_MS);
    }
  }
  Serial.println("Zenoh session opened successfully");

  // Start read and lease tasks for zenoh-pico
  Serial.print("Starting read and lease tasks... ");
  if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 ||
      zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
    Serial.println("Unable to start read and lease tasks");
    z_session_drop(z_session_move(&s));
    while (1) {
      delay(1000);
    }
  }
  Serial.println("Tasks leased successfully");
}

void loop() {
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Red
  delay(BLINK_DURATION_MS);
  digitalWrite(LED_BUILTIN, LOW);
  delay(BLINK_DURATION_MS);
}