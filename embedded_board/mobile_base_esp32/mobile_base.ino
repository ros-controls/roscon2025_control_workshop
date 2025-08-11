#include <Arduino.h>

extern "C" {
#include "zenoh-pico.h"
}

#define MODE "client"
#define CONNECT "serial/dev/ttyUSB0#baudrate=115200"
#define ZENOH_BAUDRATE 115200

// Zenoh session and publishers
z_owned_session_t s;
z_owned_publisher_t left_pos_pub;
z_owned_publisher_t left_vel_pub;
z_owned_publisher_t right_pos_pub;
z_owned_publisher_t right_vel_pub;

// Robot state
struct WheelState {
  float position;
  float velocity;
  float cmd_velocity;
};

WheelState left_wheel = {0.0, 0.0, 0.0};
WheelState right_wheel = {0.0, 0.0, 0.0};

// Timing
unsigned long last_update = 0;
unsigned long last_publish = 0;
const unsigned long UPDATE_INTERVAL = 10;
const unsigned long PUBLISH_INTERVAL = 50;
const float VELOCITY_GAIN = 5.0;

// Subscriber callbacks
void left_wheel_callback(const z_sample_t* sample, void* ctx) {
  char cmd_str[32];
  size_t len = min(sample->payload.len, sizeof(cmd_str) - 1);
  memcpy(cmd_str, sample->payload.start, len);
  cmd_str[len] = '\0';
  left_wheel.cmd_velocity = atof(cmd_str);
}

void right_wheel_callback(const z_sample_t* sample, void* ctx) {
  char cmd_str[32];
  size_t len = min(sample->payload.len, sizeof(cmd_str) - 1);
  memcpy(cmd_str, sample->payload.start, len);
  cmd_str[len] = '\0';
  right_wheel.cmd_velocity = atof(cmd_str);
}

void setup() {
  Serial.begin(ZENOH_BAUDRATE);
  while (!Serial) {
    delay(1000);
  }
  delay(500);

  // Initialize Zenoh
  z_owned_config_t config = z_config_default();
  zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(MODE));
  if (strcmp(CONNECT, "") != 0) {
    zp_config_insert(z_config_loan(&config), Z_CONFIG_CONNECT_KEY, z_string_make(CONNECT));
  }

  s = z_open(z_config_move(&config));
  if (!z_session_check(&s)) {
    while (1);
  }

  zp_start_read_task(z_session_loan(&s), NULL);
  zp_start_lease_task(z_session_loan(&s), NULL);

  // Declare publishers
  left_pos_pub = z_declare_publisher(z_session_loan(&s), z_keyexpr("/left_wheel/position_state"), NULL);
  left_vel_pub = z_declare_publisher(z_session_loan(&s), z_keyexpr("/left_wheel/velocity_state"), NULL);
  right_pos_pub = z_declare_publisher(z_session_loan(&s), z_keyexpr("/right_wheel/position_state"), NULL);
  right_vel_pub = z_declare_publisher(z_session_loan(&s), z_keyexpr("/right_wheel/velocity_state"), NULL);

  // Declare subscribers
  z_owned_subscriber_t left_sub = z_declare_subscriber(z_session_loan(&s), z_keyexpr("/left_wheel/velocity_command"), z_closure_sample(left_wheel_callback), NULL);
  z_owned_subscriber_t right_sub = z_declare_subscriber(z_session_loan(&s), z_keyexpr("/right_wheel/velocity_command"), z_closure_sample(right_wheel_callback), NULL);

  last_update = millis();
  last_publish = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - last_update >= UPDATE_INTERVAL) {
    updateWheelPhysics(now - last_update);
    last_update = now;
  }

  if (now - last_publish >= PUBLISH_INTERVAL) {
    publishStates();
    last_publish = now;
  }

  delay(1);
}

void updateWheelPhysics(unsigned long dt_ms) {
  float dt = dt_ms / 1000.0;

  left_wheel.velocity += (left_wheel.cmd_velocity - left_wheel.velocity) * VELOCITY_GAIN * dt;
  right_wheel.velocity += (right_wheel.cmd_velocity - right_wheel.velocity) * VELOCITY_GAIN * dt;

  left_wheel.position += left_wheel.velocity * dt;
  right_wheel.position += right_wheel.velocity * dt;

  left_wheel.position = fmod(left_wheel.position, 2.0 * PI);
  right_wheel.position = fmod(right_wheel.position, 2.0 * PI);
  if (left_wheel.position < 0) left_wheel.position += 2.0 * PI;
  if (right_wheel.position < 0) right_wheel.position += 2.0 * PI;
}

void publishStates() {
  char buffer[64];

  snprintf(buffer, sizeof(buffer), "%.4f", left_wheel.position);
  z_publisher_put(z_publisher_loan(&left_pos_pub), (const uint8_t*)buffer, strlen(buffer), NULL);

  snprintf(buffer, sizeof(buffer), "%.4f", left_wheel.velocity);
  z_publisher_put(z_publisher_loan(&left_vel_pub), (const uint8_t*)buffer, strlen(buffer), NULL);

  snprintf(buffer, sizeof(buffer), "%.4f", right_wheel.position);
  z_publisher_put(z_publisher_loan(&right_pos_pub), (const uint8_t*)buffer, strlen(buffer), NULL);

  snprintf(buffer, sizeof(buffer), "%.4f", right_wheel.velocity);
  z_publisher_put(z_publisher_loan(&right_vel_pub), (const uint8_t*)buffer, strlen(buffer), NULL);
}
