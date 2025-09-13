#include <math.h>  // For fmod() and M_PI
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "led_strip_esp32.h"
#include "math.h"
#include "nvs_flash.h"
#include "picoros.h"
#include "picoserdes.h"

// Communication mode
#define MODE "client"
// The baudrate is set to 460800 to keep up with the 250Hz publishing rate
// of the JointState publisher task.
#define ROUTER_ADDRESS "serial/UART_0#baudrate=460800"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Subscriber callback
void joint_state_callback(uint8_t * rx_data, size_t data_len);

// Constraint helper function
template <typename T>
T constrain(T value, T lower_bound, T upper_bound)
{
  if (value < lower_bound)
  {
    return lower_bound;
  }
  else if (value > upper_bound)
  {
    return upper_bound;
  }
  else
  {
    return value;
  }
}

// A helper function to get milliseconds
uint32_t get_milliseconds() { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

/* -------------------------------------- */
// Setup JointState publisher and subscriber
picoros_publisher_t pub_js = {
  .topic =
    {
      .name = "picoros/joint_states",
      .type = ROSTYPE_NAME(ros_JointState),
      .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

picoros_subscriber_t sub_js = {
  .topic =
    {
      .name = "picoros/joint_commands",
      .type = ROSTYPE_NAME(ros_JointState),
      .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
  .user_callback = joint_state_callback,
};

// Embedded mobile base node (you can see this on the host if you run `ros2 node list`)
picoros_node_t node = {
  .name = "wbot_picoros_node",
};

static const int num_joints = 3;
// the latest velocity command from the subscriber
volatile double cmd_vel[num_joints];
// the calculated desired position for the joints given the velocity command
volatile double desired_position[num_joints];
// Variables to hold time in milliseconds
unsigned long current_time_ms = 0;
unsigned long last_update_time_ms = 0;
// Buffer for publication, used from this thread
uint8_t pub_buf[1024];

portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void update_led_based_on_velocity(double velocity_left, double velocity_right)
{
  // Calculate motion characteristics
  double avg_vel = (velocity_left + velocity_right) / 2.0;  // Average velocity (forward/backward)
  double diff_vel = velocity_left - velocity_right;         // Velocity difference (turning)

  // Thresholds
  const double min_vel = 0.05;        // Minimum velocity to consider "moving"
  const double max_vel = 2.0;         // Maximum expected velocity for scaling
  const double turn_threshold = 0.1;  // Minimum difference to consider "turning"

  // Check if robot is essentially stopped
  if (abs(avg_vel) < min_vel && abs(diff_vel) < turn_threshold)
  {
    // Robot stopped - turn off LED
    set_led_color(0, 0, 0);
    return;
  }

  // Initialize RGB values
  int red = 0, green = 0, blue = 0;

  // Check if mainly driving straight (forward/backward)
  if (abs(diff_vel) < turn_threshold)
  {
    // Driving straight - use RED proportional to speed
    int intensity = (int)(abs(avg_vel) / max_vel * 255.0);
    intensity = constrain(intensity, 0, 255);
    red = intensity;
  }
  // Check if mainly turning in place
  else if (abs(avg_vel) < min_vel)
  {
    // Pure turning
    int intensity = (int)(abs(diff_vel) / max_vel * 255.0);
    intensity = constrain(intensity, 0, 255);

    if (diff_vel > 0)
    {
      // Turning left - BLUE
      blue = intensity;
    }
    else
    {
      // Turning right - GREEN
      green = intensity;
    }
  }
  // Arc movement - mix colors
  else
  {
    // Calculate base intensities
    int forward_intensity = (int)(abs(avg_vel) / max_vel * 255.0);
    int turn_intensity = (int)(abs(diff_vel) / max_vel * 255.0);

    forward_intensity = constrain(forward_intensity, 0, 255);
    turn_intensity = constrain(turn_intensity, 0, 255);

    // Mix red (forward/backward) with turn color
    red = forward_intensity;

    if (diff_vel > 0)
    {
      // Arc left - mix RED + BLUE
      blue = turn_intensity;
    }
    else
    {
      // Arc right - mix RED + GREEN
      green = turn_intensity;
    }

    // Scale down to prevent oversaturation
    double scale = 1.0;
    int max_component = MAX(red, MAX(green, blue));
    if (max_component > 255)
    {
      scale = 255.0 / max_component;
    }

    red = (int)(red * scale);
    green = (int)(green * scale);
    blue = (int)(blue * scale);
  }

  // Update the LED
  set_led_color(red, green, blue);
}

// This function calculates joints positions given the current velocity commands.
void execute_commands_and_update_robot_state()
{
  taskENTER_CRITICAL(&my_spinlock);  // Begin critical section

  current_time_ms = get_milliseconds();
  double dt = (double)(current_time_ms - last_update_time_ms) / 1000.0;
  last_update_time_ms = current_time_ms;

  // Update desired position for each joint
  for (int i = 0; i < num_joints; ++i)
  {
    desired_position[i] += cmd_vel[i] * dt;

    // Normalize the position to a range of [-pi, pi]
    desired_position[i] = fmod(desired_position[i], 2.0 * M_PI);
    if (desired_position[i] > M_PI)
    {
      desired_position[i] -= 2.0 * M_PI;
    }
    if (desired_position[i] < -M_PI)
    {
      desired_position[i] += 2.0 * M_PI;
    }
  }

  taskEXIT_CRITICAL(&my_spinlock);  // End of the critical section
}

void joint_state_callback(uint8_t * rx_data, size_t data_len)
{
  // Define arrays to hold the data
  char * joint_names[num_joints] = {};
  double joint_positions[num_joints] = {};
  double joint_velocities[num_joints] = {};
  double joint_efforts[num_joints] = {};

  // Create the JointState message and initialize it with the desired array sizes
  // ros2_control HW interface is only sending each joint a velocity command because
  // this example is paired with a diff drive controller.
  ros_JointState js = {
    .name = {.data = joint_names, .n_elements = num_joints},
    .position = {.data = joint_positions, .n_elements = 0},
    .velocity = {.data = joint_velocities, .n_elements = num_joints},
    .effort = {.data = joint_efforts, .n_elements = 0},
  };
  if (ps_deserialize(rx_data, &js, data_len))
  {
    for (auto i = 0; i < num_joints; i++)
    {
      cmd_vel[i] = js.velocity.data[i];
    }
    execute_commands_and_update_robot_state();
    update_led_based_on_velocity(cmd_vel[0], cmd_vel[1]);
  }
  else
  {
    printf("JointState message deserialization error\n");
    update_led_based_on_velocity(0., 0.);
  }
}

static void publish_joint_state(void * pvParameters)
{
  const double efforts[] = {0.5, 0, 0.2};
  const char * const names[] = {
    "wbot_wheel_left_joint", "wbot_wheel_right_joint", "arbitrary_made_up_joint"};
  while (true)
  {
    taskENTER_CRITICAL(&my_spinlock);  // Begin critical section
    double positions[] = {desired_position[0], desired_position[1], 1};
    double velocities[] = {cmd_vel[0], cmd_vel[1], cmd_vel[2]};
    taskEXIT_CRITICAL(&my_spinlock);  // End of the critical section

    z_clock_t now = z_clock_now();
    ros_JointState joint_state = {
      .header =
        {
          .stamp =
            {
              .sec = (int32_t)now.tv_sec,
              .nanosec = (uint32_t)now.tv_nsec,
            },
        },
      .name = {.data = (char **)names, .n_elements = 3},
      .position = {.data = positions, .n_elements = 3},
      .velocity = {.data = velocities, .n_elements = 3},
      .effort = {.data = (double *)efforts, .n_elements = 3},
    };
    size_t len = ps_serialize(pub_buf, &joint_state, 1024);
    if (len > 0)
    {
      picoros_publish(&pub_js, pub_buf, len);
    }
    else
    {
      ESP_LOGI(node.name, "JointState message serialization error.");
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Publish at ~100Hz
  }
}

void initialize_desired_positions()
{
  for (int i = 0; i < num_joints; ++i)
  {
    desired_position[i] = 0.0;
    cmd_vel[i] = 0.0;
  }
  // Set the initial last_update_time_ms to the current time
  last_update_time_ms = get_milliseconds();
}

extern "C" void app_main(void)
{
  led_strip_init();
  set_led_color(0, 0, 0);        // Turn off LED at start
  blink_rgb(200, 165, 0, 1000);  // Blink orange to signal start of app_main

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  set_led_color(0, 0, 255);  // Solid blue to signal NVS init complete
  vTaskDelay(pdMS_TO_TICKS(1000));
  set_led_color(255, 0, 0);  // Switch to solid red to signal start of RMW init

  // Initialize Pico ROS interface
  picoros_interface_t ifx = {
    .mode = MODE,
    .locator = ROUTER_ADDRESS,
  };

  ESP_LOGI(
    node.name, "Starting pico-ros interface:[%s] on router address:[%s]\n", ifx.mode, ifx.locator);
  while (picoros_interface_init(&ifx) == PICOROS_NOT_READY)
  {
    printf("Waiting RMW init...\n");
    // Blink LED Red to signal we are waiting for RMW router
    blink_rgb(255, 0, 0, 1000);
    z_sleep_ms(1);
  }

  ESP_LOGI(node.name, "Starting Pico-ROS node:[%s] domain:[%ld]\n", node.name, node.domain_id);
  picoros_node_init(&node);

  initialize_desired_positions();

  ESP_LOGI(node.name, "Declaring publisher on [%s]\n", pub_js.topic.name);
  picoros_publisher_declare(&node, &pub_js);
  ESP_LOGI(node.name, "Declaring subscriber on [%s]\n", sub_js.topic.name);
  picoros_subscriber_declare(&node, &sub_js);

  // Publisher task
  xTaskCreate(publish_joint_state, "publish_joint_state_task", 4096, NULL, 1, NULL);
}
