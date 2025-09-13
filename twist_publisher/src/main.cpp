#include <stdlib.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "led_strip_esp32.h"
#include "math.h"
#include "nvs_flash.h"
#include "picoros.h"
#include "picoserdes.h"

// Pico-ROS config
#define TOPIC_NAME "picoros/cmd_vel"
#define NODE_NAME "cmd_vel_pub"

// Communication mode
#define MODE "client"
#define ROUTER_ADDRESS "serial/UART_0#baudrate=115200"

// ROS node
picoros_node_t node = {
  .name = "cmd_vel_pub",
};

// ROS Publisher
picoros_publisher_t pub_twist = {
  .topic =
    {
      .name = "picoros/cmd_vel",
      .type = ROSTYPE_NAME(ros_Twist),
      .rihs_hash = ROSTYPE_HASH(ros_Twist),
    },
};

// Buffer for publication, used from this thread
#define PUB_BUF_SIZE 1024
uint8_t pub_buf[PUB_BUF_SIZE];

static void publish_twist(void * pvParameters)
{
  uint32_t counter = 0;
  const float amplitude = 1.0;
  const float divisions = 20.0;
  while (true)
  {
    ros_Vector3 linear = {
      .x = amplitude * sin(float(counter) / divisions),
      .y = 0.0,
      .z = 0.0,
    };
    ros_Vector3 angular = {
      .x = 0.0,
      .y = 0.0,
      .z = amplitude * cos(float(counter) / divisions),
    };
    ros_Twist twist = {
      .linear = linear,
      .angular = angular,
    };
    printf("Publishing Twist message X[%.3f]m/sec ...\n", linear.x);
    size_t len = ps_serialize(pub_buf, &twist, PUB_BUF_SIZE);
    if (len > 0)
    {
      picoros_publish(&pub_twist, pub_buf, len);
      // sleep for 100 ms and blink the LED Green to let the user know the message went out
      blink_rgb(0, 100, 0, 100);
    }
    else
    {
      printf("Twist message serialization error.");
      // sleep for 100 ms and blink the LED Red to let the user know the message failed
      blink_rgb(100, 0, 0, 100);
    }
    counter++;
  }
}

extern "C" void app_main(void)
{
  // Blink R,G,B to show startup
  led_strip_init();
  blink_rgb(255, 0, 0, 500);
  blink_rgb(0, 255, 0, 500);
  blink_rgb(0, 0, 255, 500);

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  set_led_color(255, 255, 0);  // Yellow when starting the pico-ros interface

  // Init Pico-ROS
  picoros_interface_t ifx = {
    .mode = MODE,
    .locator = ROUTER_ADDRESS,
  };

  ESP_LOGI(node.name, "Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator);
  while (picoros_interface_init(&ifx) == PICOROS_NOT_READY)
  {
    ESP_LOGI(node.name, "Waiting RMW init...\n");
    blink_rgb(255, 255, 0, 1000);  // blink yellow while waiting to connect
  }

  set_led_color(0, 0, 255);  // Blue when connected to the ROS 2 network
  ESP_LOGI(node.name, "Starting Pico-ROS node %s domain:%lu\n", node.name, node.domain_id);
  picoros_node_init(&node);

  ESP_LOGI(node.name, "Declaring publisher on %s\n", pub_twist.topic.name);
  picoros_publisher_declare(&node, &pub_twist);

  // Publisher task
  xTaskCreate(publish_twist, "publish_twist_task", 4096, NULL, 1, NULL);
}
