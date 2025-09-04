#include <stdlib.h>
// #include "wifi_connect.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "picoros.h"
#include "picoserdes.h"


// Wifi Configuration
#define WIFI_SSID                   "yourWIFI SSID"
#define WIFI_PASS                   "password"
#define WIFI_MAXIMUM_RETRY          5

// Joystick config
#define ADC1_CH_X                   ADC1_CHANNEL_5
#define ADC1_CH_Y                   ADC1_CHANNEL_4
#define DEAD_BAND_PERCENT           10
#define UPDATE_PERIOD_MS            50

// Pico-ROS config
#define TOPIC_NAME                  "joy"
#define MODE                        "client"
#define ROUTER_ADDRESS              "serial/UART_0#baudrate=115200"


// ROS node
picoros_node_t node = {
    .name = "picoros_joystick",
};

// Publisher
picoros_publisher_t pub_joy = {
    .topic = {
        .name = TOPIC_NAME,
        .type = ROSTYPE_NAME(ros_Twist),
        .rihs_hash = ROSTYPE_HASH(ros_Twist),
    },
};

// Buffer for publication, used from this thread
#define PUB_BUF_SIZE 1024
uint8_t pub_buf[PUB_BUF_SIZE];


enum{
    X_AXIS,
    Y_AXIS,
    NUM_AXIS,
};

static void publish_joy_task(void *pvParameters)
{
    while(true){
        int middle_range = 0x07FF;
        int x_raw = adc1_get_raw( ADC1_CH_X);
        int y_raw = adc1_get_raw( ADC1_CH_Y);
        int x_pcnt = ((x_raw - middle_range) * 100) / middle_range; 
        int y_pcnt = ((y_raw - middle_range) * 100) / middle_range; 
        if (abs(x_pcnt) < DEAD_BAND_PERCENT){ x_pcnt = 0; }
        if (abs(y_pcnt) < DEAD_BAND_PERCENT){ y_pcnt = 0; }
        z_clock_t clk = z_clock_now();
        // ros_Twist joy = {
        //     .header.stamp.nanosec = clk.tv_nsec, 
        //     .header.stamp.sec = clk.tv_sec,
        //     .header.frame_id = "esp32-joystick",
        //     .axes = {
        //         .data = (float[NUM_AXIS]){ 
        //             [X_AXIS] = x_pcnt / 100.0f,  
        //             [Y_AXIS] = y_pcnt / 100.0f
        //         },
        //         .n_elements = NUM_AXIS
        //     },
        // };
        // size_t len = ps_serialize(pub_buf, &joy, PUB_BUF_SIZE);
        // if (len > 0){
        //     ESP_LOGI(node.name, "Publishing joystick data x:%.2f y:%.2f",
        //          joy.axes.data[X_AXIS], joy.axes.data[Y_AXIS] );
        //     picoros_publish(&pub_joy, pub_buf, len);
        // }
        // else{
        //     ESP_LOGE(node.name, "ros_Joy message serialization error.");
        // }
        z_sleep_ms(UPDATE_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten( ADC1_CH_X, ADC_ATTEN_DB_12 );
    adc1_config_channel_atten( ADC1_CH_Y, ADC_ATTEN_DB_12 );
    
    // Init WIFI
    
    // wifi_init_sta(WIFI_SSID, WIFI_PASS, WIFI_MAXIMUM_RETRY);
    
    // Init Pico-ROS
    picoros_interface_t ifx = {
        .mode = MODE,
        .locator = ROUTER_ADDRESS,
    };

    ESP_LOGI(node.name, "Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator );
    while (picoros_interface_init(&ifx) == PICOROS_NOT_READY){
        ESP_LOGI(node.name, "Waiting RMW init...\n");
        z_sleep_s(1);
    }
    
    ESP_LOGI(node.name, "Starting Pico-ROS node %s domain:%lu\n", node.name, node.domain_id);
    picoros_node_init(&node);
    
    ESP_LOGI(node.name, "Declaring publisher on %s\n", pub_joy.topic.name);
    picoros_publisher_declare(&node, &pub_joy);
    
    // Publisher task
    xTaskCreate(publish_joy_task, "publish_twist_task", 4096, NULL, 1, NULL);
}