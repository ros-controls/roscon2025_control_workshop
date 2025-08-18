#include <Arduino.h>
#include <WiFi.h>

#include <stdio.h>
#include <stdint.h>
#include "picoros.h"
#include "picoserdes.h"

// WiFi-specific parameters
#define SSID "your_ssid"
#define PASS "your_pw"

// Zenoh-specific parameters
#define MODE "client"
#define ROUTER_ADDRESS "tcp/192.168.9.241:7447" // change this to match your ROS 2 host router's ip address

/* ---------- LED Functions ----------- */
void blinkRGB(int r, int g, int b, int sleep_ms)
{
    neopixelWrite(RGB_BUILTIN, r, g, b);
    delay(sleep_ms/2);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(sleep_ms/2);
}

/* -------------------------------------- */
// Example Publisher
picoros_publisher_t pub_log = {
    .topic = {
        .name = "picoros/joint_state",
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

// Example node
picoros_node_t node = {
    .name = "picoros_jointstate_pub",
};

// Buffer for publication, used from this thread
uint8_t pub_buf[1024];

void publish_joint_state(){
    static uint32_t counter = 0;
    float amplitude = 1.0;
    float divisions = 20.0;

    z_clock_t clk = z_clock_now();
    ros_Time t = {
        .sec = clk.tv_sec,
        .nsec = clk.tv_nsec,
    };
    // ros_JointState joint_state;
    ros_JointState joint_state = {
        .header = {
            .stamp = t,
            .frame_id = "base_link",
        },
        .name = {
            "left_wheel_joint",
            "right_wheel_joint"
        },
        .position = {
            amplitude*sin(float(counter)/divisions),
            amplitude*cos(float(counter)/divisions)
        },
        .velocity = {
            0.0, 0.0
        },
        .effort = {
            0.0, 0.0
        }
    };
    
    Serial.printf("Publishing JointState message [%zu] bytes ...\n", sizeof(joint_state));
    size_t len = ps_serialize(pub_buf, &joint_state, 1024);
    if (len > 0){
        picoros_publish(&pub_log, pub_buf, len);
    }
    else{
        Serial.printf("JointState message serialization error.");
    }
    counter++;
}

void setup(void)
{
    pinMode(RGB_BUILTIN, OUTPUT);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    // Initialize Serial for debug
    Serial.begin(115200);
    while (!Serial)
    {
        delay(1000);
    }
    int num_boot_loops = 1;
    for (auto i = 1; i<=num_boot_loops; i++)
    {
      Serial.printf("Booting... [%d/%d]\n", i, num_boot_loops);
      blinkRGB(255, 255, 0, 500);
      Serial.flush();
    }

    Serial.printf("Connecting to WiFi %s!\n", SSID);
    // Set WiFi in STA mode and trigger attachment
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
      blinkRGB(0, 0, 255, 500);
    }
    Serial.printf("Connected to WiFi [%s] with address [%s]\n", SSID, WiFi.localIP().toString());
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    delay(2000); // go solid indicating success
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);

    // Initialize Pico ROS interface
    picoros_interface_t ifx = {
        .mode = MODE,
        .locator = ROUTER_ADDRESS,
    };

    Serial.printf("Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator );
    while (picoros_interface_init(&ifx) == PICOROS_NOT_READY){
        printf("Waiting RMW init...\n");
        blinkRGB(255, 0, 0, 500);
        z_sleep_s(1);
    }

    Serial.printf("Starting Pico-ROS node %s domain:%d\n", node.name, node.domain_id);
    picoros_node_init(&node);

    Serial.printf("Declaring publisher on %s\n", pub_log.topic.name);
    picoros_publisher_declare(&node, &pub_log);
}

// loop rate is controlled with LED blink sleeping
void loop()
{
    publish_joint_state();
    blinkRGB(0, 100, 0, 100); // sleep for 100 ms and blink the LED Green to let the user know the message went out
}
