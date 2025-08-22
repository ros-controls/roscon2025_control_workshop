#include <Arduino.h>
#include <WiFi.h>

#include <stdio.h>
#include <stdint.h>
#include "picoros.h"
#include "picoserdes.h"

// WiFi-specific parameters
#define SSID "ros2_control_workshop_1"
#define PASS "roscon2025"

// Zenoh-specific parameters
#define MODE "client"
// change this to match your ROS 2 host router's ip address
#define ROUTER_ADDRESS "tcp/10.42.0.1:7447" 

/* ---------- LED Functions ----------- */
void blinkRGB(int r, int g, int b, int sleep_ms)
{
    neopixelWrite(RGB_BUILTIN, r, g, b);
    delay(sleep_ms/2);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(sleep_ms/2);
}

// Subscriber callback
void joint_state_callback(uint8_t* rx_data, size_t data_len);

/* -------------------------------------- */
// Example Publisher
picoros_publisher_t pub_js = {
    .topic = {
        .name = "picoros/joint_states",
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

picoros_subscriber_t sub_js = {
    .topic = {
        .name = "picoros/joint_commands",
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
    .user_callback = joint_state_callback,
};

// Example node
picoros_node_t node = {
    .name = "wbot_picoros_node",
};

static const int num_joints = 3;
// the latest velocity command from the subscriber
double cmd_vel[num_joints];
// the calculated desired position for the joints given the velocity command
double desired_position[num_joints];
ros_Time current_time_{};
ros_Time last_update_time_{};
// Buffer for publication, used from this thread
uint8_t pub_buf[1024];

double current_velocity[num_joints] = {0}; // Current joint velocities
double joint_inertia[num_joints] = {0.05, 0.05, 0.05};
double friction_coeff[num_joints] = {0.1, 0.1, 0.1};
const double VELOCITY_CONTROL_P_GAIN = 5.0;

void update_led_based_on_velocity(double velocity_left, double velocity_right) {
    // Calculate motion characteristics
    double avg_vel = (velocity_left + velocity_right) / 2.0;      // Average velocity (forward/backward)
    double diff_vel = velocity_left - velocity_right;             // Velocity difference (turning)
    
    // Thresholds
    const double min_vel = 0.05;               // Minimum velocity to consider "moving"
    const double max_vel = 2.0;                // Maximum expected velocity for scaling
    const double turn_threshold = 0.1;         // Minimum difference to consider "turning"
    
    // Check if robot is essentially stopped
    if (abs(avg_vel) < min_vel && abs(diff_vel) < turn_threshold) {
        // Robot stopped - turn off LED
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        return;
    }
    
    // Initialize RGB values
    int red = 0, green = 0, blue = 0;
    
    // Check if mainly driving straight (forward/backward)
    if (abs(diff_vel) < turn_threshold) {
        // Driving straight - use RED proportional to speed
        int intensity = (int)(abs(avg_vel) / max_vel * 255.0);
        intensity = constrain(intensity, 0, 255);
        red = intensity;
    }
    // Check if mainly turning in place
    else if (abs(avg_vel) < min_vel) {
        // Pure turning
        int intensity = (int)(abs(diff_vel) / max_vel * 255.0);
        intensity = constrain(intensity, 0, 255);
        
        if (diff_vel > 0) {
            // Turning left - BLUE
            blue = intensity;
        } else {
            // Turning right - GREEN
            green = intensity;
        }
    }
    // Arc movement - mix colors
    else {
        // Calculate base intensities
        int forward_intensity = (int)(abs(avg_vel) / max_vel * 255.0);
        int turn_intensity = (int)(abs(diff_vel) / max_vel * 255.0);
        
        forward_intensity = constrain(forward_intensity, 0, 255);
        turn_intensity = constrain(turn_intensity, 0, 255);
        
        // Mix red (forward/backward) with turn color
        red = forward_intensity;
        
        if (diff_vel > 0) {
            // Arc left - mix RED + BLUE
            blue = turn_intensity;
        } else {
            // Arc right - mix RED + GREEN  
            green = turn_intensity;
        }
        
        // Scale down to prevent oversaturation
        double scale = 1.0;
        int max_component = max(red, max(green, blue));
        if (max_component > 255) {
            scale = 255.0 / max_component;
        }
        
        red = (int)(red * scale);
        green = (int)(green * scale);
        blue = (int)(blue * scale);
    }
    
    // Update the LED
    neopixelWrite(RGB_BUILTIN, red, green, blue);
}

void execute_commands_and_update_robot_state() {
    z_clock_t now = z_clock_now();
    current_time_ = {
        .sec = now.tv_sec,
        .nanosec = (uint32_t)now.tv_nsec,
    };
    
    // Calculate time difference in seconds
    double dt = (current_time_.sec - last_update_time_.sec) +
                (current_time_.nanosec - last_update_time_.nanosec) / 1e9;
    
    // Safety check for time step
    if (dt <= 0 || dt > 0.1) {
        last_update_time_ = current_time_;
        return;
    }
    
    // Update desired position for each joint with physics
    noInterrupts();
    for (int i = 0; i < num_joints; ++i) {
        // Safety check for valid values - fix any NaNs immediately
        if (isnan(current_velocity[i])) {
            current_velocity[i] = 0.0;
        }
        if (isnan(desired_position[i])) {
            desired_position[i] = 0.0;
        }
        if (isnan(cmd_vel[i])) {
            cmd_vel[i] = 0.0;
        }
        
        // Dead zone: if commanded velocity is near zero AND current velocity is small, stop completely
        if (abs(cmd_vel[i]) < 0.01 && abs(current_velocity[i]) < 0.05) {
            current_velocity[i] = 0.0;
            // Don't update position - keep it exactly where it is
            continue;
        }
        
        // Calculate velocity error
        double velocity_error = cmd_vel[i] - current_velocity[i];
        
        // Simple proportional control to generate force
        double control_force = VELOCITY_CONTROL_P_GAIN * velocity_error; // Adjust gain as needed
        
        // Apply friction (opposes current motion)
        double friction_force = -friction_coeff[i] * current_velocity[i];
        
        // Total force
        double total_force = control_force + friction_force;
        
        // Calculate acceleration from force and inertia
        double acceleration = total_force / joint_inertia[i];
        
        // Update velocity with acceleration
        current_velocity[i] += acceleration * dt;
        
        // Update position with current velocity
        desired_position[i] += current_velocity[i] * dt;
        
        // Safety check before normalization
        if (isnan(desired_position[i])) {
            desired_position[i] = 0.0;
        }
        
        // Normalize the position to a range of [-pi, pi]
        desired_position[i] = fmod(desired_position[i] + M_PI, 2.0 * M_PI);
        if (desired_position[i] < 0) {
            desired_position[i] += 2.0 * M_PI;
        }
        desired_position[i] -= M_PI;
        
        // Final safety check after normalization
        if (isnan(desired_position[i])) {
            desired_position[i] = 0.0;
        }
    }
    interrupts();
    
    last_update_time_ = current_time_;
}
void joint_state_callback(uint8_t* rx_data, size_t data_len){
    // Define arrays to hold the data
    char* joint_names[num_joints] = {};
    double joint_positions[num_joints] = {};
    double joint_velocities[num_joints] = {};
    double joint_efforts[num_joints] = {};

    // Create the JointState message and initialize it with the desired array sizes
    // ros2_control HW interface is only sending each joint a velocity command
    ros_JointState js = {
        .name = {.data = joint_names, .n_elements = num_joints},
        .position = {.data = joint_positions, .n_elements = 0},
        .velocity = {.data = joint_velocities, .n_elements = num_joints},
        .effort = {.data = joint_efforts, .n_elements = 0},
    };
    if (ps_deserialize(rx_data, &js, data_len)){
        for(auto i = 0; i < num_joints; i++){
            cmd_vel[i] = js.velocity.data[i];
        }
        execute_commands_and_update_robot_state();
    }
    else{
        printf("JointState message deserialization error\n");
    }
}

void publish_joint_state(){
    static const double efforts[] = {0.5, 0, 0.2};
    static const char* const names[] = {"wbot_wheel_left_joint", "wbot_wheel_right_joint", "arbitrary_made_up_joint"};

    noInterrupts(); // Disable interrupts 
    double positions[] = {desired_position[0], desired_position[1], 1};    
    double velocities[] = {cmd_vel[0], cmd_vel[1], cmd_vel[2]};
    interrupts(); // Re-enable interrupts

    z_clock_t now = z_clock_now();
    ros_JointState joint_state = {
        .header = {
            .stamp = {
                .sec = now.tv_sec,
                .nanosec = (uint32_t)now.tv_nsec,
            },
        },
        .name = {.data = (char**)names, .n_elements = num_joints},
        .position = {.data = positions, .n_elements = num_joints},
        .velocity = {.data = velocities, .n_elements = num_joints},
        .effort = {.data = (double*)efforts, .n_elements = num_joints},
    };
    // Serial.printf("Publishing JointState message number %d ...\n", counter);
    size_t len = ps_serialize(pub_buf, &joint_state, 1024);
    if (len > 0){
        picoros_publish(&pub_js, pub_buf, len);
    }
    else{
        Serial.printf("JointState message serialization error.");
    }
}

void initialize_desired_positions() {
    for (int i = 0; i < num_joints; ++i) {
        desired_position[i] = 0.0; // Start at zero
        cmd_vel[i] = 0.0;
    }
    z_clock_t now = z_clock_now();
    current_time_ = {
        .sec = now.tv_sec,
        .nanosec = (uint32_t)now.tv_nsec,
    };
    last_update_time_ = current_time_;
}

void setup(void)
{
    pinMode(RGB_BUILTIN, OUTPUT);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    // Initialize Serial for debug
    Serial.begin(115200);
    do
    {// blink the LED Orange to signal start of the program
        blinkRGB(200, 165, 0, 1000);
    } while (!Serial);

    Serial.printf("Connecting to WiFi %s!\n", SSID);
    // Set WiFi in STA mode and trigger attachment
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED)
    {// blink the LED Blue to signal we are connecting to WiFi
      blinkRGB(0, 0, 255, 500);
    }
    Serial.printf("Connected to WiFi [%s] with address [%s]\n", SSID, WiFi.localIP().toString());
    // Hold Blue solid indicating success
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    delay(2000);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);

    // Initialize Pico ROS interface
    picoros_interface_t ifx = {
        .mode = MODE,
        .locator = ROUTER_ADDRESS,
    };

    Serial.printf("Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator );
    while (picoros_interface_init(&ifx) == PICOROS_NOT_READY){
        printf("Waiting RMW init...\n");
        // Blink LED Red to signal we are waiting for RMW router
        blinkRGB(255, 0, 0, 1000);
        z_sleep_ms(1);
    }

    Serial.printf("Starting Pico-ROS node %s domain:%d\n", node.name, node.domain_id);
    picoros_node_init(&node);

    initialize_desired_positions();

    Serial.printf("Declaring publisher on %s\n", pub_js.topic.name);
    picoros_publisher_declare(&node, &pub_js);
    Serial.printf("Declaring subscriber on %s\n", sub_js.topic.name);
    picoros_subscriber_declare(&node, &sub_js);
}

// loop rate is controlled by topic pub-sub & LED blink if desired
void loop()
{
    publish_joint_state();
    // uncomment to slow down the control rate.
    // blinkRGB(0, 100, 0, 10); // sleep for 10 ms and blink the LED Green to let the user know the message went out
    delay(100);
}
