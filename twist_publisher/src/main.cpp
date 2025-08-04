
#include <Arduino.h>
#include <WiFi.h>
// #define RGB_BUILTIN 48 // RGB pin for esp32-s3-devkitm-1 is GPIO48

extern "C"
{
#include "zenoh-pico.h"
}

// WiFi-specific parameters
#define SSID "your_ssid"
#define PASS "your_pw"

// Zenoh-specific parameters
#define MODE "client"
#define CONNECT "" // example: tcp/192.168.9.241:7447

#define TWIST_TOPIC "rt/turtle1/cmd_vel"

#define MAX_STOP_TWISTS 200

/* --------------- Structs -------------- */
struct Vector3
{
    double x;
    double y;
    double z;
};

struct Twist
{
    Vector3 linear;
    Vector3 angular;
};

/* -------- Serialize Functions --------- */
char *serialize_float_as_f64_little_endian(double val, char *buf)
{
    long long *c_val = (long long *)&val;
    for (int i = 0; i < sizeof(double); ++i, ++buf)
    {
        *buf = 0xFF & (*c_val >> (i * 8));
    }

    return buf;
}

char *serialize_vector3(Vector3 *v, char *buf)
{
    buf = serialize_float_as_f64_little_endian(v->x, buf);
    buf = serialize_float_as_f64_little_endian(v->y, buf);
    buf = serialize_float_as_f64_little_endian(v->z, buf);

    return buf;
}

void serialize_twist(Twist *t, char *buf)
{
    // Serialize Twist header for little endian
    *(buf++) = 0x00;
    *(buf++) = 0x01;
    *(buf++) = 0x00;
    *(buf++) = 0x00;
    buf = serialize_vector3(&t->linear, buf);
    buf = serialize_vector3(&t->angular, buf);
}

/* ---------- Print Functions ----------- */
void printVector(struct Vector3 *v)
{
    Serial.print("X: ");
    Serial.print(v->x);
    Serial.print(", Y: ");
    Serial.print(v->y);
    Serial.print(", Z: ");
    Serial.print(v->z);
}

void printTwist(struct Twist *t)
{
    Serial.print("Linear ");
    printVector(&t->linear);
    Serial.println("");

    Serial.print("Angular ");
    printVector(&t->angular);
    Serial.println("");
}

/* ---------- LED Functions ----------- */
void blinkRGB(int r, int g, int b, int sleep_ms)
{
    neopixelWrite(RGB_BUILTIN, r, g, b);
    delay(sleep_ms/2);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(sleep_ms/2);
}

/* -------------------------------------- */
z_owned_publisher_t pub;

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

    // Initialize Zenoh Session and other parameters
    z_owned_config_t config = z_config_default();
    zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(MODE));
    zp_config_insert(z_config_loan(&config), Z_CONFIG_CONNECT_KEY, z_string_make(CONNECT));

    // Open Zenoh session
    Serial.printf("Opening Zenoh Session... %s\n", CONNECT);
    z_owned_session_t s = z_open(z_config_move(&config));
    if (!z_session_check(&s))
    {
        Serial.println("Unable to open session!\n");
        while (1)
        {
            blinkRGB(255, 0, 0, 300);
        }
    }
    Serial.println("OK");

    // Start the receive and the session lease loop for zenoh-pico
    zp_start_read_task(z_session_loan(&s), NULL);
    zp_start_lease_task(z_session_loan(&s), NULL);

    // Declare Zenoh publisher
    Serial.print("Declaring publisher for ");
    Serial.print(TWIST_TOPIC);
    Serial.println("...");
    pub = z_declare_publisher(z_session_loan(&s), z_keyexpr(TWIST_TOPIC), NULL);
    if (!z_publisher_check(&pub))
    {
        Serial.println("Unable to declare publisher!\n");
        while (1)
        {
            blinkRGB(255, 0, 0, 300);
        }
    }
    Serial.println("OK");
    Serial.println("Zenoh setup finished!");

    delay(300);
}

// number of consecutive Stop Twists sent (to stop sending those after a while)
short nb_stop_twist = 0;

// loop rate is controlled with LED blink sleeping
void loop()
{

    // Create ROS twist message
    Twist measure;
    measure.linear.x = 0.0;
    measure.linear.y = 0.0;
    measure.linear.z = 1.0;
    measure.angular.x = 2.0;
    measure.angular.y = 0.0;
    measure.angular.z = 0.0;

    if (measure.linear.x == 0 && measure.angular.z == 0)
    {
        nb_stop_twist++;
    }
    else
    {
        nb_stop_twist = 0;
    }

    if (nb_stop_twist >= MAX_STOP_TWISTS)
    {
        // this is a Stop Twist again, don't publish it
        // Serial.print("No move after more than ");
        // Serial.print(MAX_STOP_TWISTS);
        // Serial.println(" loops - Don't publish");
        nb_stop_twist = MAX_STOP_TWISTS;
        delay(100);
    }
    else
    {
        // printTwist(&measure);
        // Serial.println("");

        uint8_t twist_serialized_size = 4 + sizeof(double) * 6;
        char buf[twist_serialized_size];
        serialize_twist(&measure, buf);
        if (z_publisher_put(z_publisher_loan(&pub), (const uint8_t *)buf, twist_serialized_size, NULL) < 0)
        {
            Serial.println("Error while publishing data");
        }
        blinkRGB(0, 255, 0, 100);
    }
}
