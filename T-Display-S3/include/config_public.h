/* This file contains your configuration used to:
 *  1- connect to WIFI (enterprise or non-EAP)
 *  2- Set the device configuration (e.g., sensor type , device name, etc)
 *  3- Set the MQTT configuration
 */

// ============ device configuration =======================================
//const char *sensor_type="SGP30"; // Grove - VOC and eCO2 Gas Sensor(SGP30) by Seeeds
const char *sensor_type="SHT35"; // SHT35, SGP30, or Multi

#define device_location "Lab/ambient" // Where the device is located?
#define device_pref "SC-" // Give the device a name!

// for OTA drive
#define firmware_version "v@1.0.1.9"
#define ProductKey "5d0d1086-4c6b-4053-9932-5e7ff0b1acba"
#define update_timeTick 3600

// I2C pins in lilygo T-Display-S3 board
#define SDAPIN  43
#define SCLPIN  44

#define SCREEN_TIMEOUT 30000 // 30 sec - Screen timeout in milliseconds
#define WIFI_TIMEOUT 300000 // 5 min - Wifi timeout in milliseconds
#define EN_PIN 14 // GPIO pin connected to EN button

//----------------- OTA config variables
//int _update_timeTick = 900;
int _device_number = 99;
int _power_saving = 0;
int _MQTT_PUBLISH_DELAY_ms = 5000;
//const char _MQTT_SERVER = "petra.phys.yorku.ca";
//const char _EAP_ssid = "AirYorkPLUS";

// ============ NETWORK setting =======================================

const bool EAP_wifi= false; // change according to the WIFI type [true for enterprise networks, false for normal ]

//------ ENT Wifi setting (e.g. eduroam)-------------------
#define EAP_ANONYMOUS_IDENTITY "user@domain.ca"
#define EAP_IDENTITY "user@domain.ca"
#define EAP_PASSWORD "your_pass" // hardcode your wifi password here. It may not a good idea if you want to share your code
const char* EAP_ssid = "your_network_ssid"; //  network's SSID

//------ non-ENT Wifi setting-------------------
const char *ssid = "AirYorkGUEST";
//const char *password = none;


// ============ MQTT setting =======================================
const char *MQTT_SERVER = "Your_mqtt_server"; // the server that hosts the mqtt
const int MQTT_PORT = 1883;

//#define MQTT_PUBLISH_DELAY 5000
#define MQTT_CLIENT_ID "esp32_single_sht35"
const char *MQTT_USER = "mqttuser"; // NULL for no authentication
const char *MQTT_PASSWORD = "mqttpassword"; // NULL for no authentication
