/* This file contains your configuration used to:
 *  1- connect to WIFI (enterprise or non-EAP)
 *  2- Set the device configuration (e.g., sensor type , device name, etc)
 *  3- Set the MQTT configuration
 */

// ============ device configuration =======================================
//const char *sensor_type="SGP30"; // Grove - VOC and eCO2 Gas Sensor(SGP30) by Seeeds
const bool sensor_sht = true; // SHT_xx humidity & temperature
const bool sensor_water = false; // Capacitive Moisture Sensor
const bool sensor_gas = false; // SGP30 gas sensor

#define device_location "FSC_YORKU/ambient" // Where the device is located?
#define device_pref "FSC-" // Give the device a name!
int _device_number = 99;


// I2C pins in lilygo T-Display-S3 board
#define SDAPIN  43
#define SCLPIN  44

//#define SCREEN_TIMEOUT 30000 // 30 sec - Screen timeout in milliseconds
#define WIFI_TIMEOUT 300000 // 5 min - Wifi timeout in milliseconds
#define MQTT_TIMEOUT 3000 // 5 sec - mqtt timeout in milliseconds
#define EN_PIN 14 // GPIO pin connected to EN button

// ============ NETWORK setting =======================================
const bool do_wifi = true; // true for wifi, false for no wifi functionality

const bool EAP_wifi= true; // change according to the WIFI type [true for enterprise networks, false for normal ]

//------ ENT Wifi setting (e.g. eduroam)-------------------
#define EAP_ANONYMOUS_IDENTITY "Your Anonymous Identity"
#define EAP_IDENTITY "Your Identity"
#define EAP_PASSWORD "Your password" // hardcode your wifi password here
const char* EAP_ssid = "AirYorkPLUS"; //  network's SSID

//------ non-ENT Wifi setting-------------------
const char *ssid = "AirYorkGUEST";
const char *password = NULL;

// ============ OTA  =======================================
const bool do_ota = true; // true for OTA, false for no OTA functionality
#define firmware_version "v@1.0.3.0"
#define ProductKey "Your Product Key"
#define update_timeTick 3600 // checks OTA server for updates every 1 hour

//int _power_saving = 0;

// ============ MQTT setting =======================================
//const char *MQTT_SERVER = "petra.phys.yorku.ca";
const char *MQTT_SERVER = "Your MQTT server";
const int MQTT_PORT = 1883;

int _MQTT_PUBLISH_DELAY_ms = 5000;
const char *MQTT_USER = "public"; // NULL for no authentication
const char *MQTT_PASSWORD = "public"; // NULL for no authentication