/* This file contains your configuration used to:
 *  1- connect to WIFI (enterprise or non-EAP)
 *  2- Set the device configuration (sensor type and multiplicity)
 *  3- Set the MQTT configuration
 */

#define TCAADDR 0x70 // I2C multiplexer address

// ============ device configuration =======================================
const bool multiplexer=false; // true if there are multiple sensors connected via I2C, false otherwise

//const char *sensor_type="SGP30"; // Grove - VOC and eCO2 Gas Sensor(SGP30) by Seeeds
const char *sensor_type="Multi"; // SHT35, SGP30, or Multi

#define device_location "Lab/ambient" // Where the device is located?
#define device_name "esp32_air" // Give the device a name!


// ============ NETWORK setting =======================================
const bool EAP_wifi=false; // change according to the WIFI type [true for enterprise networks, false for normal ]


//------ ENT Wifi setting (e.g. eduroam)-------------------
#define EAP_ANONYMOUS_IDENTITY "user@domain.ca"
#define EAP_IDENTITY "user@domain.ca"
#define EAP_PASSWORD "your_pass" // hardcode your wifi password here. It may not a good idea if you want to share your code
const char* EAP_ssid = "your_network_ssid"; //  network's SSID

//------ non-ENT Wifi setting-------------------
const char *ssid = "your_network_ssid";
const char *password = "your_password";


// ============ MQTT setting =======================================
const char *MQTT_SERVER = "Your_mqtt_server"; // the server that hosts the mqtt
const int MQTT_PORT = 1883;

#define MQTT_PUBLISH_DELAY 2000 // in mili-seconds
#define MQTT_CLIENT_ID "esp32_single" // you can choose any name
const char *MQTT_USER = "mqttuser"; // NULL for no authentication
const char *MQTT_PASSWORD = "mqttpassword"; // NULL for no authentication
