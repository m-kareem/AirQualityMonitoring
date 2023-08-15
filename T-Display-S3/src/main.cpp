#include <Arduino.h>
#include <EEPROM.h>

#include "SHTSensor.h" // sensirion/arduino-sht bySensirion AG
#include "Adafruit_seesaw.h" // Adafruit seesaw Library by Adafruit Industries
#include "sgp30.h" // for CO2 sensor

#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <otadrive_esp.h>

#include <SPI.h>
#include <PubSubClient.h>
#include <MQTT.h>

#include "TFT_eSPI.h"

#include "YU_logo.h" // image convertor: https://javl.github.io/image2cpp/
#include "config.h" // Update this file with your configuration

#include <NTPClient.h> // for the time server

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Global constants
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define LOOP_TIME_INTERVAL_MS  2000
#define BASELINE_IS_STORED_FLAG  (0X55)

// Display.
#define     DISPLAY_HEIGHT          170                         // T-Display-S3 display height in pixels.
#define     DISPLAY_WIDTH           320                         // T-Display-S3 display width in pixels.
#define     DISPLAY_BRIGHTNESS_MAX  252                         // T-Display-S3 display brightness maximum.
#define     DISPLAY_BRIGHTNESS_OFF  0                           // T-Display-S3 display brightness minimum.  
#define     TFT_BL                  38                          // T-Display-S3 backlight pin  

#define gray 0x6B6D
#define blue 0x0967
#define orange 0xC260
#define purple 0x604D
#define green 0x1AE9

// Sprites.
#define     SPRITE_HEADER_FONT     2                           // HEADER sprite font size.
#define     SPRITE_HEADER_HEIGHT   30                          // HEADER sprite height in pixels.
#define     SPRITE_HEADER_WIDTH    DISPLAY_WIDTH               // HEADER sprite width in pixels.

#define     SPRITE_BOADY_FONT     2                           // BOADY sprite font size.
#define     SPRITE_BOADY_HEIGHT   140                         // BOADY sprite height in pixels.
#define     SPRITE_BOADY_WIDTH    DISPLAY_WIDTH               // BOADY sprite width in pixels.

#define darkred 0xA041

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Global variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// tft display
unsigned long last_activity_time = 0; // Last time the screen was active
int currentLine_y = 0;
bool screen_on = true;

// Battery
uint32_t    uVolt;                                              // Battery voltage.

// Display (T-Display-S3 lcd display).
TFT_eSPI    tft = TFT_eSPI();                                   // T-Display-S3 lcd.
//int         lcdBacklighBrightness = DISPLAY_BRIGHTNESS_MAX;     // T-Display-S3 brightness.

// Sprites.
TFT_eSprite spriteHeader = TFT_eSprite(& tft);                 // header sprite.
TFT_eSprite spriteNet = TFT_eSprite(& tft);                 // NET sprite.
TFT_eSprite spriteBoady = TFT_eSprite(& tft);                 // Boady sprite.

String      stringIP;                                           // IP address.
String      macaddress;                                           // IP address.

int counter = 0;
float CO2_list;
float VOC_list;
float CO2VOC_list[2];

float rH_list;
float T_list;
float TrH_list[2];


uint16_t water_detection ;
float water_leakage = 0;
float water_leakage_list[2];

float TrHW_list[4];
float AirQuality_list[4];

const float invalidData = -999;

//--------------- time -------------------------
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
String tt="";
String curSeconds="";

//------------- MQTT topics --------------------
char MQTT_TOPIC_STATE[100]= "";
char MQTT_TOPIC_CO2VOC[100]= "";
char MQTT_TOPIC_TRH[100]= "";
char MQTT_TOPIC_WATER[100]= "";

char MQTT_TOPIC_AIRQUALITY[100]= "";
char MQTT_TOPIC_TRHW[100]= "";

char device_name[100]= "";

//------------- objects --------------------
String sensor_type = "";
SHTSensor sht; // temperature & humidity sensor
Adafruit_seesaw ss; // water sensor

WiFiClient espClient;
PubSubClient mqttClient(espClient);


void ota_proggress(size_t downloaded, size_t total);

//================== Functions ==============================
void setupWifi() {
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  
  WiFi.disconnect(true);
  delay(100);  // <- fixes some issues with WiFi stability
  WiFi.begin(ssid, password);
}
//---------------------------------------------------------------
void setupEAP_Wifi() {  
  Serial.print("Connecting to EAP network: ");
  Serial.println(EAP_ssid);

  WiFi.disconnect(true);
  delay(100);  // <- fixes some issues with WiFi stability
  WiFi.mode(WIFI_STA); //init wifi mode
  WiFi.begin(EAP_ssid, WPA2_AUTH_PEAP, EAP_ANONYMOUS_IDENTITY, EAP_IDENTITY, EAP_PASSWORD);
}
//---------------------------------------------------------------
void setupNetwork() {
  currentLine_y = 0;
  spriteNet.drawString("MAC Address: " + WiFi.macAddress(), 5, currentLine_y, 2);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  currentLine_y += 16;
  spriteNet.drawString("Awaiting wifi connection ...", 5, currentLine_y, 2);
  
  currentLine_y += 16;
  if (EAP_wifi){
    spriteNet.drawString("Connecting to EAP network: " +String(EAP_ssid) , 5, currentLine_y, 2);
    setupEAP_Wifi();
  }
  else{
    spriteNet.drawString("Connecting to network: " +String(ssid) , 5, currentLine_y, 2);
    setupWifi();
  }

  last_activity_time = millis(); // Update last activity time
  // Wait for a connection.
    while (WiFi.status() != WL_CONNECTED)
    {
      spriteNet.fillCircle(300, 8, 5,TFT_RED);
      spriteNet.pushSprite(0,0);
      delay(500);
      Serial.print(".");
      spriteNet.fillCircle(300, 8, 5,TFT_BLACK);
      spriteNet.pushSprite(0,0);
      
      if (millis() - last_activity_time > WIFI_TIMEOUT ) // Check wifi timeout
        ESP.restart();
      
      delay(500);
    }

    // Connected.
    spriteNet.fillCircle(300, 8, 5,TFT_GREEN);
    
    stringIP = WiFi.localIP().toString();
    
    currentLine_y += 16;
    if(do_ota)
      spriteNet.drawString("Wifi connected: " + stringIP +",  OTA enabled.", 5, currentLine_y, 2);
    else
      spriteNet.drawString("Wifi connected: " + stringIP +",  OTA disabled.", 5, currentLine_y, 2);

    spriteNet.pushSprite(0,0);
}
//------------------------------
void setupTime() {
  // Init and get the time
  timeClient.begin();
  timeClient.setTimeOffset(-3600*4); // Eastern time zone
  timeClient.update();
}

//------------------------------
void mqttReconnect() {
  int counter2 =0;
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    } else {
      counter2++;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
      if (counter2>=20)//after 20 iteration timeout - reset board
        ESP.restart();
    }
  }
}

//---------------------------------------------------------------
void mqtt_jason_publish(String topic, String dataType, float mylist[], int list_size )
{
  // this function works only with ArduinoJson 5

  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["device"] = device_name;
  JSONencoder["sensorType"] = sensor_type;

  JsonArray& values = JSONencoder.createNestedArray(dataType);
  
  for (int i = 0; i < list_size; i++){
  values.add(mylist[i]);
  }
  
  char JSONmessageBuffer[400];

  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  if (mqttClient.publish(topic.c_str(), JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

}

//----------------------------------------------
void ReadSystem(){
  s16 err = 0;
  u16 _co2_eq_ppm, _tvoc_ppb;
  float co2_eq_ppm, tvoc_ppb, temp, hum;

    if(sensor_gas)
    {
        err = sgp_measure_iaq_blocking_read(&_tvoc_ppb, &_co2_eq_ppm);
        co2_eq_ppm = (float) _co2_eq_ppm;
        tvoc_ppb = (float) _tvoc_ppb;

        if (err == STATUS_OK) {
            CO2_list=co2_eq_ppm;
            VOC_list=tvoc_ppb;
            CO2VOC_list[0]=CO2_list;
            CO2VOC_list[1]=VOC_list;
        }
        else{
          CO2_list= invalidData; VOC_list= invalidData; CO2VOC_list[0]=invalidData; CO2VOC_list[1]=invalidData;
          Serial.println("error reading IAQ values\n");
        }
    }
    if(sensor_sht){
      if (sht.readSample()) {
        T_list=round(sht.getTemperature()*10)/10;
        rH_list=round(sht.getHumidity()*10)/10;
        TrH_list[0]=T_list;
        TrH_list[1]=rH_list;
      }
      else{T_list= invalidData; rH_list= invalidData; TrH_list[0]=invalidData; TrH_list[1]=invalidData;
        Serial.print("sht_xx: Error in readSample()\n");
      }  
    }
    if(sensor_water){
      water_detection = ss.touchRead(0);
      Serial.print("water_detection: "); Serial.println(water_detection);
      if (water_detection > 550 && water_detection < 2000) water_leakage = 1;
      else water_leakage = 0;
      water_leakage_list[0]= water_leakage;
      water_leakage_list[1]= water_detection;

    }
    
    if(sensor_gas && sensor_sht){
      AirQuality_list[0]=T_list;
      AirQuality_list[1]=rH_list;
      AirQuality_list[2]=CO2_list;
      AirQuality_list[3]=VOC_list; 
    }

    if(sensor_sht && sensor_water){
        TrHW_list[0]=T_list;
        TrHW_list[1]=rH_list;
        TrHW_list[2]= water_leakage;
        TrHW_list[3]= water_detection;
    }
    
}

//---------------------------------------------
void PrintResults_TFT(){
  spriteBoady.fillRect(0 , 0, SPRITE_BOADY_WIDTH, SPRITE_BOADY_HEIGHT, TFT_BLACK);
  
  spriteBoady.fillRoundRect(5,5,130,55,4, purple);
  spriteBoady.fillRoundRect(5,70,130,55,4, TFT_BLUE);
  
  if (sensor_sht){
    spriteBoady.setTextColor(TFT_WHITE, purple);
    spriteBoady.drawString("T (C)" , 10, 10, 2);
    spriteBoady.drawString(String(T_list,1) , 50, 30, 4);
      
    spriteBoady.setTextColor(TFT_WHITE, TFT_BLUE);
    spriteBoady.drawString("rH (%)", 10, 75, 2);
    spriteBoady.drawString(String(rH_list,1) , 50, 95, 4);
  }
  
  if (sensor_water){
    if (water_leakage){
      spriteBoady.fillRoundRect(140,5,175,120,4, TFT_RED);
      spriteBoady.setTextColor(TFT_WHITE, TFT_RED);
      spriteBoady.drawString("Water leakage" , 150, 10, 4);
      spriteBoady.drawString("YES" , 200, 60, 4);
    }  
    else{
      spriteBoady.fillRoundRect(140,5,175,120,4, TFT_DARKGREEN);
      spriteBoady.setTextColor(TFT_WHITE, TFT_DARKGREEN);
      spriteBoady.drawString("Water leakage" , 150, 10, 4);
      spriteBoady.drawString("NO" , 200, 60, 4);
    }
    if (water_detection > 2000){
      spriteBoady.fillRoundRect(140,5,175,120,4, gray);
      spriteBoady.setTextColor(TFT_WHITE, gray);
      spriteBoady.drawString("Water leakage" , 150, 10, 4);
      spriteBoady.drawString("NO INFO" , 170, 60, 4);
    }
  }
  
      
  spriteBoady.pushSprite(0, 40);
}


//-----------------------Other functions ------------
void array_to_u32(u32* value, u8* array) {
    (*value) = (*value) | (u32)array[0] << 24;
    (*value) = (*value) | (u32)array[1] << 16;
    (*value) = (*value) | (u32)array[2] << 8;
    (*value) = (*value) | (u32)array[3];
}
void u32_to_array(u32 value, u8* array) {
    if (!array) {
        return;
    }
    array[0] = value >> 24;
    array[1] = value >> 16;
    array[2] = value >> 8;
    array[3] = value;
}

/*
    Reset baseline per hour,store it in EEPROM;
*/
void store_baseline(void) {
    static u32 i = 0;
    u32 j = 0;
    u32 iaq_baseline = 0;
    u8 value_array[4] = {0};
    i++;
    //Serial.println(i);
    if (i == 3600) {
        i = 0;
        if (sgp_get_iaq_baseline(&iaq_baseline) != STATUS_OK) {
            Serial.println("get baseline failed!");
        } else {
            Serial.println(iaq_baseline, HEX);
            Serial.println("get baseline");
            u32_to_array(iaq_baseline, value_array);
            for (j = 0; j < 4; j++) {
                EEPROM.write(j, value_array[j]);
                Serial.print(value_array[j]);
                Serial.println("...");
            }
            EEPROM.write(j, BASELINE_IS_STORED_FLAG);
        }
    }
    delay(LOOP_TIME_INTERVAL_MS);
}

/*  Read baseline from EEPROM and set it.If there is no value in EEPROM,retrun .
    Another situation: When the baseline record in EEPROM is older than seven days,Discard it and return!!

*/
void set_baseline(void) {
    u32 i = 0;
    u8 baseline[5] = {0};
    u32 baseline_value = 0;
    for (i = 0; i < 5; i++) {
        baseline[i] = EEPROM.read(i);
        Serial.print(baseline[i], HEX);
        Serial.print("..");
    }
    Serial.println("!!!");
    if (baseline[4] != BASELINE_IS_STORED_FLAG) {
        Serial.println("There is no baseline value in EEPROM");
        return;
    }
    /*
        if(baseline record in EEPROM is older than seven days)
        {
        return;
        }
    */
    array_to_u32(&baseline_value, baseline);
    sgp_set_iaq_baseline(baseline_value);
    Serial.println(baseline_value, HEX);
}

//--------------------------------------------------------
void I2C_scanner()
{
  byte error, address;
  int nDevices = 0;

  currentLine_y += 2*16;
  tft.drawString("Scanning I2C ...", 5, currentLine_y, 2);

  currentLine_y += 16;
  int y_offset = 1;
  
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16){
        Serial.print("0");
        tft.drawString("I2C device found at address 0x0", 5, currentLine_y, 2);
      } 
      Serial.print(address,HEX);
      Serial.println("  !");
      
      nDevices++;
      y_offset = (currentLine_y-16) + nDevices*16;
      tft.drawString("I2C device found at address 0x"+String(address, HEX) + " !", 5, y_offset, 2);
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
        tft.drawString("Unknown error at address 0x0", 5, currentLine_y, 2);
      }
      Serial.println(address,HEX);
      tft.drawString("Unknown error at address 0x"+String(address, HEX), 5, currentLine_y, 2);
    }    
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found\n");
    tft.drawString("No I2C devices found", 5, currentLine_y, 2);
  } else
    Serial.println("Scan finished\n");
}

void welcome_screen(){
  // YU logo
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_WHITE);
  tft.pushImage(0,0,DISPLAY_WIDTH, 146,YU_logo);
  
  delay(3000);

  // wipe screen
  tft.fillScreen(TFT_BLACK); currentLine_y = 0;
  // welcome message
  tft.drawString("Air Quality Monitoring", 35, currentLine_y+5, 4);
  
  currentLine_y = currentLine_y + 2*16;
  tft.drawString("Designed by Mohammad Kareem, PhD", 45, currentLine_y, 2);

  currentLine_y = currentLine_y + 2*16;
  tft.drawString("mkareem@yorku.ca", 85, currentLine_y, 2);

  currentLine_y = currentLine_y + 2*16;
  tft.drawString("Department of Physics & Astronomy", 45, currentLine_y, 2);

  currentLine_y = currentLine_y + 16;
  tft.drawString("York University, Toronto, Canada", 45, currentLine_y, 2);

  tft.drawString("firmware: "+String(firmware_version), DISPLAY_WIDTH - 130, DISPLAY_HEIGHT - 20, 2);

  delay(5000);
  tft.fillScreen(TFT_BLACK);
}

void saveConfigs(){
  EEPROM.writeInt(4, _device_number);
  EEPROM.writeInt(8, _power_saving);
  EEPROM.writeInt(12, _MQTT_PUBLISH_DELAY_ms);
  EEPROM.commit();
}

void updateConfigs(){
  String payload = OTADRIVE.getConfigs();
  StaticJsonBuffer<300> jb;
  JsonObject& obj = jb.parseObject(payload);
  
  if (!obj.success())
  {
    Serial.println("parseObject() failed");
    return;
  }
  
  char JSONmessageBuffer[50];
  obj.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Config content: " + String(JSONmessageBuffer));
  
 _device_number = (int)obj["device_number"];
 _power_saving = (int)obj["power_saving"];
 _MQTT_PUBLISH_DELAY_ms = (int)obj["measuring_rate"];
 Serial.println("_device_number: " + String(_device_number));
 Serial.println("_power_saving: " + String(_power_saving));
 Serial.println("_MQTT_PUBLISH_DELAY_ms: " + String(_MQTT_PUBLISH_DELAY_ms));
 saveConfigs();
}

void loadConfigs(){
  // check configs initialized in eeprom or not
  if (EEPROM.readInt(0) != 0x4A)
  {
    // configs not initialized yet. write for first time
    EEPROM.writeInt(0, 0x4A); // memory sign
    EEPROM.writeInt(4, _device_number);  // _device_number default value
    EEPROM.writeInt(8, _power_saving);  // _power_saving default value
    EEPROM.writeInt(12, _MQTT_PUBLISH_DELAY_ms);  // _power_saving default value
    EEPROM.commit();
  }
  else
  {
    // configs initialized and valid. read values
    _device_number = EEPROM.readInt(4);
    _power_saving = EEPROM.readInt(8);
    _MQTT_PUBLISH_DELAY_ms = EEPROM.readInt(12);
  }
}

void ota_proggress(size_t downloaded, size_t total)
{
  int percent = downloaded / (total / 100 + 1);
  tft.setCursor(0, 20);
  tft.printf("%d/%d   %d%%", downloaded, total, percent);
  tft.fillRect(0, 56, percent * 2, 10, TFT_GREEN);
}

void MQTT_publish(){
  char buffer[16];
  itoa(_device_number, buffer, 10);
  const char* char_helper = buffer;
  // first wipe the chars 
  memset(device_name, 0, 100);
  memset(MQTT_TOPIC_STATE, 0, 100);
  memset(MQTT_TOPIC_CO2VOC, 0, 100);
  memset(MQTT_TOPIC_TRH, 0, 100);
  memset(MQTT_TOPIC_AIRQUALITY, 0, 100);
      
  strcat(device_name, device_pref); strcat(device_name, char_helper);
  strcat(MQTT_TOPIC_STATE, device_location); strcat(MQTT_TOPIC_STATE, "/"); strcat(MQTT_TOPIC_STATE, device_name); strcat(MQTT_TOPIC_STATE, "/status");
    
  if (!mqttClient.connected()){
    mqttReconnect();
  }

  if (sensor_type =="Multi"){
    if (sensor_sht && sensor_gas){
      strcat(MQTT_TOPIC_AIRQUALITY, device_location); strcat(MQTT_TOPIC_AIRQUALITY, "/"); strcat(MQTT_TOPIC_AIRQUALITY, device_name); strcat(MQTT_TOPIC_AIRQUALITY, "/measurements/AQ");
      mqtt_jason_publish(MQTT_TOPIC_AIRQUALITY, "AQ", AirQuality_list, 4);
    }
    else if (sensor_sht && sensor_water){
      strcat(MQTT_TOPIC_TRHW, device_location); strcat(MQTT_TOPIC_TRHW, "/"); strcat(MQTT_TOPIC_TRHW, device_name); strcat(MQTT_TOPIC_TRHW, "/measurements/TrHW");
      mqtt_jason_publish(MQTT_TOPIC_TRHW, "TrHW", TrHW_list, 4);
    }
    
  } 
  else{ // for single sensors
    if(sensor_sht){
      strcat(MQTT_TOPIC_TRH, device_location); strcat(MQTT_TOPIC_TRH, "/"); strcat(MQTT_TOPIC_TRH, device_name); strcat(MQTT_TOPIC_TRH, "/measurements/TrH");
      mqtt_jason_publish(MQTT_TOPIC_TRH, "TrH", TrH_list, 2);
    }
    else if(sensor_gas) {
      strcat(MQTT_TOPIC_CO2VOC, device_location); strcat(MQTT_TOPIC_CO2VOC, "/"); strcat(MQTT_TOPIC_CO2VOC, device_name); strcat(MQTT_TOPIC_CO2VOC, "/measurements/CO2VOC");
      mqtt_jason_publish(MQTT_TOPIC_CO2VOC, "CO2VOC", CO2VOC_list, 2);
    }
    else if(sensor_water) {
      strcat(MQTT_TOPIC_WATER, device_location); strcat(MQTT_TOPIC_WATER, "/"); strcat(MQTT_TOPIC_WATER, device_name); strcat(MQTT_TOPIC_WATER, "/measurements/Water");
      mqtt_jason_publish(MQTT_TOPIC_WATER, "WATER", water_leakage_list, 2);
    }
  }
  
  
        
  mqttClient.loop();
    
  Serial.println("-------------");
}

String GetTime(){
  while(
    !timeClient.update()) {
    timeClient.forceUpdate();
  }
  String formattedTime = timeClient.getFormattedDate();
  int splitT = formattedTime.indexOf("T");
  String curTime = formattedTime.substring(splitT+1, formattedTime.length()-1);
  return curTime;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// setup().
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Serial.
  Serial.begin(115200);
  delay(100);
  
  if (do_ota) //OTA finctionality
  {
    // Register your firmware info to the library
    OTADRIVE.setInfo(String(ProductKey), String(firmware_version));
    // Register your update progrees handler
    OTADRIVE.onUpdateFirmwareProgress(ota_proggress);
  }
  
  Wire.begin(SDAPIN, SCLPIN); // SDA, SCL

  // Analog.
  analogReadResolution(12);
    
  // TFT Display.
  pinMode(EN_PIN, INPUT_PULLUP); // Set EN button as input with pull-up resistor
  tft.init();
  tft.setRotation(3);
  ledcSetup(0, 10000, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, DISPLAY_BRIGHTNESS_MAX);

  // Enable the modeul to operate from an external LiPo battery.
  pinMode(15, OUTPUT);
  digitalWrite(15, 1);

  // Creating Sprites
  spriteHeader.createSprite(SPRITE_HEADER_WIDTH, SPRITE_HEADER_HEIGHT);
  spriteHeader.setSwapBytes(true);

  spriteNet.createSprite(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  spriteNet.setSwapBytes(true);

  spriteBoady.createSprite(SPRITE_BOADY_WIDTH, SPRITE_BOADY_HEIGHT);
  spriteBoady.setSwapBytes(true);

  welcome_screen();
  
  if (do_wifi)
  { setupNetwork();
    setupTime();
  }
  
  I2C_scanner();

  EEPROM.begin(32);
  
  if(do_ota)
    loadConfigs();
  
  
  //--- sensors managements
  if (sensor_sht + sensor_water + sensor_gas >= 2)
    sensor_type = "Multi";
  else if (sensor_sht) sensor_type = "TrH";
  else if (sensor_water) sensor_type = "Water";
  else if (sensor_gas) sensor_type = "CO2VOC";
  else sensor_type = "None";
  
  if (sensor_sht){
    if(sht.init()) {
      Serial.print("sht_xx: init success\n");
    }
    else {
      Serial.print("sht_xx: init failed\n");
    }
  }

  if (sensor_water){
    if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw (water sensor) not found");
    }
    else {
      Serial.print("seesaw (water sensor) started! version: ");
      Serial.println(ss.getVersion(), HEX);
    }
  }
    
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  
  s16 err;
  u16 scaled_ethanol_signal, scaled_h2_signal;

  //Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
  //all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc
    
  // Read H2 and Ethanol signal in the way of blocking
    if (sensor_gas){
      while (sgp_probe() != STATUS_OK) {
          Serial.println("SGP failed");
          while (1);
      }
      err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
      if (err == STATUS_OK) {
          Serial.println("get ram signal!");
      } else {
          Serial.println("error reading signals");
      }
      // err = sgp_iaq_init();
      set_baseline();
    }
    
  delay(5000);
  
  // wipe entire screen
  tft.fillScreen(TFT_BLACK); currentLine_y = 0;
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// loop
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Read the battery voltage.  
  uVolt = (analogRead(4) * 2 * 3.3 * 1000) / 4096;

  // Update the Header sprite
  spriteHeader.fillRect(0 , 0, SPRITE_HEADER_WIDTH, SPRITE_HEADER_HEIGHT, TFT_BLACK);
  spriteHeader.setTextColor(TFT_YELLOW, TFT_BLACK);

  spriteHeader.drawString(String(uVolt / 1000) + "." + String(uVolt % 1000) + " vDC", DISPLAY_WIDTH -70, 16, SPRITE_HEADER_FONT);
  
  if(do_wifi)
  {
    if(EAP_wifi)
      spriteHeader.drawString(String(EAP_ssid), 0, 0, SPRITE_HEADER_FONT);
    else
      spriteHeader.drawString(String(ssid), 0, 0, SPRITE_HEADER_FONT);
    
    spriteHeader.drawString("IP: "+ stringIP, 100, 0, SPRITE_HEADER_FONT);
    spriteHeader.drawString("Device: "+ String(device_name), 0, 16, SPRITE_HEADER_FONT);
  }
  else
  {
    spriteHeader.drawString("Wifi disabled", 0, 0, SPRITE_HEADER_FONT);
  }
  
  ReadSystem();

  tft.drawLine(0,37, DISPLAY_WIDTH, 37, TFT_RED);
  tft.drawLine(0,38, DISPLAY_WIDTH, 38, TFT_RED);
  
  PrintResults_TFT();
  
  if (_power_saving==0) screen_on =true;
  else{
    if (digitalRead(EN_PIN) == LOW) { // Check if EN button is pressed
      screen_on = !screen_on; // Toggle screen state
      last_activity_time = millis(); // Update last activity time
    }
    
    if (millis() - last_activity_time > SCREEN_TIMEOUT && screen_on) // Check screen timeout
      screen_on = false; // Update screen state
  }

  if(screen_on) ledcWrite(0, DISPLAY_BRIGHTNESS_MAX);
  else ledcWrite(0, DISPLAY_BRIGHTNESS_OFF); 

  if(do_wifi)
  {
    if(WiFi.status() != WL_CONNECTED){
      ledcWrite(0, DISPLAY_BRIGHTNESS_MAX);
      tft.fillScreen(TFT_BLACK);
      tft.drawString("Retry " + String(counter) + ": connecting to network:",0,0, 2);
      
      setupNetwork();

      if (millis() - last_activity_time > WIFI_TIMEOUT ) // Check wifi timeout
        ESP.restart();
    }
    else{ // when wifi is connected
      timeStamp = GetTime();
      spriteHeader.drawString(timeStamp.substring(0,5), DISPLAY_WIDTH -40, 0, SPRITE_HEADER_FONT);
   
      if (do_ota) //OTA finctionality
      {
        if (OTADRIVE.timeTick(update_timeTick))
        {
          updateConfigs();
      
          auto inf = OTADRIVE.updateFirmwareInfo();
          if (inf.available)
          {
            ledcWrite(0, DISPLAY_BRIGHTNESS_MAX);
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(1, 1);
            tft.printf("Downloading new firmware: v%s", inf.version.c_str());
            OTADRIVE.updateFirmware();
            delay(2000);
            tft.fillScreen(TFT_BLACK);
          }
        }
      }
      
      MQTT_publish();
     
    } // end of wifi connected
  } // end of do_wifi functionality

  spriteHeader.pushSprite(2, 5); // draw on coordinates 2,5
  
  if(sensor_gas) store_baseline();
    
  delay(_MQTT_PUBLISH_DELAY_ms);
}