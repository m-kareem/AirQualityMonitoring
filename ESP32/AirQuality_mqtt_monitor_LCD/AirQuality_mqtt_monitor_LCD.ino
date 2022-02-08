#include <Arduino.h>
#include <EEPROM.h>

#include "sensirion_common.h"
#include "sgp30.h" // for CO2 sensor
#include "Seeed_SHT35.h" // for temperature / humidity SHT sensor

#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <MQTT.h>

#include "AirQuality_mqtt_monitor_LCD_config.h" // Update this file with your configuration

#include <LiquidCrystal_I2C.h>
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

#define LED 2
/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SDAPIN  20
  #define SCLPIN  21
  #define RSTPIN  7
  #define SERIAL SerialUSB
#else
  #define SDAPIN  A4
  #define SCLPIN  A5
  #define RSTPIN  2
  #define SERIAL Serial
#endif


#define LOOP_TIME_INTERVAL_MS  2000
#define BASELINE_IS_STORED_FLAG  (0X55)
//#define ARRAY_TO_U32(a)  (a[0]<<24|a[1]<<16|a[2]<<8|a[3])    //MSB first  //Not suitable for 8-bit platform

// ----------- variable definitions
int counter = 0;
int n_chan;
float CO2_list[8];
float VOC_list[8];
float CO2VOC_list[8][2];

float rH_list[8];
float T_list[8];
float TrH_list[8][2];
//float AirQuality_list[8][4];

const float invalidData = -999;

//-------------creating MQTT topics --------------------
char MQTT_TOPIC_STATE[100]= "";
char MQTT_TOPIC_CO2[100]= "";
char MQTT_TOPIC_VOC[100]= "";
char MQTT_TOPIC_CO2VOC[100]= "";

char MQTT_TOPIC_HUMIDITY[100]= "";
char MQTT_TOPIC_TEMPERATURE[100]= "";
char MQTT_TOPIC_TRH[100]= "";

//char MQTT_TOPIC_AIRQUALITY[200]= "";

//---- objects
SHT35 sensor(SCLPIN);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

//================== Functions ==============================

void setupNetwork() {
  //bool wifi_status = false;
  if (EAP_wifi){
   setupEAP_Wifi();
  }
  else{
    setupWifi();
  }

  if(WiFi.status() == WL_CONNECTED){
    Serial.println("Connected to network");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

//------------------------------
void setupWifi() {
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true);
  delay(100);  // <- fixes some issues with WiFi stability
  WiFi.begin(ssid, password);
}

//---------------------------------------------------------------
void setupEAP_Wifi() {
  Serial.print("Connecting to network: ");
  Serial.println(EAP_ssid);
  WiFi.disconnect(true);
  delay(100);  // <- fixes some issues with WiFi stability

  WiFi.mode(WIFI_STA); //init wifi mode
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ANONYMOUS_IDENTITY, strlen(EAP_ANONYMOUS_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
  esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function

  WiFi.begin(EAP_ssid); //connect to wifi
}
//---------------------------------------------------------------
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
      if (counter2>=10)//after 10 iteration timeout - reset board
        ESP.restart();

    }
  }
}

//---------------------------------------------------------------
void mqttPublish(const char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}
//---------------------------------------------------------------
void mqtt_jason_publish(String topic, String dataType, float list[][2])
{
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["device"] = device_name;
  JSONencoder["sensorType"] = sensor_type;

  JsonArray& values = JSONencoder.createNestedArray(dataType);

  for (int i = 0; i < n_chan; i++){
      values.add(list[i][0]);
      values.add(list[i][1]);
      //values.add(list[i][2]);
      //values.add(list[i][3]);
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
//------------------------------------------
void SetI2CConnection(uint8_t i) {
  if (i > n_chan-1) return;                // If more than 7 out of range, do nothing

  Wire.beginTransmission(TCAADDR);     // All data to go to address 0x70 (The Multiplexer address)
  Wire.write(1 << i);               // write value as a single bit in an 8bit byte, MSB being I2C no. 7 LSB being I2C no. 0
  Wire.endTransmission();           // Signal sending stopped
  //Serial.printf("\nI2C channel %d is selected! \n", i);
}
//----------------------------------------------
void ReadSystem(){
  s16 err = 0;
  u16 _co2_eq_ppm, _tvoc_ppb;
  float co2_eq_ppm, tvoc_ppb, temp, hum;
  for (int i = 0; i < n_chan; i++){

    if(multiplexer) SetI2CConnection(i);   // Loop through each connected I2C sensor on the I2C buses

    if(sensor_type=="SGP30"){
        err = sgp_measure_iaq_blocking_read(&_tvoc_ppb, &_co2_eq_ppm);
        co2_eq_ppm = (float) _co2_eq_ppm;
        tvoc_ppb = (float) _tvoc_ppb;

        if (err == STATUS_OK) {
            CO2_list[i]=co2_eq_ppm;
            VOC_list[i]=tvoc_ppb;
            CO2VOC_list[i][0]=CO2_list[i];
            CO2VOC_list[i][1]=VOC_list[i];
        }
        else{
          CO2_list[i]= invalidData; VOC_list[i]= invalidData; CO2VOC_list[i][0]=invalidData; CO2VOC_list[i][1]=invalidData;
          Serial.println("error reading IAQ values\n");
        }
    }
    else if(sensor_type=="SHT35"){
      if(NO_ERROR==sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)){
        T_list[i]=round(temp*100)/100;
        rH_list[i]=round(hum*100)/100;
        TrH_list[i][0]=T_list[i];
        TrH_list[i][1]=rH_list[i];
      }
      else{T_list[i]= invalidData; rH_list[i]= invalidData; TrH_list[i][0]=invalidData; TrH_list[i][1]=invalidData;}
      
    }
    else if(sensor_type=="Multi"){
      err = sgp_measure_iaq_blocking_read(&_tvoc_ppb, &_co2_eq_ppm);
        co2_eq_ppm = (float) _co2_eq_ppm;
        tvoc_ppb = (float) _tvoc_ppb;

        if (err == STATUS_OK) {
            CO2_list[i]=co2_eq_ppm;
            VOC_list[i]=tvoc_ppb;
            CO2VOC_list[i][0]=CO2_list[i];
            CO2VOC_list[i][1]=VOC_list[i];
        }
        else{
          CO2_list[i]= invalidData; VOC_list[i]= invalidData; CO2VOC_list[i][0]=invalidData; CO2VOC_list[i][1]=invalidData;
          Serial.println("error reading IAQ values\n");
        }
        
        if(NO_ERROR==sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)){
        T_list[i]=round(temp*100)/100;
        rH_list[i]=round(hum*100)/100;
        TrH_list[i][0]=T_list[i];
        TrH_list[i][1]=rH_list[i];
      }
      else{T_list[i]= invalidData; rH_list[i]= invalidData; TrH_list[i][0]=invalidData; TrH_list[i][1]=invalidData;}
    }

    else{
      Serial.println("Sensor is not supported!");
      return;
      }
  }// end looping over i2c channels
}
//----------------------------------------------
void PrintResults(){
    Serial.print("CO2(ppm): ");
    for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",CO2_list[i]);
    }
    Serial.print("\n");

    Serial.print("VOC(ppb): ");
    for (int i = 0; i < n_chan; i++){
      //if(T_list[i]== invalidData)
       // Serial.printf("%s,  ","none");
      //else
        Serial.printf("%4.1f,  ",VOC_list[i]);
    }
    Serial.print("\n");
}
//----------------------------------------------
void PrintResults_LCD(){
    if(sensor_type=="SGP30"){
      Serial.print("CO2(ppm): ");
      lcd.setCursor(0, 0);
      lcd.print("CO2(ppm): ");
    
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",CO2_list[i]);
        lcd.setCursor(11, 0);
        if(CO2_list[i]==invalidData){lcd.printf("n.a");}
        else{lcd.printf("%4.0f  ",CO2_list[i]);}
        
      }
      Serial.print("\n");

      Serial.print("VOC(ppb): ");
      lcd.setCursor(0, 1);
      lcd.print("VOC(ppb): ");
      for (int i = 0; i < n_chan; i++){
        //if(T_list[i]== invalidData)
        // Serial.printf("%s,  ","none");
      //else
        Serial.printf("%4.1f,  ",VOC_list[i]);
        lcd.setCursor(11, 1);
        lcd.printf("%4.0f  ",VOC_list[i]);
      }
      
    }
    else if (sensor_type=="SHT35"){
      Serial.print("Humidity(%RH): ");
      lcd.setCursor(0, 1);
      lcd.print("rH:");
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",rH_list[i]);
        lcd.setCursor(2, 1);
        lcd.printf("%4.0f  ",rH_list[i]);
      }
      Serial.print("Temperature(C): ");
      lcd.setCursor(7, 1);
      lcd.print("-T(C): ");
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",T_list[i]);
        lcd.setCursor(12, 1);
        lcd.printf("%4.0f  ",T_list[i]);
      }
    }
    else if (sensor_type=="Multi"){
      Serial.print("CO2(ppm):");
      lcd.setCursor(0, 0);
      lcd.print("CO2(ppm): ");
    
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",CO2_list[i]);
        if(CO2_list[i]==invalidData){lcd.setCursor(13, 0); lcd.printf("N.A");}
        else{lcd.setCursor(12, 0); lcd.printf("%4.0f  ",CO2_list[i]);}
      }
      Serial.print("\n");

      Serial.print("VOC(ppb): ");
      //lcd.setCursor(0, 1);
      //lcd.print("VOC(ppb): ");
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",VOC_list[i]);
        //lcd.setCursor(11, 1);
        //lcd.printf("%4.0f  ",VOC_list[i]);   
    }
    Serial.print("Humidity(%RH): ");
      lcd.setCursor(0, 1);
      lcd.print("H%:");
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",rH_list[i]);
        lcd.setCursor(2, 1);
        if(rH_list[i]==invalidData){lcd.setCursor(3, 1); lcd.printf("N.A");}
        else{lcd.printf("%4.0f  ",rH_list[i]);}
      }
      Serial.print("Temperature(C): ");
      lcd.setCursor(7, 1);
      lcd.print(",T(C): ");
      for (int i = 0; i < n_chan; i++){
        Serial.printf("%4.1f,  ",T_list[i]);
        lcd.setCursor(12, 1);
        if(T_list[i]==invalidData){lcd.setCursor(13, 1); lcd.printf("N.A");}
        else{lcd.printf("%4.0f  ",T_list[i]);}
      }
    }

    
    Serial.print("\n");
}
//---------------------------------------------
void setNchan(){
  if(multiplexer) n_chan=8;
  else n_chan=1;
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
    Serial.println(i);
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


//########################################################################
void setup() {
    while (!Serial);
    delay(500);
    Serial.begin(115200);
    Wire.begin(); // SDA, SCL
    //delay(100);
    pinMode(LED,OUTPUT);

    setupNetwork();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    setNchan();
    strcat(MQTT_TOPIC_STATE, device_location); strcat(MQTT_TOPIC_STATE, "/"); strcat(MQTT_TOPIC_STATE, device_name); strcat(MQTT_TOPIC_STATE, "/status");
    
    strcat(MQTT_TOPIC_CO2, device_location); strcat(MQTT_TOPIC_CO2, "/"); strcat(MQTT_TOPIC_CO2, device_name); strcat(MQTT_TOPIC_CO2, "/measurements/co2");
    strcat(MQTT_TOPIC_VOC, device_location); strcat(MQTT_TOPIC_VOC, "/"); strcat(MQTT_TOPIC_VOC, device_name); strcat(MQTT_TOPIC_VOC, "/measurements/voc");
    strcat(MQTT_TOPIC_CO2VOC, device_location); strcat(MQTT_TOPIC_CO2VOC, "/"); strcat(MQTT_TOPIC_CO2VOC, device_name); strcat(MQTT_TOPIC_CO2VOC, "/measurements/CO2VOC");

    strcat(MQTT_TOPIC_HUMIDITY, device_location); strcat(MQTT_TOPIC_HUMIDITY, "/"); strcat(MQTT_TOPIC_HUMIDITY, device_name); strcat(MQTT_TOPIC_HUMIDITY, "/measurements/rH");
    strcat(MQTT_TOPIC_TEMPERATURE, device_location); strcat(MQTT_TOPIC_TEMPERATURE, "/"); strcat(MQTT_TOPIC_TEMPERATURE, device_name); strcat(MQTT_TOPIC_TEMPERATURE, "/measurements/T");
    strcat(MQTT_TOPIC_TRH, device_location); strcat(MQTT_TOPIC_TRH, "/"); strcat(MQTT_TOPIC_TRH, device_name); strcat(MQTT_TOPIC_TRH, "/measurements/TrH");

    //strcat(MQTT_TOPIC_AIRQUALITY, device_location); strcat(MQTT_TOPIC_AIRQUALITY, "/"); strcat(MQTT_TOPIC_AIRQUALITY, device_name); strcat(MQTT_TOPIC_AIRQUALITY, "/measurements/AirQuality");


    s16 err;
    u16 scaled_ethanol_signal, scaled_h2_signal;


    /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
        all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/
    while (sgp_probe() != STATUS_OK) {
        Serial.println("SGP failed");
        while (1);
    }
    /*Read H2 and Ethanol signal in the way of blocking*/
    err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
    if (err == STATUS_OK) {
        Serial.println("get ram signal!");
    } else {
        Serial.println("error reading signals");
    }
    // err = sgp_iaq_init();
    set_baseline();
    //

    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
}
//------------------------------
void loop() {
    ReadSystem();
    //PrintResults();
    PrintResults_LCD();

    if(WiFi.status() != WL_CONNECTED){
      counter++;
      digitalWrite(LED,HIGH);
      Serial.print("Retry connecting to network: ");
      Serial.println(counter);
      setupNetwork();

      if(counter>=10) //after 10 iteration timeout - reset board
        ESP.restart();
    }
    else{
      if (!mqttClient.connected()){
        mqttReconnect();
      }

      //ReadSystem();
      //  //PrintResults();
      //PrintResults_LCD();

      mqtt_jason_publish(MQTT_TOPIC_CO2VOC, "CO2VOC", CO2VOC_list);
      delay(MQTT_PUBLISH_DELAY);
      mqtt_jason_publish(MQTT_TOPIC_TRH, "TrH", TrH_list);  
      //mqtt_jason_publish(MQTT_TOPIC_AIRQUALITY, "AirQuality", AirQuality_list);  
      
      mqttClient.loop();

      Serial.println("-------------");

      //--- LED signal on successful submission
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
   }
    store_baseline();
    delay(MQTT_PUBLISH_DELAY);
    //lcd.clear();
}
