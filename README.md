# Remote Air Quality Monitoring Module

## Introduction
The purpose of this project is to design a modular Remote Air Quality Monitoring Device for controlled atmosphere environment (cleanroom) to measure, record and monitor real-time temperature, humidity, and other air quality measures such as CO2 and Volatile Organic Compounds (VOC). The system is based on I2C sensors and ESP32 micro-controller which connects to a local WiFi Network and transmit the measured data over MQTT protocol to a hosting database. Grafana is used for data visualization and real-time monitoring.

## Required Hardwares
- ESP32 Development Board
- I2C Temp & Humi Sensor(SHT35). I've used [this](https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/Grove_I2C_HighAccuracy_Temp_Humi_Sensor(SHT35)_Web.pdf) sensor by Seeed.
- I2C VOC & ECO2 gas sensor (SGP30). I've used [this](https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/101020512_Web.pdf) sensor by Seeed.
- A 1602 LCD Display Screen with I2C module interface adapter. This is optional but nice to have to display the real-time measurements on the devise.

* A more elegent option would be to use LilyGo T-Display-S3 module which has a built-in LCD display. You can find more information about this module [here](https://github.com/Xinyuan-LilyGO/T-Display-S3)
* An updated code with Over The Air firmware updating feature is provided in 'T-Display-S3' directory. You can use this code to program the T-Display-S3 module. The code is based on PlatformIO IDE. You can use the provided platformio.ini file to configure your project. You may need to install other standard libraries depending on your operating system.

### Custom PCB design
For a more practical application, I've made a custom designed PCB to mount the micro-controller and sensors. You can use the provided Gerber and Drill Files and order yours from any PCB manufacturer. I've used [JLCPCB](https://jlcpcb.com). Other parts you may need for this PCB are:
- Two 15 Position Header Connector (2.54mm) Through Hole Tin, like [this](https://www.digikey.ca/en/products/detail/sullins-connector-solutions/PPTC151LFBN-RC/810153) one.
- Two Connector Header Through Hole 4 position (2mm), like [this](https://www.digikey.ca/en/products/detail/sullins-connector-solutions/SWR201-NRTN-S04-SA-WH/2769602) one.


## Micro-controller Software
The ESP32 micro-controller is programmed by Arduino IDE. The code is provided in ESP directory in 'AirQuality_mqtt_monitor_LCD'. The main dependencies are provided. You should directly copy them to 'Arduino/libraries'. You may also need to install other standard libraries depending on your operating system.

## Python bridge-script
Once the data is transmitted to the server machine and you where able to subscribe to the corresponding MQTT channel successfully, a python script needs to be running in background, listening to the incoming MQTT messages, paring the messages and write the data to an [InfluxDB](https://www.influxdata.com/get-influxdb/) database. The bridge script is provided in 'bridge_script' directory. Check it carefully and modify it based on your database configs.

This script needs to be running 24/7 in background. In order to make sure any system restart or power outage won't affect the data-flow, I've created a docker container with auto restart option and run the script in the container. You may find other solutions based on your needs.

## Data visualization
I've chosen [Grafana](https://grafana.com) (a multi-platform open source analytics and interactive visualization web application) for the data visualization. You need to link the InfluxDB as a data source to have a nice real-time monitoring system, which can be configured to be accessible from anywhere. There are many interesting features to implement in Grafana, such as setting Alerts, i.e., you can create a Discord channel and link it to Grafana to receive alerts and notify the subscribed users when a specified condition happen.

## 3D box design
You can 3D-print a box to accommodate the components. An stl file is provided in 'Box_3Ddesign' directory. Feel free to design a better looking box if you did not like this one.
