# AirQualityMonitoring

## Introduction
The purpose of this project is to design a modular Air Quality Monitoring Device fro controlled atmosphere environment (cleanroom) to measure, record and monitor real-time temperature, humidity, and other air quality measures such as CO2 and Volatile Organic Compounds (VOC). The system is based on I2C sensors and ESP32 micro-controller which connects to a local WIFI Network and transmit the measured data over MQTT protocol to a hosting database. Grafana is used for data visualization and real-time monitoring.

## Required Hardwares
- ESP32 Development Board
- I2C Temp&Humi Sensor(SHT35). I've used [this](https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/Grove_I2C_HighAccuracy_Temp_Humi_Sensor(SHT35)_Web.pdf) sensor by Seeed.
- I2C VOC & ECO2 gas sensor (SGP30). I've used [this](https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/101020512_Web.pdf) sensor by Seeed.
- A 1602 LCD Display Screen with I2C module interface adapter. This is optional but nice to have to display the real-time measurements on the devise.

### Custom PCB design
For a more practical application, I've designed a custom PCB to mount the micro-controller and sensors. You can use the provided Gerber and Drill Files and order yours from any PCB manufacturer. I've used [JLCPCB](https://jlcpcb.com). Other parts you may need:
- two 15 Position Header Connector (2.54mm) Through Hole Tin, like [this](https://www.digikey.ca/en/products/detail/sullins-connector-solutions/PPTC151LFBN-RC/810153) one.
- two Connector Header Through Hole 4 position (2mm), like [this](https://www.digikey.ca/en/products/detail/sullins-connector-solutions/SWR201-NRTN-S04-SA-WH/2769602) one.


## Software







## 3D box
