#!/usr/bin/env python3

"""A MQTT to InfluxDB Bridge
This script receives MQTT data and saves those to InfluxDB.

##--ATTENTION--:
This file is supposed to be running in a docker image.
By modifying this file, do the following:
1. stop: docker stop mqttbridge
2. delete container: docker rm mqttbridge
3. delete image: docker rmi mqttbridge
4. rebuild (in the directory containing the Dockerfile): docker build -t mqttbridge .
5. rerun: docker run -d --restart always --name mqttbridge mqttbridge
6. check the status: docker ps -a , the container's status must be Up
"""

import re
from typing import NamedTuple, List

import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient

import time,datetime
import json


#INFLUXDB_ADDRESS = 'SC-L-PH-DEV-GP1.yorku.yorku.ca' # insert your database server address here
INFLUXDB_ADDRESS = 'influxdb' # this is the influxdb docker container name, defined in docker-compose
INFLUXDB_USER = 'admin'
INFLUXDB_PASSWORD = 'admin'
INFLUXDB_DATABASE = 'FSC_DB' # insert your database name here

#MQTT_ADDRESS = 'SC-L-PH-DEV-GP1.yorku.yorku.ca' # insert your MQTT server address here (usually same as database server)
MQTT_ADDRESS = 'mosquitto' # insert your MQTT server address here (usually same as database server)
MQTT_USER = 'mqttuser'
MQTT_PASSWORD = 'mqttpassword'
MQTT_TOPIC = '+/+/+/measurements/+' # The MQTT topic 'Lab' is defined in "device_location" in 'AirQuality_mqtt_monitor_LCD_config.h'
MQTT_REGEX = r'([^/]+/([^/]+)/([^/]+)/measurements/([^/]+))'

MQTT_CLIENT_ID = 'MQTTInfluxDBBridge'
noDataValue = -999
n_chan=8

influxdb_client = InfluxDBClient(INFLUXDB_ADDRESS, 8086, INFLUXDB_USER, INFLUXDB_PASSWORD, None)


class SensorData(NamedTuple):
    location: str
    device: str
    sensorType: str
    measurement: str
    value: str
    validity: str


def check_validity(value_str):
    validity = ['false'] * len(value_str)
    for i in range(len(value_str)):
        if(float(value_str[i])!=noDataValue):
            validity[i]='true'
    return validity


def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    #print('Connected with result code ' + str(rc))
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ' --- ' + str(msg.payload))
    #print(str(msg.payload))
    #x=msg.payload
    #y = json.loads(x)
    #print(y["temp"][0])
    sensor_data = _parse_mqtt_message(msg.topic, msg.payload.decode('utf-8'))


    if sensor_data is not None:
        print('Sending sensor data to influxDB')
        _send_sensor_data_to_influxdb(sensor_data)
        print(' ---------- ')


def _parse_mqtt_message(topic, payload):
    match = re.match(MQTT_REGEX, topic)
    if match:
        match = re.findall(r'[\w\.-]+',topic)
        location = match[1]
        print('location= '+location)
        device = match[2]
        print('device= '+device)
        measurement = match[4]
        print('measurement= '+measurement)
        SensorReading=json.loads(payload)
        print('SensorReading= '+str(SensorReading[measurement]))
        validity=check_validity(SensorReading[measurement])
        print(validity)
        return SensorData(location, device, SensorReading["sensorType"], measurement, SensorReading[measurement], validity)
    else:
        return None


def _send_sensor_data_to_influxdb(sensor_data):
    
    json_body = [
                {
                    'measurement': sensor_data.measurement,
                    'tags': {
                        'location': sensor_data.location,
                        'device': sensor_data.device,
                        'sensor': sensor_data.sensorType,
                        'validity': sensor_data.validity[0],
                        },
                        'time': datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z',
                        'fields': {
                            'value_1': sensor_data.value[0],
                            'value_2': sensor_data.value[1]
                        }
                    }
                ]

    status = influxdb_client.write_points(json_body)
    print('Write to DB: ', status)



def _init_influxdb_database():
    databases = influxdb_client.get_list_database()
    if len(list(filter(lambda x: x['name'] == INFLUXDB_DATABASE, databases))) == 0:
        influxdb_client.create_database(INFLUXDB_DATABASE)
    influxdb_client.switch_database(INFLUXDB_DATABASE)



def main():
    _init_influxdb_database()

    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()


if __name__ == '__main__':
    print('MQTT to InfluxDB bridge')
    main()
