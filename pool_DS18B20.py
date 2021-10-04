#!/usr/bin/env python
import os
import paho.mqtt.client as mqtt
import time

#mqtt assign
Broker = "192.168.0.107"
pub_topic = "sensor/pool/temperature"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(pub_topic)

def sensor():
    for i in os.listdir('/sys/bus/w1/devices'):
        if i != 'w1_bus_master1':
            ds18b20 = i
    return ds18b20

def read(ds18b20):
    location = '/sys/bus/w1/devices/' + ds18b20 + '/w1_slave'
    tfile = open(location)
    text = tfile.read()
    tfile.close()
    secondline = text.split("\n")[1]
    temperaturedata = secondline.split(" ")[9]
    temperature = float(temperaturedata[2:])
    celsius = temperature / 1000
    return round(celsius, 1)

def loop(ds18b20):
    
    while True:
        client = mqtt.Client()
        client.on_connect = on_connect
        #client.on_message = on_message
        client.connect(Broker, 1883, 60)
        if read(ds18b20) != None:
            print("Current temperature : %0.3f C" % read(ds18b20))
            client.publish(pub_topic, read(ds18b20))
        time.sleep(60)    

    
if __name__ == '__main__':
        serialNum = sensor()
        loop(serialNum)

