#!/usr/bin/env python
import os
import paho.mqtt.client as mqtt
import time

#mqtt assign
Broker = "192.168.0.107"
pub_topic_outside = "sensor/pool/outside_temperature"
pub_topic_pool = "sensor/pool/temperature"

sleep_time = 120

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(pub_topic_pool)


def sensor():
    ds18b20 =[]
    for i in os.listdir('/sys/bus/w1/devices'):
        if i != 'w1_bus_master1':
            ds18b20.append(i)
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
        if read(ds18b20[1]) != None:
            print("Pool temperature : %0.3f C" % read(ds18b20[1]))
            client.publish(pub_topic_pool, read(ds18b20[1]))
        if read(ds18b20[0]) != None:
            print("Pool outside temperature : %0.3f C" % read(ds18b20[0]))
            client.publish(pub_topic_outside, read(ds18b20[0]))
        time.sleep(sleep_time)    
def kill():
    quit()
if __name__ == '__main__':
    try:
        serialNum = sensor()
        loop(serialNum)
        
    except KeyboardInterrupt:
        kill()
