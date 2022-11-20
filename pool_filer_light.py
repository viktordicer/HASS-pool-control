import smbus
import sys
import getopt
import time 
import paho.mqtt.client as mqtt
import threading
from array import *
from datetime import datetime
import os

i2c_addr = 0x20 #I2C address for devices
i2c_register_out = [0x00, 0x01] #Set A and B to output
i2c_register = [0x12, 0x13] # 0x012 is A register and 0x13 is B register

bus = smbus.SMBus(1)

##	SET REGISTER AS OUTPUTS
#Device1
bus.write_byte_data(i2c_addr,i2c_register_out[1],0x00) # Set all of bank B to outputs 
bus.write_byte_data(i2c_addr,i2c_register_out[0],0x00) # Set all of bank A to outputs 

# Clear registers
	#Device1
bus.write_byte_data(i2c_addr,i2c_register[0],0)
bus.write_byte_data(i2c_addr,i2c_register[1],0)

#mqtt assign
Broker = "192.168.0.107"
sub_topic_filter = "command/pool/filter"
sub_topic_light = "command/pool/light"
sub_topic_state = "command/pool/state"
sub_topic_ph = "command/pool/ph"
sub_topic_batery = "command/pool/batery"

pub_topic_filter = "sensor/pool/filter"
pub_topic_light = "sensor/pool/light"
pub_topic_ph = "sensor/pool/ph"
pub_topic_batery = "sensor/pool/batery"

#Set bit function	
def set_bit(value, bit):
	return value | bit

#Clear bit function
def clear_bit(value, bit):
	return value & ~ bit

def write_data(device,register,bit):
    bus.write_byte_data(device,register,bit)

#read data from register
def read_data(device,register):
    return bus.read_byte_data(device, register)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("command/pool/#")

def is_bit_set(value, bit):
    if value & (1 <<(bit-1)):
        return "On"
    else:
        return "Off"

def on_message_filter(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    reg_A = read_data(i2c_addr, i2c_register[0])
    if message == "0":
        write_data(i2c_addr,i2c_register[0],clear_bit(reg_A,1))
        client.publish(pub_topic_filter, "Off")
    elif message == "1":
        write_data(i2c_addr,i2c_register[0],set_bit(reg_A,1))
        client.publish(pub_topic_filter, "On")

def on_message_light(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    reg_B = read_data(i2c_addr, i2c_register[1])
    print(reg_B)
    if message == "0":
        write_data(i2c_addr,i2c_register[1],clear_bit(reg_B,1))
        client.publish(pub_topic_light, "Off")
    elif message == "1":
        write_data(i2c_addr,i2c_register[1],set_bit(reg_B,1))
        client.publish(pub_topic_light, "On")
        
def on_message_ph(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    reg_A = read_data(i2c_addr, i2c_register[0])
    print(reg_A)
    if message == "0":
        write_data(i2c_addr,i2c_register[0],clear_bit(reg_A,2))
        client.publish(pub_topic_ph, "Off")
    elif message == "1":
        write_data(i2c_addr,i2c_register[0],set_bit(reg_A,2))
        client.publish(pub_topic_ph, "On")
        
def on_message_batery(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    reg_B = read_data(i2c_addr, i2c_register[1])
    print(reg_B)
    if message == "0":
        write_data(i2c_addr,i2c_register[1],clear_bit(reg_B,2))
        client.publish(pub_topic_ph, "Off")
    elif message == "1":
        write_data(i2c_addr,i2c_register[1],set_bit(reg_B,2))
        client.publish(pub_topic_ph, "On")

def on_message_state(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    reg_A = read_data(i2c_addr, i2c_register[0])
    reg_B = read_data(i2c_addr, i2c_register[1])
    client.publish(pub_topic_filter, is_bit_set(reg_A,1))
    client.publish(pub_topic_light, is_bit_set(reg_B,1))
    client.publish(pub_topic_ph, is_bit_set(reg_A,2))
    client.publish(pub_topic_batery, is_bit_set(reg_B,2))

client = mqtt.Client()
client.on_connect = on_connect
client.message_callback_add(sub_topic_filter, on_message_filter)
client.message_callback_add(sub_topic_light, on_message_light)
client.message_callback_add(sub_topic_ph, on_message_ph)
client.message_callback_add(sub_topic_batery, on_message_batery)
client.message_callback_add(sub_topic_state, on_message_state)
client.username_pw_set(username="viktor", password="viktor")
client.connect(Broker, 1883, 60)
client.loop_forever()
