#!/bin/python3

import paho.mqtt.client as mqtt
import time


client = mqtt.Client()


def on_log(client, userdata, level, buf):
    print("log: ", buf)


client.on_log = on_log

client.tls_set(
    '/client_certs/ca.crt', '/client_certs/client.crt',
    '/client_certs/client.key'
)

client.connect('iotgw.local', 8883, 60)
client.loop_start()

value = 0
while True:
    client.publish("/value", value)
    value += 1
    time.sleep(1)
