#!/bin/python3

import pydbus
import paho.mqtt.client as mqtt
from gi.repository import GLib
import datetime

dev_id = '7C:DF:A1:E7:C6:DD'
bluez_service = 'org.bluez'
adapter_path = '/org/bluez/hci0'
device_path = f"{adapter_path}/dev_{dev_id.replace(':', '_')}"
temperature_uuid = '25ed47ec-2800-46f0-903d-c75afdd6cc45'
humidity_uuid = '9b5c4bed-5ffb-4c38-af8f-c52f01bd7c63'

# Setup MQTT connection
ble_client = mqtt.Client()


def on_log(client, userdata, level, buf):
    print("log: ", buf)


ble_client.on_log = on_log
ble_client.tls_set('/ble_certs/ca.crt', '/ble_certs/ca.crt', '/ble_certs/ble.key')
ble_client.connect('iotgw.local', 8883, 60)
ble_client.loop_start()

# Setup DBus information for adapter and remote device
bus = pydbus.SystemBus()
mngr = bus.get('org.bluez', '/')
adapter = bus.get('org.bluez', adapter_path)
device = bus.get('org.bluez', device_path)
# Connect to device (needs to have already been paired via bluetoothctl)
device.Connect()


def get_characteristic_path(dev_path, uuid):
    mng_objs = mngr.GetManagedObjects()
    for path in mng_objs:
        chr_uuid = mng_objs[path].get('org.bluez.GattCharacteristic1', {}).get('UUID')
        if path.startswith(dev_path) and chr_uuid == uuid:
            return path


while True:
    char_path = get_characteristic_path(device._path, temperature_uuid)
    temperature = bus.get(bluez_service, char_path)
    char_path = get_characteristic_path(device._path, humidity_uuid)
    humidity = bus.get(bluez_service, char_path)
    if (callable(getattr(temperature, "ReadValue", None)) and
            callable(getattr(humidity, "ReadValue", None))):
        break
    print('Retry...')

print(f"Temperature: {temperature.ReadValue({})}")
print(f"Humidity: {humidity.ReadValue({})}")


def temperature_change_handler(iface, prop_changed, prop_removed):
    if 'Value' in prop_changed:
        temp = prop_changed['Value'][1] + prop_changed['Value'][0] / 10.0
        t = datetime.datetime.now().strftime("%Y-%m-%d, %H:%M:%S")
        print(f"Time: {t} Temperature: {temp}C")
        ble_client.publish("/Temperature", f"Time: {t} Temperature: {temp}C")


def humidity_change_handler(iface, prop_changed, prop_removed):
    if 'Value' in prop_changed:
        humidity = prop_changed['Value'][1]
        t = datetime.datetime.now().strftime("%Y-%m-%d, %H:%M:%S")
        print(f"Time: {t} Humidity: {humidity}%")
        ble_client.publish("/Humidity", f"Time: {t} Humidity: {humidity}%")


mainloop = GLib.MainLoop()
temperature.onPropertiesChanged = temperature_change_handler
temperature.StartNotify()
humidity.onPropertiesChanged = humidity_change_handler
humidity.StartNotify()

try:
    mainloop.run()
except KeyboardInterrupt:
    mainloop.quit()
    temperature.StopNotify()
    humidity.StopNotify()
    device.Disconnect()
