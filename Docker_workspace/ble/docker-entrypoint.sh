#!/bin/bash

bluetoothctl --timeout=2 scan on
sleep 5
python3 -u ./ble.py

