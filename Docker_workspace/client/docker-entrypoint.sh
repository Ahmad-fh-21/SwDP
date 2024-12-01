#!/bin/bash

echo "IoT-GW debug client"

while true; do
  echo "Subscribing to sensors ..."
  stdbuf -o0 mosquitto_sub --cafile /client_certs/ca.crt --cert /client_certs/client.crt --key /client_certs/client.key -h iotgw.local -p 8883 --tls-version tlsv1.3  -t "/sensors/#"
done

