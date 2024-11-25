FROM debian:latest
RUN apt-get update && apt-get install -y bluez python3 python3-pydbus procps mosquitto-clients python3-paho-mqtt avahi-utils

RUN useradd -m bluezuser && passwd -d bluezuser

COPY docker-entrypoint.sh .
COPY ble.py .
RUN chmod a+x /docker-entrypoint.sh
RUN chmod a+x /ble.py

USER bluezuser
CMD ./docker-entrypoint.sh

