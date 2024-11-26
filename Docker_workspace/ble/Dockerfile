FROM debian:latest
RUN apt-get update && apt-get install -y \
    bluez \
    python3 \
    python3-pydbus \
    python3-paho-mqtt \
    avahi-utils

RUN useradd -m bluezuser \
 && passwd -d bluezuser

COPY docker-entrypoint.sh .
COPY ble.py .
RUN chown bluezuser:bluezuser /docker-entrypoint.sh \
    && chmod a+x /docker-entrypoint.sh
RUN chown bluezuser:bluezuser /ble.py \
    && chmod a+x /ble.py

USER bluezuser
CMD ./docker-entrypoint.sh

