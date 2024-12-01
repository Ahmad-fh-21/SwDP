FROM debian:latest
RUN apt-get update && \
    apt-get install -y mosquitto-clients avahi-utils
COPY docker-entrypoint.sh .
RUN chmod a+x docker-entrypoint.sh
CMD ./docker-entrypoint.sh

