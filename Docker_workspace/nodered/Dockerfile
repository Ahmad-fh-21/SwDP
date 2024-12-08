FROM nodered/node-red:3.1.0-16-bullseye
RUN cd /usr/src/node-red && npm install --save  node-red-contrib-influxdb
USER root
RUN  apt-get update && apt-get install -y avahi-utils
USER node-red

