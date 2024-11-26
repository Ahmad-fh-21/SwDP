ARG DOCKERHUB_TAG=latest
FROM nodered/node-red:${DOCKERHUB_TAG}
RUN cd /usr/src/node-red && npm install --save  node-red-contrib-influxdb
USER root
RUN  apt-get update && apt-get install -y avahi-utils
USER node-red

