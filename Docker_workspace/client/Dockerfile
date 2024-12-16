FROM debian:latest                                                #use latest debian image as base
RUN apt-get update && \                                           #update the packetlist and install required packets mosquitto clients and avahi utils
    apt-get install -y mosquitto-clients avahi-utils
COPY docker-entrypoint.sh .                                       #copy the docker-entrypoint.sh script into the working directory of the container
RUN chmod a+x docker-entrypoint.sh                                #make the script executable
CMD ./docker-entrypoint.sh                                        #define the default command to run when the container starts

