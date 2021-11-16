#!/bin/sh

# Get Mapping Service URL from parameters
if [ "$1" ]
then
    echo "Mapping Service URL = $1"
    # Set Mapping Service URL
    export MAPPING_SERVICE_URL=$1
else
    echo "You need to give Mapping Service URL as first parameter!"
    exit 1
fi

# Get host IP for display
if [ "$2" ]
then
    echo "Display IP = $2"
else
    echo "You need to give host IP address for display as second parameter!"
    exit 1
fi

# Set Display IP
export DISPLAY=$2:0.0

# Set application log level
# Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

docker rm -f solarservicemappingmultiviewer
docker run -it -d -e DISPLAY -e MAPPING_SERVICE_URL -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARServiceMappingMultiViewer" -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarservicemappingmultiviewer artwin/solar/services/mapping-multi-viewer:latest
