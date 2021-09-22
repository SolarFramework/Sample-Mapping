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

# Get image data set from parameters
if [ "$3" ]
then
    # Set image data set
    export HOLOLENS_DATA_SET=$3
else
    echo "You can choose Hololens data set to use by setting A or B as third parameter (A by default)"
    # Set image data set to default
    export HOLOLENS_DATA_SET="A"
fi

echo "Hololens data set = $HOLOLENS_DATA_SET"

# Set application log level
# Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

docker rm -f solarpipelinemappingmultiproducer
docker run -it -d -e DISPLAY -e MAPPING_SERVICE_URL -e HOLOLENS_DATA_SET -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARPipelineMappingMultiProducer" -v /tmp/.X11-unix:/tmp/.X11-unix --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarpipelinemappingmultiproducer artwin/solar/pipeline/mapping-multi-producer:latest
