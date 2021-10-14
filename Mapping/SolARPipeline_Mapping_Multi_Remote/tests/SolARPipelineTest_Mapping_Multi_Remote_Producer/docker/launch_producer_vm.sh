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

# Set Display IP
export DISPLAY=${DISPLAY}
xhost local:docker

# Get image data set from parameters
if [ "$2" ]
then
    # Set image data set
    export HOLOLENS_DATA_SET=$2
else
    echo "You can choose Hololens data set to use by setting A or B as second parameter (A by default)"
    # Set image data set to default
    export HOLOLENS_DATA_SET="A"
fi

echo "Hololens data set = $HOLOLENS_DATA_SET"

# Set application log level
# Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

docker rm -f solarpipelinemappingmultiproducer
docker run -it -d -e DISPLAY -e MAPPING_SERVICE_URL -e HOLOLENS_DATA_SET -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARPipelineMappingMultiProducer" -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarpipelinemappingmultiproducer artwin/solar/pipeline/mapping-multi-producer:latest
