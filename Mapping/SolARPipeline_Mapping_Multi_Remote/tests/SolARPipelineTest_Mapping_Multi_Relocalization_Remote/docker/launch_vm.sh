#!/bin/sh

# Get Relocalization Service URL from parameters
if [ "$1" ]
then
    echo "Relocalization Service URL = $1"
    # Set Relocalization Service URL
    export RELOCALIZATION_SERVICE_URL=$1
else
    echo "You need to give Relocalization Service URL as first parameter!"
    exit 1
fi

# Get Mapping Service URL from parameters
if [ "$2" ]
then
    echo "Mapping Service URL = $2"
    # Set Mapping Service URL
    export MAPPING_SERVICE_URL=$2
else
    echo "You need to give Mapping Service URL as second parameter!"
    exit 1
fi

# Set Display IP
export DISPLAY=${DISPLAY}
xhost local:docker

# Get image data set from parameters
if [ "$3" ]
then
    # Set image data set
    export HOLOLENS_DATA_SET=$3
else
    echo "You can choose Hololens data set to use by setting A or B as 3rd parameter (A by default)"
    # Set image data set to default
    export HOLOLENS_DATA_SET="A"
fi

echo "Hololens data set = $HOLOLENS_DATA_SET"

# Set application log level
# Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

docker rm -f solarpipelinemappingmultirelocalizationclient
docker run -it -d -e DISPLAY -e RELOCALIZATION_SERVICE_URL -e MAPPING_SERVICE_URL -e HOLOLENS_DATA_SET -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARPipelineMappingMultiRelocalizationClient" -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarpipelinemappingmultirelocalizationclient artwin/solar/pipeline/mapping-multi-relocalization-client:latest
