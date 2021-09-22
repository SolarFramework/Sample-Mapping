ECHO off

REM Get Mapping Service URL from parameters
IF "%1"=="" (
    ECHO You need to give Mapping Service URL as first parameter!
    GOTO end
) ELSE (
    ECHO Mapping Service URL = %1
)

REM Set Mapping Service URL
SET MAPPING_SERVICE_URL=%1

REM Get host IP for display
IF "%2"=="" (
    ECHO You need to give host IP address for display as second parameter!
    GOTO end
) ELSE (
    ECHO Set display IP
    SET DISPLAY=%2:0.0
)

ECHO Display IP = %DISPLAY%

REM Get image data set from parameters
IF "%3"=="" (
    ECHO You can choose Hololens data set to use by setting A or B as third parameter - A by default
    REM Set image data set to default
    SET HOLOLENS_DATA_SET=A
) ELSE (
    ECHO Set image data set
    SET HOLOLENS_DATA_SET=%3
)

ECHO Hololens data set = %HOLOLENS_DATA_SET%

REM Set application log level
REM Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
SET SOLAR_LOG_LEVEL=INFO

docker rm -f solarpipelinemappingmultiproducer
docker run -it -d -e DISPLAY -e MAPPING_SERVICE_URL -e HOLOLENS_DATA_SET -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARPipelineMappingMultiProducer" -v /tmp/.X11-unix:/tmp/.X11-unix --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarpipelinemappingmultiproducer artwin/solar/pipeline/mapping-multi-producer:latest

:end