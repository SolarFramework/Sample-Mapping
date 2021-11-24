ECHO off

REM Get Map Update Service URL from parameters
IF "%1"=="" (
    ECHO You need to give Map Update Service URL as parameter!
    GOTO end
) ELSE (
    ECHO Map Update Service URL = %1
)

REM Set MapUpdate Service URL
SET MAPUPDATE_SERVICE_URL=%1

REM Set application log level
REM Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
SET SOLAR_LOG_LEVEL=INFO

docker rm -f solarservicemappingmultinodrop
docker run -d -p 50051:8080 -e SOLAR_LOG_LEVEL -e MAPUPDATE_SERVICE_URL -e "SERVICE_NAME=SolARServiceMappingMultiNoDrop" --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarservicemappingmultinodrop artwin/solar/services/mapping-multi-nodrop-service:latest

:end
