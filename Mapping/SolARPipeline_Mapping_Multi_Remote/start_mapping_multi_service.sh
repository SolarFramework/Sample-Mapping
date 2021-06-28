#!/bin/bash

## Set application gRPC server url
export XPCF_GRPC_SERVER_URL=0.0.0.0:50051
## Set application gRPC max receive message size
export XPCF_GRPC_MAX_RECV_MSG_SIZE=7000000
## Set application gRPC max send message size
export XPCF_GRPC_MAX_SEND_MSG_SIZE=20000000
## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

./SolARPipeline_Mapping_Multi_Remote -m SolARPipeline_Mapping_Multi_Remote_modules.xml -p SolARPipeline_Mapping_Multi_Remote_properties.xml

