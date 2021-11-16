FROM ubuntu:18.04
MAINTAINER Christophe Cutullic christophe.cutullic@b-com.com

## Configure Ubuntu environment
RUN apt-get update -y
RUN apt-get install -y libgtk-3-0
RUN apt-get install -y libva-dev
RUN apt-get install -y libvdpau-dev

## Copy SolARServiceMappingMulti app files
RUN mkdir SolARServiceMappingMulti

## Data files (fbow vocabulary)
RUN mkdir SolARServiceMappingMulti/data
RUN mkdir SolARServiceMappingMulti/data/fbow_voc
ADD data/fbow_voc/* /SolARServiceMappingMulti/data/fbow_voc/

## Libraries and modules
RUN mkdir SolARServiceMappingMulti/modules
ADD modules/* /SolARServiceMappingMulti/modules/

## Project files
ADD SolARService_Mapping_Multi /SolARServiceMappingMulti/
RUN chmod +x /SolARServiceMappingMulti/SolARService_Mapping_Multi
RUN mkdir .xpcf
ADD *.xml /.xpcf/
ADD docker/start_server.sh .
RUN chmod +x start_server.sh

## Set application gRPC server url
ENV XPCF_GRPC_SERVER_URL=0.0.0.0:8080
## Set application gRPC max receive message size
ENV XPCF_GRPC_MAX_RECV_MSG_SIZE=7000000
## Set application gRPC max send message size
ENV XPCF_GRPC_MAX_SEND_MSG_SIZE=-1

## Set url to Map Update Service
ENV XPCF_GRPC_MAP_UPDATE_URL=map-update-pipeline.artwin.svc.cluster.local:80

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_server.sh"  ]
