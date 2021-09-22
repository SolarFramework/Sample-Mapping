FROM ubuntu:18.04
MAINTAINER Christophe Cutullic christophe.cutullic@b-com.com

## Configure Ubuntu environment
RUN apt-get update -y
RUN apt-get install -y libgtk-3-0
RUN apt-get install -y libva-dev
RUN apt-get install -y libvdpau-dev

# Manage graphical display
RUN apt-get install -y xterm
RUN useradd -ms /bin/bash xterm

## Copy SolARPipelineMappingMultiViewer app files
RUN mkdir SolARPipelineMappingMultiViewer

## Libraries and modules
RUN mkdir SolARPipelineMappingMultiViewer/modules
ADD modules/* /SolARPipelineMappingMultiViewer/modules/

## Project files
ADD SolARPipelineTest_Mapping_Multi_Remote_Viewer /SolARPipelineMappingMultiViewer/
RUN chmod +x /SolARPipelineMappingMultiViewer/SolARPipelineTest_Mapping_Multi_Remote_Viewer
RUN mkdir .xpcf
ADD *.xml /.xpcf
ADD docker/start_viewer.sh .
RUN chmod +x start_viewer.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_viewer.sh"  ]
