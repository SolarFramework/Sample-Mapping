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

## Copy SolARServiceMappingMultiViewer app files
RUN mkdir SolARServiceMappingMultiViewer

## Libraries and modules
RUN mkdir SolARServiceMappingMultiViewer/modules
ADD modules/* /SolARServiceMappingMultiViewer/modules/

## Project files
ADD SolARServiceTest_Mapping_Multi_Viewer /SolARServiceMappingMultiViewer/
RUN chmod +x /SolARServiceMappingMultiViewer/SolARServiceTest_Mapping_Multi_Viewer
RUN mkdir .xpcf
ADD *.xml /.xpcf/
ADD docker/start_viewer.sh .
RUN chmod +x start_viewer.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_viewer.sh"  ]
