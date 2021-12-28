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

## Copy SolARServiceMappingMultiRelocalizationClient app files
RUN mkdir SolARServiceMappingMultiRelocalizationClient

## Data files (hololens data set, hololens configuration)
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens
ADD data/data_hololens/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A
ADD data/data_hololens/loop_desktop_A/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/000
ADD data/data_hololens/loop_desktop_A/000/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/000/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/001
ADD data/data_hololens/loop_desktop_A/000/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/001/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B
ADD data/data_hololens/loop_desktop_B/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/000
ADD data/data_hololens/loop_desktop_B/000/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/000/
RUN mkdir SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/001
ADD data/data_hololens/loop_desktop_B/000/* /SolARServiceMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/001/

## Libraries and modules
RUN mkdir SolARServiceMappingMultiRelocalizationClient/modules
ADD modules/* /SolARServiceMappingMultiRelocalizationClient/modules/

## Project files
ADD SolARServiceTest_Mapping_Multi_Relocalization /SolARServiceMappingMultiRelocalizationClient/
RUN chmod +x /SolARServiceMappingMultiRelocalizationClient/SolARServiceTest_Mapping_Multi_Relocalization
RUN mkdir .xpcf
ADD *.xml /.xpcf/
ADD docker/start_client.sh .
RUN chmod +x start_client.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_client.sh"  ]
