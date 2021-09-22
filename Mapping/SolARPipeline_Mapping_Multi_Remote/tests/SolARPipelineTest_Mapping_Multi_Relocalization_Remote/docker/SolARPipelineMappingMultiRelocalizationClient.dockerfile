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

## Copy SolARPipelineMappingMultiRelocalizationClient app files
RUN mkdir SolARPipelineMappingMultiRelocalizationClient

## Data files (hololens data set, hololens configuration)
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens
ADD data/data_hololens/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A
ADD data/data_hololens/loop_desktop_A/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/000
ADD data/data_hololens/loop_desktop_A/000/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/000/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/001
ADD data/data_hololens/loop_desktop_A/000/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_A/001/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B
ADD data/data_hololens/loop_desktop_B//* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/000
ADD data/data_hololens/loop_desktop_B/000/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/000/
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/001
ADD data/data_hololens/loop_desktop_B/000/* /SolARPipelineMappingMultiRelocalizationClient/data/data_hololens/loop_desktop_B/001/

## Libraries and modules
RUN mkdir SolARPipelineMappingMultiRelocalizationClient/modules
ADD modules/* /SolARPipelineMappingMultiRelocalizationClient/modules/

## Project files
ADD SolARPipelineTest_Mapping_Multi_Relocalization_Remote /SolARPipelineMappingMultiRelocalizationClient/
RUN chmod +x /SolARPipelineMappingMultiRelocalizationClient/SolARPipelineTest_Mapping_Multi_Relocalization_Remote
RUN mkdir .xpcf
ADD *.xml /.xpcf
ADD docker/start_client.sh .
RUN chmod +x start_client.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_client.sh"  ]
