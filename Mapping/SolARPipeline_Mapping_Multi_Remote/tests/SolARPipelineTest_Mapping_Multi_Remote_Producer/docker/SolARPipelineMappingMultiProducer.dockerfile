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

## Copy SolARPipelineMappingMultiProducer app files
RUN mkdir SolARPipelineMappingMultiProducer

## Data files (maps, hololens configuration)
RUN mkdir SolARPipelineMappingMultiProducer/data
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens
ADD data/data_hololens/* /SolARPipelineMappingMultiProducer/data/data_hololens/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A
ADD data/data_hololens/loop_desktop_A/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A/000
ADD data/data_hololens/loop_desktop_A/000/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A/000/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A/001
ADD data/data_hololens/loop_desktop_A/000/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_A/001/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B
ADD data/data_hololens/loop_desktop_B/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B/000
ADD data/data_hololens/loop_desktop_B/000/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B/000/
RUN mkdir SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B/001
ADD data/data_hololens/loop_desktop_B/000/* /SolARPipelineMappingMultiProducer/data/data_hololens/loop_desktop_B/001/

## Libraries and modules
RUN mkdir SolARPipelineMappingMultiProducer/modules
ADD modules/* /SolARPipelineMappingMultiProducer/modules/

## Project files
ADD SolARPipelineTest_Mapping_Multi_Remote_Producer /SolARPipelineMappingMultiProducer/
RUN chmod +x /SolARPipelineMappingMultiProducer/SolARPipelineTest_Mapping_Multi_Remote_Producer
RUN mkdir .xpcf
ADD *.xml /.xpcf
ADD docker/start_producer.sh .
RUN chmod +x start_producer.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_producer.sh"  ]
