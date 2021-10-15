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

## Copy SolARServiceMappingMultiProducer app files
RUN mkdir SolARServiceMappingMultiProducer

## Data files (maps, hololens configuration)
RUN mkdir SolARServiceMappingMultiProducer/data
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens
ADD data/data_hololens/* /SolARServiceMappingMultiProducer/data/data_hololens/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A
ADD data/data_hololens/loop_desktop_A/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A/000
ADD data/data_hololens/loop_desktop_A/000/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A/000/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A/001
ADD data/data_hololens/loop_desktop_A/000/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_A/001/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B
ADD data/data_hololens/loop_desktop_B/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B/000
ADD data/data_hololens/loop_desktop_B/000/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B/000/
RUN mkdir SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B/001
ADD data/data_hololens/loop_desktop_B/000/* /SolARServiceMappingMultiProducer/data/data_hololens/loop_desktop_B/001/

## Libraries and modules
RUN mkdir SolARServiceMappingMultiProducer/modules
ADD modules/* /SolARServiceMappingMultiProducer/modules/

## Project files
ADD SolARServiceTest_Mapping_Multi_Producer /SolARServiceMappingMultiProducer/
RUN chmod +x /SolARServiceMappingMultiProducer/SolARServiceTest_Mapping_Multi_Producer
RUN mkdir .xpcf
ADD *.xml /.xpcf
ADD docker/start_producer.sh .
RUN chmod +x start_producer.sh

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_producer.sh"  ]
