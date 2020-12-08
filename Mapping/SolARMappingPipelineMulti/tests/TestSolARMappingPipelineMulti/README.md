# TestSolARMappingPipelineMulti

[![License](https://img.shields.io/github/license/SolARFramework/Sample-Mapping?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

**TestSolARMappingPipelineMulti** is a program used to test the multi threads mapping vision pipeline module based on SolAR Framework.

TestSolARMappingPipelineMulti is open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

## Configuration

* :warning: Don't forget to download the [fbow vocabularies](https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip), unzip this archive and put the `akaze.fbow` in your working directory. Then, you have to set the 'VOCpath' property of the 'SolARKeyframeRetrieverFBOW' component in the 'xpcf_SolARMappingPipelineMulti_registry.xml' file.

* TestSolARMappingPipelineMulti sample requires a full set of data to simulate a real camera capture (camera parameters, fiducial marker, camera images and poses). You could find such data set here (hololens 2 captures): https://artifact.b-com.com/webapp/#/artifacts/browse/tree/General/solar-generic-local/captures/hololens/bcomLab
* :warning: Don't forget to set the properties in the 'TestSolARMappingPipelineProducer_conf.xml' file for 'SolARFiducialMarkerLoaderOpencv' and 'SolARDeviceDataLoader' components.

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com
