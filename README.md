# Sample-Mapping
[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

The samples demonstrates mapping features such as mapping, map extension, map visualization.

The samples for mapping are open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

## How to run

### Install required data

Before running the samples, you need to download data such as Hololens captures, maps and the vocabulary of the bag of word used for image retrieval.
To install the required data, just launch the following script:

> #### Windows
>
	installData.bat

> #### Linux
>
	./installData.sh

This script will install the following data into the `./data` folder:
- The bag of words downloaded from our [GitHub releases](https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip) and unzipped in the `./data` folder.
- Hololens captures (image and poses) downloaded from our Artifactory ([Loop_Desktop_A](https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip) and [Loop_Desktop_B](https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopB.zip)) and copied into the `./data/data_hololens` folder.

<strong>Loop_Desktop_A</strong> is a video sequence captured with a Hololens 1 around a desktop starting and finishing with the fiducial Marker A with a loop trajectory. A fiducial marker B is captured during the trajectory.

<strong>Loop_Dekstop_B</strong> is a video sequence captured with a Hololens 1 within an open-space starting and finishing with the fiducial Marker B with a loop trajectory. A part of the trajectory is common to the `Loop_Desktop_A` trajectory to test map overlap detection and map fusion.


### Install required modules

Some samples require several SolAR modules such as OpenGL, OpenCV (with or without Cuda), FBOW (with or without Cuda), G20, Tools and PopSift. If they are not yet installed on your machine, please run the following command from the test folder:

<pre><code>remaken install packagedependencies.txt</code></pre>

and for debug mode:

<pre><code>remaken install packagedependencies.txt -c debug</code></pre>

For more information about how to install remaken on your machine, visit the [install page](https://solarframework.github.io/install/) on the SolAR website.

## Run the samples

### Mapping samples

These samples construct a 3D map from a AR device capture producing an image sequence with corresponding poses based on an AR runtime (e.g. Hololens SDK, ARkit, or ARCore). The samples can detect loop and correct the drift due to vision tracking. When the user exits the sample, the map is saved into a dedicated folder.

Also, the samples offer different modes of initialization, whether based on a fiducial marker which will define the reference coordinate system of the map, an existing 3D map previoulsy built, or no initial knowledge. In this last case, an initial map will be built based on a translation movement of the camera, and the reference coordinate system of the map will be defined by the first pose of the camera.

Four samples are available for the mapping in the `Mapping` folder:
* <strong>Sample Mono</strong>: It is a mono-threaded implementation of a standalone application which will load the AR device capture and will build the 3D map (recommended to better understand the pipeline with the source code).

> #### Windows
>
	SolARSample_Mapping_Mono.exe

> #### Linux
>
	./run.sh ./SolARSample_Mapping_Mono

Add `SolARSample_Mapping_Mono_Cuda_conf.xml` at the end of the command to run the sample with Cuda optimization (required CUDA to be installed on your computer).

* <strong>Sample Multi</strong>: It is a multi-threaded implementation of a standalone application which will load the AR device capture and will build the 3D map (recommended for better performances).

> #### Windows
>
	SolARSample_Mapping_Multi.exe

> #### Linux
>
	./run.sh ./SolARSample_Mapping_Multi

Add `SolARSample_Mapping_Multi_Cuda_conf.xml` at the end of the command to run the sample with Cuda optimization (required CUDA to be installed on your computer).

* <strong>Pipeline Mono</strong>: It is a mono-threaded implementation of a mapping pipeline which will consume images and corresponding poses, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.

> #### Windows
>
	SolARPipelineTest_Mapping_Mono.exe

> #### Linux
>
	./run.sh ./SolARPipelineTest_Mapping_Mono

Add `SolARPipelineTest_Mapping_Mono_Processing_Cuda_conf.xml` at the end of the command to run the pipeline test with Cuda optimization (required CUDA to be installed on your computer).

* <strong>Pipeline Multi</strong>: It is a multi-threaded implementation of a mapping pipeline which will consume images and corresponding poses, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.

> #### Windows
>
	SolARPipelineTest_Mapping_Multi.exe

> #### Linux
>
	./run.sh ./SolARPipelineTest_Mapping_Multi

Add `SolARPipelineTest_Mapping_Multi_Processing_Cuda_conf.xml` at the end of the command to run the pipeline test with Cuda optimization (required CUDA to be installed on your computer).

* <strong>Pipeline Multi No Drop</strong>: It is a multi-threaded implementation of a mapping pipeline which will consume images and corresponding poses without any image drop, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.

> #### Windows
>
	SolARPipelineTest_Mapping_Multi_NoDrop.exe

> #### Linux
>
	./run.sh ./SolARPipelineTest_Mapping_Multi_NoDrop

Add `SolARPipelineTest_Mapping_Multi_NoDrop_Processing_Cuda_conf.xml` at the end of the command to run the pipeline test with Cuda optimization (required CUDA to be installed on your computer).

We recommend first to run the `Sample Multi` which by default will load the `loop_desktop_A` AR device capture and which will build the `mapA`. To go deeper with this sample, you can change some properties available in the `SolARSample_Mapping_Multi_conf.xml` file and test on your own data:
* component `SolARDeviceDataLoader`, properties `calibrationFile` and `pathToData`: these two properties defined the path to the calibration file of the camera of your AR device and to the captured sequence.
* component `SolARMarker2DSquaredBinaryOpencv`, property `filePath`: the path to your fiducial marker if you want to initialize the mapping with it.
* component `SolARMapper`, property `directory`: here, you can set the folder path of your map used to load an existing map (in case of initial relocalization based on an existing map), and to save the resulting map.
* component `SolARSLAMMBootstrapper`, property `hasPose`: If `1`, the mapping sample wil use whether a fiducial marker or the map if available for initializing the mapping. If `0`, the initialization will require a translation movement of the camera to create the initial map (which reference coordinate system will be set to the initial pose of the camera).

You can also try to build the `mapB` based on the AR device capture `loop_desktop_B` and `fiducialMarkerB` by changing these properties.

### Map Visualizer sample

This sample allows to visualize any 3D map previously built by the mapping samples. You can set the path to your map by changing into the `SolARSample_Mapping_MapVisualizer_conf.xml` file the property `directory` of the `SolARMapper` component.

> #### Windows
>
	SolARSample_Mapping_MapVisualizer.exe

> #### Linux
>
	./run.sh ./SolARSample_Mapping_MapVisualizer

### Map Extension sample

This sample allows to extend a map with a new capture. It will load a 3D map previously built as well as a new AR device capture. It will first try to relocalize the camera of the new capture in relation to the 3D map, and then will extend the 3D map by triangulating new 3D points of the real environment. When the mapping is completed, the extended map is saved when exiting the application by pressing escape key. By default, the initial 3D map is `mapA` and the AR device capture is `loop_desktop_B`.

> #### Windows
>
	SolARSample_Mapping_MapExtension.exe

> #### Linux
>
	./run.sh ./SolARSample_Mapping_MapExtension
