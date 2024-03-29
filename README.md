# DEPRECATED - This repo is no longer maintained

----

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

* <strong>Pipeline Multi</strong>: It is a multi-threaded implementation of a mapping pipeline which will consume images and corresponding poses, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.

> #### Windows
>
	SolARPipelineTest_Mapping_Multi.exe

> #### Linux
>
	./run.sh ./SolARPipelineTest_Mapping_Multi

Add `SolARPipelineTest_Mapping_Multi_Processing_Cuda_conf.xml` at the end of the command to run the pipeline test with Cuda optimization (required CUDA to be installed on your computer).

We recommend first to run the `Pipeline test` which by default will load the `loop_desktop_A` AR device capture and which will build the `mapA`. To go deeper with this sample, you can change some properties available in the `SolARPipelineTest_Mapping_Multi_Processing_Cuda_conf.xml` file and test on your own data:
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

