# Sample-Mapping
[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

The samples demonstrates mapping features such as mapping, map extension, map fusion, map visualization.

The samples for mapping are open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

## Before running the samples

Some sample require external data which must be downloaded before executing them.

### Bag Of Word Vocabulary

This vocabulary is required for keyframe retrieval. Download the vocabularies required for the bag of words available on the [GitHub](https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip), and extract the `akaze.fbow` file and copy it in the `./data/fbow_voc` folder.

### AR device captures

The mapping samples create maps from AR device captures containing both an image sequence and the corresponding poses. You can use two captures available on the solar artifactory:

* <strong>Loop_Desktop_A</strong>: A video sequence captured with a Hololens 1 around a desktop starting and finishing with the fiducial Marker A with a loop trajectory. A fiducial marker B is captured during the trajectory.

* <strong>Loop_Dekstop_B</strong>: A video sequence captured with a Hololens 1 within an open-space starting and finishing with the fiducial Marker B with a loop trajectory. A part of the trajectory is common to the `Loop_Desktop_A` trajectory to test map overlap detection and map fusion.

Download the video sequences [loopDesktopA.zip](https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip) and [loopDesktopB.zip](https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopB.zip), and extract the `./loop_desktop_A` and `./loop_desktop_B` folders into the `./data/data_hololens` folder.

Extract also the `fiducialMarkerA.yml`, `fiducialMarkerB.yml` and `hololens_calibration.yml` files into the `./data/data_hololens` folder.

### Maps

The map visualization, and fusion samples use one or two maps. You can create them with the mapping samples, or directly download them from the artifactory:
* [loopDesktopA.zip](https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopA.zip)
* [loopDesktopB.zip](https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopB.zip)  

Extract the `./mapA` and `./mapB` folders and respective `fiducialMarkerA.yml` and `fiducialMarkerB.yml` files into the `./data/map_hololens` folder.

### Required modules

Some samples require several SolAR modules such as OpenGL, OpenCV, FBOW and G20. If they are not yet installed on your machine, please run the following command from the test folder:

<pre><code>remaken install packagedependencies.txt</code></pre>

and for debug mode:

<pre><code>remaken install packagedependencies.txt -c debug</code></pre>

For more information about how to install remaken on your machine, visit the [install page](https://solarframework.github.io/install/) on the SolAR website.

## The samples

### Mapping samples

These samples construct a 3D map from a AR device capture producing an image sequence with corresponding poses based on an AR runtime (e.g. Hololens SDK, ARkit, or ARCore). The samples can detect loop and correct the drift due to vision tracking. When the user exits the sample, the map is saved into a dedicated folder.

Also, the samples offer different modes of initialization, whether based on a fiducial marker which will define the reference coordinate system of the map, an existing 3D map previoulsy built, or no initial knowledge. In this last case, an initial map will be built based on a translation movement of the camera, and the reference coordinate system of the map will be defined by the first pose of the camera.

Four samples are available for the mapping in the `Mapping` folder:
* <strong>Sample Mono</strong>: It is a mono-threaded implementation of a standalone application which will load the AR device capture and will build the 3D map (recommended to better understand the pipeline with the source code).
* <strong>Sample Multi</strong>: It is a multi-threaded implementation of a standalone application which will load the AR device capture and will build the 3D map (recommended for better performances).
* <strong>Pipeline Mono</strong>: It is a mono-threaded implementation of a mapping pipeline which will consume images and corresponding poses, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.
* <strong>Pipeline Multi</strong>: It is a multi-threaded implementation of a mapping pipeline which will consume images and corresponding poses, and will build the 3D map. A project to test the pipeline is also provided in the `tests` folder.

We recommend first to run the `Sample Multi` which by default will load the `loop_desktop_A` AR device capture and which will build the `mapA`. To go deeper with this sample, you can change some properties available in the `SolARSample_Mapping_Multi_conf.xml` file and test on your own data:
* component `SolARDeviceDataLoader`, properties `calibrationFile` and `pathToData`: these two properties defined the path to the calibration file of the camera of your AR device and to the captured sequence.
* component `SolARMarker2DSquaredBinaryOpencv`, property `filePath`: the path to your fiducial marker if you want to initialize the mapping with it.
* component `SolARMapper`, property `directory`: here, you can set the folder path of your map used to load an existing map (in case of initial relocalization based on an existing map), and to save the resulting map.
* component `SolARSLAMMBootstrapper`, property `hasPose`: If `1`, the mapping sample wil use whether a fiducial marker or the map if available for initializing the mapping. If `0`, the initialization will require a translation movement of the camera to create the initial map (which reference coordinate system will be set to the initial pose of the camera).

You can also try to build the `mapB` based on the AR device capture `loop_desktop_B` and `fiducialMarkerB` by changing these properties.

### Map Visualizer sample

This sample allows to visualize any 3D map previously built by the mapping samples. You can set the path to your map by changing into the `SolARSample_Mapping_MapVisualizer_conf.xml` file the property `directory` of the `SolARMapper` component.  

### Map Extension sample

This sample allows to extend a map with a new capture. It will load a 3D map previously built as well as a new AR device capture. It will first try to relocalize the camera of the new capture in relation to the 3D map, and then will extend the 3D map by triangulating new 3D points of the real environment. When the mapping is completed, the extended map is saved when exiting the application by pressing escape key. By default, the initial 3D map is `mapA` and the AR device capture is `loop_desktop_B`.  

### Map Fusion samples

The two samples allow to fuse two maps previoulsy built:
* <strong>Local Map Fusion</strong>: This fusion requires to specify the 3D transform between the two maps in the `TransformLocalToGlobal.txt` file. This 3D transform can be estimated with the OpenCV module test called `SolARTest_ModuleOpenCV_DeviceDualMarkerCalibration` which will estimate the transform between two markers, here the `FiducialMarkerA` reference of the `mapA`, and `FiducialMarkerB` reference of the `mapB` and also visible in the `loop_desktop_A` AR device capture.
* <strong>Floating Map Fusion</strong>: This fusion will automatically detect overlaps between two maps based on a keyframe retrieval approach. The overlaps detection will estimate the 3D transform between the two maps, and the the sample will merge them.
