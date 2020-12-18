# Sample-Mapping
[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

The samples demonstrates mapping features such as mapping, map extension, map fusion, map visualization.

The samples for mapping are open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

## Before running the samples

Some sample require external data which must be downloaded before executing them.

### Bag Of Word Vocabulary

This vocabulary is required for keyframe retrieval. Download the vocabularies required for the bag of words available on the [GitHub](https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip), and extract the `akaze.fbow` file and copy it in the `./data` folder.

### Video captures

The mapping samples create maps from video captures. You can use two video captures available on the solar artifactory:

* <strong>Loop_Desktop_A</strong>: A video sequence captured with a Hololens 1 around a desktop starting and finishing with the fiducial Marker A with a loop trajectory. A fiducial marker B is capture during the trajectory.

* <strong>Loop_Dekstop_B</strong>: A video sequence captured with a Hololens 1 within an open-space starting and finishing with the fiducial Marker B with a loop trajectory. A part of the trajectory is common to the Loop_Desktop_A trajectory to test map overlap detection and map fusion.

Download the video sequences [loopDesktopA.zip](https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip) and [loopDesktopB.zip](https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopB.zip), and extract the `./loop_desktop_A` and `./loop_desktop_B` folders into the `./data` folder.

### Maps

The map visualization, and fusion samples use one or two maps. You can create them with the mapping samples, or directly download them from the artifactory:
* [loopDesktopA.zip](https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopA.zip)
* [loopDesktopB.zip](https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopB.zip)  

Extract the `./mapA` and `./mapB` folders and respective `fiducialMarkerA.yml` and `fiducialMarkerB.yml` files into the `./data` folder.

### Required modules

Some samples require sevral SolAR modules such as OpenGL, OpenCV, FBOW and G20. If they are not yet installed on your machine, please run the following command from the test folder:

<pre><code>remaken install packagedependencies.txt</code></pre>

and for debug mode:

<pre><code>remaken install packagedependencies.txt -c debug</code></pre>

For more information about how to install remaken on your machine, visit the [install page](https://solarframework.github.io/install/) on the SolAR website.

## The samples

###Sample Mapping
