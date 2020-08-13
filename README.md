# object_map
By [Or Tslil](https://github.com/ortslil64), [Amit Elbaz](https://github.com/elbazam)

[Paper link](https://www.researchgate.net/publication/342735084_Representing_and_updating_objects'_identities_in_semantic_SLAM)

ROS implementation of online semantic SLAM, based on the not yet published paper - "Representing and updating object identities in semantic SLAM". The object detection node is based on SSD300 architecture and forked from https://github.com/balancap/SSD-Tensorflow.
## Example - Gazebo simulation
[![Watch the video](https://img.youtube.com/vi/-H25q_Vcol8/default.jpg)](https://youtu.be/-H25q_Vcol8)
## Example - Experiment
[![Watch the video](https://img.youtube.com/vi/mQHh478gTg8/default.jpg)](https://youtu.be/mQHh478gTg8)
## Pipelines
![demo](https://github.com/or-tal-robotics/object_map/blob/master/images/pipelines.png?raw=true "General piplines"){:height="50%" width="50%"}
## Dependencies
The following python packges are required:
* python 2.*
* numpy
* sklearn
* sciPy
* openCV
* TensorFlow 1.1* (GPU version)
* hector_mapping (http://wiki.ros.org/hector_mapping)
* currently tested in ros melodic in ubuntu 18.04
## Setup
1. Download repository to your catkin workspace:
```bash
git clone https://github.com/or-tal-robotics/object_map.git
```
2. Build:
```bash
catkin_make
```
3. Install SSD image detector for ROS:
```bash
pip install -e object_detector_ssd_tf_ros
```
4. Unzip SSD weights in `object_map/object_detector_ssd_tf_ros/ssd/model/ssd_300_vgg.ckpt.zip`

## Runing
* For a demo simulation use:
```bash
roslaunch gazebo_demo demo.launch
```
* For a demo simulation working with the "Bhattacharyya coefficient" method of updating the map use:
```bash
roslaunch gazebo_demo Test.launch
```
