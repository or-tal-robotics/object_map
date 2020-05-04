# object_map
By [Or Tslil](https://github.com/ortslil64), [Amit Elbaz](https://github.com/elbazam)

ROS implementation of online semantic SLAM, based on the not yet published paper - "Representing and updating object identities in semantic SLAM". The object detection node is based on SSD300 architecture and forked from https://github.com/balancap/SSD-Tensorflow.
## Example - Gazebo simulation
[![Watch the video](https://img.youtube.com/vi/-H25q_Vcol8/default.jpg)](https://youtu.be/-H25q_Vcol8)
## Example - Experiment
[![Watch the video](https://img.youtube.com/vi/mQHh478gTg8/default.jpg)](https://youtu.be/mQHh478gTg8)
## Dependencies
The following python packges are required:
* python 2.*
* numpy
* sklearn
* sciPy
* openCV
* TensorFlow 1.1* (GPU version)
* hector_mapping (http://wiki.ros.org/hector_mapping)

## Runing
For a demo simulation use:


```
roslaunch gazebo_demo demo.launch
```

For a demo simulation working with the "Bhattacharyya coefficient" method of updating the map use:


```
roslaunch gazebo_demo Test.launch
```
