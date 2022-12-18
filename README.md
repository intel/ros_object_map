DISCONTINUATION OF PROJECT.

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project. 

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project. 
# ros_object_map

## 1 Introduction
ros_object_map is ROS package which designes to mark tag of objects on map when SLAM. It uses [ros_object_analytics](https://github.com/intel/ros_object_analytics) for object detection.

## 2 Prerequisite
  * [ros_object_analytics](https://github.com/intel/ros_object_analytics) installed

## 3 Build Dependencies
  * [object_analytics_msgs](https://github.com/intel/ros_object_analytics)
  * [object_msgs](https://github.com/intel/object_msgs)

## 4 Building
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/intel/ros_object_analytics.git
  git clone https://github.com/intel/object_msgs.git
  git clone https://github.com/intel/object_map.git
  cd ~/catkin_ws
  catkin_make install
  source install/setup.bash
  ```

## 5 Running the demo
### Step1: Launch realsense
```
- roslaunch realsense_ros_camera rs_camera.launch enable_pointcloud:=true enable_sync:=true enable_infra1:=false enable_infra2:=false
```

### Step2: Launch object_analytics
```
# launch with Movidius NCS as detection backend
- roslaunch object_analytics_launch analytics_movidius_ncs.launch 
# or launch with OpenCL caffe as detection backend
- roslaunch object_analytics_launch analytics_opencl_caffe.launch
```

### Step3: Launch cartographer and Rviz
```
- roslaunch object_map turtlebot_urg_lidar_2d.launch
- roslaunch object_analytics_visualization rviz.launch
```

### Step4: Launch Object Map
```
- roslaunch object_map object_map.launch
```

## 6 Interface
### 6.1 Topic
  * ```/object_map/Markers``` : Publish MarkerArray on Rviz
  * ```/object_map/map_save``` : Subscribe map_save topic to save object maps
  * ```/object_analytics/detection```: Subscribe ObjectsInBoxes from object_analytics
  * ```/object_analytics/tracking```: Subscribe TrackedObjects from object_analytics
  * ```/object_analytics/localization```: Subscribe ObjectsInBoxes3D from object_analytics
### 6.2 Save object map
  ```
  - rostopic pub --once /object_map/map_save std_msgs/Int32  1
  ```


###### *Any security issue should be reported using process at https://01.org/security*
