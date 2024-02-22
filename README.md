# Winter Project-Caring moisturizing robot


## REAL SENSE

The project is depended on realsense camera, to start, you could run the following command

run first line in one terminal


```
First step:

```
ros2 launch hello_moveit moveit.launch.xml hardware_type:=actual
```
Second: 

```
 ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true
```

Third step acticvate the face detection node and publish the transform from camera to robot

```
 ros2 launch care care_facedetec.launch.xml

```

Fourth step runs the moveit group command:
```
ros2 launch hello_moveit hello_moveit.launch.xml
```


## Reference

```https://github.com/ostadabbas/3d-facial-landmark-detection-and-tracking/blob/master/face_landmark_detection.py```