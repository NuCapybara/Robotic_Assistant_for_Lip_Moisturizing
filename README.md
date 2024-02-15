# Winter Project-Caring moisturizing robot


## REAL SENSE

The project is depended on realsense camera, to start, you could run the following command

run first line in one terminal
```
realsense-viewer
```

run second line in the seperate terminal
```
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```

```
ros2 launch ros_april_tag detect_tag.launch.xml
```

```
python video_facial_landmarks.py \--shape-predictor shape_predictor_68_face_landmarks.dat
```

```
ros2 launch care care_facedetec.launch.xml

```

```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=fake 

```

## Reference

```https://github.com/ostadabbas/3d-facial-landmark-detection-and-tracking/blob/master/face_landmark_detection.py```