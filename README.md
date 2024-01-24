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
