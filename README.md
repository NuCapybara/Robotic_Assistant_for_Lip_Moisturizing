# Robotic Assistant for Lip Moisturizing

This 10-week project aims to develop a small, affordable robot that can recognize lip movements and accurately apply a cotton swab to the lips. It’s designed to help patients who are unconscious and suffering from dry lips and mouth due to their inability to drink water on their own. Typically, caregivers moisten the patient’s lips with a moist cotton swab to hydrate and clean the area, which is a repetitive task that requires constant attention. By automating this process, the robot can alleviate the burden on healthcare workers and caregivers, freeing them up to focus on other important duties.

# Flow Chart on Nodes Communication

![winter_flow_chart](https://github.com/NuCapybara/Winter_Project/assets/144244355/cd4c8be8-1732-4c18-acc5-e2cae292a040)

## Launching Nodes

The project is depended on realsense camera, to start, you should be connected to realsense D435i camera and WX200 Robot. You could run the following launchfiles in each terminal window.

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

Fourth step runs the moveit group command and control the robot.
```
ros2 launch hello_moveit hello_moveit.launch.xml
```


## Reference

```https://github.com/ostadabbas/3d-facial-landmark-detection-and-tracking/blob/master/face_landmark_detection.py```


