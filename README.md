# turtlebot3_camera

##  Notes: We will use cv_camera ros package
1.  Installing cv_camera ros package:
    1.  git clone https://github.com/OTL/cv_camera.git
2.  (OPTIONAL) -- if error appear:
    1.  git clone https://github.com/ros/roslint.git -- some package require for cv_camera
#### or
    2.  rosdep install --from-paths src --ignore-src -r -y (the best; will install any require dependencies of all packages)

## launch:
### BringUp Camera
#### Both launch file xxx_robot.launch and xxx_remote.launch act like bringup in original TurtleBot3 wiki.
1.  Using Raspberry Pi Camera on RPi:
    1.  roscore (Remote PC)
    2.  cameraPi_robot.launch
        1.  launch on TurtleBot3
        2.  publishing:
            1.  /camPi/camera_info
            2.  /camPi/image_raw
            3.  /camPi/image_raw/compressed
            4.  /camPi/image_raw/compressed/parameter_descriptions
            5.  /camPi/image_raw/compressed/parameter_updates
            6.  /camPi/image_raw/compressedDepth
            7.  /camPi/image_raw/compressedDepth/parameter_descriptions
            8.  /camPi/image_raw/compressedDepth/parameter_updates
            9.  /camPi/image_raw/theora
            10. /camPi/image_raw/theora/parameter_descriptions
            11. /camPi/image_raw/theora/parameter_updates

    3.  cameraPi_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /pi_opencv_img
        3.  subscribing:
            1.  /camPi/camera_info
            2.  /camPi/image_raw

2.  Using USB Camera on RPi:
    1.  roscore (Remote PC)
    2.  cameraPi_USB_robot.launch
        1.  launch on TurtleBot3
        2.  publishing:
            1.  /camPi_USB/camera_info
            2.  /camPi_USB/image_raw
            3.  /camPi_USB/image_raw/compressed
            4.  /camPi_USB/image_raw/compressed/parameter_descriptions
            5.  /camPi_USB/image_raw/compressed/parameter_updates
            6.  /camPi_USB/image_raw/compressedDepth
            7.  /camPi_USB/image_raw/compressedDepth/parameter_descriptions
            8.  /camPi_USB/image_raw/compressedDepth/parameter_updates
            9.  /camPi_USB/image_raw/theora
            10. /camPi_USB/image_raw/theora/parameter_descriptions
            11. /camPi_USB/image_raw/theora/parameter_updates

    3.  cameraPi_USB_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /pi_usb_opencv_img
        3.  subscribing:
            1.  /camPi_USB/camera_info
            2.  /camPi_USB/image_raw

3.  Using USB Camera (PC use only)
    1.  roscore (Remote PC)
    2.  cameraUSB_robot.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /camUSB/camera_info
            2.  /camUSB/image_raw
            3.  /camUSB/image_raw/compressed
            4.  /camUSB/image_raw/compressed/parameter_descriptions
            5.  /camUSB/image_raw/compressed/parameter_updates
            6.  /camUSB/image_raw/compressedDepth
            7.  /camUSB/image_raw/compressedDepth/parameter_descriptions
            8.  /camUSB/image_raw/compressedDepth/parameter_updates
            9.  /camUSB/image_raw/theora
            10. /camUSB/image_raw/theora/parameter_descriptions
            11. /camUSB/image_raw/theora/parameter_updates

    3.  cameraUSB_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /usb_opencv_img
        3.  subscribing:
            1.  /camUSB/camera_info
            2.  /camUSB/image_raw

# Important:
### All launch file with xxx_remote.launch will run Image converter node ("image_converter_pi_node.py", or "image_converter_pi_usb_node.py" or "image_converter_usb_node.py") which require args -- args="38 80 10" ---> gamma, alpha, beta. The value depending on the camera-calibration shapen the image quality --- rosrun turtlebot3_camera sharpenCalibrate_xxx_node.py

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraPi_robot.launch (TurtleBot3)
3.  roslaunch turtlebot3_camera cameraPi_remote.launch (Remote PC)
4.  rosrun turtlebot3_camera sharpenCalibrate_pi_node.py (Remote PC)

#### or

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraPi_USB_robot.launch (TurtleBot3)
3.  roslaunch turtlebot3_camera rangeDetectorPi_USB.launch  (Remote PC)
4.  rosrun turtlebot3_camera sharpenCalibrate_pi_usb_node.py (Remote PC)

#### or

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraUSB_robot.launch (Remote PC)
3.  roslaunch turtlebot3_camera rangeDetectorUSB.launch  (Remote PC)
4.  rosrun turtlebot3_camera sharpenCalibrate_usb_node.py (Remote PC)

###  Range Detector
####  Use to find the color range (upper/lower) -- require an args -- args="HSV 38 80 10" where HSV is colorspace, and 38, 80, and 18 (gamma, alpha, beta) from image sharpenCalibrate_xxx_node.

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraPi_robot.launch (TurtleBot3)
3.  roslaunch turtlebot3_camera rangeDetectorPi.launch  (Remote PC)

#### or

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraPi_USB_robot.launch (TurtleBot3)
3.  roslaunch turtlebot3_camera rangeDetectorPi_USB.launch  (Remote PC)

#### or

1.  roscore (Remote PC)
2.  roslaunch turtlebot3_camera cameraUSB_robot.launch (Remote PC)
3.  roslaunch turtlebot3_camera rangeDetectorUSB.launch  (Remote PC)
