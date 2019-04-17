# turtlebot3_camera

## launch:
1.  Using Raspberry Pi Camera on RPi:
    1.  cameraPi_robot.launch
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

    2.  cameraPi_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /pi_opencv_img
        3.  subscribing:
            1.  /camPi/camera_info
            2.  /camPi/image_raw

2.  Using USB Camera on RPi:
    1.  cameraPi_USB_robot.launch
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

    2.  cameraPi_USB_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /pi_usb_opencv_img
        3.  subscribing:
            1.  /camPi_USB/camera_info
            2.  /camPi_USB/image_raw

3.  Using USB Camera (PC use only)
    1.  cameraUSB_robot.launch
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

    2.  cameraUSB_remote.launch
        1.  launch on Remote PC
        2.  publishing:
            1.  /usb_opencv_img
        3.  subscribing:
            1.  /camUSB/camera_info
            2.  /camUSB/image_raw


# Important:
### All launch file with xxxxx_remote.launch will run Image converter node ("image_converter_pi_node.py", or "image_converter_pi_usb_node.py" or "image_converter_usb_node.py") which require args -- args="38 80 10" ---> gamma, alpha, beta.the value depending on the camera-calibration shapen the image quality --- rosrun turtlebot3_camera sharpenCalibrate_xxx_node.py
