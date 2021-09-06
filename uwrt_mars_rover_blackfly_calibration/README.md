# Brief
This package can be used for calibrating the blackfly cameras. The calibration process consists of 2 nodes. The first node (broadcaster) broadcasts images taken from the blackfly to some topic. The 2nd node (cameracalibrator) uses these images to compute camera intrinsics.

# Pre-req
Make sure the [Spinnaker SDK](https://www.flir.ca/products/spinnaker-sdk/) is installed. As a sanity test, run the SpinView application.

Become familiar with the [camera calibration package](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

# Usage
> ros2 launch uwrt_mars_rover_blackfly_calibration blackfly_calibration.launch.xml

Also, run the cameracalibrator node as specified in the camera calibration tutorial linked above.

# Debugging
Make sure camera is plugged in, SDK is installed, and that the broadcaster node has the required params specified.