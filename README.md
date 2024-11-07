# USB Camera Node for ROS

This project provides a ROS node to capture images from a USB camera and publish them to a ROS topic.

## Installation

1. **Download the Package**  
   Clone the repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/adnroboticsfr/USB-Camera-Node-ROS.git
   ```

2. **Build the Package**  
   From the root of your workspace, compile the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## Usage

1. **Launch the Node**  
   Start the USB Camera node using the launch file:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch usb_camera_node usb_cam.launch
   ```

## Verification

1. **Check Topics**  
   In another terminal, verify if the `usb_cam/image_raw` topic is publishing messages:
   ```bash
   rostopic list
   ```
   To view the published images, you can use `rqt_image_view`:
   ```bash
   rosrun rqt_image_view rqt_image_view
   ```

2. **Monitor Connection Status**  
   You can also check the connection status by echoing the `usb_cam/connected` topic:
   ```bash
   rostopic echo usb_cam/connected
   ```

## Configuration

If you wish to customize camera parameters (such as camera index, frame rate, resolution, etc.), you can modify the `usb_cam.launch` file:

```xml
<launch>
    <node name="usb_cam_node" pkg="usb_cam" type="usb_cam_node" output="screen">
        <param name="camera_index" value="0" />
        <param name="rate_hz" value="10" />
        <param name="flip_image" value="true" />
        <param name="width" value="640" />
        <param name="height" value="480" />
    </node>
</launch>
```

## Support

For questions or issues, please refer to the ROS documentation or create an issue on the GitHub repository.

---

With this guide, you are ready to use the USB Camera node in ROS to capture and view images. Happy experimenting with ROS!
```

