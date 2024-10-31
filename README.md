# USB Camera Node for ROS

This project provides a ROS node that captures images from a USB camera and publishes them to a ROS topic.

## Usage Steps

1. **Set Up Your ROS Environment**:
   - Ensure you have a working ROS environment (like ROS Noetic) and all necessary packages installed (such as `cv_bridge`, `sensor_msgs`, etc.).

2. **Create a ROS Package**:
   - If not already done, create a new ROS package in your workspace:
     ```bash
     cd ~/catkin_ws/src
     catkin_create_pkg usb_cam_node std_msgs sensor_msgs cv_bridge rospy
     ```
   - This will create a new directory `usb_cam_node` with the necessary dependencies.

3. **Add the Script to the Package**:
   - Create a `scripts` directory in the newly created package:
     ```bash
     cd usb_cam_node
     mkdir scripts
     ```
   - Create a Python file named `usb_cam_node.py` in the `scripts` directory and paste the provided code.

4. **Make the Script Executable**:
   - Grant execution permissions to the script:
     ```bash
     chmod +x scripts/usb_cam_node.py
     ```

5. **Configure the Launch File (Optional)**:
   - If you want to launch the node with specific parameters, create a launch file. Create a file `usb_cam.launch` in the `launch` directory:
     ```bash
     mkdir launch
     touch launch/usb_cam.launch
     ```
   - Edit the `usb_cam.launch` file to include the following content:
     ```xml
     <launch>
         <node name="usb_cam_node" pkg="usb_camera_node" type="usb_cam_node.py" output="screen">
             <param name="camera_index" value="0" />
             <param name="rate_hz" value="10" />
             <param name="flip_image" value="true" />
             <param name="width" value="640" />
             <param name="height" value="480" />
         </node>
     </launch>
     ```

6. **Build the Package**:
   - Go back to the root of your workspace and build the package:
     ```bash
     cd ~/catkin_ws
     catkin_make
     ```

7. **Launch ROS**:
   - Start the ROS master:
     ```bash
     roscore
     ```

8. **Run Your Node**:
   - Open a new terminal, source your workspace, and launch the node:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     roslaunch usb_camera_node usb_cam.launch
     ```

## Verification

- **Check Topics**:
  - In another terminal, use the following command to see if the `usb_cam/image_raw` topic is publishing messages:
    ```bash
    rostopic list
    ```
  - To view the published images, you can use `rqt_image_view`:
    ```bash
    rosrun rqt_image_view rqt_image_view
    ```

- **Monitor Connection Status**:
  - You can also check the connection status by echoing the topic `usb_cam/connected`:
    ```bash
    rostopic echo usb_cam/connected
    ```

