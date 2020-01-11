# ASIMcar
This is the repository for autonomous RC car of ASIM Lab. The project is entirely based on ROS.

* Information about apply realtime patch to the kernel and build can be found on [kernel_rt_patch](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/kernel_rt_patch.md).

* To enable extra 2 CPUs, read through [cpu_setup](https://github.com/xihuiwu/ASIMcar/blob/master/docs/asimcar_setup/cpu_setup.md).

* To install OpenCV, refer to this [tutorial](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/opencv_installation.md).
After OpenCV installation, delete ROS cv2.so which is located
```
/opt/ros/{ROS_version}/lib/python2.7/dist-packages
```
  
* When building the workspace, using the following build command to avoid opencv library conflict.
```
catkin build -DOpenCV_DIR=/usr/local/share/OpenCV
```
