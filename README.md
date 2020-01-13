##### ASIMcar-ROS1
This is the repository for autonomous RC car of ASIM Lab. The project is entirely based on ROS. The other packages run on ROS2 and can be located [here](https://github.com/xihuiwu/ASIMcar-ROS2).

* Information about apply realtime patch to the kernel and build can be found on [kernel_rt_patch](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/kernel_rt_patch.md).

* To change CPU setup, read through the [cpu_setup](https://github.com/xihuiwu/ASIMcar/blob/master/docs/asimcar_setup/cpu_setup.md).

* To install OpenCV with CUDA support, refer to this [install_tutorial](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/opencv_installation.md).
After OpenCV installation, ROS cv2.so which is located
```
/opt/ros/{ROS_version}/lib/python2.7/dist-packages
```
could be deleted or renamed.
  
* When building the workspace, using the following build command to avoid opencv library conflict.
```
catkin build -DOpenCV_DIR=/usr/local/share/OpenCV
```

# To Do List
2020-1-13
- [ ] Collect lane pictures
- [ ] Add Laser SLAM package

2020-1-12
- [x] Reconfigure joy to low level control topic
- [x] Create ROS2 package
- [x] Change Dense layer to CNN layer in the ROS2 package.
