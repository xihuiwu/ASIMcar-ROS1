# ASIMcar
This is the repository for autonomous RC car of ASIM Lab. The project is entirely based on ROS.

* Information about apply realtime patch to the kernel and build can be found on [kernel_rt_patch](https://github.com/xihuiwu/ASIMcar/blob/master/docs/nvidia/kernel_rt_patch.md).

* To enable extra 2 CPUs, read through [cpu_enable](https://github.com/xihuiwu/ASIMcar/blob/master/docs/asimcar_setup/cpu_enable.md).

* To install OpenCV 3.4.5, refer to this [repository](https://github.com/xihuiwu/buildOpenCVTX2).
After OpenCV installation, delete ROS cv2.so which is located
```
/opt/ros/{ROS_version}/lib/python2.7/dist-packages
```
 to avoid version conflict.
