# ASIMcar-ROS1
This is the repository for autonomous RC car of ASIM Lab. The project is entirely based on ROS. The other packages run on ROS2 and can be located [here](https://github.com/xihuiwu/ASIMcar-ROS2).

* Information about apply realtime patch to the kernel and build can be found on [kernel_rt_patch](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/kernel_rt_patch.md).

* To change CPU configuration for better computation performance, read through the [cpu_setup](https://github.com/xihuiwu/ASIMcar-ROS1/blob/master/docs/JetsonTX2_setup/cpu_setup.md).

* To install OpenCV with CUDA support, refer to this [install_tutorial](https://github.com/xihuiwu/ASIMcar/blob/master/docs/JetsonTX2_setup/opencv_installation.md).
  
* When building the workspace, using the following build command to avoid opencv library conflict.
```
catkin build -DOpenCV_DIR=/usr/local/share/OpenCV
```
This issue would be avoided by using newer version of ROS (>= Melodic)

# Extra Dependencies
* [PyGame](https://www.pygame.org/wiki/CompileUbuntu?parent=)

# Citation
If you think this platform is helpful to your research, please cite the paper. This is the [paper link](https://asmedigitalcollection.asme.org/DSCC/proceedings/DSCC2019/59148/Park%20City,%20Utah,%20USA/1070503). Here is a BibTex entry:
```
@inproceedings{ASIMcar,
author={Wu, Xihui and Eskandarian, Azim},
title={An Improved Small-Scale Connected Autonomous Vehicle Platform},
year={2019},
month={Nov},
journal={Dynamic Systems and Control Conference | ASME Digital Collection},
publisher={American Society of Mechanical Engineers Digital Collection},
url={https://asmedigitalcollection.asme.org/DSCC/proceedings/DSCC2019/59148/Park City, Utah, USA/1070503}
}
```
