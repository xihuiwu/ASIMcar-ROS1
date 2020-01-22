Download the opencv from [here](https://github.com/opencv/opencv) and opencv_contrib from [here](https://github.com/opencv/opencv_contrib). Then merge the opencv_contrib folder into the opencv folder.

Go to the opencv directory
```
$ cd opencv-[version]
$ mkdir build
$ cd build
```


# Build Command
With libjpeg-turbo
```
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D WITH_CUDA=ON \
        -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON \
        -D WITH_LIBV4L=ON -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
        -D WITH_QT=ON -D WITH_OPENGL=ON -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
        -D WITH_JPEG=ON -D BUILD_JPEG=ON ..
$ make -j4
$ sudo make install
```
