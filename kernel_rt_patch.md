Pre-requisite
sudo apt install qt5-default -y

1. Create environment
touch $HOME/nvidia/environment
#!/bin/sh
# kernel flags
export TEGRA_KERNEL_OUT=$HOME/nvidia/tegra-jetson-tx2-kernel-rt

2. Compile
$ source environment
$ mkdir -p $TEGRA_KERNEL_OUT
$ cd $HOME/nvidia/kernel/kernel-4.9
$ zcat /proc/config.gz > .config
# list and apply real-time patches
$ cd scripts
$ ./rt-patch apply-patches
# create default config
$ cd ..
$ make prepare
$ make modules_prepare
$ make -j4 Image
$ make -j4 modules

3. Copy Image
$ sudo cp arch/arm64/boot/Image /boot/Image
