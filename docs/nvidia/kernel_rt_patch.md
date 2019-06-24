# Pre-requisite
```
sudo apt install qt5-default -y
```

# Create environment
```
touch $HOME/nvidia/environment
```
```
gedit $HOME/nvidia/environment
#!/bin/sh
export TEGRA_KERNEL_OUT=$HOME/nvidia/tegra-jetson-tx2-kernel-rt
```

# Compile
```
source environment
mkdir -p $TEGRA_KERNEL_OUT
cd $HOME/nvidia/kernel/kernel-4.9
zcat /proc/config.gz > .config
cd scripts
./rt-patch apply-patches
cd ..
make prepare
make modules_prepare
make -j4 Image
make -j4 modules
```

# Copy Image
```
sudo cp arch/arm64/boot/Image /boot/Image
```
