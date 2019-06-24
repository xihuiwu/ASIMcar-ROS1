1. Rule Creation
Create a file /etc/udev/rules.d/99-usb-serial.rules with the following line (template)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", SYMLINK+="your_device_name" 

2. Load Rule
sudo udevadm trigger

3. Verify Rule
ls -l /dev/your_device_name

