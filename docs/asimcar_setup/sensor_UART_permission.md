# Rule Creation
Create a file /etc/udev/rules.d/99-uart-serial.rules with the following line (template)
```
SUBSYSTEM=="ttyTHS#", MODE="0666", SYMLINK+="your_device_name"
```
Here "0666" is granting the port read and write permissions.

# Load Rule
```
sudo udevadm trigger
```

# Verify Rule
```
ls -l /dev/your_device_name
```

# Add User to Dialout
```
sudo usermod -aG dialout {user_name}
```

