1. Create new user group called i2c:
```
sudo groupadd i2c
```

2. Change the group ownership of /dev/i2c-1 to i2c:
```
sudo chown :i2c /dev/i2c-1
```

3. Change the file permissions of the device /dev/i2c-1 so users of the i2c group can read and write to the device:
```
sudo chmod g+rw /dev/i2c-1
```

4. Add your user to the group i2c:
```
sudo usermod -aG i2c {username}
```
