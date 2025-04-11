sudo cp esp32.rules rplidar.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger
