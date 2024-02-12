# To run, enter the following command to give permission to file
# chmod +x setup.sh
# following instructions from https://github.com/NVIDIA/jetson-gpio
# and https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
# run these commands to add user (jetson) to correct groups to access gpio
sudo groupadd -f -r gpio
sudo usermod -a -G gpio jetson


# need to run these commands to ensure that python can connect to serial port without interference
# https://jetsonhacks.com/2019/10/10/jetson-nano-uart/
systemctl stop nvgetty
systemctl disable nvgetty
udevadm trigger
# reboot


