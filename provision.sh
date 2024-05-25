#!/bin/bash -e

# Setup for Raspberry Pi 5
echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' >> /boot/firmware/config.txt
echo 'usb_max_current_enable=1' >> /boot/firmware/config.txt
sed -i '1s/^/video=HDMI-A-1:720x720M@60D,rotate=270 /' /boot/firmware/cmdline.txt

# Set up avahi-daemon
sudo apt install -y avahi-daemon

# Install low-latency kernel
sudo wget https://github.com/raspberrypi/firmware/raw/master/boot/bcm2712-rpi-5-b.dtb -P /etc/flash-kernel/dtbs/
sudo apt update && sudo apt install -y linux-lowlatency

# Adafruit GPIO setup
sudo apt install -y python-is-python3 python3-pip i2c-tools libgpiod-dev python3-libgpiod
sudo rm /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install Adafruit-Blinka RPi.GPIO

# Bluetooth
sudo apt install -y bluez

# Audio
sudo apt install -y portaudio19-dev python3-pyaudio alsa-utils
pip install --upgrade pyaudio deepgram-sdk

# DepthAI
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
echo 'export OPENBLAS_CORETYPE=ARMV8' >> ~/.bashrc && source ~/.bashrc
pip install blobconverter

# Install ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
sudo apt update && sudo apt install -y ros-dev-tools
sudo apt install -y ros-jazzy-ros-base
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/G-Levine/control_board_hardware_interface.git
git clone https://github.com/G-Levine/neural_controller.git --recurse-submodules 
git clone https://github.com/G-Levine/pupper_v3_description.git
git clone https://github.com/Nate711/pupper_feelings.git

# Install dependencies
cd ~/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
rosdep install --from-paths src -y --ignore-src

# Install additional ROS2 packages
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge

# Build ROS2 workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/ros2_ws/install/setup.bash

# Install utils
cd ~
git clone https://github.com/Nate711/utils.git -b launch_neural_controller
bash ~/utils/install_battery_monitor.sh
bash ~/utils/install_robot_auto_start_service.sh
