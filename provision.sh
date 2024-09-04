#!/bin/bash -e

set -x
export DEBIAN_FRONTEND=noninteractive

cat /etc/hostname
cat /etc/hosts
hostname

DEFAULT_USER=pi
mkdir -p /home/$DEFAULT_USER

# PiOS debian ros2
sudo apt update
wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo apt install ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo pip install vcstool colcon-common-extensions
echo 'source /opt/ros/jazzy/setup.bash' >> /home/$DEFAULT_USER/.bashrc
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt upgrade -y

# Setup for Raspberry Pi 5
echo 'dtparam=i2c_arm=on,i2c_arm_baudrate=400000' >> /boot/firmware/config.txt
echo 'usb_max_current_enable=1' >> /boot/firmware/config.txt
echo 'dtoverlay=waveshare-4dpic-3b' >> /boot/firmware/config.txt
echo 'dtoverlay=waveshare-4dpic-4b' >> /boot/firmware/config.txt
echo 'dtoverlay=waveshare-4dpic-5b' >> /boot/firmware/config.txt
echo 'hdmi_force_hotplug=1' >> /boot/firmware/config.txt
echo 'config_hdmi_boost=10' >> /boot/firmware/config.txt
echo 'hdmi_group=2' >> /boot/firmware/config.txt
echo 'hdmi_mode=87' >> /boot/firmware/config.txt
echo 'hdmi_timings=720 0 100 20 100 720 0 20 8 20 0 0 0 60 0 48000000 6' >> /boot/firmware/config.txt
echo 'start_x=0' >> /boot/firmware/config.txt
echo 'gpu_mem=128' >> /boot/firmware/config.txt
echo 'dtoverlay=hifiberry-dac' >> /boot/firmware/config.txt
sed -i '1s/^/video=HDMI-A-1:720x720M@60D,rotate=270 /' /boot/firmware/cmdline.txt

# Download and extract the display overlays
wget 'https://files.waveshare.com/wiki/4inch%20HDMI%20LCD%20(C)/4HDMIB_DTBO.zip' -O 4HDMIB_DTBO.zip
sudo apt install -y unzip
unzip 4HDMIB_DTBO.zip
sudo cp 4HDMIB_DTBO/*.dtbo /boot/firmware/overlays/
rm -r 4HDMIB_DTBO 4HDMIB_DTBO.zip

# Set up avahi-daemon
sudo apt install -y avahi-daemon net-tools openssh-server curl

# Install low-latency kernel
# sudo wget https://github.com/raspberrypi/firmware/raw/master/boot/bcm2712-rpi-5-b.dtb -P /etc/flash-kernel/dtbs/
# sudo apt install -y linux-lowlatency

# # Adafruit GPIO setup
sudo apt install -y python-is-python3 python3-pip i2c-tools libgpiod-dev python3-libgpiod

# IRL pi said already newest
# sudo apt-get install -y gcc-aarch64-linux-gnu build-essential

sudo rm /usr/lib/python3.*/EXTERNALLY-MANAGED
pip install Adafruit-Blinka RPi.GPIO

# Bluetooth
sudo apt install -y bluez

# Audio
sudo apt install -y portaudio19-dev python3-pyaudio alsa-utils
pip install --upgrade pyaudio deepgram-sdk

# Install ROS2
sudo apt install -y software-properties-common
# sudo add-apt-repository universe
# sudo apt update 


# Create ROS2 workspace
mkdir -p /home/$DEFAULT_USER/ros2_ws/src
cd /home/$DEFAULT_USER/ros2_ws/src

# Get dependencies and pupper packages
# Array of repositories and branches
repos=(
    "https://github.com/pal-robotics/backward_ros.git"
    "https://github.com/PickNikRobotics/RSL.git"
    "https://github.com/PickNikRobotics/generate_parameter_library.git"
    "https://github.com/ros-controls/realtime_tools.git"
    "https://github.com/ros-controls/control_msgs.git"
    "https://github.com/ros/diagnostics.git"
    "https://github.com/ros2/teleop_twist_joy.git"
    "https://github.com/ros-drivers/joystick_drivers.git:ros2"
    "https://github.com/ros/xacro.git:ros2"
    "https://github.com/G-Levine/control_board_hardware_interface.git"
    "https://github.com/G-Levine/neural_controller.git"
    "https://github.com/G-Levine/pupper_v3_description.git"
    "https://github.com/Nate711/pupper_feelings.git"
)

# Clone each repository and checkout the specified branch if provided
for repo in "${repos[@]}"; do
    IFS=':' read -r url branch <<< "$repo"
    repo_name=$(basename "$url" .git)
    
    if [ -d "$repo_name" ]; then
        echo "Directory $repo_name already exists. Skipping clone."
    else
        git clone "$url" --recurse-submodules
        if [ -n "$branch" ]; then
            cd "$repo_name"
            git checkout "$branch"
            cd ..
        fi
    fi
done

# Install dependencies
cd /home/$DEFAULT_USER/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
rosdep install --from-paths src -y --ignore-src

# Install additional ROS2 packages
# sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-teleop-twist-joy ros-jazzy-foxglove-bridge ros-jazzy-xacro

# Build ROS2 workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo 'source ~/ros2_ws/install/setup.bash' >> /home/$DEFAULT_USER/.bashrc
source /home/$DEFAULT_USER/ros2_ws/install/setup.bash

# Install utils
cd /home/$DEFAULT_USER
git clone https://github.com/Nate711/utils.git -b launch_neural_controller
bash /home/$DEFAULT_USER/utils/install_battery_monitor.sh
bash /home/$DEFAULT_USER/utils/install_robot_auto_start_service.sh
