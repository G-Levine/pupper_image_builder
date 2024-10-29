# pupper_image_builder

## Description
An automated OS image builder for the Pupper robot. Pulls the latest Ubuntu image, applies all the necessary configurations, and installs ROS2 and Pupper-specific packages.

## How to use
### Prerequisites
1. Install Docker Desktop or some Docker engine
2. Make sure your Docker is up-to-date. After upgrading MacOS, older versions of Docker will not work.


### Build Ubuntu server image:
```
./make_server_image.sh
```
Image saved as `pupOS_ubuntu_server.img`.

### Build full Ubuntu desktop image 
```
./make_desktop_image.sh
```
Image saved as `pupOS_ubuntu_desktop_full.img`. The script will also build the desktop base image (details see below) and save it as `pupOS_ubuntu_desktop_ros_base.img` during the build process if it doesn't already exist in this directory.

### Build desktop base image

To just build the base for the desktop image which includes Ubuntu desktop, ros-jazzy-desktop, and low-latency kernel:
```
./make_base_image.sh
```
Image saved as `pupOS_ubuntu_desktop_ros_base.img`

The build process takes about 5-30 minutes on an M1 MacBook Pro depending on which image you build.

### Default credentials
Hostname: `pupper`

User: `pi`

Password: `rhea123`

### First boot
1. Disable a NetworkManager service to decrease boot time from ~90s to ~30s:

`sudo systemctl disable NetworkManager-wait-online.service`

2. Pair PS4/5 controller over bluetooth.
3. Optionally connect to WIFI
4. Optionally enable Pi to automatically log in to the `pi` user on boot by writing the following to `/etc/gdm3/custom.conf`:
```
[daemon]
# Enabling automatic login
AutomaticLoginEnable=true
AutomaticLogin=pi
```
(TODO: add this to some startup script)

## Operating systems
* Ubuntu - see above.
* PiOS - checkout `pios` branch. 

## Troubleshooting
###  Checksum error with the ubuntu desktop image
* Most likely the image was updated but packer has cached the checksum and has not realized it should get the new checksum. 
* Delete .packer_plugins to make packer download the checksum again.

### Out of memory / build taking a long time
* Increase RAM memory limit in the Docker Desktop application (go to settings (gear icon) -> resources). Primarily needed to building ROS on PiOS.

## Issues
* `NetworkManager-wait-online.service` adds 1 min to startup time
    * Have to run `sudo systemctl disable NetworkManager-wait-online.service` after booting actual RPi. Could not disable it for some reason in the provisioning scripts.
* Sometimes get errors like `arm.ubuntu:         <urlopen error <urlopen error [Errno -3] Temporary failure in name resolution> (https://raw.githubusercontent.com/ros/rosdistro/master/humble/distribution.yaml)>` during image build where the container can't properly get web resources. 
* Using the pre-built Ubuntu Desktop image does not work. Any window takes a very long time to open. So instead we install ubuntu-desktop on top of the pre-built Ubuntu Server image.
* Sometimes user-data is not correctly installed. You'll know if the Pi boots and you don't see the pi user to log in to. PiOS uses a system daemon to run a script on first boot. Maybe cloud-init isn't working. Can try to put setup script in cloud-init per-boot folder or once folder.