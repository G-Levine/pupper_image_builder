# pupper_image_builder

## Description
An automated OS image builder for the Pupper robot. Pulls the latest Ubuntu image, applies all the necessary configurations, and installs ROS2 and Pupper-specific packages.

## How to use
Requires Docker to be installed and running.

```
docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init .
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build .
```

This takes about 10 minutes on an M1 MacBook Pro.

The output is written to `pupOS.img`.

### Default credentials
Hostname: `pupper`

User: `pi`

Password: `rhea123`

## Operating systems
* Ubuntu Server - checkout master branch. 
* Ubuntu Desktop - checkout gui branch. Issue: GUI is super slow
* PiOS - checkout pios branch. 

## Known issues
* With Ubuntu desktop, opening an application (e.g. terminal) takes forever. And need to switch to x11 somehow to work with rviz
* user-data is not correctly installed. PiOS uses a system daemon to run a script on first boot. Maybe cloud-init isn't working. Can try to put setup script in cloud-init per-boot folder or once folder.

## Troubleshooting
Checksum error with the ubuntu desktop image
* Most likely the image was updated but packer has cached the checksum and has not realized it should get the new checksum. 
* Delete .packer_plugins/github.com/ethanmdavidson/git/packer-plugin-git_v0.6.3_x5.0_linux_arm64_SHA256SUM to make packer download the checksum again.

Out of memory / build taking a long time
* Increase RAM memory limit in the Docker Desktop application (go to settings->resources)

## Issues
* Ubuntu shows you set up wizard on first boot (user-data not applied)
* user-data is not applied (still need to check if it copied over correctly). You can see in /etc/security/limits.conf that user-data has not been applied
* Takes forever to open any application (Terminal, Files, etc)
* Display orientation is wrong