# pupper_image_builder

## Description
An automated OS image builder for the Pupper robot. Pulls the latest Ubuntu image, applies all the necessary configurations, and installs ROS2 and Pupper-specific packages.

## How to use
Requires Docker to be installed and running.

```
docker pull mkaczanowski/packer-builder-arm:latest
docker run --memory 16g --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init .
docker run --memory 16g --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build .
```

This takes about 10 minutes on an M1 MacBook Pro.

The output is written to `pupOS.img`.

### Default credentials
Hostname: `pupper`

User: `pi`

Password: `rhea123`

## Troubleshooting
Checksum error with the ubuntu desktop image
* Most likely the image was updated but packer has cached the checksum and has not realized it should get the new checksum. 
* Delete .packer_plugins/github.com/ethanmdavidson/git/packer-plugin-git_v0.6.3_x5.0_linux_arm64_SHA256SUM to make packer download the checksum again.

## Issues
* Ubuntu shows you set up wizard on first boot (user-data not applied)
* user-data is not applied (still need to check if it copied over correctly). You can see in /etc/security/limits.conf that user-data has not been applied
* Takes forever to open any application (Terminal, Files, etc)
* Display orientation is wrong