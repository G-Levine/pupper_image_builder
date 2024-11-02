#!/bin/bash -e

set -x

sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
echo '[Service]\nExecStart=\nExecStart=-/sbin/agetty --autologin pi --noclear %I $TERM' | sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf
sudo systemctl daemon-reload
sudo systemctl restart getty@tty1.service