#!/bin/sh

set -e

password=$1

# install JETANK
echo $password | sudo -S systemctl stop nvgetty
echo $password | sudo -S systemctl disable nvgetty

echo $password | sudo -S touch /etc/udev/rules.d/20-myserial.rules
echo $password | sudo -S chmod 666 /etc/udev/rules.d/20-myserial.rules
echo $password | sudo -S echo 'KERNEL=="ttyTHS1",MODE="0666"' >> /etc/udev/rules.d/20-myserial.rules || true

echo $password | sudo -S udevadm control --reload

echo $password | sudo -S wget http://www.waveshare.net/w/upload/e/eb/Camera_overrides.tar.gz
echo $password | sudo -S tar zxvf Camera_overrides.tar.gz
echo $password | sudo -S cp camera_overrides.isp /var/nvidia/nvcam/settings/

echo $password | sudo -S chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
echo $password | sudo -S chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
echo 'reboot'
echo $password | sudo -S reboot