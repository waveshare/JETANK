#!/bin/sh

set -e

password=$1

# install SCSservo pyserial imutils
apt update
python3 setup.py install

cp -r JETANK_1_servos //workspace/jetbot/notebooks
cp -r JETANK_2_ctrl //workspace/jetbot/notebooks
cp -r JETANK_3_motionDetect //workspace/jetbot/notebooks
cp -r JETANK_4_colorRecognition //workspace/jetbot/notebooks
cp -r JETANK_5_colorTracking //workspace/jetbot/notebooks
cp -r JETANK_6_gamepadCtrl //workspace/jetbot/notebooks