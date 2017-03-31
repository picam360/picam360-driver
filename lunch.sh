#!/bin/bash

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

CAMERA_WIDTH=2048
CAMERA_HEIGHT=2048
#CAMERA_WIDTH=1440
#CAMERA_HEIGHT=1440
CAM0=1
CAM1=0

sudo killall socat
sudo killall raspivid
sudo killall picam360-driver.bin

if [ -e cam0 ]; then
	rm cam0
fi
mkfifo cam0
chmod 0666 cam0

if [ -e cam1 ]; then
	rm cam1
fi
mkfifo cam1
chmod 0666 cam1

if [ -e cmd ]; then
	rm cmd
fi
mkfifo cmd
chmod 0666 cmd


# cam0
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 30 -cs $CAM0 -b 2000000 -o - > cam0 &
#/usr/bin/raspivid -cd MJPEG -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &

# cam1
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM1 -b 8000000 -o - > cam1 &

#wait for i2c available
sleep 3

# driver
./picam360-driver.bin