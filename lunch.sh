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


if [ -e rtp_rx ]; then
	rm rtp_rx
fi
mkfifo rtp_rx
chmod 0666 rtp_rx

if [ -e rtp_tx ]; then
	rm rtp_tx
fi
mkfifo rtp_tx
chmod 0666 rtp_tx

sudo killall socat
#socat -u udp-recv:9004 - > rtp_rx &
socat PIPE:rtp_tx UDP-DATAGRAM:192.168.4.2:9002 &
socat tcp-connect:192.168.4.2:9002 PIPE:rtp_tx &
	
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