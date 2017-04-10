#!/bin/bash

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

CAMERA_WIDTH=2048
CAMERA_HEIGHT=2048
#CAMERA_WIDTH=1440
#CAMERA_HEIGHT=1440
CAM0=1
CAM1=0
RPICAM=true
USBCAM=false

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

#use tcp
#	sudo killall nc
#   nc -l -p 9006 < rtp_tx > rtp_rx &

sudo killall socat
socat -u udp-recv:9004 - > rtp_rx &
socat PIPE:rtp_tx UDP-DATAGRAM:192.168.4.2:9002 &
#socat tcp-connect:192.168.4.2:9002 PIPE:rtp_tx &

if [ $RPICAM = true ]; then

sudo killall raspivid

# cam0
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 30 -cs $CAM0 -b 2000000 -o - > cam0 &
#/usr/bin/raspivid -cd MJPEG -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &

# cam1
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs $CAM1 -b 8000000 -o - > cam1 &

elif [ $USBCAM = true ]; then

sudo killall ffmpeg

ffmpeg -r 15 -s 2048x1536 -f video4linux2 -input_format mjpeg -i /dev/video0 -vcodec copy pipe:1.mjpeg > cam0 2> /dev/null &
ffmpeg -r 15 -s 2048x1536 -f video4linux2 -input_format mjpeg -i /dev/video1 -vcodec copy pipe:1.mjpeg > cam1 2> /dev/null &

fi

#wait for i2c available
sleep 3

# driver
./picam360-driver.bin