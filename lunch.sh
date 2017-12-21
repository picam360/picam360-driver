#!/bin/bash

source ~/.picam360rc

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

CAM_WIDTH=2048
CAM_HEIGHT=2048
CAM0=0
CAM1=1
VIEW_COODINATE=
DEBUG=false

while getopts n:w:h:t:v:g OPT
do
    case $OPT in
        n)  NUM_OF_CAMERA=$OPTARG
            ;;
        w)  CAM_WIDTH=$OPTARG
            ;;
        h)  CAM_HEIGHT=$OPTARG
            ;;
        v)  VIEW_COODINATE="-v $OPTARG"
            ;;
        g)  DEBUG=true
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ $CAMERA_TYPE = "RASPICAM" ]; then

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

sudo killall raspivid

# cam0
/usr/bin/raspivid -cd MJPEG -n -t 0 -co 20 -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w $CAM_WIDTH -h $CAM_HEIGHT -fps 30 -cs $CAM0 -b 2000000 -o - > cam0 &
#/usr/bin/raspivid -cd MJPEG -n -t 0 -ex sports -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &

if [ $NUM_OF_CAMERA = "2" ]; then

# cam1
/usr/bin/raspivid -cd MJPEG -n -t 0 -co 20 -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM1 -b 8000000 -o - > cam1 &

fi

elif [ $CAMERA_TYPE = "USB_WDR" ]; then

./tools/Linux_UVC_TestAP/H264_UVC_TestAP --xuset-mjb 30000000 /dev/video0
if [ $NUM_OF_CAMERA = "2" ]; then
./tools/Linux_UVC_TestAP/H264_UVC_TestAP --xuset-mjb 30000000 /dev/video2
fi

fi

#wait for i2c available
sleep 3

# picam360-driver

sudo killall picam360-driver.bin

if [ $DEBUG = "true" ]; then

echo b main > gdbcmd
echo r $VIEW_COODINATE >> gdbcmd
gdb ./picam360-driver.bin -x gdbcmd

else

./picam360-driver.bin $VIEW_COODINATE

fi