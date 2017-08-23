#!/bin/bash

source ~/.picam360rc

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

CAM_NUM=1
CAM_WIDTH=2048
CAM_HEIGHT=2048
#[RASPI,USB]
TYPE=RASPI
CAM0=0
CAM1=1
VIEW_COODINATE=
DEBUG=false

while getopts n:w:h:t:v:g OPT
do
    case $OPT in
        n)  CAM_NUM=$OPTARG
            ;;
        w)  CAM_WIDTH=$OPTARG
            ;;
        h)  CAM_HEIGHT=$OPTARG
            ;;
        t)  TYPE=$OPTARG
            ;;
        v)  VIEW_COODINATE="-v $OPTARG"
            ;;
        g)  DEBUG=true
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ -e cmd ]; then
	rm cmd
fi
mkfifo cmd
chmod 0666 cmd

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

if [ -e rtcp_rx ]; then
	rm rtcp_rx
fi
mkfifo rtcp_rx
chmod 0666 rtcp_rx

if [ -e rtcp_tx ]; then
	rm rtcp_tx
fi
mkfifo rtcp_tx
chmod 0666 rtcp_tx

if [ "$CAPTURE_IP" = "" ]; then
	CAPTURE_IP="192.168.4.2"
fi

#use tcp
#	sudo killall nc
#   nc -l -p 9006 < rtp_tx > rtp_rx &

sudo killall socat
socat -u udp-recv:9003 - > rtcp_rx &
socat PIPE:rtp_tx UDP-DATAGRAM:$CAPTURE_IP:9002 &

#socat tcp-connect:$CAPTURE_IP:9002 PIPE:rtp_tx &

if [ $TYPE = "RASPI" ]; then

sudo killall raspivid

# cam0
/usr/bin/raspivid -cd MJPEG -n -t 0 -co 20 -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w $CAM_WIDTH -h $CAM_HEIGHT -fps 30 -cs $CAM0 -b 2000000 -o - > cam0 &
#/usr/bin/raspivid -cd MJPEG -n -t 0 -ex sports -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM0 -b 8000000 -o - > cam0 &

if [ $CAM_NUM = "2" ]; then

# cam1
/usr/bin/raspivid -cd MJPEG -n -t 0 -co 20 -w $CAM_WIDTH -h $CAM_HEIGHT -fps 5 -cs $CAM1 -b 8000000 -o - > cam1 &

fi

elif [ $TYPE = "USB" ]; then

bash v4l2-ctl.sh /dev/video0
if [ $CAM_NUM = "2" ]; then
bash v4l2-ctl.sh /dev/video2
fi

sudo killall ffmpeg

CAM_RESOLUTION=${CAM_WIDTH}x${CAM_HEIGHT}

ffmpeg -r 15 -s $CAM_RESOLUTION -f video4linux2 -input_format mjpeg -i /dev/video0 -vcodec copy pipe:1.mjpeg > cam0 2> /dev/null &

if [ $CAM_NUM = "2" ]; then

ffmpeg -r 15 -s $CAM_RESOLUTION -f video4linux2 -input_format mjpeg -i /dev/video2 -vcodec copy pipe:1.mjpeg > cam1 2> /dev/null &

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