CAMERA_WIDTH=2048
CAMERA_HEIGHT=2048
#CAMERA_WIDTH=1440
#CAMERA_HEIGHT=1440

# cam0
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs 0 -b 8000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9100 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 30 -cs 0 -b 2000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9100 &
#/usr/bin/raspivid -cd MJPEG -t 0 -ex sports -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs 0 -b 8000000 -o - | nc -l 9100 &

# cam1
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w $CAMERA_WIDTH -h $CAMERA_HEIGHT -fps 5 -cs 1 -b 8000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9101 &

#wait for i2c available
sleep 3

# driver
/home/pi/picam360/picam360-driver/picam360-driver.bin | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9000 &