OBJS=picam360_driver.o rtp.o rtcp.o video_mjpeg.o v4l2_handler.o mrevent.o quaternion.o manual_mpu.o
PLUGINS=plugins/mpu9250 plugins/rov_driver
TOOLS=tools/Linux_UVC_TestAP
BIN=picam360-driver.bin

include Makefile.include


