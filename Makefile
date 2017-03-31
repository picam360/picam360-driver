OBJS=picam360_driver.o rtp.o video_mjpeg.o mrevent.o \
	 MotionSensor/libMotionSensor.a libs/libI2Cdev.a
BIN=picam360-driver.bin
LDFLAGS+=

include Makefile.include


