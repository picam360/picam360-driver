OBJS=picam360_driver.o rtp.o video_mjpeg.o mrevent.o quaternion.o manual_mpu.o \
	 libs/MotionSensor/libMotionSensor.a libs/libI2Cdev.a \
	 plugins/rov_driver/rov_driver.o plugins/mpu9250/mpu9250.o
BIN=picam360-driver.bin
LDFLAGS+=-ljansson

include Makefile.include


