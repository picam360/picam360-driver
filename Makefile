OBJS=attitude_injector.o \
	 MotionSensor/libMotionSensor.a libs/libI2Cdev.a
BIN=attitude-injector.bin
LDFLAGS+=

include Makefile.include


