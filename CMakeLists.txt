cmake_minimum_required(VERSION 3.1.3)

project(picam360-driver)

if(APPLE)
	message("OSX")
	add_executable(picam360-driver.bin
		picam360_driver.c
		rtp.cc
		video_mjpeg.cc
		mrevent.c
		quaternion.c
		manual_mpu.c
	)
elseif(UNIX)
	message("UNIX or LINUX")
	add_executable(picam360-driver.bin
		picam360_driver.c
		rtp.cc
		video_mjpeg.cc
		mrevent.c
		quaternion.c
		manual_mpu.c
		v4l2_handler.c
	)
	add_definitions(-DENABLE_V4L2)
elseif(WIN32)
	message("WINDOWS")
endif()

set_target_properties(picam360-driver.bin PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(JANSSON jansson REQUIRED)
pkg_check_modules(LIBEDIT libedit REQUIRED)

include_directories(
	include
	${JANSSON_INCLUDE_DIRS}
	${LIBEDIT_INCLUDE_DIRS}
)
link_directories(
	${JANSSON_LIBRARY_DIRS}
	${LIBEDIT_LIBRARY_DIRS}
)

target_link_libraries(picam360-driver.bin
	${JANSSON_LIBRARIES}
	${LIBEDIT_LIBRARIES}
	pthread
	dl
)

option(USE_MPU9250 "Use mpu9250" OFF)
option(USE_ROV_DRIVER "Use rov_driver" OFF)

#plugins
if(USE_MPU9250)
	message("enable mpu9250")
	add_subdirectory(plugins/mpu9250)
endif()
if(USE_ROV_DRIVER)
	message("enable rov_driver")
	add_subdirectory(plugins/rov_driver)
endif()