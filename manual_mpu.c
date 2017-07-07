#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>

#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#include "manual_mpu.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define MPU_NAME "manual"

static VECTOR4D_T lg_compass = { .ary = { 0, 0, 0, 1 } };
static VECTOR4D_T lg_quat = { .ary = { 0, 0, 0, 1 } };
static float lg_north = 0;
static float lg_temp = 0;

static VECTOR4D_T get_quaternion() {
	return lg_quat;
}

static VECTOR4D_T get_compass() {
	return lg_compass;
}

static float get_temperature() {
	return lg_temp;
}

static float get_north() {
	return lg_north;
}

static void release(void *user_data) {
	free(user_data);
}

void create_manual_mpu(MPU_T **_mpu) {
	MPU_T *mpu = (MPU_T*) malloc(sizeof(MPU_T));
	strcpy(mpu->name, MPU_NAME);
	mpu->release = release;
	mpu->get_quaternion = get_quaternion;
	mpu->get_compass = get_compass;
	mpu->get_temperature = get_temperature;
	mpu->get_north = get_north;
	mpu->user_data = mpu;

	*_mpu = mpu;
}

void manual_mpu_set(MPU_T *mpu, float x_deg, float y_deg, float z_deg) {
	lg_quat = quaternion_init();
	lg_quat = quaternion_multiply(lg_quat, quaternion_get_from_z(z_deg));
	lg_quat = quaternion_multiply(lg_quat, quaternion_get_from_x(x_deg));
	lg_quat = quaternion_multiply(lg_quat, quaternion_get_from_y(y_deg));
}
