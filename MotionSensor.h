#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#define YAW 0
#define ROLL 1
#define PITCH 2
#define DIM 3
#ifdef __cplusplus
extern "C"{
#endif

extern float quatanion[4];
extern float ypr[3]; //yaw, pitch, roll
extern float accel[3];
extern float gyro[3];
extern float temp;
extern float compass[3];

extern int ms_open();
extern int ms_update();
extern int ms_close();

#ifdef __cplusplus
}
#endif

#endif
