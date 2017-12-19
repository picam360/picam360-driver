#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

#include "MotionSensor.h"

#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#include "mpu9250.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "mpu9250"
#define MPU_NAME "mpu9250"

static PLUGIN_HOST_T *lg_plugin_host = NULL;
static pthread_mutex_t lg_mutex = { };
static struct timeval lg_base_time = { };

static int lg_i2c_ch = 1;

static int lg_average_count = 1;
static bool lg_is_compass_calib = false;
static float lg_compass_min[3] = { -317.000000, -416.000000, -208.000000 };
//static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { 221.000000, -67.000000, 98.000000 };
//static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };
static VECTOR4D_T lg_compass = { };
static VECTOR4D_T lg_quat = { };
static float lg_north = 0;
static int lg_north_count = 0;

static float lg_offset_pitch = 0;
static float lg_offset_yaw = 0;
static float lg_offset_roll = 0;

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
static void *threadFunc(void *data) {
	pthread_setname_np(pthread_self(), "MPU9250");

	do {
		float _quaternion[4] = { };
		for (int i = 0; i < lg_average_count; i++) {
			ms_update();
			for (int j = 0; j < 4; j++) {
				_quaternion[j] += quaternion[j];
			}
		}
		for (int j = 0; j < 4; j++) {
			_quaternion[j] /= lg_average_count;
		}

		VECTOR4D_T quat = { };
		{ //compas : calibration
			float calib[3];
			float bias[3];
			float gain[3];
			for (int i = 0; i < 3; i++) {
				if (lg_is_compass_calib) {
					lg_compass_min[i] = MIN(lg_compass_min[i], compass[i]);
					lg_compass_max[i] = MAX(lg_compass_max[i], compass[i]);
				}
				bias[i] = (lg_compass_min[i] + lg_compass_max[i]) / 2;
				gain[i] = (lg_compass_max[i] - lg_compass_min[i]) / 2;
				calib[i] = (compass[i] - bias[i]) / (gain[i] == 0 ? 1 : gain[i]);
			}
			float norm = sqrt(calib[0] * calib[0] + calib[1] * calib[1] + calib[2] * calib[2]);
			for (int i = 0; i < 3; i++) {
				calib[i] /= (norm == 0 ? 1 : norm);
			}
			//convert from mpu coodinate to opengl coodinate
			lg_compass.ary[0] = calib[1];
			lg_compass.ary[1] = -calib[0];
			lg_compass.ary[2] = -calib[2];
			lg_compass.ary[3] = 1.0;
		}
		{ //quat : convert from mpu coodinate to opengl coodinate
			quat.ary[0] = _quaternion[1];	//x
			quat.ary[1] = _quaternion[3];	//y : swap y and z
			quat.ary[2] = -_quaternion[2];	//z : swap y and z
			quat.ary[3] = _quaternion[0];	//w
		}
		{ //north
			float north = 0;

			float matrix[16];
			mat4_fromQuat(matrix, quat.ary);
			mat4_invert(matrix, matrix);

			float compass_mat[16] = { };
			memcpy(compass_mat, lg_compass.ary, sizeof(float) * 4);
			if (lg_debugdump) {
				printf("compass1 %f : %f, %f, %f\n", lg_north, compass_mat[0], compass_mat[1], compass_mat[2]);
			}

			mat4_transpose(compass_mat, compass_mat);
			mat4_multiply(compass_mat, compass_mat, matrix);
			mat4_transpose(compass_mat, compass_mat);

			if (lg_debugdump) {
				printf("compass2 %f : %f, %f, %f\n", lg_north, compass_mat[0], compass_mat[1], compass_mat[2]);
			}

			north = -atan2(compass_mat[0], -compass_mat[2]) * 180 / M_PI; // start from z axis

			lg_north = lg_north + sub_angle(north, lg_north) / (lg_north_count + 1);
			lg_north_count++;
			if (lg_north_count > 100) {
				lg_north_count = 100;
			}
		}
		{ //calib
			float x, y, z;
			if (lg_debugdump) {
				quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("original %f : %f, %f, %f\n", lg_north, x * 180 / M_PI, y * 180 / M_PI, z * 180 / M_PI);
			}
			VECTOR4D_T quat_offset = quaternion_init();
			quat_offset = quaternion_multiply(quat_offset, quaternion_get_from_z(lg_offset_roll));
			quat_offset = quaternion_multiply(quat_offset, quaternion_get_from_x(lg_offset_pitch));
			quat_offset = quaternion_multiply(quat_offset, quaternion_get_from_y(lg_offset_yaw));
			quat = quaternion_multiply(quat, quat_offset); // Rv=RvoRv
			if (lg_debugdump) {
				quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("offset   %f : %f, %f, %f\n", lg_north, x * 180 / M_PI, y * 180 / M_PI, z * 180 / M_PI);
			}
			quat = quaternion_multiply(quaternion_get_from_y(-lg_north * M_PI / 180), quat); // Rv=RvoRvRn
			if (lg_debugdump) {
				quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("north   %f : %f, %f, %f\n", lg_north, x * 180 / M_PI, y * 180 / M_PI, z * 180 / M_PI);
			}
		}
		{ //time
			struct timeval diff;
			static struct timeval time = { };
			gettimeofday(&time, NULL);
			timersub(&time, &lg_base_time, &diff);
			quat.t = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		}
		pthread_mutex_lock(&lg_mutex);
		lg_quat = quat;
		pthread_mutex_unlock(&lg_mutex);

		usleep(5000);
	} while (1);
	return NULL;
}

#if (1) //status block

#define STATUS_VAR(name) lg_status_ ## name
#define STATUS_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_status(STATUS_VAR(name));

static STATUS_T *STATUS_VAR(is_compass_calib);
static STATUS_T *STATUS_VAR(compass_min);
static STATUS_T *STATUS_VAR(compass_max);

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(is_compass_calib)) {
		snprintf(buff, buff_len, "%d", lg_is_compass_calib ? 1 : 0);
	} else if (status == STATUS_VAR(compass_min)) {
		snprintf(buff, buff_len, "%f,%f,%f", lg_compass_min[0], lg_compass_min[1], lg_compass_min[2]);
	} else if (status == STATUS_VAR(compass_max)) {
		snprintf(buff, buff_len, "%f,%f,%f", lg_compass_max[0], lg_compass_max[1], lg_compass_max[2]);
	}
}

static void status_set_value(void *user_data, const char *value) {
	//STATUS_T *status = (STATUS_T*) user_data;
}

static STATUS_T *new_status(const char *name) {
	STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
	strcpy(status->name, name);
	status->get_value = status_get_value;
	status->set_value = status_set_value;
	status->release = status_release;
	status->user_data = status;
	return status;
}

static void init_status() {
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", is_compass_calib);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", compass_min);
	STATUS_INIT(lg_plugin_host, PLUGIN_NAME ".", compass_max);
}

#endif //status block

static bool is_init = false;
static void init() {
	if (is_init) {
		return;
	} else {
		is_init = true;
	}
	if (ms_open(lg_i2c_ch) != 0) {
		//error
		return;
	}

	pthread_mutex_init(&lg_mutex, 0);
	gettimeofday(&lg_base_time, NULL);

	init_status();

//do{
//	ms_update();
//	printf("%f,%f,%f,%f\n",  (float)_q[0] / (1<<30),  (float)_q[1] / (1<<30),  (float)_q[2] / (1<<30),  (float)_q[3] / (1<<30));
//	usleep(5000);
//}while(1);
	pthread_t f1_thread;
	pthread_create(&f1_thread, NULL, threadFunc, NULL);
}

static VECTOR4D_T get_quaternion() {
	VECTOR4D_T quat;
	pthread_mutex_lock(&lg_mutex);
	quat = lg_quat;
	pthread_mutex_unlock(&lg_mutex);
	return quat;
}

static VECTOR4D_T get_compass() {
	return lg_compass;
}

static float get_temperature() {
	return temp;
}

static float get_north() {
	return lg_north;
}

static void release(void *user_data) {
	free(user_data);
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".start_compass_calib", sizeof(buff)) == 0) {
		lg_is_compass_calib = true;
		for (int i = 0; i < 3; i++) {
			lg_compass_min[i] = INT_MAX;
			lg_compass_max[i] = -INT_MAX;
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_compass_calib", sizeof(buff)) == 0) {
		lg_is_compass_calib = false;
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		break;
	default:
		break;
	}
}

static void init_options(void *user_data, json_t *options) {
	lg_offset_pitch = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_pitch"));
	lg_offset_yaw = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_yaw"));
	lg_offset_roll = json_number_value(json_object_get(options, PLUGIN_NAME ".offset_roll"));
	lg_i2c_ch = json_number_value(json_object_get(options, PLUGIN_NAME ".i2c_ch"));
	lg_average_count = json_number_value(json_object_get(options, PLUGIN_NAME ".average_count"));
	if (lg_average_count <= 0) {
		lg_average_count = 1;
	}

	for (int i = 0; i < 3; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".compass_min_%d", i);
		lg_compass_min[i] = json_number_value(json_object_get(options, buff));
		sprintf(buff, PLUGIN_NAME ".compass_max_%d", i);
		lg_compass_max[i] = json_number_value(json_object_get(options, buff));
	}

	init();
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".offset_pitch", json_real(lg_offset_pitch));
	json_object_set_new(options, PLUGIN_NAME ".offset_yaw", json_real(lg_offset_yaw));
	json_object_set_new(options, PLUGIN_NAME ".offset_roll", json_real(lg_offset_roll));
	json_object_set_new(options, PLUGIN_NAME ".i2c_ch", json_real(lg_i2c_ch));
	json_object_set_new(options, PLUGIN_NAME ".average_count", json_real(lg_average_count));

	for (int i = 0; i < 3; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".compass_min_%d", i);
		json_object_set_new(options, buff, json_real(lg_compass_min[i]));
		sprintf(buff, PLUGIN_NAME ".compass_max_%d", i);
		json_object_set_new(options, buff, json_real(lg_compass_max[i]));
	}
}

#define MAX_INFO_LEN 1024
static wchar_t lg_info[MAX_INFO_LEN];
static wchar_t *get_info(void *user_data) {
	int cur = 0;
	if (lg_is_compass_calib) {
		cur += swprintf(lg_info + cur, MAX_INFO_LEN - cur, L"\ncompass calib : min[%.1f,%.1f,%.1f] max[%.1f,%.1f,%.1f]", lg_compass_min[0], lg_compass_min[1], lg_compass_min[2], lg_compass_max[0],
				lg_compass_max[1], lg_compass_max[2]);
	}
	return lg_info;
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
		memset(plugin, 0, sizeof(PLUGIN_T));
		strcpy(plugin->name, PLUGIN_NAME);
		plugin->release = release;
		plugin->command_handler = command_handler;
		plugin->event_handler = event_handler;
		plugin->init_options = init_options;
		plugin->save_options = save_options;
		plugin->get_info = get_info;
		plugin->user_data = plugin;

		*_plugin = plugin;
	}
	{
		MPU_T *mpu = (MPU_T*) malloc(sizeof(MPU_T));
		memset(mpu, 0, sizeof(MPU_T));
		strcpy(mpu->name, MPU_NAME);
		mpu->release = release;
		mpu->get_quaternion = get_quaternion;
		mpu->get_compass = get_compass;
		mpu->get_temperature = get_temperature;
		mpu->get_north = get_north;
		mpu->user_data = mpu;

		lg_plugin_host->add_mpu(mpu);
	}
}
