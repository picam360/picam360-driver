#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <wchar.h>
#include <limits.h>
#include <dirent.h>
#include <pthread.h>

#include "rov_driver.h"

#include <mat4/identity.h>
#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PLUGIN_NAME "rov_driver"

#define PT_STATUS 100
#define PT_CMD 101
#define PT_CAM_BASE 110

static PLUGIN_HOST_T *lg_plugin_host = NULL;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static float lg_motor_center = 0.0737;
static float lg_motor_margin = 0.0013;
static float lg_motor_range = 0.005;
#define MOTOR_BASE(value) lg_motor_center + lg_motor_margin * ((value == 0) ? 0 : (value > 0) ? 1 : -1)

static int lg_light_id[LIGHT_NUM] = { 4, 34 };
static int lg_motor_id[MOTOR_NUM] = { 18, 36, 35, 17 };

static int lg_light_value[LIGHT_NUM] = { 0, 0 };
static int lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_motor_dir[4] = { -1, 1, -1, 1 };

static float lg_light_strength = 0; //0 to 100
static float lg_thrust = 0; //-100 to 100
static float lg_brake_ps = 5; // percent
static VECTOR4D_T lg_target_quaternion = { .ary = { 0, 0, 0, 1 } };

static bool lg_lowlevel_control = false;
static bool lg_pid_enabled = false;
static float lg_yaw_diff = 0;
static float lg_pitch_diff = 0;
static float lg_p_gain = 1.0;
static float lg_i_gain = 1.0;
static float lg_d_gain = 1.0;
static float lg_pid_value[3] = { }; //x, z, delta yaw
static float lg_delta_pid_target[3][3] = { }; //x, z, delta yaw
static struct timeval lg_delta_pid_time[3] = { };

static void release(void *user_data) {
	free(user_data);
}

static bool init_pwm() {
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd > 0) {
		char cmd[256];
		int len;
		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[0], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[1], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[2], lg_motor_center);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[3], lg_motor_center);
		write(fd, cmd, len);
		close(fd);
	}
	return true;
}

static void update_pwm() {
	int len = 0;
	char cmd[256] = { };
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd < 0) {
		return;
	}

	{
		len = sprintf(cmd, "sync=off\n");
		write(fd, cmd, len);
	}

	{
		float value = lg_light_value[0];
		value = pow(value / 100, 3);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_light_value[1];
		value = pow(value / 100, 3);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[0];
		value = lg_motor_dir[0] * (value / 100)
				* lg_motor_range+ MOTOR_BASE(lg_motor_dir[0] * value);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[0], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[1];
		value = lg_motor_dir[1] * (value / 100)
				* lg_motor_range+ MOTOR_BASE(lg_motor_dir[1] * value);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[1], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[2];
		value = lg_motor_dir[2] * (value / 100)
				* lg_motor_range+ MOTOR_BASE(lg_motor_dir[2] * value);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[2], value);
		write(fd, cmd, len);
	}

	{
		float value = lg_motor_value[3];
		value = lg_motor_dir[3] * (value / 100)
				* lg_motor_range+ MOTOR_BASE(lg_motor_dir[3] * value);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[3], value);
		write(fd, cmd, len);
	}

	{
		len = sprintf(cmd, "sync=on\n");
		write(fd, cmd, len);
	}

	close(fd);
}

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
void *pid_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "DA PID");

	static struct timeval last_time = { };
	gettimeofday(&last_time, NULL);
	while (1) {
		usleep(10 * 1000); //less than 100Hz
		if(lg_lowlevel_control){
			continue;
		}

		struct timeval time = { };
		gettimeofday(&time, NULL);
		struct timeval diff;
		timersub(&time, &last_time, &diff);
		float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		//cal
		//trancate min max
		lg_light_strength = MIN(MAX(lg_light_strength, 0), 100);
		lg_light_value[0] = lg_light_strength;
		lg_light_value[1] = lg_light_strength;

		//trancate min max
		lg_thrust = MIN(MAX(lg_thrust, -100), 100);
		//brake
		lg_thrust *= exp(log(1.0 - lg_brake_ps / 100) * diff_sec);

		if (lg_pid_enabled) {
			float x, y, z;
			if (lg_debugdump) {
				quaternion_get_euler(lg_target_quaternion, &y, &x, &z,
						EULER_SEQUENCE_YXZ);
				printf("target  : %f, %f, %f\n", x * 180 / M_PI, y * 180 / M_PI,
						z * 180 / M_PI);
			}
			VECTOR4D_T quat = lg_plugin_host->get_quaternion();
			//quat = quaternion_multiply(quat, quaternion_get_from_z(M_PI)); //mpu offset
			if (lg_debugdump) {
				quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("vehicle : %f, %f, %f\n", x * 180 / M_PI, y * 180 / M_PI,
						z * 180 / M_PI);
			}
			//(RcRt-1Rc-1)*(Rc)*vtg, target coordinate will be converted into camera coordinate
			float vtg[16] = { 0, -1, 0, 1 }; // looking at ground
			float unif_matrix[16];
			float camera_matrix[16];
			float target_matrix[16];
			mat4_identity(unif_matrix);
			mat4_identity(camera_matrix);
			mat4_identity(target_matrix);
			mat4_fromQuat(camera_matrix, quat.ary);
			mat4_fromQuat(target_matrix, lg_target_quaternion.ary);
			mat4_invert(target_matrix, target_matrix);
			mat4_multiply(unif_matrix, unif_matrix, target_matrix); // Rt-1
			mat4_multiply(unif_matrix, unif_matrix, camera_matrix); // RcRt-1

			mat4_transpose(vtg, vtg);
			mat4_multiply(vtg, vtg, unif_matrix);
			mat4_transpose(vtg, vtg);

			if (lg_debugdump) {
				printf("vehicle : %f, %f, %f\n", vtg[0], vtg[1], vtg[2]);
			}

			float xz = sqrt(vtg[0] * vtg[0] + vtg[2] * vtg[2]);
			lg_yaw_diff = -atan2(vtg[2], vtg[0]) * 180 / M_PI;
			lg_pitch_diff = atan2(xz, -vtg[1]) * 180 / M_PI; //[-180:180]

			static float last_yaw = 0;
			lg_delta_pid_time[0] = time;
			lg_delta_pid_target[0][0] = cos(lg_yaw_diff * M_PI / 180)
					* (lg_pitch_diff / 180); // x [-1:1]
			lg_delta_pid_target[1][0] = sin(lg_yaw_diff * M_PI / 180)
					* (lg_pitch_diff / 180); // z [-1:1]
			lg_delta_pid_target[2][0] = sub_angle(lg_yaw_diff, last_yaw) / 180; // delta yaw [-1:1]

			timersub(&lg_delta_pid_time[0], &lg_delta_pid_time[1], &diff);
			diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			diff_sec = MAX(MIN(diff_sec, 1.0), 0.001);

			for (int k = 0; k < 3; k++) {
				float p_value =
						lg_p_gain
								* (lg_delta_pid_target[k][0]
										- lg_delta_pid_target[k][1]);
				float i_value = lg_i_gain * lg_delta_pid_target[k][0]
						* diff_sec;
				float d_value = lg_d_gain
						* (lg_delta_pid_target[k][0]
								- 2 * lg_delta_pid_target[k][1]
								+ lg_delta_pid_target[k][2]) / diff_sec;
				float delta_value = p_value + i_value + d_value;
				lg_pid_value[k] += delta_value;
				lg_pid_value[k] = MIN(MAX(lg_pid_value[k], -2500), 2500);
			}

			//increment
			for (int j = 3 - 1; j >= 1; j--) {
				for (int k = 0; k < 3; k++) {
					lg_delta_pid_target[k][j] = lg_delta_pid_target[k][j - 1];
				}
				lg_delta_pid_time[j] = lg_delta_pid_time[j - 1];
			}
			last_yaw = lg_yaw_diff;

			// 0 - 1
			// |   |
			// 3 - 2
			float motor_pid_gain[3][MOTOR_NUM] = { //
					//
							{ 1, -1, -1, 1 },				// x
							{ -1, -1, 1, 1 },				// z
							{ -1, 1, -1, 1 }			// delta yaw
					};
			float motor_pid_value[MOTOR_NUM] = { };
			for (int k = 0; k < 3; k++) {
				for (int i = 0; i < MOTOR_NUM; i++) {
					float power_calib = (lg_pid_value[k] > 0 ? 1 : -1)
							* sqrt(abs(lg_pid_value[k]));
					motor_pid_value[i] += power_calib * motor_pid_gain[k][i];
				}
			}
			for (int i = 0; i < MOTOR_NUM; i++) {
				float value = lg_thrust + motor_pid_value[i];
				float diff = value - lg_motor_value[i];
				int max_diff = 10;
				if (abs(diff) > max_diff) {
					diff = (diff > 0) ? max_diff : -max_diff;
				}
				value = lg_motor_value[i] + diff;
				if (value * lg_motor_value[i] < 0) {
					value = 0;
				}
				lg_motor_value[i] = value;
			}
			// end of pid control
		} else {
			for (int i = 0; i < MOTOR_NUM; i++) {
				float value = lg_thrust;
				float diff = value - lg_motor_value[i];
				int max_diff = 10;
				if (abs(diff) > max_diff) {
					diff = (diff > 0) ? max_diff : -max_diff;
				}
				value = lg_motor_value[i] + diff;
				if (value * lg_motor_value[i] < 0) {
					value = 0;
				}
				lg_motor_value[i] = MIN(MAX(value, -100), 100);
			}
		}
		update_pwm();

		last_time = time;
	} // end of while
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_thrust", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v, x, y, z, w;
			int num = sscanf(param, "%f %f,%f,%f,%f", &v, &x, &y, &z, &w);

			if (num == 1) {
				lg_thrust = v;
			} else if (num == 5) {
				lg_thrust = v;
				lg_target_quaternion.x = x;
				lg_target_quaternion.y = y;
				lg_target_quaternion.z = z;
				lg_target_quaternion.w = w;
			}
			printf("set_thrust : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_strength", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float v;
			int num = sscanf(param, "%f", &v);

			if (num == 1) {
				lg_light_strength = v;
			}
			printf("set_light_strength : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_target_quaternion", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float x, y, z, w;
			int num = sscanf(param, "%f,%f,%f,%f", &x, &y, &z, &w);

			if (num == 4) {
				lg_target_quaternion.x = x;
				lg_target_quaternion.y = y;
				lg_target_quaternion.z = z;
				lg_target_quaternion.w = w;
			}
			printf("set_target_quaternion : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_lowlevel_control", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_lowlevel_control = (value != 0);
			printf("set_lowlevel_control : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_pid_enabled", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_pid_enabled = (value != 0);
			printf("set_pid_enabled : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_value", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < LIGHT_NUM) {
				lg_light_value[id] = value;
			}
			sscanf(param, "%f", &value);
			printf("set_light_value : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_motor_value", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int id = 0;
			float value = 0;
			sscanf(param, "%d=%f", &id, &value);
			if (id < MOTOR_NUM) {
				lg_motor_value[id] = value;
			}
			printf("set_motor_value : completed\n");
		}
	} else {
		printf(":unknown command : %s\n", buff);
	}
	return 0;
}

static void event_handler(void *user_data, uint32_t node_id, uint32_t event_id) {
	switch (node_id) {
	case PICAM360_HOST_NODE_ID:
		switch (event_id) {
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void init_options(void *user_data, json_t *options) {
	lg_p_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".p_gain"));
	lg_i_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".i_gain"));
	lg_d_gain = json_number_value(
			json_object_get(options, PLUGIN_NAME ".d_gain"));

	for (int i = 0; i < LIGHT_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".light%d_id", i);
		int id = (int) json_number_value(json_object_get(options, buff));
		if (id != 0) {
			lg_light_id[i] = id;
		}
	}

	for (int i = 0; i < MOTOR_NUM; i++) {
		char buff[256];
		int value;
		sprintf(buff, PLUGIN_NAME ".motor%d_id", i);
		value = (int) json_number_value(json_object_get(options, buff));
		if (value != 0) {
			lg_motor_id[i] = value;
		}
		sprintf(buff, PLUGIN_NAME ".motor%d_dir", i);
		value = (int) json_number_value(json_object_get(options, buff));
		if (value != 0) {
			lg_motor_dir[i] = value;
		}
	}
	{
		float value;
		value = json_number_value(
				json_object_get(options, PLUGIN_NAME ".motor_center"));
		if (value != 0) {
			lg_motor_center = value;
		}
		value = json_number_value(
				json_object_get(options, PLUGIN_NAME ".motor_margin"));
		if (value != 0) {
			lg_motor_margin = value;
		}
		value = json_number_value(
				json_object_get(options, PLUGIN_NAME ".motor_range"));
		if (value != 0) {
			lg_motor_range = value;
		}
	}
}

static void save_options(void *user_data, json_t *options) {
	json_object_set_new(options, PLUGIN_NAME ".p_gain", json_real(lg_p_gain));
	json_object_set_new(options, PLUGIN_NAME ".i_gain", json_real(lg_i_gain));
	json_object_set_new(options, PLUGIN_NAME ".d_gain", json_real(lg_d_gain));

	for (int i = 0; i < LIGHT_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".light%d_id", i);
		json_object_set_new(options, buff, json_real(lg_light_id[i]));
	}

	for (int i = 0; i < MOTOR_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".motor%d_id", i);
		json_object_set_new(options, buff, json_real(lg_motor_id[i]));
		sprintf(buff, PLUGIN_NAME ".motor%d_dir", i);
		json_object_set_new(options, buff, json_real(lg_motor_dir[i]));
	}

	{
		json_object_set_new(options, PLUGIN_NAME ".motor_center",
				json_real(lg_motor_center));
		json_object_set_new(options, PLUGIN_NAME ".motor_margin",
				json_real(lg_motor_margin));
		json_object_set_new(options, PLUGIN_NAME ".motor_range",
				json_real(lg_motor_range));
	}
}

static bool is_init = false;
static void init() {
	if (is_init) {
		return;
	}
	is_init = true;

	bool succeeded = init_pwm();
	if (!succeeded) {
		perror("An error happen in init_pwm().");
		exit(-1);
	}

	pthread_t pid_thread;
	pthread_create(&pid_thread, NULL, pid_thread_func, (void*) NULL);
}

#define MAX_INFO_LEN 1024
static wchar_t lg_info[MAX_INFO_LEN];
static wchar_t *get_info(void *user_data) {
	lg_info[0] = L'\0';
	return lg_info;
}

void create_plugin(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
	init();
	lg_plugin_host = plugin_host;

	{
		PLUGIN_T *plugin = (PLUGIN_T*) malloc(sizeof(PLUGIN_T));
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
}
