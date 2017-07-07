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
#include "rtp.h"

#include <mat4/identity.h>
#include <mat4/rotateY.h>
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
static bool lg_lowlevel_control = false;
static VECTOR4D_T lg_target_quaternion = { .ary = { 0, 0, 0, 1 } };

static bool lg_pid_enabled = false;
static float lg_yaw_diff = 0;
static float lg_pitch_diff = 0;
static float lg_p_gain = 1.0;
static float lg_i_gain = 1.0;
static float lg_d_gain = 1.0;
static float lg_pid_value[3] = { }; //x, z, delta yaw
static float lg_delta_pid_target[3][3] = { }; //x, z, delta yaw
static struct timeval lg_delta_pid_time[3] = { };

static int lg_ack_command_id = 0;
static int lg_command_id = 0;
static char lg_command[1024] = { };

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

static int picam360_driver_xmp(char *buff, int buff_len, float light0_value,
		float light1_value, float motor0_value, float motor1_value,
		float motor2_value, float motor3_value) {
	int xmp_len = 0;

	xmp_len = 0;
	buff[xmp_len++] = 0xFF;
	buff[xmp_len++] = 0xE1;
	buff[xmp_len++] = 0; // size MSB
	buff[xmp_len++] = 0; // size LSB
	xmp_len += sprintf(buff + xmp_len, "http://ns.adobe.com/xap/1.0/");
	buff[xmp_len++] = '\0';
	xmp_len += sprintf(buff + xmp_len, "<?xpacket begin=\"﻿");
	buff[xmp_len++] = 0xEF;
	buff[xmp_len++] = 0xBB;
	buff[xmp_len++] = 0xBF;
	xmp_len += sprintf(buff + xmp_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-capture rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	{
		xmp_len +=
				sprintf(buff + xmp_len,
						"<picam360_driver"
								" light0_value=\"%f\" light1_value=\"%f\""
								" motor0_value=\"%f\" motor1_value=\"%f\" motor2_value=\"%f\" motor3_value=\"%f\"",
						light0_value, light1_value, motor0_value, motor1_value,
						motor2_value, motor3_value);
		if (lg_ack_command_id != lg_command_id) {
			xmp_len += sprintf(buff + xmp_len,
					" command_id=\"%d\" command=\"%s\"", lg_command_id,
					lg_command);
		}
		sprintf(buff + xmp_len, " />");
	}
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

static void parse_xml(char *xml) {
//	char *q_str = NULL;
//	q_str = strstr(xml, "<quaternion ");
//	if (q_str) {
//		VECTOR4D_T quat = { .ary = { 0, 0, 0, 1 } };
//		sscanf(q_str, "<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />",
//				&quat.x, &quat.y, &quat.z, &quat.w);
//
//		lg_plugin_host->set_camera_quaternion(-1, quat);
//	}
//	q_str = strstr(xml, "<compass ");
//	if (q_str) {
//		VECTOR4D_T compass = { .ary = { 0, 0, 0, 1 } };
//		sscanf(q_str, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />", &compass.x,
//				&compass.y, &compass.z);
//
//		lg_plugin_host->set_camera_compass(compass);
//
//		if (lg_is_compass_calib) {
//			q_str = strstr(xml, "<compass_min ");
//			if (q_str) {
//				sscanf(q_str, "<compass_min x=\"%f\" y=\"%f\" z=\"%f\" />",
//						&lg_compass_min[0], &lg_compass_min[1],
//						&lg_compass_min[2]);
//			}
//			q_str = strstr(xml, "<compass_max ");
//			if (q_str) {
//				sscanf(q_str, "<compass_max x=\"%f\" y=\"%f\" z=\"%f\" />",
//						&lg_compass_max[0], &lg_compass_max[1],
//						&lg_compass_max[2]);
//			}
//		}
//	}
//	q_str = strstr(xml, "<temperature ");
//	if (q_str) {
//		float temperature;
//		sscanf(q_str, "<temperature v=\"%f\" />", &temperature);
//
//		lg_plugin_host->set_camera_temperature(temperature);
//	}
//	q_str = strstr(xml, "<bandwidth ");
//	if (q_str) {
//		float bandwidth;
//		sscanf(q_str, "<bandwidth v=\"%f\" />", &bandwidth);
//		lg_bandwidth = bandwidth;
//	}
//	{
//		int offset = 0;
//		do {
//			q_str = strstr(xml + offset, "<video_info ");
//			offset = (unsigned long) q_str - (unsigned long) xml + 1;
//			if (q_str) {
//				int id;
//				float fps;
//				int frameskip;
//				sscanf(q_str,
//						"<video_info id=\"%d\" fps=\"%f\" frameskip=\"%d\" />",
//						&id, &fps, &frameskip);
//				if (id >= 0 && id < NUM_OF_CAM) {
//					lg_fps[id] = fps;
//					lg_frameskip[id] = frameskip;
//				}
//			}
//		} while (q_str);
//	}
//	q_str = strstr(xml, "<ack_command_id ");
//	if (q_str) {
//		sscanf(q_str, "<ack_command_id v=\"%d\" />", &lg_ack_command_id);
//	}
}

//static void status_handler(unsigned char *data, int data_len) {
//	if (data[0] == 0xFF && data[1] == 0xE1) { //xmp
//		int xmp_len = 0;
//		xmp_len = ((unsigned char*) data)[2] << 8;
//		xmp_len += ((unsigned char*) data)[3];
//
//		char *xml = (char*) data + strlen((char*) data) + 1;
//		parse_xml(xml);
//	}
//}

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
void *transmit_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "DA TRANSMIT");

	int xmp_len = 0;
	int buff_size = 4096;
	char buff[buff_size];
	static struct timeval last_time = { };
	gettimeofday(&last_time, NULL);
	while (1) {
		struct timeval time = { };
		gettimeofday(&time, NULL);
		struct timeval diff;
		timersub(&time, &last_time, &diff);
		float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
		//cal
		if (!lg_lowlevel_control) {
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
					printf("target  : %f, %f, %f\n", x * 180 / M_PI,
							y * 180 / M_PI, z * 180 / M_PI);
				}
				VECTOR4D_T quat = lg_plugin_host->get_quaternion();
				//quat = quaternion_multiply(quat, quaternion_get_from_z(M_PI)); //mpu offset
				if (lg_debugdump) {
					quaternion_get_euler(quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
					printf("vehicle : %f, %f, %f\n", x * 180 / M_PI,
							y * 180 / M_PI, z * 180 / M_PI);
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
				lg_delta_pid_target[2][0] = sub_angle(lg_yaw_diff, last_yaw)
						/ 180; // delta yaw [-1:1]

				timersub(&lg_delta_pid_time[0], &lg_delta_pid_time[1], &diff);
				diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
				diff_sec = MAX(MIN(diff_sec, 1.0), 0.001);

				for (int k = 0; k < 3; k++) {
					float p_value = lg_p_gain
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
						lg_delta_pid_target[k][j] =
								lg_delta_pid_target[k][j - 1];
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
						motor_pid_value[i] += power_calib
								* motor_pid_gain[k][i];
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
		} // end of !low_motor_control

		xmp_len = picam360_driver_xmp(buff, sizeof(buff), lg_light_value[0],
				lg_light_value[1], lg_motor_value[0], lg_motor_value[1],
				lg_motor_value[2], lg_motor_value[3]);

		rtp_sendpacket((unsigned char*) buff, xmp_len, PT_CMD);

		last_time = time;
		usleep(100 * 1000); //less than 10Hz
	} // end of while
}

static int command_handler(void *user_data, const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_value", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			lg_light_value[0] = value;
			lg_light_value[1] = value;
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

	sprintf(lg_command, "picam360_driver.save");
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

	pthread_t transmit_thread;
	pthread_create(&transmit_thread, NULL, transmit_thread_func, (void*) NULL);
}

#define MAX_INFO_LEN 1024
static wchar_t lg_info[MAX_INFO_LEN];
static wchar_t *get_info(void *user_data) {
	lg_info[0] = L'\0';
	return lg_info;
}


void create_rov_driver(PLUGIN_HOST_T *plugin_host, PLUGIN_T **_plugin) {
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
