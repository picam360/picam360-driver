/**
 * picam360-driver @ picam360 project
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>

#include "rtp.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PT_CAM_BASE 110

static bool lg_cam_run = false;
pthread_t lg_cam0_thread;
pthread_t lg_cam1_thread;

static void *camx_thread_func(void* arg) {
	int cam_num = (int) arg;
	char buff[RTP_MAXPACKETSIZE];
	int buff_size = RTP_MAXPACKETSIZE;
	sprintf(buff, "cam%d", cam_num);
	int camd_fd = open(buff, O_RDONLY);
	if (camd_fd < 0) {
		return NULL;
	}
	while (lg_cam_run) {
		int data_len = read(camd_fd, buff, buff_size);

		// send the packet
		rtp_sendpacket((unsigned char*)buff, data_len, PT_CAM_BASE + cam_num);
	}
	close(camd_fd);
	return NULL;
}
static bool is_init = false;
void init_video() {
	if (is_init) {
		return;
	}
	is_init = true;

	lg_cam_run = true;
	pthread_create(&lg_cam0_thread, NULL, camx_thread_func, (void*) 0);
	pthread_create(&lg_cam1_thread, NULL, camx_thread_func, (void*) 1);
}
void deinit_video() {
	if (!is_init) {
		return;
	}

	lg_cam_run = false;
	pthread_join(lg_cam0_thread, NULL);
	pthread_join(lg_cam1_thread, NULL);

	is_init = false;
}
