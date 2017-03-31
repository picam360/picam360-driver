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
#include <sys/time.h>
#include <list>

#include "video_mjpeg.h"
#include "rtp.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mrevent.h"

#ifdef __cplusplus
}
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PT_CAM_BASE 110

#define NUM_OF_CAM 2
static bool lg_cam_run = false;
static pthread_t lg_cam_thread[NUM_OF_CAM] = { };
static float lg_fps[NUM_OF_CAM] = { };
static int lg_frameskip[NUM_OF_CAM] = { };

#define MAX_IMAGE_SIZE (16*1024*1024) //16M
class _PACKET_T {
public:
	_PACKET_T() {
		len = 0;
		eof = false;
	}
	int len;
	char data[RTP_MAXPAYLOADSIZE];
	bool eof;
};
class _FRAME_T {
public:
	_FRAME_T() {
		pthread_mutex_init(&packets_mlock, NULL);
		mrevent_init(&packet_ready);
	}
	~_FRAME_T() {
		_FRAME_T *frame = this;
		while (!frame->packets.empty()) {
			_PACKET_T *packet;
			pthread_mutex_lock(&frame->packets_mlock);
			packet = *(frame->packets.begin());
			frame->packets.pop_front();
			if (frame->packets.empty()) {
				mrevent_reset(&frame->packet_ready);
			}
			pthread_mutex_unlock(&frame->packets_mlock);
			delete packet;
		}
	}
	std::list<_PACKET_T *> packets;
	pthread_mutex_t packets_mlock;
	MREVENT_T packet_ready;
};
class _SENDFRAME_ARG_T {
public:
	_SENDFRAME_ARG_T() {
		pthread_mutex_init(&frames_mlock, NULL);
		mrevent_init(&frame_ready);
	}
	int cam_num;
	struct timeval last_time;
	std::list<_FRAME_T *> frames;
	pthread_mutex_t frames_mlock;
	MREVENT_T frame_ready;
};

static void *sendframe_thread_func(void* arg) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	while (lg_cam_run) {
		int res = mrevent_wait(&send_frame_arg->frame_ready, 1000);
		if (res != 0) {
			continue;
		}
		_FRAME_T *frame;
		pthread_mutex_lock(&send_frame_arg->frames_mlock);
		while (1) {
			frame = *(send_frame_arg->frames.begin());
			send_frame_arg->frames.pop_front();
			if (send_frame_arg->frames.empty()) {
				mrevent_reset(&send_frame_arg->frame_ready);
				break;
			}
			lg_frameskip[send_frame_arg->cam_num]++;
			delete frame; //skip frame
		}
		pthread_mutex_unlock(&send_frame_arg->frames_mlock);
		while (lg_cam_run) {
			int res = mrevent_wait(&frame->packet_ready, 1000);
			if (res != 0) {
				continue;
			}
			_PACKET_T *packet;
			pthread_mutex_lock(&frame->packets_mlock);
			packet = *(frame->packets.begin());
			frame->packets.pop_front();
			if (frame->packets.empty()) {
				mrevent_reset(&frame->packet_ready);
			}
			pthread_mutex_unlock(&frame->packets_mlock);
			// send the packet
			rtp_sendpacket((unsigned char*) packet->data, packet->len,
			PT_CAM_BASE + send_frame_arg->cam_num);
			if (packet->eof) {
				delete packet;
				break;
			} else {
				delete packet;
			}
		}
		delete frame;
	}
	return NULL;
}

static void *camx_thread_func(void* arg) {
	int cam_num = (int) arg;
	char buff[RTP_MAXPAYLOADSIZE];
	int buff_size = RTP_MAXPAYLOADSIZE;
	sprintf(buff, "cam%d", cam_num);
	int camd_fd = open(buff, O_RDONLY);
	if (camd_fd < 0) {
		return NULL;
	}
	_SENDFRAME_ARG_T send_frame_arg;
	send_frame_arg.cam_num = cam_num;

	pthread_t sendframe_thread;
	pthread_create(&sendframe_thread, NULL, sendframe_thread_func,
			(void*) &send_frame_arg);

	gettimeofday(&send_frame_arg.last_time, NULL);

	int marker = 0;
	int soicount = 0;
	_FRAME_T *active_frame = NULL;
	while (lg_cam_run) {
		int soi_pos = INT_MIN;
		int data_len = read(camd_fd, buff, buff_size);

		for (int i = 0; i < data_len; i++) {
			if (marker) {
				marker = 0;
				if (buff[i] == 0xD8) { //SOI
					if (soicount == 0) {
						soi_pos = (i - 1);
						active_frame = new _FRAME_T;

						pthread_mutex_lock(&send_frame_arg.frames_mlock);
						send_frame_arg.frames.push_back(active_frame);
						pthread_mutex_unlock(&send_frame_arg.frames_mlock);
						mrevent_trigger(&send_frame_arg.frame_ready);
					}
					soicount++;
				}
				if (buff[i] == 0xD9 && active_frame != NULL) { //EOI
					soicount--;
					if (soicount == 0) {
						_PACKET_T *packet = new _PACKET_T;
						packet->eof = true;
						if (soi_pos >= 0) { //soi
							packet->len = (i + 1) - soi_pos;
							memcpy(packet->data, buff + soi_pos, packet->len);
						} else {
							packet->len = i + 1;
							memcpy(packet->data, buff, packet->len);
						}
						pthread_mutex_lock(&active_frame->packets_mlock);
						active_frame->packets.push_back(packet);
						pthread_mutex_unlock(&active_frame->packets_mlock);
						mrevent_trigger(&active_frame->packet_ready);

						active_frame = NULL;
						{ //fps
							struct timeval time = { };
							gettimeofday(&time, NULL);

							struct timeval diff;
							timersub(&time, &send_frame_arg.last_time, &diff);
							float diff_sec = (float) diff.tv_sec
									+ (float) diff.tv_usec / 1000000;
							float tmp = 1.0f / diff_sec;
							float frame_sec = ((
									(lg_fps[cam_num] != 0) ?
											1.0 / lg_fps[cam_num] : 0) * 0.9
									+ diff_sec * 0.1);
							lg_fps[cam_num] =
									(frame_sec != 0) ? 1.0 / frame_sec : 0;
							send_frame_arg.last_time = time;
						}
					}
				}
			} else if (buff[i] == 0xFF) {
				marker = 1;
			}
		}
		if (active_frame != NULL) {
			_PACKET_T *packet = new _PACKET_T;
			if (soi_pos >= 0) { //soi
				packet->len = data_len - soi_pos;
				memcpy(packet->data, buff + soi_pos, packet->len);
			} else {
				packet->len = data_len;
				memcpy(packet->data, buff, packet->len);
			}
			pthread_mutex_lock(&active_frame->packets_mlock);
			active_frame->packets.push_back(packet);
			pthread_mutex_unlock(&active_frame->packets_mlock);
			mrevent_trigger(&active_frame->packet_ready);
		}
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
	for (int i = 0; i < NUM_OF_CAM; i++) {
		pthread_create(&lg_cam_thread[i], NULL, camx_thread_func, (void*) i);
	}
}
void deinit_video() {
	if (!is_init) {
		return;
	}

	lg_cam_run = false;
	for (int i = 0; i < NUM_OF_CAM; i++) {
		pthread_join(lg_cam_thread[i], NULL);
	}

	is_init = false;
}

float video_get_fps(int cam_num) {
	return lg_fps[MAX(MIN(cam_num,NUM_OF_CAM-1), 0)];
}
int video_get_frameskip(int cam_num) {
	return lg_frameskip[MAX(MIN(cam_num,NUM_OF_CAM-1), 0)];
}
