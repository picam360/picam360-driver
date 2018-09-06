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
#ifdef ENABLE_V4L2
#include "v4l2_handler.h"
#endif

#ifdef __cplusplus
}
#endif

#ifdef _WIN64
   //define something for Windows (64-bit)
#elif _WIN32
   //define something for Windows (32-bit)
#elif __APPLE__
    #include "TargetConditionals.h"
    #if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
        // define something for simulator
    #elif TARGET_OS_IPHONE
        // define something for iphone
    #else
        #define TARGET_OS_OSX 1
        // define something for OSX
		#define pthread_setname_np(a, b) pthread_setname_np(b)
    #endif
#elif __linux
    // linux
#elif __unix // all unices not caught above
    // Unix
#elif __posix
    // POSIX
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PT_CAM_BASE 110

#define NUM_OF_CAM 2

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
		num_of_bytes = 0;
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
	int num_of_bytes;
	std::list<_PACKET_T *> packets;
	pthread_mutex_t packets_mlock;
	MREVENT_T packet_ready;
};
class _SENDFRAME_ARG_T {
public:
	_SENDFRAME_ARG_T() {
		vstream_type = VIDEO_STREAM_TYPE_NONE;
		memset(vstream_filepath, 0, sizeof(vstream_filepath));
		cam_run = false;
		cam_num = 0;
		cam_width = 0;
		cam_height = 0;
		cam_fps = 0;
		rtp = NULL;
		skip_frame = 0;
		framecount = 0;
		recieved_framecount = 0;
		fps = 0;
		frameskip = 0;
		pthread_mutex_init(&frames_mlock, NULL);
		mrevent_init(&frame_ready);
	}
	enum VIDEO_STREAM_TYPE vstream_type;
	char vstream_filepath[256];
	bool cam_run;
	int cam_num;
	int cam_width;
	int cam_height;
	int cam_fps;
	RTP_T *rtp;
	int skip_frame;
	unsigned int framecount;
	unsigned int recieved_framecount;
	float fps;
	int frameskip;
	std::list<_FRAME_T *> frames;
	pthread_mutex_t frames_mlock;
	MREVENT_T frame_ready;
	pthread_t cam_thread;
	void *user_data;
};

static _SENDFRAME_ARG_T *lg_send_frame_arg[NUM_OF_CAM] = { };

static VIDEO_MJPEG_XMP_CALLBACK lg_video_mjpeg_xmp_callback = NULL;

void set_video_mjpeg_xmp_callback(VIDEO_MJPEG_XMP_CALLBACK callback) {
	lg_video_mjpeg_xmp_callback = callback;
}

static void *sendframe_thread_func(void* arg) {
	pthread_setname_np(pthread_self(), "SEND");
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	int last_framecount = send_frame_arg->framecount;
	struct timeval last_time = { };
	gettimeofday(&last_time, NULL);
	while (send_frame_arg->cam_run) {
		int res = mrevent_wait(&send_frame_arg->frame_ready, 100 * 1000);
		if (res != 0) {
			continue;
		}
		_FRAME_T *frame;
		pthread_mutex_lock(&send_frame_arg->frames_mlock);
		while (1) {
			frame = *(send_frame_arg->frames.begin());
			send_frame_arg->frames.pop_front();
			send_frame_arg->framecount++;
			if (send_frame_arg->frames.empty()) {
				mrevent_reset(&send_frame_arg->frame_ready);
				break;
			}
			send_frame_arg->frameskip++;
			delete frame; //skip frame
		}
		pthread_mutex_unlock(&send_frame_arg->frames_mlock);
		while (send_frame_arg->cam_run) {
			{ //fps
				struct timeval time = { };
				gettimeofday(&time, NULL);

				struct timeval diff;
				timersub(&time, &last_time, &diff);
				float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
				if (diff_sec > 1.0) {
					float tmp = (float) (send_frame_arg->framecount - last_framecount) / diff_sec;
					float w = diff_sec / 10;
					send_frame_arg->fps = send_frame_arg->fps * (1.0 - w) + tmp * w;

					last_framecount = send_frame_arg->framecount;
					last_time = time;
				}
			}
			int res = mrevent_wait(&frame->packet_ready, 100 * 1000);
			if (res != 0) {
				continue;
			}
			_PACKET_T *packet = NULL;
			pthread_mutex_lock(&frame->packets_mlock);
			if (!frame->packets.empty()) {
				packet = *(frame->packets.begin());
				frame->packets.pop_front();
			}
			if (frame->packets.empty()) {
				mrevent_reset(&frame->packet_ready);
			}
			pthread_mutex_unlock(&frame->packets_mlock);
			if (packet == NULL) {
				fprintf(stderr, "packet is null\n");
				continue;
			}
			// send the packet
			rtp_sendpacket(send_frame_arg->rtp, (unsigned char*) packet->data, packet->len,
			PT_CAM_BASE + send_frame_arg->cam_num);
			if (packet->eof) {
				rtp_flush(send_frame_arg->rtp);
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

static void *camx_thread_func_fifo(void* arg) {
	pthread_setname_np(pthread_self(), "CAM");
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;
	unsigned char buff[RTP_MAXPAYLOADSIZE];
	int buff_size = RTP_MAXPAYLOADSIZE;

	int camd_fd = open(send_frame_arg->vstream_filepath, O_RDONLY);
	if (camd_fd < 0) {
		return NULL;
	}

	pthread_t sendframe_thread;
	pthread_create(&sendframe_thread, NULL, sendframe_thread_func, (void*) send_frame_arg);

	int stat_ave = 0;
	int stat_sigma = 0;
	int stat_n = 0;
	float stat_ave_sum = 0;
	float stat_ave2_sum = 0;

	int marker = 0;
	int soicount = 0;
	_FRAME_T *active_frame = NULL;
	while (send_frame_arg->cam_run) {
		int soi_pos = INT_MIN;
		int data_len = read(camd_fd, buff, buff_size);

		for (int i = 0; i < data_len; i++) {
			if (marker) {
				marker = 0;
				if (buff[i] == 0xD8) { //SOI
					if ((send_frame_arg->recieved_framecount++ % (send_frame_arg->skip_frame + 1)) != 0) {
						continue;
					}
					if (soicount == 0) {
						soi_pos = (i - 1);
						active_frame = new _FRAME_T;

						pthread_mutex_lock(&send_frame_arg->frames_mlock);
						send_frame_arg->frames.push_back(active_frame);
						pthread_mutex_unlock(&send_frame_arg->frames_mlock);
						mrevent_trigger(&send_frame_arg->frame_ready);
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
							if (packet->len > RTP_MAXPAYLOADSIZE) {
								fprintf(stderr, "packet length exceeded. %d\n", packet->len);
								packet->len = RTP_MAXPAYLOADSIZE;
							}
							memcpy(packet->data, buff + soi_pos, packet->len);
						} else {
							packet->len = i + 1;
							if (packet->len > RTP_MAXPAYLOADSIZE) {
								fprintf(stderr, "packet length exceeded. %d\n", packet->len);
								packet->len = RTP_MAXPAYLOADSIZE;
							}
							memcpy(packet->data, buff, packet->len);
						}

						{ //stat
							float num_of_bytes = active_frame->num_of_bytes + packet->len;
							stat_n++;
							stat_ave_sum += num_of_bytes;
							stat_ave2_sum += num_of_bytes * num_of_bytes;
							if (stat_n > 100) {
								float ave = (float) stat_ave_sum / (float) stat_n;
								float ave2 = (float) stat_ave2_sum / (float) stat_n;
								stat_ave = (int) ave;
								stat_sigma = (int) sqrt(ave2 - ave * ave);
								stat_sigma += stat_ave / 15; //offset 1/5
								stat_n = 100;
								stat_ave_sum = ave * stat_n;
								stat_ave2_sum = ave2 * stat_n;
								if ((float) num_of_bytes > (float) stat_ave * 1.5) {
									stat_n = 0;
									stat_ave_sum = 0;
									stat_ave2_sum = 0;
									stat_ave = 0;
									stat_sigma = 0;
									printf("reset stat\n");
								}
							}
						}

						pthread_mutex_lock(&active_frame->packets_mlock);
						active_frame->num_of_bytes += packet->len;
						active_frame->packets.push_back(packet);
						mrevent_trigger(&active_frame->packet_ready);
						pthread_mutex_unlock(&active_frame->packets_mlock);

						active_frame = NULL;
					}
				}
			} else if (buff[i] == 0xFF) {
				marker = 1;
			} else if (active_frame != NULL && stat_sigma != 0) {
				int num_of_bytes = active_frame->num_of_bytes + (data_len - i);
				if ((stat_ave - num_of_bytes) > 3 * stat_sigma) { //skip
					break;
				}
			}
		}
		if (active_frame != NULL) {
			_PACKET_T *packet = new _PACKET_T;
			if (soi_pos >= 0) { //soi
				if (lg_video_mjpeg_xmp_callback) { //xmp injection
					packet->len = data_len - soi_pos;
					packet->data[0] = 0xFF;
					packet->data[1] = 0xD8; //soi marker
					int xmp_len = lg_video_mjpeg_xmp_callback(packet->data + 2,
					RTP_MAXPAYLOADSIZE - 2, send_frame_arg->cam_num);
					if (packet->len + xmp_len > RTP_MAXPAYLOADSIZE) { // split packet
						packet->len = 2 + xmp_len;

						pthread_mutex_lock(&active_frame->packets_mlock);
						active_frame->num_of_bytes += packet->len;
						active_frame->packets.push_back(packet);
						pthread_mutex_unlock(&active_frame->packets_mlock);
						mrevent_trigger(&active_frame->packet_ready);

						soi_pos += 2; //increment soi marker
						if (data_len - soi_pos > 0) {
							packet = new _PACKET_T;
							packet->len = data_len - soi_pos;

							memcpy(packet->data, buff + soi_pos, packet->len);
						} else {
							packet = NULL;
						}
					} else {
						soi_pos += 2; //increment soi marker
						memcpy(packet->data + 2 + xmp_len, buff + soi_pos, packet->len - 2);
						packet->len += xmp_len;
					}
				} else {
					packet->len = data_len - soi_pos;
					if (packet->len > RTP_MAXPAYLOADSIZE) {
						fprintf(stderr, "packet length exceeded. %d\n", packet->len);
						packet->len = RTP_MAXPAYLOADSIZE;
					}
					memcpy(packet->data, buff + soi_pos, packet->len);
				}
			} else {
				packet->len = data_len;
				if (packet->len > RTP_MAXPAYLOADSIZE) {
					fprintf(stderr, "packet length exceeded. %d\n", packet->len);
					packet->len = RTP_MAXPAYLOADSIZE;
				}
				memcpy(packet->data, buff, packet->len);
			}
			if (packet) {
				pthread_mutex_lock(&active_frame->packets_mlock);
				active_frame->num_of_bytes += packet->len;
				active_frame->packets.push_back(packet);
				pthread_mutex_unlock(&active_frame->packets_mlock);
				mrevent_trigger(&active_frame->packet_ready);
			}
		}
	}
	close(camd_fd);
	return NULL;
}

#ifdef ENABLE_V4L2
static int v4l2_progress_image(const void *p, int size, void* arg) {
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;

	//remove space
	for (; size > 0;) {
		if (((unsigned char*) p)[size - 1] == 0xD9) {
			break;
		} else {
			size--;
		}
	}

	if ((send_frame_arg->recieved_framecount++ % (send_frame_arg->skip_frame + 1)) == 0 && size != 0) {
		_FRAME_T *active_frame = new _FRAME_T;

		pthread_mutex_lock(&send_frame_arg->frames_mlock);
		send_frame_arg->frames.push_back(active_frame);
		pthread_mutex_unlock(&send_frame_arg->frames_mlock);
		mrevent_trigger(&send_frame_arg->frame_ready);

		for (int i = 0; i < size;) {
			_PACKET_T *packet = new _PACKET_T;
			if (i == 0) {
				i += 2; //increment soi marker
				packet->len = 2;
				packet->data[0] = 0xFF;
				packet->data[1] = 0xD8; //soi marker
				if (lg_video_mjpeg_xmp_callback) { //xmp injection
					int xmp_len = lg_video_mjpeg_xmp_callback(packet->data + 2,
					RTP_MAXPAYLOADSIZE - 2, send_frame_arg->cam_num);
					if (packet->len + xmp_len <= RTP_MAXPAYLOADSIZE) {
						packet->len += xmp_len;
					}
				}
			}
			int len;
			if (packet->len + (size - i) > RTP_MAXPAYLOADSIZE) {
				len = RTP_MAXPAYLOADSIZE - packet->len;
			} else {
				len = (size - i);
			}
			if (len) {
				memcpy(packet->data + packet->len, (unsigned char*) p + i, len);
				packet->len += len;
				i += len;
			}
			pthread_mutex_lock(&active_frame->packets_mlock);
			active_frame->num_of_bytes += packet->len;
			if (i == size) { // active_frame->num_of_bytes is size + xmp_len
				packet->eof = true;
			}
			active_frame->packets.push_back(packet);
			mrevent_trigger(&active_frame->packet_ready);
			pthread_mutex_unlock(&active_frame->packets_mlock);
		}
	}

	return send_frame_arg->cam_run ? 1 : 0;
}

static void *camx_thread_func_v4l2(void* arg) {
	pthread_setname_np(pthread_self(), "CAM");
	_SENDFRAME_ARG_T *send_frame_arg = (_SENDFRAME_ARG_T*) arg;

	pthread_t sendframe_thread;
	pthread_create(&sendframe_thread, NULL, sendframe_thread_func, arg);

	handle_v4l2(send_frame_arg->vstream_filepath, send_frame_arg->cam_width, send_frame_arg->cam_height, send_frame_arg->cam_fps, v4l2_progress_image, arg);

	return NULL;
}
#endif

void init_video_mjpeg(int cam_num, enum VIDEO_STREAM_TYPE vstream_type, const char *vstream_filepath, int width, int height, int fps, RTP_T *rtp, void *user_data) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (lg_send_frame_arg[cam_num]) {
		return;
	}
	lg_send_frame_arg[cam_num] = new _SENDFRAME_ARG_T;
	lg_send_frame_arg[cam_num]->cam_num = cam_num;
	lg_send_frame_arg[cam_num]->vstream_type = vstream_type;
	strncpy(lg_send_frame_arg[cam_num]->vstream_filepath, vstream_filepath, sizeof(lg_send_frame_arg[cam_num]->vstream_filepath));
	lg_send_frame_arg[cam_num]->cam_width = width;
	lg_send_frame_arg[cam_num]->cam_height = height;
	lg_send_frame_arg[cam_num]->cam_fps = fps;
	lg_send_frame_arg[cam_num]->rtp = rtp;
	lg_send_frame_arg[cam_num]->user_data = user_data;

	lg_send_frame_arg[cam_num]->cam_run = true;

	void *(*start_routine)(void *);
	if (lg_send_frame_arg[cam_num]->vstream_type == VIDEO_STREAM_TYPE_FIFO) {
		start_routine = camx_thread_func_fifo;
#ifdef ENABLE_V4L2
	} else if (lg_send_frame_arg[cam_num]->vstream_type == VIDEO_STREAM_TYPE_V4L2) {
		start_routine = camx_thread_func_v4l2;
#endif
	} else {
		return;
	}
	pthread_create(&lg_send_frame_arg[cam_num]->cam_thread, NULL, start_routine, (void*) lg_send_frame_arg[cam_num]);
}
void deinit_video_mjpeg(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}

	lg_send_frame_arg[cam_num]->cam_run = false;
	pthread_join(lg_send_frame_arg[cam_num]->cam_thread, NULL);

	delete lg_send_frame_arg[cam_num];
	lg_send_frame_arg[cam_num] = NULL;
}

float video_mjpeg_get_fps(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->fps;
}
int video_mjpeg_get_frameskip(int cam_num) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return 0;
	}
	return lg_send_frame_arg[cam_num]->frameskip;
}

void video_mjpeg_set_skip_frame(int cam_num, int value) {
	cam_num = MAX(MIN(cam_num,NUM_OF_CAM-1), 0);
	if (!lg_send_frame_arg[cam_num]) {
		return;
	}
	lg_send_frame_arg[cam_num]->skip_frame = value;
}
