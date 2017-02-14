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

#include "picam360_driver.h"
#include "MotionSensor.h"

bool init() {
	ms_open();
	return true;
}

static float lg_quat[4];

float *get_quatanion_mpu9250() {
	ms_update();
	lg_quat[0] = -quatanion[1];
	lg_quat[1] = -quatanion[3];
	lg_quat[2] = quatanion[2];
	lg_quat[3] = quatanion[0];
	return lg_quat;
}

int xmp(char *buff, int buff_len) {
	int xmp_len = 0;

	float *quat = get_quatanion_mpu9250();

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
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-drive rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:Description rdf:about=\"\">");
	xmp_len += sprintf(buff + xmp_len,
					"<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />",
					quat[0], quat[1], quat[2], quat[3]);
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

int main(int argc, char *argv[]) {
	bool succeeded;

	succeeded = init();
	if (!succeeded) {
		perror("An error happen in init().");
		exit(-1);
	}
	int xmp_len = 0;
	int buff_size = 4096;
	char buff[buff_size];
	while (1) {

		xmp_len = xmp(buff, buff_size);
		write(STDOUT_FILENO, buff, xmp_len);

		usleep(10 * 1000); //less than 100Hz
	}

	return 0;
}
