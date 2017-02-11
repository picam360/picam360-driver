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


int main(int argc, char *argv[]) {
	bool is_init = false;
	int marker = 0;
	int buff_size = 4096;
	char buff[buff_size];
	while (1) {
		int inj_len = 0;
		char inj[buff_size];
		int data_len = read(STDIN_FILENO, buff, buff_size);
		int inj_pos = 0;
		for (int i = 0; i < data_len; i++) {
			if (marker) {
				marker = 0;
				if (buff[i] == 0xD8) { //SOI
					if(is_init) {
						inj_pos = i + 1;
						ms_update();

						inj_len = 0;
						inj[inj_len++] = 0xFF;
						inj[inj_len++] = 0xE1;
						inj[inj_len++] = 0; // size MSB
						inj[inj_len++] = 0; // size LSB
						inj_len += sprintf(inj + inj_len, "http://ns.adobe.com/xap/1.0/");
						inj[inj_len++] = '\0';
						inj_len += sprintf(inj + inj_len, "<?xpacket begin=\"﻿");
						inj[inj_len++] = 0xEF;
						inj[inj_len++] = 0xBB;
						inj[inj_len++] = 0xBF;
						inj_len += sprintf(inj + inj_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
						inj_len += sprintf(inj + inj_len, "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"attitude-injector rev1\">");
						inj_len += sprintf(inj + inj_len, "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
						inj_len += sprintf(inj + inj_len, "<rdf:Description xmlns:GPano=\"http://ns.google.com/photos/1.0/panorama/\" rdf:about=\"\">");
						inj_len += sprintf(inj + inj_len, "<GPano:PoseHeadingDegrees>%f</GPano:PoseHeadingDegrees>", ypr[YAW]);
						inj_len += sprintf(inj + inj_len, "<GPano:PosePitchDegrees>%f</GPano:PosePitchDegrees>", ypr[PITCH]);
						inj_len += sprintf(inj + inj_len, "<GPano:PoseRollDegrees>%f</GPano:PoseRollDegrees>", ypr[ROLL]);
						inj_len += sprintf(inj + inj_len, "<Picam360:Quaternion>%f,%f,%f,%f</Picam360:Quaternion>", quatanion[0], quatanion[1], quatanion[2], quatanion[3]);
						inj_len += sprintf(inj + inj_len, "</rdf:Description>");
						inj_len += sprintf(inj + inj_len, "</rdf:RDF>");
						inj_len += sprintf(inj + inj_len, "</x:xmpmeta>");
						inj_len += sprintf(inj + inj_len, "<?xpacket end=\"w\"?>");
						inj[inj_len++] = '\0';
						inj[2] = ((inj_len - 2) >> 8) & 0xFF; // size MSB
						inj[3] = (inj_len - 2) & 0xFF; // size LSB
					}
				}
				if (buff[i] == 0xD9) { //EOI
					if(!is_init) {
						is_init = true;
						ms_open();
					}
				}
			} else if (buff[i] == 0xFF) {
				marker = 1;
			}
		}
		if(is_init) {
			if(inj_pos > 0) {
				write(STDOUT_FILENO, buff, inj_pos);
			}
			if(inj_len > 0) {
				write(STDOUT_FILENO, inj, inj_len);
			}
			if(inj_pos < data_len) {
				write(STDOUT_FILENO, buff + inj_pos, data_len - inj_pos);
			}
		}
	}

	return 0;
}
