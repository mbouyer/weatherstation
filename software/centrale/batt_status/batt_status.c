/*
 * Copyright (c) 2021 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/if.h>
#ifdef __NetBSD__
#include <netcan/can.h>
#else
#include <linux/can.h>
#include <linux/can/raw.h>
#endif
#include "nmea2000_pgn.h"

int s;

static void
usage()
{
	fprintf(stderr, "usage: batt_status <interface>\n");
	exit(1);
}

int
main(int argc, const char *argv[])
{
	struct ifreq ifr;
	struct sockaddr_can sa;
	struct can_frame cf;
	int r;
	static struct nmea2000_dc_status_data dc_status;
	static struct nmea2000_charger_status_data charger_status;
	static struct nmea2000_battery_status_data batt_status;

	if (argc != 2) {
		usage();
	}

	if ((s = socket(AF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		err(1, "CAN socket");
	}
	strncpy(ifr.ifr_name, argv[1], IFNAMSIZ );
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		err(1, "SIOCGIFINDEX for %s", argv[1]);
	}
	sa.can_family = AF_CAN;
	sa.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		err(1, "bind socket");
	}

	while (1) {
		int n;
		uint pgn;
		r = read(s, &cf, sizeof(cf));
		if (r < 0) {
			err(1, "read from socket");
		}
		if (r == 0)
			continue;
		if (((cf.can_id >> 16) & 0xff) < 240)
			continue; /* non-broadcast frame */
		pgn = (cf.can_id >> 8) & 0x1ffff;
		switch(pgn) {
		case NMEA2000_DC_STATUS:
		{
			char *d = (void *)&dc_status;
			if ((cf.data[0] & 0x7) == 0)
				memcpy(&d[0], &cf.data[2], 6);
			else 
				memcpy(&d[6], &cf.data[1],
				    sizeof(dc_status) - 6);
			break;
		}
		case NMEA2000_CHARGER_STATUS:
			memcpy(&charger_status, &cf.data[0],
			    sizeof(charger_status));
			break;
		case NMEA2000_BATTERY_STATUS:
		{
			int temp;
			memcpy(&batt_status, &cf.data[0],
			    sizeof(batt_status));
			temp = batt_status.temp - 27300;
			printf("BATT %d.%02dV %d.%01dmA %d.%02d dC",
			    batt_status.voltage/100, batt_status.voltage%100,
			    batt_status.current/10, abs(batt_status.current%10),
			    temp/100, temp%100
			    );
			printf(" charger ");
			switch(charger_status.op_state) {
			case CHARGER_STATE_NOCHRG:
				printf("NOCHRG");
				break;
			case CHARGER_STATE_BULK:
				printf("BULK");
				break;
			case CHARGER_STATE_ABS:
				printf("ABS");
				break;
			case CHARGER_STATE_FLOAT:
				printf("FLOAT");
				break;
			case CHARGER_STATE_DIS:
				printf("DIS");
				break;
			case CHARGER_STATE_FAULT:
				printf("FAULT");
				break;
			case CHARGER_STATE_UNAVAIL:
				printf("UNAVAIL");
				break;
			default:
				printf("state %d", charger_status.op_state);
			}
			printf(" DC ");
			switch(dc_status.type) {
			case DCSTAT_TYPE_BATT:
				printf("BATT soc %d\n", dc_status.soc);
				break;
			case DCSTAT_TYPE_CONV:
				printf("CONV soh %d\n", dc_status.soh);
				break;
			}
			break;
		}
		}
	}
	exit(0);
}
