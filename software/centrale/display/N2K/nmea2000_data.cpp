/*
 * Copyright (c) 2022 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the distribution.
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

#include "NMEA2000.h"
#include "nmea2000_defs_rx.h"
#include "nmea2000_defs_tx.h"

#include <lv_meteo/meteo_data.h>

bool nmea2000_wind_rx::handle(const nmea2000_frame &f)
{
	uint16_t speed = f.frame2uint16(1);
	uint16_t dir = f.frame2uint16(3);
	uint8_t ref = f.frame2uint16(5);

	return true;
}

bool nmea2000_env_rx::handle(const nmea2000_frame &f)
{
	uint8_t src = f.frame2uint8(1);
	uint16_t temp = f.frame2uint16(2);
	uint16_t hum = f.frame2uint16(4);
	uint16_t press = f.frame2uint16(6);

	if (temp != 0xffff) {
		switch(src & 0x3f) {
		case 0: // sea
			break;
		case 1: // outside
			meteo_set_temp(TEMP_EXT,
			    ((double)temp / 100.0) - 273.15,
			    (double)hum / 250.0,
			    true);
			break;
		case 2: // inside
			meteo_set_temp(TEMP_INT,
			    ((double)temp / 100.0) - 273.15,
			    (double)hum / 250.0,
			    true);
			break;
		default:
			break;
		}
	}
	return true;
}

bool nmea2000_env_tx::senddata(double _temp, double _hum, double _press)
{
	bool ret;
	static uint8_t sid;
	uint8_t src = 0x02; // inside
	uint16_t temp = (_temp + 273.15) * 100;
	uint16_t hum = (_hum * 250);
	uint16_t press = _press * 10;

	sid++;
	if (sid == 0xfe)
		sid = 0;

	uint82frame(sid, 0);
	uint82frame(src, 1);
	uint162frame(temp, 2);
	uint162frame(hum, 4);
	uint162frame(press, 6);
	valid = true;
	ret = nmea2000P->send_bypgn(NMEA2000_ENV_PARAM, true);
	valid = false;
	return ret;
}

extern "C" {
bool
n2ks_env(double temp, double hum, double press)
{
        nmea2000_env_tx *f = (nmea2000_env_tx *)nmea2000P->get_frametx(nmea2000P->get_tx_bypgn(NMEA2000_ENV_PARAM));
	return f->senddata(temp, hum, press);
}       
}
