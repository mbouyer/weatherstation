/*
 * Copyright (c) 2019 Manuel Bouyer
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

#ifndef NMEA2000_DEFS_H_
#define NMEA2000_DEFS_H_

class nmea2000_desc {
    public:
        const char *descr;
	const bool isuser;
	const int pgn;
	bool enabled;

	inline nmea2000_desc(const char *desc, bool isuser, int pgn) :
	    descr(desc), isuser(isuser), pgn(pgn) {enabled = false;}
	virtual ~nmea2000_desc() {};
};

#define NMEA2000_PRIORITY_HIGH          0
#define NMEA2000_PRIORITY_SECURITY      1
#define NMEA2000_PRIORITY_CONTROL       3
#define NMEA2000_PRIORITY_REQUEST       6
#define NMEA2000_PRIORITY_INFO          6
#define NMEA2000_PRIORITY_ACK           6
#define NMEA2000_PRIORITY_LOW           7

#define NMEA2000_ADDR_GLOBAL    255
#define NMEA2000_ADDR_NULL      254
#define NMEA2000_ADDR_MAX       251

#define NMEA2000_INDUSTRY_GROUP 4

#define ISO_ADDRESS_CLAIM	60928U
#define ISO_REQUEST		59904U

#define NMEA2000_DATETIME	129033U

#define NMEA2000_WIND_DATA	130306U
#define NMEA2000_ENV_PARAM 	130311U


#endif
