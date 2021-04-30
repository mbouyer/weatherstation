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

/*
 * thermistor constants array
 * It has to be in its own file because of sdcc bug 
 */

#include <stdlib.h> 
#include "ntc_tab.h"
const struct temp_val const temps[] = {
	{
		.temp = 33300,
		.val = 3308, /* R = 2374 Ohm */
	},
	{
		.temp = 32800,
		.val = 3184, /* R = 2862 Ohm */
	},
	{
		.temp = 32300,
		.val = 3040, /* R = 3469 Ohm */
	},
	{
		.temp = 31800,
		.val = 2876, /* R = 4231 Ohm */
	},
	{
		.temp = 31300,
		.val = 2696, /* R = 5192 Ohm */
	},
	{
		.temp = 30800,
		.val = 2492, /* R = 6414 Ohm */
	},
	{
		.temp = 30300,
		.val = 2276, /* R = 7980 Ohm */
	},
	{
		.temp = 29800,
		.val = 2048, /* R = 10000 Ohm */
	},
	{
		.temp = 29300,
		.val = 1808, /* R = 12629 Ohm */
	},
	{
		.temp = 28800,
		.val = 1568, /* R = 16079 Ohm */
	},
	{
		.temp = 28300,
		.val = 1336, /* R = 20646 Ohm */
	},
	{
		.temp = 27800,
		.val = 1112, /* R = 26750 Ohm */
	},
	{
		.temp = 27300,
		.val = 908, /* R = 34989 Ohm */
	},
	{
		.temp = 26800,
		.val = 728, /* R = 46227 Ohm */
	},
	{
		.temp = 26300,
		.val = 568, /* R = 61723 Ohm */
	},
	{
		.temp = 25800,
		.val = 436, /* R = 83343 Ohm */
	},
	{
		.temp = 25300,
		.val = 328, /* R = 113880 Ohm */
	},
	{
		.temp = 0,
		.val = 0, /* OEL */
	}
};
