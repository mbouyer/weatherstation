/*
 *  Copyright (c) 2022 Manuel Bouyer.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "events_buttons.h"

#include <pthread.h>
#include <err.h>
#include <unistd.h>

static pthread_mutex_t eventsi_mtx;
static int meteo_buttons_ev;

static pthread_t eventsi_t;
static void * meteo_input(void *);

void
meteo_buttons_init(void)
{
	if (pthread_mutex_init(&eventsi_mtx, NULL) != 0) {
		err(1, "meteoi mutex init failed");
	}
	meteo_buttons_ev = 0;
	if (pthread_create(&eventsi_t, NULL, meteo_input, NULL) != 0) {
		err(1, "pthread_create(meteo_input) failed");
	}
}

static void *
meteo_input(void *a)
{
	while (1) {
		sleep(1);
	}
}

bool
meteo_buttons_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
	static int last_button = 0;

	data->state = LV_INDEV_STATE_REL;
	pthread_mutex_lock(&eventsi_mtx);
	for (int i = 0; i < 4; i++) {
		if (meteo_buttons_ev & (1 << i)) {
			last_button = i;
			data->state = LV_INDEV_STATE_PR;
			break;
		}
	}
	data->btn_id = last_button;
	pthread_mutex_unlock(&eventsi_mtx);
	return false;
}
