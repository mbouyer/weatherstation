/*
 *  Copyright (c) 2020 Manuel Bouyer.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/atomic.h>
#include "lvgl/lvgl.h"
#include "meteo_pages.h"
#include "meteo_data.h"
#include "hal.h"



static lv_obj_t *temp_value[NTEMP];
static lv_obj_t *hum_value[NTEMP];
static lv_obj_t *baro_value;

static lv_task_t *set_temp_task[NTEMP];
static lv_task_t *set_baro_task;

static volatile double received_temp[NTEMP];
static volatile double received_hum[NTEMP];
static volatile bool temp_valid[NTEMP];
static volatile uint temp_gen[NTEMP];
static volatile double received_baro;
static volatile bool baro_valid;
static volatile uint baro_gen;


static void meteo_create_main(void);

meteo_page_t mpage_main = {
	meteo_create_main,
	activate_page,
	"main",
	true,
	false,
	NULL
};

#define meteo_page (mpage_main.mpage_page)

static void
meteo_set_temp_timeout(lv_task_t *task)
{
	intptr_t idx = (intptr_t)task->user_data;
	lv_label_set_text(temp_value[idx], " --.-" DEGSTR);
	lv_label_set_text(hum_value[idx], "--%");
	temp_valid[idx] = false;
}

void
meteo_set_temp(int idx, double _temp, double _hum, bool valid)
{
	temp_gen[idx]++;
	membar_producer();
	received_temp[idx] = _temp;
	received_hum[idx] = _hum;
	temp_valid[idx] = valid;
	membar_producer();
	temp_gen[idx]++;
}

static void
meteo_temp_update(int idx)
{
	static uint gen = 0;
	char buf[7];
	double _temp;
	int _hum;
	bool valid;

	if (gen == temp_gen[idx])
		return;

	while (temp_gen[idx] != gen || (gen % 1) != 0) {
		gen = temp_gen[idx];
		membar_consumer(); 
		_temp = received_temp[idx];
		_hum = (int)(received_hum[idx] + 0.5);
		valid = temp_valid[idx];
		membar_consumer();    
	}

	if (valid) {
		snprintf(buf, 7, "%3.1f" DEGSTR, _temp );
		lv_label_set_text(temp_value[idx], buf);
		snprintf(buf, 7, "%2d%%", _hum );
		lv_label_set_text(hum_value[idx], buf);
	} else {
		lv_label_set_text(temp_value[idx], " --.-" DEGSTR);
		lv_label_set_text(hum_value[idx], "--%");
	}
	lv_task_reset(set_temp_task[idx]);
}

static void
meteo_set_baro_timeout(lv_task_t *task)
{
	lv_label_set_text(baro_value, "----.-hPa");
	baro_valid = false;
}

void
meteo_set_baro(double value, bool valid)
{
	baro_gen++;
	membar_producer();
	received_baro = value;
	baro_valid = valid;
	membar_producer();
	baro_gen++;
}

static void
meteo_baro_update(void)
{
	static uint gen = 0;
	char buf[12];
	double value;
	bool valid;

	if (gen == baro_gen)
		return;

	while (baro_gen != gen || (gen % 1) != 0) {
		gen = baro_gen;
		membar_consumer(); 
		value = received_baro;
		valid = baro_valid;
		membar_consumer();    
	}

	if (valid) {
		snprintf(buf, 12, "%4.1fhPa", value);
		lv_label_set_text(baro_value, buf);
	} else {
		lv_label_set_text(baro_value, "----.-hPa");
	}
	lv_task_reset(set_baro_task);
}

static void
meteo_main_action(lv_obj_t * obj, lv_event_t event)
{
        printf("main event ");   
	print_ev(event);
	printf("\n");
}

static void
meteo_create_main()
{
	int w, h;

	for (int i = 0; i < NTEMP; i++) {
		temp_value[i] = lv_label_create(meteo_page, NULL);
		lv_label_set_style(temp_value[i], LV_LABEL_STYLE_MAIN,
		    &style_large_text);
		lv_label_set_text(temp_value[i], " --.-" DEGSTR);
		w = lv_obj_get_width(temp_value[i]);
		h = lv_obj_get_height(temp_value[i]);
		lv_obj_align(temp_value[i], NULL, LV_ALIGN_OUT_TOP_LEFT,
		    20, h + 10 + h * i);
		temp_valid[i] = false;

		hum_value[i] = lv_label_create(meteo_page, NULL);
		lv_label_set_style(hum_value[i], LV_LABEL_STYLE_MAIN,
		    &style_large_text);
		lv_label_set_text(hum_value[i], "--%");
		w = lv_obj_get_width(hum_value[i]);
		h = lv_obj_get_height(hum_value[i]);
		lv_obj_align(hum_value[i], temp_value[i],
		    LV_ALIGN_OUT_RIGHT_MID, 5, 0);
		set_temp_task[i] = lv_task_create(
		    meteo_set_temp_timeout, 10000, LV_TASK_PRIO_MID, (void *)(intptr_t)i);
	}
	baro_value = lv_label_create(meteo_page, NULL);
	lv_label_set_style(baro_value, LV_LABEL_STYLE_MAIN,
	    &style_large_text);
	lv_label_set_text(baro_value, "----.-hPa");
	w = lv_obj_get_width(baro_value);
	h = lv_obj_get_height(baro_value);
	lv_obj_align(baro_value, NULL, LV_ALIGN_OUT_TOP_LEFT,
	    20, h + 10 + h * NTEMP);
	baro_valid = false;
	set_baro_task = lv_task_create(
	    meteo_set_baro_timeout, 10000, LV_TASK_PRIO_MID, NULL);

	lv_obj_set_click(meteo_page, 1);
	lv_obj_set_event_cb(meteo_page, meteo_main_action);

}

void
meteo_update_main(void)
{
	for (int i = 0; i < NTEMP; i++) {
		meteo_temp_update(i);
	}
	meteo_baro_update();
}
