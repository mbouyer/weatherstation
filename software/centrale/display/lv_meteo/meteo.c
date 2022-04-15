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
#include <sys/time.h>
#include "lvgl/lvgl.h"
#include "meteo.h"
#include "meteo_data.h"
#include "meteo_pages.h"
#include "meteo_font.h"
#include "hal.h"

lv_style_t style_large_text;
lv_style_t style_medium_text;
lv_style_t style_small_text;

static meteo_page_t *mpages[] = {             
	&mpage_main,
}; 

static void page_list(void);
static int current_page = 0;

static lv_obj_t *lv_top;
       lv_obj_t *lv_top_trs; /* transient top displays (menu, err msg, ...) */

static struct button {
	lv_obj_t *button;
	lv_obj_t *label;
} buttons[4];

void
activate_page(meteo_page_t *mpage)
{
	if (mpage->mpage_page != NULL) {
		lv_scr_load(mpage->mpage_page);
	}
}

void
deactivate_page(meteo_page_t *mpage)
{
}

static void
switch_to_page(int new)
{
	if (!mpages[new]->mpage_is_transient) {
		deactivate_page(mpages[current_page]);
		current_page = new;
	}
	mpages[new]->mpage_activate(mpages[new]);
}

void
switch_to_page_o(lv_obj_t *page)
{
	for (int i = 0; i < sizeof(mpages) / sizeof(mpages[0]); i++) {
		if (page == mpages[i]->mpage_page) {
			switch_to_page(i);
			return;
		}
	}
}

static void
disp_refresh(void *p)
{
	activate_page(mpages[current_page]);
}

void
print_ev(lv_event_t event)
{
	switch(event) {
	case LV_EVENT_PRESSED:
		printf("LV_EVENT_PRESSED");
		break;
	case LV_EVENT_PRESSING:
		printf("LV_EVENT_PRESSING");
		break;
	case LV_EVENT_PRESS_LOST:
		printf("LV_EVENT_PRESS_LOST");
		break;
	case LV_EVENT_SHORT_CLICKED:
		printf("LV_EVENT_SHORT_CLICKED");
		break;
	case LV_EVENT_LONG_PRESSED:
		printf("LV_EVENT_LONG_PRESSED");
		break;
	case LV_EVENT_LONG_PRESSED_REPEAT:
		printf("LV_EVENT_LONG_PRESSED_REPEAT");
		break;
	case LV_EVENT_CLICKED:
		printf("LV_EVENT_CLICKED");
		break;
	case LV_EVENT_RELEASED:
		printf("LV_EVENT_RELEASED");
		break;
	case LV_EVENT_DRAG_BEGIN:
		printf("LV_EVENT_DRAG_BEGIN");
		break;
	case LV_EVENT_DRAG_END:
		printf("LV_EVENT_DRAG_END");
		break;
	case LV_EVENT_DRAG_THROW_BEGIN:
		printf("LV_EVENT_DRAG_THROW_BEGIN");
		break;
	case LV_EVENT_KEY:
		// LV_KEY_LEFT, LV_KEY_RIGHT
		printf("LV_EVENT_KEY %d", *((uint32_t *)lv_event_get_data()));
		break;
	case LV_EVENT_FOCUSED:
		printf("LV_EVENT_FOCUSED");
		break;
	case LV_EVENT_DEFOCUSED:
		printf("LV_EVENT_DEFOCUSED");
		break;
	case LV_EVENT_VALUE_CHANGED:
		printf("LV_EVENT_VALUE_CHANGED");
		break;
	case LV_EVENT_INSERT:
		printf("LV_EVENT_INSERT");
		break;
	case LV_EVENT_REFRESH:
		printf("LV_EVENT_REFRESH");
		break;
	case LV_EVENT_APPLY:
		printf("LV_EVENT_APPLY");
		break;
	case LV_EVENT_CANCEL:
		printf("LV_EVENT_CANCEL");
		break;
	case LV_EVENT_DELETE:
		printf("LV_EVENT_DELETE");
		break;
	default:
		printf("unkown %d", event);
	}
}

static void
back_click_action(lv_obj_t * obj, lv_event_t event)
{

	switch(event) {
	case LV_EVENT_REFRESH:
		lv_async_call(disp_refresh, NULL);
		return;
	}
	printf("back event ");
	print_ev(event);
	printf("\n");
}

void
meteo_control_page(int move)
{
	int new_page;
	new_page = current_page;
	if (move > 0) {
		do {
			new_page++;
			if (new_page >= sizeof(mpages) / sizeof(mpages[0]))
				new_page = 0;
		} while (mpages[new_page]->mpage_in_page == false);
	} else if (move < 0) {
		do {
			new_page--;
			if (new_page < 0)
				new_page =
				    sizeof(mpages) / sizeof(mpages[0]) - 1;
		} while (mpages[new_page]->mpage_in_page == false);
	}
	switch_to_page(new_page);
}

static void
btn_up_click_action(lv_obj_t * btn, lv_event_t event)
{
#if 0
	switch(event) {
	   case LV_EVENT_LONG_PRESSED:
		page_list();
		return;
	   case LV_EVENT_SHORT_CLICKED:
		meteo_control_page(1);
		return;
	}
#endif
		
	printf("buttons[BUTTON_UP].button event ");
	print_ev(event);
	printf("\n");
}
static void
btn_down_click_action(lv_obj_t * btn, lv_event_t event)
{
#if 0
	switch(event) {
	   case LV_EVENT_LONG_PRESSED:
		page_list();
		return;
	   case LV_EVENT_SHORT_CLICKED:
		meteo_control_page(1);
		return;
	}
#endif
		
	printf("buttons[BUTTON_DOWN].button event ");
	print_ev(event);
	printf("\n");
}

static void
btn_left_click_action(lv_obj_t * btn, lv_event_t event)
{
#if 0
	switch(event) {
	   case LV_EVENT_LONG_PRESSED:
		page_list();
		return;
	   case LV_EVENT_SHORT_CLICKED:
		meteo_control_page(1);
		return;
	}
#endif
		
	printf("buttons[BUTTON_LEFT].button event ");
	print_ev(event);
	printf("\n");
}

static void
btn_right_click_action(lv_obj_t * btn, lv_event_t event)
{
#if 0
	switch(event) {
	   case LV_EVENT_LONG_PRESSED:
		page_list();
		return;
	   case LV_EVENT_SHORT_CLICKED:
		meteo_control_page(1);
		return;
	}
#endif
		
	printf("buttons[BUTTON_RIGHT].button event ");
	print_ev(event);
	printf("\n");
}


void
transient_open(lv_obj_t *obj)
{
	lv_obj_move_foreground(obj);
}

void
transient_close(lv_obj_t *obj)
{
	lv_obj_del_async(obj);
}

lv_obj_t *
transient_list(const char *names, int select,
    void (*action)(lv_obj_t *, lv_event_t))
{
	lv_obj_t *list = lv_ddlist_create(lv_top_trs, NULL);
	lv_ddlist_set_options(list, names);
	lv_ddlist_set_selected(list, select);
	lv_obj_set_event_cb(list, action);
	lv_ddlist_set_stay_open(list, true);
	lv_coord_t h = lv_obj_get_height(list);
	transient_open(list);
	lv_obj_align(list, lv_top_trs, LV_ALIGN_CENTER, 0, -h/2 + 5);
	return list;
}

static void
page_action(lv_obj_t *list, lv_event_t event)
{
	int key;
	int value;
	switch(event) {
	case LV_EVENT_VALUE_CHANGED:
		value = lv_ddlist_get_selected(list);
		transient_close(list);
		switch_to_page(value);
		break;
	case LV_EVENT_KEY:
		key = *((uint32_t *)lv_event_get_data());
		if (key == LV_KEY_ESC) {
			transient_close(list);
		}
		break;
	}
}

static void
page_list(void)
{
	char pagesnames[100];

	pagesnames[0] = '\0';

	for (int i = 0; i < sizeof(mpages) / sizeof(mpages[0]); i++) {
		if (i > 0)
			strlcat(pagesnames, "\n", sizeof(pagesnames));
		strlcat(pagesnames, mpages[i]->mpage_menu, sizeof(pagesnames));
	}
	transient_list(pagesnames, current_page, page_action);

}

static void
meteo_buttons_create(void)
{
	lv_coord_t h, w;
	static lv_style_t style_btn;
	static lv_style_t style_btn_pr;
	lv_style_copy(&style_btn, &lv_style_transp_tight);
	lv_style_copy(&style_btn_pr, &lv_style_transp_tight);
	style_btn_pr.glass = 0;
	style_btn_pr.body.opa = LV_OPA_COVER;
	style_btn_pr.body.main_color = LV_COLOR_BLACK;
	style_btn_pr.body.grad_color = LV_COLOR_BLACK;
	style_btn_pr.text.color = LV_COLOR_WHITE;
	style_btn_pr.image.color = LV_COLOR_WHITE;
	style_btn_pr.line.color = LV_COLOR_WHITE;

	buttons[BUTTON_UP].button = lv_btn_create(lv_top, NULL);
	lv_cont_set_fit(buttons[BUTTON_UP].button, LV_FIT_TIGHT);
	lv_obj_set_event_cb(buttons[BUTTON_UP].button, btn_up_click_action);
	lv_btn_set_style(buttons[BUTTON_UP].button, LV_BTN_STYLE_REL,
	    &style_btn);
	lv_btn_set_style(buttons[BUTTON_UP].button, LV_BTN_STYLE_PR,
	    &style_btn_pr);

	buttons[BUTTON_UP].label =
	    lv_label_create(buttons[BUTTON_UP].button, NULL);
	lv_label_set_static_text(buttons[BUTTON_UP].label, "U" LV_SYMBOL_UP);
	h = lv_obj_get_height(buttons[BUTTON_UP].button);
	lv_obj_align(buttons[BUTTON_UP].button, NULL,
	    LV_ALIGN_OUT_TOP_MID, 0, h);

	buttons[BUTTON_RIGHT].button = lv_btn_create(lv_top, NULL);
	lv_cont_set_fit(buttons[BUTTON_RIGHT].button, LV_FIT_TIGHT);
	lv_obj_set_event_cb(buttons[BUTTON_RIGHT].button,
	    btn_right_click_action);
	lv_btn_set_style(buttons[BUTTON_RIGHT].button, LV_BTN_STYLE_REL,
	    &style_btn);
	lv_btn_set_style(buttons[BUTTON_RIGHT].button, LV_BTN_STYLE_PR,
	    &style_btn_pr);

	buttons[BUTTON_RIGHT].label =
	    lv_label_create(buttons[BUTTON_RIGHT].button, NULL);
	lv_label_set_static_text(buttons[BUTTON_RIGHT].label, "R" LV_SYMBOL_RIGHT);
	h = lv_obj_get_height(buttons[BUTTON_RIGHT].button);
	w = lv_obj_get_width(buttons[BUTTON_RIGHT].button);
	lv_obj_align(buttons[BUTTON_RIGHT].button, NULL,
	    LV_ALIGN_OUT_RIGHT_MID, -w, 0);

	buttons[BUTTON_DOWN].button = lv_btn_create(lv_top, NULL);
	lv_cont_set_fit(buttons[BUTTON_DOWN].button, LV_FIT_TIGHT);
	lv_obj_set_event_cb(buttons[BUTTON_DOWN].button, btn_down_click_action);
	lv_btn_set_style(buttons[BUTTON_DOWN].button, LV_BTN_STYLE_REL,
	    &style_btn);
	lv_btn_set_style(buttons[BUTTON_DOWN].button, LV_BTN_STYLE_PR,
	    &style_btn_pr);

	buttons[BUTTON_DOWN].label =
	    lv_label_create(buttons[BUTTON_DOWN].button, NULL);
	lv_label_set_static_text(buttons[BUTTON_DOWN].label, "D" LV_SYMBOL_DOWN);
	h = lv_obj_get_height(buttons[BUTTON_DOWN].button);
	lv_obj_align(buttons[BUTTON_DOWN].button, NULL,
	    LV_ALIGN_OUT_BOTTOM_MID, 0, -h);

	buttons[BUTTON_LEFT].button = lv_btn_create(lv_top, NULL);
	lv_cont_set_fit(buttons[BUTTON_LEFT].button, LV_FIT_TIGHT);
	lv_obj_set_event_cb(buttons[BUTTON_LEFT].button,
	    btn_left_click_action);
	lv_btn_set_style(buttons[BUTTON_LEFT].button, LV_BTN_STYLE_REL,
	    &style_btn);
	lv_btn_set_style(buttons[BUTTON_LEFT].button, LV_BTN_STYLE_PR,
	    &style_btn_pr);

	buttons[BUTTON_LEFT].label =
	    lv_label_create(buttons[BUTTON_LEFT].button, NULL);
	lv_label_set_static_text(buttons[BUTTON_LEFT].label, "L" LV_SYMBOL_LEFT);
	h = lv_obj_get_height(buttons[BUTTON_LEFT].button);
	w = lv_obj_get_width(buttons[BUTTON_LEFT].button);
	lv_obj_align(buttons[BUTTON_LEFT].button, NULL,
	    LV_ALIGN_OUT_LEFT_MID, w, 0);
}

/* empty function for focus/edit style */
static void
empty_style_mod_cb(lv_group_t * group, lv_style_t * style)
{
}

/**
 * Create a meteo application
 */
void
meteo_app_init(void)
{
	int i;
	static lv_style_t style_page;

	lv_init();

	hal_init();

	lv_coord_t hres = lv_disp_get_hor_res(NULL);
	lv_coord_t vres = lv_disp_get_ver_res(NULL);
	lv_theme_t * th = lv_theme_mono_init(20, NULL);
	lv_theme_set_current(th);

	lv_style_copy(&style_small_text, &lv_style_transp_tight);
	style_small_text.text.font = &lv_font_unscii_8;

	lv_style_copy(&style_medium_text, &lv_style_transp_tight);
	style_medium_text.text.font = &lib16;

	lv_style_copy(&style_large_text, &lv_style_transp_tight);
	style_large_text.text.font = &lib24;

	lv_top = lv_disp_get_layer_top(NULL);
	lv_obj_set_click(lv_top, 1);
	lv_obj_set_event_cb(lv_top, back_click_action);

	lv_top_trs = lv_obj_create(lv_top, lv_top);

	meteo_buttons_create();

	lv_style_copy(&style_page, lv_obj_get_style(lv_scr_act()));
	
	for (i = 0; i < sizeof(mpages) / sizeof(mpages[0]); i++) {
		mpages[i]->mpage_page = lv_obj_create(NULL, NULL);
		lv_obj_set_style(mpages[i]->mpage_page, &style_page);
		if (mpages[i]->mpage_init != NULL)
			mpages[i]->mpage_init();
	}

	current_page = 0;
	mpages[current_page]->mpage_activate(mpages[current_page]);
}

void
meteo_app_run(void)
{
	struct timeval tv, tv_prev, tv_diff;
	time_t ticktime;

	if (gettimeofday(&tv_prev, NULL) < 0) {
		err(1, "gettimeofday");
	}

	while(1) {
		lv_task_handler();
		meteo_update_main();
		if (gettimeofday(&tv, NULL) < 0) {
			err(1, "gettimeofday");
		}
		timersub(&tv, &tv_prev, &tv_diff);
		tv_prev = tv;
		ticktime = tv_diff.tv_sec * 1000 + tv_diff.tv_usec / 1000;
		lv_tick_inc(ticktime);
		usleep(10 * 1000);
	}
}
