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

#define DEGSTR "°"

typedef struct meteo_page {
	void (*mpage_init)(void);
	void (*mpage_activate)(struct meteo_page *);
	const char *mpage_menu;
	bool mpage_in_page;
	bool mpage_is_transient;
	lv_obj_t *mpage_page;
} meteo_page_t;

extern lv_style_t style_large_text;
extern lv_style_t style_medium_text;
extern lv_style_t style_small_text;

extern lv_obj_t *lv_top_trs;

void activate_page(meteo_page_t *);
void deactivate_page(meteo_page_t *);
void transient_open(lv_obj_t *);
void transient_close(lv_obj_t *);
lv_obj_t *transient_list(const char *, int, void (*)(lv_obj_t *, lv_event_t));
void switch_to_page_o(lv_obj_t *);

void print_ev(lv_event_t event);

extern meteo_page_t mpage_main;

void meteo_update_main(void);

#define BUTTON_UP 0
#define BUTTON_RIGHT 1
#define BUTTON_DOWN 2
#define BUTTON_LEFT 3
