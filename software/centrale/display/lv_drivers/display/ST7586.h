/**
 *
 */

#ifndef ST7586_H
#define ST7586_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifndef LV_DRV_NO_CONF
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lv_drv_conf.h"
#else
#include "../../lv_drv_conf.h"
#endif
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include <libusb-1.0/libusb.h>
#include <pthread.h>

typedef struct _meteo_ctx {
        libusb_context *edctx_usb;     
	libusb_device_handle *edctx_dev;
	pthread_mutex_t edctx_mtx;   
	int             edctx_refcount;
	volatile int    edctx_fail;
} meteo_ctx_t;

meteo_ctx_t *meteo_get(void);
void meteo_put(meteo_ctx_t *);

void meteo_init(void);
void meteo_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
void meteo_set_px(lv_disp_drv_t *, uint8_t *, lv_coord_t, lv_coord_t, lv_coord_t, lv_color_t, lv_opa_t);
void meteo_rounder(lv_disp_drv_t *, lv_area_t *);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ST7586_H */
