/*
 * Copyright (c) 2022 Manuel Bouyer
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <dev/i2c/i2c_io.h>

#include <lvgl/lvgl.h>
#include <lv_meteo/hal.h>
#include <lv_meteo/meteo_data.h>

#include "bme280.h"

static struct bme280_dev bme280_dev;
static uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

static lv_task_t *get_bme280_task; 

static int iic_fd;

void
user_delay_us(uint32_t period, void *intf_ptr)
{
	usleep(period);
}

int8_t
user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	i2c_ioctl_exec_t iie;

	iie.iie_op = I2C_OP_READ_WITH_STOP;
	iie.iie_addr = BME280_I2C_ADDR_SEC;
	iie.iie_cmd = &reg_addr;
	iie.iie_cmdlen = 1;
	iie.iie_buf = reg_data;
	iie.iie_buflen = len;

	if (ioctl(iic_fd, I2C_IOCTL_EXEC, &iie) == -1) {
		perror("ioctl read");
		return BME280_E_COMM_FAIL;
	}
	return BME280_OK;

}

int8_t
user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
    void *intf_ptr)
{
	i2c_ioctl_exec_t iie;

	iie.iie_op = I2C_OP_WRITE_WITH_STOP;
	iie.iie_addr = BME280_I2C_ADDR_SEC;
	iie.iie_cmd = &reg_addr;
	iie.iie_cmdlen = 1;
	iie.iie_buf = __UNCONST(reg_data);
	iie.iie_buflen = len;

	if (ioctl(iic_fd, I2C_IOCTL_EXEC, &iie) == -1) {
		perror("ioctl write");
		return BME280_E_COMM_FAIL;
	}
	return BME280_OK;
}

void
get_bme280_values(lv_task_t *task)
{
	int rslt;
	struct bme280_data comp_data;

	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_dev);
	if (rslt != BME280_OK) {
		warnx("bme280_get_sensor_data failed: %d\n", rslt);
		meteo_set_temp(TEMP_INT, 0, 0, false);
		meteo_set_baro(0, false);
		return;
	}
	meteo_set_temp(TEMP_INT, comp_data.temperature, comp_data.humidity,
	    true);
	meteo_set_baro(comp_data.pressure / 100, true);
	n2ks_env(comp_data.temperature, comp_data.humidity,
	    comp_data.pressure / 100);
}

void
lv_bme280_init(void)
{
	int rslt;
	uint8_t settings_sel;
#define IIC_DEV "/dev/iic1"

	iic_fd = open(IIC_DEV, O_RDWR);

	if (iic_fd < 0) {
		err(1, "open %s", IIC_DEV);
	}
		
	bme280_dev.intf_ptr = &dev_addr;
	bme280_dev.intf = BME280_I2C_INTF;
	bme280_dev.read = user_i2c_read;
	bme280_dev.write = user_i2c_write;
	bme280_dev.delay_us = user_delay_us;

	rslt = bme280_init(&bme280_dev);
	if (rslt != BME280_OK) {
		errx(1, "bme280_init failed: %d\n", rslt);
	}

	bme280_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280_dev.settings.filter = BME280_FILTER_COEFF_16;
	bme280_dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &bme280_dev);
	if (rslt != BME280_OK) {
		errx(1, "bme280_set_sensor_settings failed: %d\n", rslt);
	}
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280_dev);
	if (rslt != BME280_OK) {
		errx(1, "bme280_set_sensor_mode failed: %d\n", rslt);
	}

	get_bme280_task = lv_task_create(
		    get_bme280_values, 5000, LV_TASK_PRIO_MID, NULL);
}
