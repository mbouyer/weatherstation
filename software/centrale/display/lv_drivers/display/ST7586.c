/**
 * driver for ST7586-based LCDs
 */

#include "ST7586.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <err.h>
#include <fcntl.h>

#include <sys/gpio.h>
#include <sys/ioctl.h>
#include <dev/spi/spi_io.h>

static int gpio_fd = -1;
static int spi_fd = -1;

#define LINE_SIZE ((LV_HOR_RES_MAX + 2) / 3)
#define FB_SIZE  ( LINE_SIZE * LV_VER_RES_MAX)

//static u_int8_t framebuf[FB_SIZE];

static void st7586_init(meteo_ctx_t *);

static void
gpio_wr(const char *name, unsigned char val)
{
	struct gpio_req gr;

	memset(&gr, 0, sizeof(gr));
	strlcpy(gr.gp_name, name, sizeof(gr.gp_name));
	gr.gp_value = val;
	if (ioctl(gpio_fd, GPIOWRITE, &gr) == -1)
		err(EXIT_FAILURE, "GPIOWRITE %s", name );
}

static void
spi_wr(unsigned char a0, unsigned char val)
{
	struct spi_ioctl_transfer spic;
	unsigned char rval = 0xda;
	gpio_wr("lcd_a0", a0);
	memset(&spic, 0, sizeof(spic));
	spic.sit_addr = 0;
	spic.sit_send = &val;
	spic.sit_sendlen = sizeof(val);
	spic.sit_recv = NULL;
	spic.sit_recvlen = 0;
	if (ioctl(spi_fd, SPI_IOCTL_TRANSFER, &spic) == -1)
		err(EXIT_FAILURE, "spi0 %d write", a0);
	// printf("spi send %d rec %d\n", val, rval);
}

static void
write_com(unsigned char val)
{
	spi_wr(0, val);
}

static void
write_dat(unsigned char val)
{
	spi_wr(1, val);
}

static void
write_fb(uint8_t *buf)
{
	struct spi_ioctl_transfer spic;
	int remain, size;
	write_com(0x2c);
	
	gpio_wr("lcd_a0", 1);
	remain = FB_SIZE;
	while (remain > 0) {
		size = 4096;
		if (remain < size)
			size = remain;
	
		memset(&spic, 0, sizeof(spic));
		spic.sit_addr = 0;
		spic.sit_send = buf;
		spic.sit_sendlen = size;
		spic.sit_recv = NULL;
		spic.sit_recvlen = 0;
		if (ioctl(spi_fd, SPI_IOCTL_TRANSFER, &spic) == -1)
			err(EXIT_FAILURE, "spi0 writebuf");
		buf += size;
		remain -= size;
	}
}

/**
 * Initialize the meteo context
 */
void
meteo_init(void)
{
	struct spi_ioctl_configure spic;
	gpio_fd = open("/dev/gpio0", O_RDWR);
	if (gpio_fd == -1)
		err(EXIT_FAILURE, "/dev/gpio0");

	spi_fd = open("/dev/spi0", O_RDWR);
	if (spi_fd == -1)
		err(EXIT_FAILURE, "/dev/spi0");

	spic.sic_addr = 0;
	spic.sic_mode = 0; /* SPI_MODE_0 */
	spic.sic_speed = 1000000;
	spic.sic_speed = 4000000;

	if (ioctl(spi_fd, SPI_IOCTL_CONFIGURE, &spic) == -1)
		err(EXIT_FAILURE, "spi0 configure");

	gpio_wr("lcd_rstb", 1);
	usleep(200000);
	gpio_wr("lcd_rstb", 0);
	usleep(200000);
	gpio_wr("lcd_rstb", 1);
	usleep(200000);

	write_com(0xD7); write_dat(0x9F); // Disable Auto Read
	write_com(0xE0); write_dat(0x00); // Enable OTP Read
	usleep(20000);	
	write_com(0xE3); // OTP Up-Load
	usleep(20000);	
	write_com(0xE1); // OTP Control Out 
	write_com(0x11); // Sleep Out
	write_com(0x28); // Display OFF
	usleep(50000);	
	write_com(0xC0); write_dat(0x1D); write_dat(0x01); // Vop - 0X11Dh
	write_com(0xC3); write_dat(0x02);	//BIAS - 1/12
	write_com(0xC4); write_dat(0x07);	// Set Booster
	write_com(0xD0); write_dat(0x1D);	// Enable Analog Circuit
	write_com(0xB5); write_dat(0x00);	// N-Line = 0 ; Frame inversion

	write_com(0x39); //Display Mode : Monochrome mode(B/W Mode)
	//............ FSTN White Temperature Compensation
	write_com(0xF1); // Frame Rate (Monochrome Mode)
	write_dat(0x06); write_dat(0x0B); write_dat(0x0D); write_dat(0x12);

	//............ FSTN White Temperature Compensation
	write_com(0xF4); //Temperature Gradient Compensation
	write_dat(0x7F); //MT1 , MTO
	write_dat(0x22); //MT3 , MT2
	write_dat(0x11); //MT5 , MT4
	write_dat(0x02); //MT7 , MT6
	write_dat(0x00); //MT9 , MT8
	write_dat(0x32); //MTB , MTA
	write_dat(0x82); //MTD , MTC
	write_dat(0xB6); //MTF , MTE
	
	write_com(0x3A); write_dat(0x02); // Enable DDRAM Interface
	write_com(0x36); write_dat(0x00); // Scan Direction Setting/Display Control
	write_com(0xB0); write_dat(0x7F); // Duty Setting  1/128Duty
	write_com(0x20);	// Normal display
	write_com(0x37); write_dat(0x00);	// Start Line
	write_com(0xB1); write_dat(0x00); 	// First Output COM
	write_com(0xB3); write_dat(0x00);	//FOSC Divider
	write_com(0x2A);	// Column Address Setting
	write_dat(0x00);
	write_dat(0x00);
	write_dat(0x00);
	write_dat(79);	

	write_com(0x2B);	// Row Address Setting
	write_dat(0x00);
	write_dat(0x00);
	write_dat(0x00);
	write_dat(127);

	write_com(0x29);	// Display ON
}

void
meteo_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	/* Return if the area is out the screen */
	if(area->x2 < 0)
		return;
	if(area->y2 < 0)
		return;
	if(area->x1 > LV_HOR_RES - 1)
		return;
	if(area->y1 > LV_VER_RES - 1)
		return;

	write_fb((uint8_t *)color_p);
	
	lv_disp_flush_ready(disp_drv);
}

void
meteo_set_px(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa) {
	(void) disp_drv;
	(void) opa;
	int bits = x % 3;
	uint8_t mask, color2;
	uint16_t idx = (x / 3) + LINE_SIZE * y;

	switch(bits) {
	case 0:
		mask = 0xe0;
		color2 = color.full ? 0 : 0xe0;
		break;
	case 1:
		mask = 0x1c;
		color2 = color.full ? 0 : 0x1c;
		break;
	case 2:
		mask = 0x03;
		color2 = color.full ? 0 : 0x03;
		break;
	}

	buf[idx] &= ~mask; /* reset pixel color */
	buf[idx] |= color2; /* write new color	 */
}

void
meteo_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
	(void) disp_drv;
	
	/* always round to full screen (for now) */
	area->x1 = 0;
	area->x2 = LV_HOR_RES_MAX - 1;
	area->y1 = 0;
	area->y2 = LV_VER_RES_MAX - 1;
}
