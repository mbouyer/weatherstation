/* $Id: main.c,v 1.36 2019/03/12 19:24:19 bouyer Exp $ */
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

#include <pic18fregs.h>
#include <stdio.h> 
#include <string.h> 
#include <stdlib.h> 
#include <nmea2000.h>
#include <nmea2000_pgn.h>
#include <raddeg.h>
#include "serial.h"
#include "i2c.h"

extern unsigned char stack; 
extern unsigned char stack_end;

#pragma stack 0x100 256

void _reset (void) __naked __interrupt 0;
void _startup (void) __naked;

unsigned long devid; 

unsigned long nmea2000_user_id; 

static unsigned char sid;

static struct nmea2000_msg msg;
static unsigned char nmea2000_data[NMEA2000_DATA_LENGTH];

unsigned int timer0_read(void);
#pragma callee_saves timer0_read

#define TIMER0_5MS 48

#define CLRWDT __asm__("clrwdt")

static char counter_10hz;
static char counter_1hz;
static volatile union softintrs {
	struct softintrs_bits {
		char int_10hz : 1;	/* 0.1s timer */
	} bits;
	char byte;
} softintrs;

static unsigned char wind_timeout;
#define WIND_TIMEOUT 5 /* seconds */

#define DAC_ADDR 0x61
#define DAC_WRITE_DAC		0x00
#define DAC_WRITE_VOL		0x02
#define DAC_WRITE_ALL		0x03
#define DAC_WRITE_CONF_VOL	0x04

#define LED_N	LATBbits.LATB1
#define LED_NE	LATBbits.LATB0
#define LED_E	LATCbits.LATC7
#define LED_SE	LATCbits.LATC6
#define LED_S	LATCbits.LATC5
#define LED_SW	LATCbits.LATC2
#define LED_W	LATCbits.LATC1
#define LED_NW	LATCbits.LATC0

static inline void
clear_leds(void) {
	LED_N = LED_NE = 1;
	LATC |= 0xe7; /* E, SE, S, SW, W, NW */
}

static void
set_leds(unsigned char v)
{
	clear_leds();
	switch(v) {
	case 0:
		LED_N = 0;
		break;
	case 1:
		LED_N = 0;
		LED_NE = 0;
		break;
	case 2:
		LED_NE = 0;
		break;
	case 3:
		LED_NE = 0;
		LED_E = 0;
		break;
	case 4:
		LED_E = 0;
		break;
	case 5:
		LED_E = 0;
		LED_SE = 0;
		break;
	case 6:
		LED_SE = 0;
		break;
	case 7:
		LED_SE = 0;
		LED_S = 0;
		break;
	case 8:
		LED_S = 0;
		break;
	case 9:
		LED_S = 0;
		LED_SW = 0;
		break;
	case 10:
		LED_SW = 0;
		break;
	case 11:
		LED_SW = 0;
		LED_W = 0;
		break;
	case 12:
		LED_W = 0;
		break;
	case 13:
		LED_W = 0;
		LED_NW = 0;
		break;
	case 14:
		LED_NW = 0;
		break;
	case 15:
		LED_NW = 0;
		LED_N = 0;
		break;
	default:
		printf("invalid LED value %d\n", v);
	}
}

static unsigned char i2cdata[4];
static inline void
write_dac(unsigned char v) {
	i2cdata[0] = (DAC_WRITE_DAC << 5);
	i2cdata[1] = v;
	i2c_write(DAC_ADDR, &i2cdata[0], 2);
}
	

void
user_handle_iso_request(unsigned long pgn)
{
	printf("ISO_REQUEST for %ld from %d\n", pgn, rid.saddr);
#if 0
	switch(pgn) {
	case PRIVATE_COMPASS_OFFSET:
		send_private_compass_offset(rid.saddr);
		break;
	case NMEA2000_RATEOFTURN:
		send_nmea2000_rateofturn();
		break;
	case NMEA2000_ATTITUDE:
		send_nmea2000_attitude();
		break;
	}
#endif
}

void
user_receive()
{
	unsigned long pgn;
	float v;
	unsigned char c;

	pgn = ((unsigned long)rid.page << 16) | ((unsigned long)rid.iso_pg << 8);
	if (rid.iso_pg > 239)
		pgn |= rid.daddr;

	switch(pgn) {
	case NMEA2000_WIND_DATA:
	{
		struct nmea2000_wind_data *wind_data = (void *)rdata;
		wind_timeout = WIND_TIMEOUT;
		/*
		 * compte dac output:
		 * 48kn  = 24.693m/s = 2.04v = 255 dac
		 */
		v = (float)wind_data->speed / 2469.3 * 255.0;
		if (v > 255)
			v = 255;
		c = (unsigned char)v;
		printf("wind %d dac %d", wind_data->speed, c);
		write_dac(c);
		/*
		 * compute leds, with appropriate rounding
		 */
		v = (float)wind_data->dir / 62831.8 * 16.0;
		c = (unsigned char)(v + 0.5);
		printf(" dir %u led %d\n", wind_data->dir, c);
		set_leds(c & 0xf);
		break;
	}
	}
}

PUTCHAR(c)
{
        if (PORTBbits.RB7) {
		usart_putchar(c);
	}
}

void
main(void) __naked
{
	char c;
	static unsigned int poll_count;

	softintrs.byte = 0;
	counter_10hz = 25;
	counter_1hz = 10;
	sid = 0;

	/* setup LEDs */
	clear_leds();
	TRISB &= 0xfc;
	TRISC &= 0x18;

	LED_NW = LED_NE = 0; /* NW & NE on */

	devid = 0;
	__asm
	movlw UPPER __DEVID1
	movwf _TBLPTRU, a
	movlw HIGH __DEVID1
	movwf _TBLPTRH, a
	movlw LOW __DEVID1
	movwf _TBLPTRL, a
	tblrd*+;
	movff _TABLAT, _devid;
	tblrd*+;
	movff _TABLAT, _devid + 1;
	__endasm;

	/* configure sleep mode: PRI_IDLE */
	OSCCONbits.SCS = 0;
	OSCCONbits.IDLEN = 1;

	/* everything is low priority by default */
	IPR1 = 0;
	IPR2 = 0;
	IPR3 = 0;
	IPR4 = 0;
	IPR5 = 0;
	INTCON = 0;
	INTCON2 = 0;
	INTCON3 = 0;
	RCONbits.IPEN=1; /* enable interrupt priority */

	/* configure timer0 as free-running counter at 9.765625Khz * 4 */
	T0CON = 0x07; /* b00000111: internal clock, 1/256 prescaler */
	INTCONbits.TMR0IF = 0;
	INTCONbits.TMR0IE = 0; /* no interrupt */
	T0CONbits.TMR0ON = 1;

	/* configure timer2 for 250Hz interrupt */
	PMD1bits.TMR2MD=0;
	T2CON = 0x23; /* b00100011: postscaller 1/5, prescaler 1/16 */
	PR2 = 125; /* 250hz output */
	T2CONbits.TMR2ON = 1;
	PIR1bits.TMR2IF = 0;
	IPR1bits.TMR2IP = 1; /* high priority interrupt */
	PIE1bits.TMR2IE = 1;

	USART_INIT(0);
	I2C_INIT;

	INTCONbits.GIE_GIEH=1;  /* enable high-priority interrupts */   
	INTCONbits.PEIE_GIEL=1; /* enable low-priority interrrupts */   

	LED_NW = LED_NE = 1;
	LED_N = LED_E = 0;

	/* read user-id from config bits */
	nmea2000_user_id = 0;
	__asm
	movlw UPPER __IDLOC0
	movwf _TBLPTRU, a
	movlw HIGH __IDLOC0
	movwf _TBLPTRH, a
	movlw LOW __IDLOC0
	movwf _TBLPTRL, a
	tblrd*+;
	movff _TABLAT, _nmea2000_user_id;
	tblrd*+;
	movff _TABLAT, _nmea2000_user_id + 1;
	__endasm;

	nmea2000_init();

	stdout = STREAM_USER; /* Use the macro PUTCHAR with printf */

	/* enable watch dog timer */
	WDTCON = 0x01;

	printf("\nready");
	poll_count = timer0_read();
	while (nmea2000_status != NMEA2000_S_OK) {
		nmea2000_poll(5);
		while ((timer0_read() - poll_count) < TIMER0_5MS) {
			nmea2000_receive();
		}
		poll_count = timer0_read();
		CLRWDT;
	}

	LED_N = LED_E = 1;
	LED_NE = LED_SE = 0;
	printf(", addr %d, id %ld\n", nmea2000_addr, nmea2000_user_id);

i2c_retry:
	i2c_read(DAC_ADDR, &i2cdata[0], sizeof(i2cdata));
	printf("i2c data:");
	for (int c = 0; c < 4; c++) {
		printf(" 0x%x", i2cdata[c]);
	}
	printf("\n");
	if ((i2cdata[0] & 0xe0) != 0xc0 || (i2cdata[2] & 0xe0) != 0xe0) {
		printf("i2c read failed\n");
		goto i2c_retry;
	}

	/* start with vref buffered, DAC 0 */
	if ((i2cdata[2] & 0x1f) != 0x18 || i2cdata[3] != 0) {
		printf("write DAC eeprom\n");
		i2cdata[0] = (DAC_WRITE_ALL << 5) |
			     (0x3 << 3); /* vref buffered */
		i2cdata[1] = 0;
		i2cdata[2] = 0;
		i2c_write(DAC_ADDR, &i2cdata[0], 3);
	}

	LED_NE = LED_SE = 1;

	/*
	 * auto test the display: rotate leds 2 times; slowly grow wind
	 * indicator
	 */
	counter_10hz = 0;
	softintrs.bits.int_10hz = 0;
	for (c = 0; c < 32;) {
		CLRWDT;
		if (PIR5bits.RXBnIF) {
			nmea2000_receive();
		}

		if (nmea2000_status == NMEA2000_S_CLAIMING) {
			if ((timer0_read() - poll_count) > TIMER0_5MS) {
				nmea2000_poll(5);
				poll_count = timer0_read();
			}
			if (nmea2000_status == NMEA2000_S_OK) {
				printf("new addr %d\n", nmea2000_addr);
			}
		};
		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;

			set_leds(c >> 1);
			write_dac(c * 8 + 7 /* (c + 1) * 256 / 32 - 1 */);
			c++;
		}
		if (nmea2000_status == NMEA2000_S_OK) {
			__asm
			SLEEP
			__endasm;
		}
	}
	wind_timeout = WIND_TIMEOUT;
again:
	printf("hello user_id 0x%lx devid 0x%lx\n", nmea2000_user_id, devid);
	while (1) {
		CLRWDT;
		if (PIR5bits.RXBnIF) {
			nmea2000_receive();
		}

		if (nmea2000_status == NMEA2000_S_CLAIMING) {
			if ((timer0_read() - poll_count) > TIMER0_5MS) {
				nmea2000_poll(5);
				poll_count = timer0_read();
			}
			if (nmea2000_status == NMEA2000_S_OK) {
				printf("new addr %d\n", nmea2000_addr);
			}
		};

		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;
			counter_1hz--;
			if (counter_1hz == 0) {
				counter_1hz = 10;
				if (wind_timeout == 0) {
					clear_leds();
					LED_E = LED_S = 0;
					write_dac(0);
				} else {
					wind_timeout--;
				}
			}
		}

		if (RCREG2 == 'r')
			break;

		if (nmea2000_status == NMEA2000_S_OK) {
			__asm
			SLEEP
			__endasm;
		}
	}
	while ((c = getchar()) != 'r') {
		printf("resumed\n");
		goto again;
	}
end:
	printf("returning\n");
	while (PIE3bits.TX2IE)
		; /* wait for transmit to complete */
	INTCONbits.PEIE=0; /* disable peripheral interrupts */
	INTCONbits.GIE=0;  /* disable interrrupts */
}

unsigned int
timer0_read() __naked
{
	/* return TMR0L | (TMR0H << 8), reading TMR0L first */
	__asm
	movf    _TMR0L, w
	movff   _TMR0H, _PRODL
	return
	__endasm;
}

/* Vectors */
void _reset (void) __naked __interrupt 0
{
	__asm__("goto __startup");
}


void _startup (void) __naked
{

  __asm
    // Initialize the stack pointer
    lfsr 1, _stack_end
    lfsr 2, _stack_end
    clrf _TBLPTRU, 0    // 1st silicon doesn't do this on POR
    
    // initialize the flash memory access configuration. this is harmless
    // for non-flash devices, so we do it on all parts.
    bsf _EECON1, 7, 0
    bcf _EECON1, 6, 0
    __endasm ;

  /* Call the user's main routine */
  main();
  __asm__("reset");
}

/*
 * high priority interrupt. Split in 2 parts; one for the entry point
 * where we'll deal with timer0, then jump to another address 
 * as we don't have enough space before the low priority vector
 */
void _irqh (void) __naked __shadowregs __interrupt 1
{
	__asm
	btfss _PIR1, 1
	bra other
	bcf   _PIR1, 1
	goto _irqh_timer2
other:
	retfie 1
	nop
	__endasm ;
}

void irqh_timer2(void) __naked
{
	/*
	 * no sdcc registers are automatically saved,
	 * so we have to be carefull with C code !
	 */
	if (--counter_10hz == 0) {
		counter_10hz = 25;
		softintrs.bits.int_10hz = 1;
	}			
	__asm
	retfie 1
	nop
	__endasm;
}

void _irql (void) __interrupt 2 /* low priority */
{
	USART_INTR;
	if (PIR5 != 0) {
		nmea2000_intr();
	}
	if (PIR5bits.RXBnIF)
		PIE5bits.RXBnIE = 0; /* main loop will check */

}
