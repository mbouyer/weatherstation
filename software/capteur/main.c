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
unsigned int timer1_read(void);
#pragma callee_saves timer0_read

#define TIMER0_5MS 48

#define CLRWDT __asm__("clrwdt")

static char counter_10hz;
static char counter_1hz;
static volatile union softintrs {
	struct softintrs_bits {
		char int_10hz : 1;	/* 0.1s timer */
		char anemo_rx : 1;	/* NMEA string ready */
	} bits;
	char byte;
} softintrs;

static enum i2c_status {
	IDLE,
	READ_TEMP,
	READ_HUM,
	SEND_TEMP,
	SEND_HUM,
	FAIL
} i2c_status;

#define STH20_ADDR	0x40
#define STH20_READT_H	0xe3
#define STH20_READH_H	0xe5
#define STH20_READT	0xf3
#define STH20_READH	0xf5
#define STH20_WRITER	0xe6
#define STH20_READR	0xe7
#define STH20_RESET	0xfe

int32_t temp; /* degK * 100 */
uint32_t hum; /* %RH * 1000 */
static char counter_sth20;
static uint16_t i2cval;

static uint16_t raincount;

#define ANEMO_BUFSIZE 128
static char anemo_rxbuf1[ANEMO_BUFSIZE];
static char anemo_rxbuf2[ANEMO_BUFSIZE];
static unsigned char anemo_rxbufi;
static unsigned char anemo_rxbufs;
static unsigned char anemo_rxbufs_ready;

#define ANEMO_DIR_OFFSET 1800 /* deg * 10 */

static uint16_t wind_speed; /* kn * 10 */
static uint16_t wind_angle; /* deg * 10 */
static unsigned char wind_ok;

static void
send_env_param(void)
{
	struct nmea2000_env_param *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_ENV_PARAM, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_env_param);
	msg.data = &nmea2000_data[0]; 
	data->sid  = sid; 
	data->tsource = ENV_TSOURCE_OUTSIDE;
	data->hsource = ENV_HSOURCE_OUTSIDE;
	data->temp = temp;
	data->hum = hum / 4;
	data->press = 0xffff;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_ENV_PARAM failed\n");
}

static void
send_rain_counter(void)
{
	struct private_rain_counter *data = (void *)&nmea2000_data[0];

	PGN2ID(PRIVATE_RAIN_COUNTER, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct private_rain_counter);
	msg.data = &nmea2000_data[0]; 
	data->sid  = sid;
	data->count = raincount;
	if (! nmea2000_send_single_frame(&msg))
		printf("send PRIVATE_RAIN_COUNTER failed\n");
}

static void
send_wind_data(void)
{
	struct nmea2000_wind_data *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_WIND_DATA, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_wind_data);
	msg.data = &nmea2000_data[0]; 
	data->sid  = sid; 
	if (wind_ok) {
		data->speed = (unsigned long)wind_speed * 1852UL / 360UL;
			/* (wind_speed / 10) * (1852 / 3600) * 100 */
		data->dir = deg2urad(wind_angle / 10);
	} else {
		data->speed = 0xffff;
		data->dir = 0xffff;
	}
	data->ref = WIND_REF_TRUE_N;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_WIND_DATA failed\n");
}

void
user_handle_iso_request(unsigned long pgn)
{
	printf("ISO_REQUEST for %ld from %d\n", pgn, rid.saddr);
	switch(pgn) {
	case NMEA2000_ENV_PARAM:
		send_env_param();
		break;
	case NMEA2000_WIND_DATA:
		send_wind_data();
		break;
	case PRIVATE_RAIN_COUNTER:
		send_rain_counter();
		break;
	}
}

void
user_receive()
{
#if 0
	unsigned long pgn;

	pgn = ((unsigned long)rid.page << 16) | ((unsigned long)rid.iso_pg << 8);
	if (rid.iso_pg > 239)
		pgn |= rid.daddr;

	switch(pgn) {
	case PRIVATE_COMMAND_STATUS:
	{
		struct private_command_status *command_status = (void *)rdata;
		last_command_data = timer0_read();
		nmea2000_command_address = rid.saddr;
		command_received_heading = command_status->heading;
		received_auto_mode = command_status->auto_mode;
		received_param_slot = command_status->params_slot;
		break;
	}
	}
#endif
}

PUTCHAR(c)
{
        if (PORTBbits.RB7) {
		usart_putchar(c);
	}
}

static void
sth20_read(void) {
	switch(i2c_status) {
	case READ_TEMP:
		if (i2c_readvalue(STH20_ADDR, &i2cval) != 0) {
			i2cval &= 0xfffc;
			temp = (int32_t)i2cval * 17572 / 65536
			    + 22630 /* - 4685 + 27315 */;
			printf("STH20 temp 0x%x %ld\n", i2cval, temp);
		} else {
			printf("READ_TEMP failed\n");
		}
		raincount = timer1_read();
		i2c_status = SEND_HUM;
		break;
	case READ_HUM:
		if (i2c_readvalue(STH20_ADDR, &i2cval) != 0) {
			float tmp = (i2cval & 0xfffc);
			tmp = tmp * 125000.0 / 65536.0 - 6000.0;
			if (tmp < 0)
				tmp = 0;
			if (tmp > 100000)
				tmp = 100000;
			hum = tmp;
			printf("STH20 hum 0x%x %lu rain %d\n", i2cval, hum, raincount);
		} else {
			printf("READ_HUM failed\n");
		}
		i2c_status = SEND_TEMP;
		send_env_param();
		send_rain_counter();
		sid++;
		if (sid == 0xfe)
			sid = 0;
		break;
	}
}

static void
sth20_send(void)
{
	switch(i2c_status) {
	case SEND_TEMP:
		if (counter_sth20 == 0) {
			if (i2c_writecmd(STH20_ADDR, STH20_READT) != 0) {
				i2c_status = READ_TEMP;
				counter_sth20 = 10; /* one measure every 10s */
			} else {
				printf("STH20_READT failed\n");
			}
		}
		break;
	case SEND_HUM:
		if (i2c_writecmd(STH20_ADDR, STH20_READH) != 0) {
			i2c_status = READ_HUM;
		} else {
			printf("STH20_READH failed\n");
		}
		break;
	}
	if (counter_sth20 != 0) 
		counter_sth20--;

}

static void
parse_anemo_rx(void)
{
	static char *rxbuf;
	static unsigned char i;
	unsigned char dot;

	i = 0;
	if (anemo_rxbufs_ready == 0) {
		rxbuf = anemo_rxbuf1;
		printf("anemo1 %s\n", rxbuf);
	} else {
		rxbuf = anemo_rxbuf2;
		printf("anemo2 %s\n", rxbuf);
	}
	if (*rxbuf != '$')
		return;
	rxbuf++; i++;

	for (; i < 10 ; i++, rxbuf++) {
		/* we should have the NMEA ID 'MWV' in the first few chars */
		if (*rxbuf == 0)
			return;
		if (rxbuf[0] == 'M' && rxbuf[1] == 'W' && rxbuf[2] == 'V') {
			rxbuf += 3;
			i += 3;
			break;
		}
		rxbuf++; i++;
	}
	printf("found MWV\n");
	if (*rxbuf != ',')
		return;
	rxbuf++; i++;
	dot = 0;
	wind_angle = 0;
	wind_ok = 0;
	while(*rxbuf != ',') {
		if (*rxbuf == 0 || i >= ANEMO_BUFSIZE)
			return;

		if (*rxbuf == '.') {
			rxbuf++; i++;
			dot++;
			continue;
		}
		if (*rxbuf < '0' || *rxbuf > '9')
			return;
		wind_angle = wind_angle * 10;
		wind_angle += *rxbuf - '0';
		rxbuf++; i++;
	}
	if (dot == 0)
		wind_angle = wind_angle * 10;
	wind_angle = wind_angle + ANEMO_DIR_OFFSET;
	if (wind_angle > 3600)
		wind_angle = wind_angle - 3600;
	printf("wangle %d\n", wind_angle);

	rxbuf++; i++; /* skip , */
	rxbuf++; i++; /* skip R/T */
	rxbuf++; i++; /* skip , */

	wind_speed = 0;
	dot = 0;
	while(*rxbuf != ',') {
		if (*rxbuf == 0 || i >= ANEMO_BUFSIZE)
			return;

		if (*rxbuf == '.') {
			rxbuf++; i++;
			dot++;
			continue;
		}
		if (*rxbuf < '0' || *rxbuf > '9')
			return;
		wind_speed = wind_speed * 10;
		wind_speed += *rxbuf - '0';
		rxbuf++; i++;
	}
	if (dot == 0)
		wind_speed = wind_speed * 10;
	printf("wspeed %d\n", wind_speed);

	rxbuf++; i++; /* skip , */
	switch(*rxbuf) {
		case 'N': /* knots */
			break;
		case 'M': /* m/s */
			wind_speed =
			    (unsigned long)wind_speed * 3600UL / 1852UL; 
			break;
		case 'K': /* km/h */
			wind_speed =
			    (unsigned long)wind_speed * 1000UL / 1852UL; 
			break;
		default:
			return;
	}
	rxbuf++; i++; /* skip N/M/K */
	rxbuf++; i++; /* skip , */
	if (*rxbuf == 'A')
		wind_ok = 1;
	printf("wok %d\n", wind_ok);
}

void
main(void) __naked
{
	char c;
	static unsigned int poll_count;

	sid = 0;

	softintrs.byte = 0;
	counter_10hz = 25;
	counter_1hz = 10;
	i2c_status = IDLE;

	anemo_rxbufi = 0;
	anemo_rxbufs = 0;
	wind_ok = 0;

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

	/* configure UART1 (anemo) for 4800Bps at 10Mhz */
	SPBRG1 = 31;
	TXSTA1 = 0x00; /* disable TX */
	BAUDCON1 = 0x10; /* TX1 low-level idle state */
	RCSTA1 = 0x10; /* enable RX */

	INTCONbits.GIE_GIEH=1;  /* enable high-priority interrupts */   
	INTCONbits.PEIE_GIEL=1; /* enable low-priority interrrupts */   

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

	printf(", addr %d, id %ld\n", nmea2000_addr, nmea2000_user_id);

	if (i2c_writecmd(STH20_ADDR, STH20_READR) == 0) {
		printf("STH20_READR fail\n");
		i2c_status = FAIL;
	} else {
		i2c_readvalue(STH20_ADDR, &i2cval);
		printf("sth20 reg 0x%x\n", i2cval);
		i2c_status = SEND_TEMP;
	}
	counter_sth20 = 0;

	/* setup timer1 */
	LATAbits.LATA5 = 1;
	ANCON0bits.ANSEL4 = 0;
	TMR1H = 0;
	TMR1L = 0;
	T1GCON = 0;
	T1CON = 0x82; /* clock on T1CKI, RD16 */
	T1CONbits.TMR1ON = 1;

	/* enable anemo RX */
	RCSTA1bits.SPEN = 1;
	PIE1bits.RC1IE = 1;
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
			sth20_read();
			counter_1hz--;
			if (counter_1hz == 0) {
				counter_1hz = 10;
				sth20_send();
			}
		}
		if (softintrs.bits.anemo_rx) {
			parse_anemo_rx();
			softintrs.bits.anemo_rx = 0;
			if (wind_ok) {
				send_wind_data();
				sid++;
				if (sid == 0xfe)
					sid = 0;
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

unsigned int
timer1_read() __naked
{
	/* return TMR1L | (TMR1H << 8), reading TMR1L first */
	__asm
	movf    _TMR1L, w
	movff   _TMR1H, _PRODL
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
	unsigned char c;

	USART_INTR;

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) {
		c = RCREG1;
		if (c == 0x0a) {
			; /* ignore */
		} else if (c == 0x0d) {
			anemo_rxbufs_ready = anemo_rxbufs;
			if (anemo_rxbufs == 0) {
				anemo_rxbuf1[anemo_rxbufi] = 0;
				anemo_rxbufs = 1;
			} else {
				anemo_rxbuf2[anemo_rxbufi] = 0;
				anemo_rxbufs = 0;
			}
			softintrs.bits.anemo_rx = 1;
			anemo_rxbufi = 0;
		} else {
			if (anemo_rxbufi < (ANEMO_BUFSIZE - 1)) {
				if (anemo_rxbufs == 0) {
					anemo_rxbuf1[anemo_rxbufi] = c;
				} else {
					anemo_rxbuf2[anemo_rxbufi] = c;
				}
				anemo_rxbufi++;
			}
		}
		if (RCSTA1bits.OERR) { 
			RCSTA1bits.CREN = 0;
			RCSTA1bits.CREN = 1; 
		}
	}

	if (PIR5 != 0) {
		nmea2000_intr();
	}
	if (PIR5bits.RXBnIF)
		PIE5bits.RXBnIE = 0; /* main loop will check */

#if 0
	if (PIR5 != 0)
		PIE5 &= _PIE5_TXBnIE;
#endif
}
