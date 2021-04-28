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

#define LEDBATT_G LATCbits.LATC2
#define LEDBATT_R LATCbits.LATC6

#define LEDPWR_G LATCbits.LATC0
#define LEDPWR_R LATCbits.LATC1

#define BATT_ON LATCbits.LATC7

#define INA226_ADDR 0x40

#define INA_CONFIG	0x00
#define INA_SHUNTV	0x01
#define INA_BUS		0x02
#define INA_POWER	0x03
#define INA_CURRENT	0x04
#define INA_CAL		0x05
#define INA_MASK	0x06
#define INA_ALERT	0x07
#define INA_MANUF	0xfe
#define INA_DIE 	0xff


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

void
main(void) __naked
{
	char c;
	static unsigned int poll_count;
	static uint16_t i2cval;

	sid = 0;

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

	LEDBATT_R = 0;
	LEDBATT_G = 0;
	LEDPWR_R = 0;
	LEDPWR_G = 0;
	BATT_ON = 0;
	TRISCbits.TRISC2 = 0;
	TRISCbits.TRISC6 = 0;
	TRISCbits.TRISC0 = 0;
	TRISCbits.TRISC1 = 0;
	TRISCbits.TRISC7 = 0;

	LEDPWR_R = 1;

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

	USART_INIT(0);
	I2C_INIT;

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
	while (nmea2000_addr_status != ADDR_STATUS_OK) {
		nmea2000_poll(5);
		while ((timer0_read() - poll_count) < TIMER0_5MS) {
			nmea2000_receive();
		}
		poll_count = timer0_read();
		CLRWDT;
	}

	printf(", addr %d, id %ld\n", nmea2000_addr, nmea2000_user_id);

	usart_putchar('c');
	usart_putchar('h');
	usart_putchar('\n');
	usart_putchar('\r');

	LEDPWR_G = 1;
	LEDPWR_R = 0;
	BATT_ON  = 1;
	LEDBATT_G = 1;

	if (i2c_readreg(INA226_ADDR, INA_MANUF, &i2cval) == 0)
		printf("i2c INA_MANUF fail\n");
	else
		printf("INA226 manuf 0x%x", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_DIE, &i2cval) == 0)
		printf("i2c INA_DIE fail\n");
	else
		printf(" die 0x%x\n", i2cval);

again:
	printf("hello user_id 0x%lx devid 0x%lx\n", nmea2000_user_id, devid);
	while (1) {
		CLRWDT;
		if (PIR5bits.RXBnIF) {
			nmea2000_receive();
		}

		if (nmea2000_addr_status == ADDR_STATUS_CLAIMING) {
			if ((timer0_read() - poll_count) > TIMER0_5MS) {
				nmea2000_poll(5);
				poll_count = timer0_read();
			}
			if (nmea2000_addr_status == ADDR_STATUS_OK) {
				printf("new addr %d\n", nmea2000_addr);
			}
		};

		if (RCREG2 == 'r')
			break;

		if (nmea2000_addr_status == ADDR_STATUS_OK) {
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

#if 0
	if (PIR5 != 0)
		PIE5 &= _PIE5_TXBnIE;
#endif
}
