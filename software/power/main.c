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
#include "ntc_tab.h"

extern unsigned char stack; 
extern unsigned char stack_end;

#pragma stack 0x100 256

void _reset (void) __naked __interrupt 0;
void _startup (void) __naked;

unsigned long devid; 

unsigned long nmea2000_user_id; 

static unsigned char sid;

static struct nmea2000_msg msg;
static unsigned char nmea2000_data[NMEA2000_DATA_FASTLENGTH];

unsigned int timer0_read(void);
#pragma callee_saves timer0_read

#define TIMER0_5MS 48

#define CLRWDT __asm__("clrwdt")

#define LEDBATT_G LATCbits.LATC2
#define LEDBATT_R LATCbits.LATC6

#define LEDPWR_G LATCbits.LATC0
#define LEDPWR_R LATCbits.LATC1

#define BATT_ON LATCbits.LATC7

#define CHRG_BULK LATBbits.LATB1

#define INA226_ADDR 0x40

#define INA_CONFIG	0x00
#define INA_SHUNTV	0x01
#define INA_BUSV	0x02
#define INA_POWER	0x03
#define INA_CURRENT	0x04
#define INA_CAL		0x05
#define INA_MASK	0x06
#define INA_ALERT	0x07
#define INA_MANUF	0xfe
#define INA_DIE 	0xff

static char counter_10hz;
static char counter_1hz;
static volatile union softintrs {
	struct softintrs_bits {
		char int_10hz : 1;	/* 0.1s timer */
	} bits;
	char byte;
} softintrs;

static unsigned long input_volt;
static uint16_t batt_v; /* Volts * 100 */
static int16_t batt_i; /* mA * 10 */
static uint16_t batt_temp; /* K * 100 */

#define BATTV_CUTOFF	1100 /* 11v */
#define BATTV_MIN	900
#define BATTV_MIN_BULK	1240
#define BATTV_STOP_BULK	1400
#define BATTI_STOP_BULK	4500 /* 450mA */
#define BATTI_STOP_ABS	1000
#define BATT_BULK_TIME	14400 /* 3h */


static uint16_t a2d_acc;

static char ina226_ready;

enum power_status {
	ON,
	OFF,
	FAIL,
	UNKOWN
} power_status;

unsigned char charger_status; /* see nmea2000_charger_status_data */
static unsigned int time_on_bulk;
static unsigned int time_on_abs;
static unsigned int time_on_batt;

static void
adctotemp(void)
{
	char i;
	for (i = 1; temps[i].val != 0; i++) {
		if (a2d_acc > temps[i].val) {
			batt_temp = temps[i - 1].temp -
			((temps[i - 1].temp - temps[i].temp)  /
		         (temps[i - 1].val - temps[i].val)  * 
			 (temps[i - 1].val - a2d_acc));
			return;
		}
	} 
	batt_temp = 0xffff;
}

static void
send_batt_status(void)
{
	struct nmea2000_battery_status_data *data = (void *)&nmea2000_data[0];

	if (charger_status == CHARGER_STATE_FAULT)
		return;

	PGN2ID(NMEA2000_BATTERY_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_battery_status_data);
	msg.data = &nmea2000_data[0];
	data->voltage = batt_v;
	data->current = batt_i;
	data->temp = batt_temp;
	data->sid = sid;
	data->instance = 0;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_BATTERY_STATUS failed\n");
}

static void
send_dc_status(void)
{
	struct nmea2000_dc_status_data *data = (void *)&nmea2000_data[0];
	static unsigned char fastid;

	if (input_volt == 0xffff)
		return;

	fastid = (fastid + 1) & 0x7;
	printf("power voltage %d.%03dV\n",
	    (int)(input_volt / 1000), (int)(input_volt % 1000));

	PGN2ID(NMEA2000_DC_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_dc_status_data);
	msg.data = &nmea2000_data[0];
	data->sid = sid;
	data->instance = 0;
	switch(power_status) {
	case UNKOWN:
		return;
	case OFF:
		data->type = DCSTAT_TYPE_BATT;
		if (batt_v > 1240) {
			data->soc = 100 - ((unsigned long)time_on_batt * 100UL / 7200UL);
		} else {
			data->soc = 50 - (1240 - batt_v) * 50 / (1240 - 1100);
		}
		data->soh = 0xff;
		data->timeremain = 0xffff; /* XXX compute */
		data->ripple = 0xffff;
		break;
	default:
		/* assume operating on mains power */
		data->type = DCSTAT_TYPE_CONV;
		data->soc = 0xff;
		data->soh =
		    ((unsigned long)input_volt * 100UL + 6000UL) / 12000;
		data->timeremain = 0xffff;
		data->ripple = 0xffff;
		break;
	}
	if (! nmea2000_send_fast_frame(&msg, fastid))
		printf("send NMEA2000_DC_STATUS failed\n");
}

static void
send_charger_status()
{
	struct nmea2000_charger_status_data *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_CHARGER_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_charger_status_data);
	msg.data = &nmea2000_data[0];
	data->instance = 0;
	data->batt_instance = 0;
	data->op_state = charger_status;
	data->mode = CHARGER_MODE_STANDALONE;
	data->enable = 1;
	data->eq_pending = 0;
	data->eq_time_remain = 0;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_CHARGER_STATUS failed\n");
}

void
user_handle_iso_request(unsigned long pgn)
{
	printf("ISO_REQUEST for %ld from %d\n", pgn, rid.saddr);
	switch(pgn) {
	case NMEA2000_BATTERY_STATUS:
		send_batt_status();
		break;
	case NMEA2000_DC_STATUS:
		send_dc_status();
		break;
	case NMEA2000_CHARGER_STATUS:
		send_charger_status();
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
handle_input_voltage()
{
	/* lsb = 2 / 4096 * 11.8 / 1.8 = 3.2mV */
	input_volt = (unsigned long)a2d_acc * 32UL / 10;
	if (input_volt > 4000) {
		/*
		 * assume we have enough voltage to run the step-up converter
		 * anyway
		 */
		 if (input_volt < 11000) {
			LEDPWR_G = 0;
			LEDPWR_R = 1;
			power_status = FAIL;
		} else {
			LEDPWR_R = 0;
			LEDPWR_G = 1;
			power_status = ON;
		}
	} else {
		LEDPWR_R = 0;
		LEDPWR_G = 0;
		power_status = OFF;
	}
}

static inline void
charger_disable(void)
{
	LEDBATT_R = 0;
	LEDBATT_G = 1;
	CHRG_BULK = 0;
	charger_status = CHARGER_STATE_DIS;
}

static inline void
charger_float(void)
{
	LEDBATT_R = 0;
	LEDBATT_G = 1;
	CHRG_BULK = 0;
	charger_status = CHARGER_STATE_FLOAT;
}

static void
read_battery_data(void)
{
	static unsigned int i2cval;
	float v;

	if (i2c_readreg(INA226_ADDR, INA_SHUNTV, &i2cval) == 0) {
		printf("i2c INA_SHUNTV fail\n");
		goto fail;
	}
	printf("shuntv %d", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_BUSV, &i2cval) == 0) {
		printf("i2c INA_BUSV fail\n");
		goto fail;
	} else {
		v = i2cval;
		v = (v * 1.25 / 10.0);
		batt_v = v;
	}
	printf(" busv %d", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_CURRENT, &i2cval) == 0) {
		printf("i2c INA_CURRENT fail\n");
		goto fail;
	} else {
		v = i2cval;
		/*
		 * current should be in A * 10 but for our small battery
		 * it's not precise enough. Use mA * 10, which matches
		 * INA_CURRENT
		 */
		batt_i = -((int16_t)i2cval); /* negative values for discharge */
	}
	printf(" current %d", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_POWER, &i2cval) == 0) {
		printf("i2c INA_POWER fail\n");
		goto fail;
	}
	printf(" power %d", i2cval);
	printf(" temp %d\n", batt_temp);

	if (power_status == OFF) {
		/* running on battery */
		charger_disable();
		if (batt_v < BATTV_CUTOFF) {
			/*
			 * switch off to protect battery
			 * this is likely going to kill us
			 */
			LEDBATT_R = 0;
			LEDBATT_G = 0;
			BATT_ON = 0;
		}
		time_on_batt++;
		return;
	}

	time_on_batt = 0;
	if (batt_temp > 30800) { /* 35 degC */
		charger_disable();
		LEDBATT_R = 1;
		BATT_ON = 0;
		return;
	}

	/* now handle charger state */
	switch(charger_status) {
	case CHARGER_STATE_FAULT:
	case CHARGER_STATE_NOCHRG:
	case CHARGER_STATE_DIS:
		/* recover from fault, or power up */
		if (batt_v < BATTV_MIN) {
			/* battery too low, don't try */
			charger_disable();
			LEDBATT_R = 1;
			BATT_ON = 0;
			return;
		}
		LEDBATT_R = 0;
		LEDBATT_G = 1;
		BATT_ON = 1;
		if (batt_v < BATTV_MIN_BULK) {
			CHRG_BULK = 1;
			charger_status = CHARGER_STATE_BULK;
			time_on_bulk = 0;
		} else {
			CHRG_BULK = 0;
			charger_status = CHARGER_STATE_FLOAT;
		}
		return;
	case CHARGER_STATE_FLOAT:
		LEDBATT_G = 1;
		LEDBATT_R = 0;
		CHRG_BULK = 0;
		return;
	case CHARGER_STATE_BULK:	
		time_on_bulk++;
		LEDBATT_G = LEDBATT_G ^ 1;
		if (time_on_bulk > BATT_BULK_TIME) {
			charger_status = CHARGER_STATE_FLOAT;
			CHRG_BULK = 0;
		}
		if (batt_v > BATTV_STOP_BULK && batt_i < BATTI_STOP_BULK) {
			charger_status = CHARGER_STATE_ABS;
			time_on_abs = 0;
		}
		return;
	case CHARGER_STATE_ABS:	
		time_on_abs++;
		LEDBATT_G = LEDBATT_G ^ 1;
		if (time_on_abs > time_on_bulk ||
		    batt_i < BATTI_STOP_ABS) {
			charger_status = CHARGER_STATE_FLOAT;
			CHRG_BULK = 0;
		}
		return;
	}
fail:
	charger_status = CHARGER_STATE_FAULT;
	LEDBATT_G = 0;
	LEDBATT_R = 0;
	BATT_ON = 0;
	CHRG_BULK = 0;
	return;
}

void
main(void) __naked
{
	char c;
	static unsigned int poll_count;
	static uint16_t i2cval;

	sid = 0;

	softintrs.byte = 0;
	counter_10hz = 25;
	counter_1hz = 10;

	batt_temp = 0xffff;

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

	LEDBATT_R = 1;
	LEDBATT_G = 1;
	LEDPWR_R = 0;
	LEDPWR_G = 0;
	BATT_ON = 0;
	CHRG_BULK = 0;
	TRISCbits.TRISC2 = 0;
	TRISCbits.TRISC6 = 0;
	TRISCbits.TRISC0 = 0;
	TRISCbits.TRISC1 = 0;
	TRISCbits.TRISC7 = 0;
	TRISBbits.TRISB1 = 0;

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

	LEDPWR_R = 1;
	LEDBATT_R = 0;
	LEDBATT_G = 0;

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

	/* set up ADC */
	PIR1bits.ADIF = 0;
	ADCON0 = (1 << 2); /* select channel 1 */
	ADCON1 = 0x20; /* vref = 2.0 */
	ADCON2 = 0xbd; /* Right justified, 20Tad Aq, Fosc/16 */
	ADCON0bits.ADON = 1;

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
	LEDPWR_R = 0;

	ina226_ready = 1;
	if (i2c_readreg(INA226_ADDR, INA_MANUF, &i2cval) == 0) {
		printf("i2c INA_MANUF fail\n");
		ina226_ready = 0;
	} else
		printf("INA226 manuf 0x%x", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_DIE, &i2cval) == 0) {
		printf("i2c INA_DIE fail\n");
		ina226_ready = 0;
	} else
		printf(" die 0x%x\n", i2cval);

	i2cval = 0x4d27; /* b0100110100100111
			  * average = 512, timev = 1.1ms, timec = 1.1ms,
			  * continous bus & shut
			  */
	 if (i2c_writereg(INA226_ADDR, INA_CONFIG, i2cval) == 0) {
		printf("i2cwr INA_CONFIG fail\n");
		ina226_ready = 0;
	}

	/*
	 * current_lsb = Imax / 2^15 = 3 / 2^15 = 91.55uA
	 * choose current_lsb = 100uA
	 * CAL = 0.00512 / (current_lsb * Rshunt) = 0.00512 / 100u * 0.039
	 * CAL = 1312.8
	 */
	 i2cval = 1313;
	 if (i2c_writereg(INA226_ADDR, INA_CAL, i2cval) == 0) {
		printf("i2cwr INA_CAL fail\n");
		ina226_ready = 0;
	}
	if (i2c_readreg(INA226_ADDR, INA_CONFIG, &i2cval) == 0) {
		printf("i2c INA_CONFIG fail\n");
		ina226_ready = 0;
	} else
		printf("INA226 config 0x%x", i2cval);
	if (i2c_readreg(INA226_ADDR, INA_CAL, &i2cval) == 0) {
		printf("i2c INA_CAL fail\n");
		ina226_ready = 0;
	} else
		printf(" cal 0x%x\n", i2cval);

	input_volt = 0xffff;
	charger_status = CHARGER_STATE_NOCHRG;
	power_status = UNKOWN;

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
		if (PIR1bits.ADIF) {
			PIR1bits.ADIF = 0;
			a2d_acc = ((unsigned int)ADRESH << 8) | ADRESL;
			if (((ADCON0 >> 2) & 0xf) == 0) {
				/* channel 0: NTC */
				adctotemp();
				ADCON0bits.ADON = 0;
				ADCON0 = (1 << 2); /* select channel 1 */
				ADCON1 = 0x20; /* vref = 2.0 */
				ADCON0bits.ADON = 1;
			} else {
				/* channel 1: voltage */
				handle_input_voltage();
				send_dc_status();
				send_charger_status();
				ADCON0bits.ADON = 0;
				ADCON0 = (0 << 2); /* select channel 1 */
				ADCON1 = 0; /* vref = Vdd */
				ADCON0bits.ADON = 1;
			}
		}
		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;
			counter_1hz--;
			if (counter_1hz == 0) {
				sid++;
				if (sid == 0xfe)
					sid = 0;
				counter_1hz = 10;
				read_battery_data();
				send_batt_status();
				ADCON0bits.GO = 1;
noi2c:
				{}
			}
		}

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

#if 0
	if (PIR5 != 0)
		PIE5 &= _PIE5_TXBnIE;
#endif
}
