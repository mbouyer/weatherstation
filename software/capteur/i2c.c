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

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

static char i2c_writeaddrreg(const char, const char);

#if 0
#define DPRINTF(x) printf x
#else
#define DPRINTF(x) /**/
#endif

static void i2c_status(void);

#define I2C_WAIT_RX { \
	int i2c_wait_count = 0; \
	while (!I2C1STAT1bits.RXBF) { \
		i2c_wait_count++; \
		if (i2c_wait_count == 30000) { \
			printf(("I2C RX timeout\n")); \
			i2c_status(); \
			for (i2c_wait_count = 0; i2c_wait_count < 512; i2c_wait_count++) ; \
			return 0; __asm__("reset"); \
		} \
	} \
    }

#define I2C_WAIT_TX { \
	int i2c_wait_count = 0; \
	while (!I2C1STAT1bits.TXBE) { \
		i2c_wait_count++; \
		if (i2c_wait_count == 30000) { \
			printf("I2C TX timeout\n"); \
			i2c_status(); \
			for (i2c_wait_count = 0; i2c_wait_count < 512; i2c_wait_count++) ; \
			return 0; __asm__("reset"); \
		} \
	} \
    }

static inline uint8_t
i2c_wait_idle(void)
{
	int i2c_wait_count = 0;
	uint8_t tries = 0;
	while (I2C1STAT0bits.MMA) {
		i2c_wait_count++;
		if (i2c_wait_count == 10000) {
			if (tries > 0 || I2C1CON1bits.P) {
				printf(("I2C idle timeout\n"));
				i2c_status();
				return 0;
			}
			I2C1CON1bits.P = 1;
			tries++;
			i2c_wait_count = 0;
		}
	}
	return 1;
}

char
i2c_readvalue(const char address, uint16_t *data)
{
	char ret = 0;
	char b;
	uint16_t i2c_wait_count;

	if (i2c_wait_idle() == 0)
		return 0;

	I2C1STAT1bits.CLRBF = 1;
	I2C1STAT1 = 0;
	I2C1PIR = 0;
	I2C1ERR = 0;

	I2C1CON1bits.ACKCNT = 1;      
	I2C1CON1bits.ACKDT = 0;       
	I2C1ADB1 = address | 1;       
	I2C1CNTH = 0;
	I2C1CNTL = 2;
	I2C1CON0bits.S = 1;
	I2C1CON0bits.RSEN = 0;

	*data = 0;
	for (b = 0; b < 2; b++) {
		for (i2c_wait_count = 10000; i2c_wait_count > 0; i2c_wait_count--) {
			if (I2C1STAT1bits.RXBF)
				break;
		}
		if (i2c_wait_count == 0) {
			printf("I2C RX timeout: ");
			i2c_status();
			I2C1STAT1 = 0;
			I2C1ERR = 0;
			return 0;
		}
		*data |= I2C1RXB;     
		if (b == 0)
			*data = *data << 8;
	}
	return 1;
}

char
i2c_writecmd(const char address, char cmd)
{
        if (i2c_wait_idle() == 0)
		return 0;

	I2C1STAT1bits.CLRBF = 1;
	I2C1STAT1 = 0;
	I2C1PIR = 0;
	I2C1ERR = 0;

	/* send register address */   
	I2C1PIR = 0;
	I2C1CON0bits.RSEN = 0;
	I2C1CON1bits.ACKDT = 0;       
	I2C1CON1bits.ACKCNT = 1;
	I2C1ADB1 = address;
	I2C1CNTH = 0;
	I2C1CNTL = 1;
	I2C1TXB = cmd;
	I2C1CON0bits.S = 1;
	I2C_WAIT_TX;
	if (I2C1ERRbits.NACKIF)       
		return 0;
	return 1;
}

char
i2c_writereg(const char address, char reg, uint8_t data)
{
	uint8_t i;

	if (i2c_wait_idle() == 0)     
		return 0;

	/* send register address */
	I2C1CON0bits.RSEN = 0;
	I2C1CON1bits.ACKDT = 0;
	I2C1CON1bits.ACKCNT = 1;
	I2C1ADB1 = address;
	I2C1CNTH = 0;
	I2C1CNTL = 2;
	I2C1TXB = reg;
	I2C1CON0bits.S = 1;
	I2C_WAIT_TX;
	I2C1TXB = data;
	return 1;
}

static void
i2c_status(void)
{
	printf("CON 0x%x 0x%x 0x%x", I2C1CON0, I2C1CON1, I2C1CON2);
	printf(" STAT 0x%x 0x%x PIR 0x%x", I2C1STAT0, I2C1STAT1, I2C1PIR);
	printf(" ERR 0x%x CNT 0x%x 0x%x\n", I2C1ERR, I2C1CNTH, I2C1CNTL);
}
