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
#include <stdlib.h>
#include <i2c.h>

static char i2c_writeaddrreg(const char, const char);

#if 0
#define DPRINTF(x) printf x
#else
#define DPRINTF(x) /**/
#endif

#define I2C_WAIT { \
	int i2c_wait_count = 0; \
	while (!PIR1bits.SSPIF) { \
		i2c_wait_count++; \
		if (i2c_wait_count == 10000) { \
			printf(("I2C timeout\n")); \
			goto end; \
		} \
	} \
    }
#define I2C_CLEAR { PIR1bits.SSPIF = 0; }

static char
i2c_writeaddrreg(const char address, const char reg)
{
	/* wait for module idle */    
	while (((SSPSTAT & 0x04) | (SSPCON2 & 0x1f)) != 0) {
		DPRINTF(("writeaddrreg transmit not idle\n"));
		; /* wait */
	}
	/* generate start */
	I2C_CLEAR;
	SSPCON2 = _SEN; /* SSPCON2bits.SEN=1; */
	I2C_WAIT;

	/* transmit address */
	I2C_CLEAR;
	SSPCON2 = _ACKSTAT; /* SSPCON2bits.ACKSTAT=1; */
	SSPBUF = (address << 1);      
	DPRINTF(("send address "));
	I2C_WAIT
	if (SSPCON2bits.ACKSTAT) {    
		printf("i2c_writeaddrreg: addr no ack\n");
		return 0;
	}
	DPRINTF(("done\n"));
	/* transmit reg */
	I2C_CLEAR;
	SSPCON2 = _ACKSTAT; /* SSPCON2bits.ACKSTAT=1; */
	SSPBUF = reg;
	DPRINTF(("send reg "));
	I2C_WAIT;

	if (SSPCON2bits.ACKSTAT) {    
		DPRINTF(("i2c_writeaddrreg: reg no ack\n"));
		return 0;
	}
	DPRINTF(("done\n"));
	return 1;
end:
	return 0;
}

char
i2c_readvalue(const char address, uint16_t *data)
{
	char ret = 0;
	char b;

	/* wait for module idle */    
	while (((SSPSTAT & 0x04) | (SSPCON2 & 0x1f)) != 0) {
		DPRINTF(("i2c_readreg transmit not idle\n"));
		; /* wait */
	}
	/* generate start */
	I2C_CLEAR;
	SSPCON2 = _SEN; /* SSPCON2bits.SEN=1; */
	I2C_WAIT;

	/* transmit address */
	I2C_CLEAR;
	SSPCON2 = _ACKSTAT; /* SSPCON2bits.ACKSTAT=1; */
	SSPBUF = ((address << 1) | 0x01);
	/* printf("send address2 "); */
	I2C_WAIT

	if (SSPCON2bits.ACKSTAT) {    
		printf("i2c_readreg: addr no ack\n");
		goto stop;
	}
	/* printf("done\n"); */       

	*data = 0;
	for (b = 0; b < 2; b++) {
		I2C_CLEAR;
		SSPCON2bits.RCEN=1;   
		/* printf("receive "); */
		I2C_WAIT;

		*data |= SSPBUF;     
		if (b == 0)
			*data = *data << 8;
		/* printf("BF OK 0x%x\n", *data); */
		I2C_CLEAR;
		if (b == 0)
			SSPCON2bits.ACKDT=0;
		else
			SSPCON2bits.ACKDT=1;
		/* send an ack */     
		SSPCON2bits.ACKEN = 1;
		/* printf("ACK "); */ 
		I2C_WAIT;
		/* printf("ACK OK\n"); */
	}
	ret = 1;
stop:
	/* generate a stop */
	I2C_CLEAR;
	SSPCON2bits.PEN=1;
	I2C_WAIT;
end:
	I2C_CLEAR;
	return ret;
}

char
i2c_writecmd(const char address, char cmd)
{
	char ret = 0;

	/* start transaction and select register */
	if (i2c_writeaddrreg(address, cmd) == 0)
		goto stop;

	ret = 1;
stop:
	/* generate a stop */
	I2C_CLEAR;
	SSPCON2bits.PEN=1;
	I2C_WAIT;
end:
	I2C_CLEAR;
	return ret;
}

char
i2c_writereg(const char address, char reg, uint8_t data)
{
	char ret = 0;

	/* start transaction and select register */
	if (i2c_writeaddrreg(address, reg) == 0)
		goto stop;

	/* transmit data */
	I2C_CLEAR;
	SSPBUF = data;
	/* printf("send data "); */   
	I2C_WAIT

	if (SSPCON2bits.ACKSTAT) {    
		printf("i2c_writereg: data no ack\n");
		goto stop;
	}
	/* printf("done\n"); */       

	ret = 1;
stop:
	/* generate a stop */
	I2C_CLEAR;
	SSPCON2bits.PEN=1;
	I2C_WAIT;
end:
	I2C_CLEAR;
	return ret;
}
