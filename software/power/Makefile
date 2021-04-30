.SUFFIXES: .o .hex

.PATH: ${.CURDIR}/../../../pic18_n2k

CC= sdcc  --use-non-free --Werror --fsigned-char
CFLAGS= --fomit-frame-pointer --denable-peeps --optimize-cmp --optimize-df
CFLAGS+= -I${.CURDIR} -I${.CURDIR}/../../../pic18_n2k
OBJECTS= main.o serial.o nmea2000.o raddeg.o i2c.o ntc_tab.o
HEADERS= serial.h nmea2000.h nmea2000_pgn.h nmea2000_user.h raddeg.h i2c.h
HEADERS+= ntc_tab.h

all: power.hex
bin: power.bin

obj:
	mkdir obj

power.bin: power.hex
	hex2bin1k ${.ALLSRC} ${.TARGET}
	@ls -lh ${.TARGET}

power.hex: ${OBJECTS}
	${CC} -V -Wl,-s${.CURDIR}/pic18f26k80.lkr --no-crt --ivt-loc=0x400 --obanksel=2 -mpic16 -p18f26k80 -Wl,libc18f.lib ${.ALLSRC} -o ${.TARGET} -llibm18f.lib

${OBJECTS}: ${HEADERS} Makefile

.c.o:
	${CC} ${CFLAGS} -V --no-crt --ivt-loc=0x400 --obanksel=2 -mpic16 -p18f26k80 -c ${.IMPSRC} -o ${.TARGET}

clean:
	rm -f power.bin *.o *.hex *.cod *.lst *.asm
