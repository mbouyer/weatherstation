ROM_BASE=600
IVECT_BASE=0x608

.SUFFIXES: .p1 .hex

.PATH: ${.CURDIR}/../../../pic18_n2k

CC= xc8-cc -mcpu=18f27q84 -mno-config -mkeep-startup -O2 -mcodeoffset=${ROM_BASE}
CFLAGS= -DIVECT_BASE=${IVECT_BASE}
CFLAGS+= -I${.CURDIR} -I${.CURDIR}/../../../pic18_n2k
CFLAGS+= -DN2K_PRINTF
OBJECTS= main.p1 serial.p1 nmea2000.p1 raddeg.p1 i2c.p1
HEADERS= serial.h nmea2000.h nmea2000_pgn.h nmea2000_user.h raddeg.h
HEADERS+= i2c.h
HEADERS+= nmea2000_pic18_ecan.c nmea2000_printf.c

all: capteur.hex

obj:
	mkdir obj

capteur.hex: ${OBJECTS}
	${CC} -Wa,-a -Wl,-Map=capteur.map  -o ${.TARGET} ${.ALLSRC}

${OBJECTS}: ${HEADERS} Makefile

.c.p1:
	${CC} ${CFLAGS} -c ${.IMPSRC} -o ${.TARGET}

clean:
	rm -f *.p1 *.hex
