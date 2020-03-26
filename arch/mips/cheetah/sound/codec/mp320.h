/*
 * Copyright 2005 Openedhand Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

//#ifndef _MP320_H
//#define _MP320_H

/* MP320 register space */

#define MP320_LINVOL    0x00
#define MP320_RINVOL    0x01
#define MP320_LOUT1V    0x02
#define MP320_ROUT1V    0x03
#define MP320_ADCDAC    0x05
#define MP320_IFACE     0x07
#define MP320_SRATE     0x08
#define MP320_LDAC      0x0a
#define MP320_RDAC      0x0b
#define MP320_BASS      0x0c
#define MP320_TREBLE    0x0d
#define MP320_RESET     0x0f
#define MP320_3D        0x10
#define MP320_ALC1      0x11
#define MP320_ALC2      0x12
#define MP320_ALC3      0x13
#define MP320_NGATE     0x14
#define MP320_LADC      0x15
#define MP320_RADC      0x16
#define MP320_ADCTL1    0x17
#define MP320_ADCTL2    0x18
#define MP320_PWR1      0x19
#define MP320_PWR2      0x1a
#define MP320_ADCTL3    0x1b
#define MP320_ADCIN     0x1f
#define MP320_LADCIN    0x20
#define MP320_RADCIN    0x21
#define MP320_LOUTM1    0x22
#define MP320_LOUTM2    0x23
#define MP320_ROUTM1    0x24
#define MP320_ROUTM2    0x25
#define MP320_MOUTM1    0x26
#define MP320_MOUTM2    0x27
#define MP320_LOUT2V    0x28
#define MP320_ROUT2V    0x29
#define MP320_MOUTV     0x2a

#define MP320_CACHE_REGNUM 0x2a

#define MP320_SYSCLK	0

//#endif
