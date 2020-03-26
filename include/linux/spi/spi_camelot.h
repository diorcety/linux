/*=============================================================================+
|                                                                              |
| Copyright 2008                                                               |
| Acrospeed Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file 
*   \brief CAMELOT 
*   \author Acrospeed
*/
#ifndef _SPI_CAMELOT_H_
#define _SPI_CAMELOT_H_
#if defined(CONFIG_MIPS_CAMELOT)
#include <asm/camelot/camelot.h>
#elif defined(CONFIG_CHEETAH)
#include <asm/mach-cheetah/cheetah.h>
#endif

#define SPI_INIT_MASK		0xFFFE00C7 //bit 16~8, 5~3
#define SPI_DATA_INVERT		1<<16
#define SPI_CLK_W_OFFSET	8
#define SPI_RESET			1<<7		/* 1: reset 0: normal operation */
#define SPI_KEEP_CS			1<<6
#define SPI_CLK_PARK_HIGH	1<<5
#define SPI_THREE_WIRE		1<<4
#define SPI_FALLING_SAMPLE	1<<3
#define SPI_TXEMPTY			1<<2
#define SPI_TXFULL			1<<1
#define SPI_RXNONEMPTY		1<<0

#define GSPI_CSR    (0x0)   /* GSPI Control/Status Register offset */
#define GSPI_CR     (0x4)   /* GSPI Control Register offset */
#define GSPI_DR     (0x8)   /* GSPI Data Register offset */

#define SPI_TX_WAIT_LOOP    10000
#define SPI_RX_WAIT_LOOP    10000

#define SPIREG(reg) (*(volatile unsigned int*)(reg))

#define spi_udelay(n) {int _www; for (_www=0; _www< 100; _www++); }

#define CML_GSPI_BUS_ID 0x1
#define CML_GSPI_CHIPSELECT_ONE 0x0
#define CML_GSPI_CHIPSELECT_TWO 0x1
#define CML_GSPI_CHIPSELECT_NUM 0x2 //number of chipselect(bank 1 bank 2 are both G-SPI)
#define CML_GSPI_MAX_HZ (SYS_CLK>>2)
#endif /* _SPI_CAMELOT_H_ */
