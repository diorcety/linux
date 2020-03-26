/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 *
 *  Routines to reset the Camelot platform
 *
 */

#include <linux/string.h>
#include <asm/mach-cheetah/cheetah.h>

void sram_init(void)
{
	if((0x80000000 & GPREG(TMIICLK)) == 0)
	{
		GPREG(SRM_LBND) = CONFIG_SRAM_BASE;
		GPREG(SRM_UBND) = CONFIG_SRAM_BASE + CONFIG_SRAM_SIZE*1024;
		GPREG(TMIICLK) = 0x80000000|GPREG(TMIICLK);
		
		memset((char *)(CONFIG_SRAM_BASE | 0xa0000000), 0, CONFIG_SRAM_SIZE*1024);
	}
}

