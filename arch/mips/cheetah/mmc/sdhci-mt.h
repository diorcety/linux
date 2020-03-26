#ifndef __SDHCI_MT_HW_H
#define __SDHCI_MT_HW_H

#ifdef CONFIG_CHEETAH
#include <asm/mach-cheetah/cheetah.h>
#endif

/* 60-6F reserved */
#define SDHCI_HOST_CONTROL3		0x70
#define  SDHCI_CLEAR_INTERNAL_FIFO	0x02000000
#define  SDHCI_INVERSE_SD_CLOCK		0x00000800
/* 74-FB reserved */

#define SDHCI_QUIRK2_FORCE_AHB				(1<<1)
#define SDHCI_QUIRK2_FORCE_SDMA				(1<<2)
#define SDHCI_QUIRK2_NO_UHSI				(1<<3)
#define SDHCI_QUIRK2_64BIT_ADMA_ADDR			(1<<4)
#define SDHCI_QUIRK2_NO_ADMA_ALIGNMENT			(1<<5)
#define SDHCI_QUIRK2_RESET_CLOCK_AFTER_PRESET		(1<<6)
#define SDHCI_QUIRK2_TRIGGER_CARD_DETECTION		(1<<7)
#define SDHCI_QUIRK2_MAX_CLOCK_VAL			(1<<8)

struct sdhci_mt {
	struct sdhci_host	*host;
	struct platform_device	*pdev;
	struct resource		*ioarea;
	unsigned int		cur_clk;
	unsigned int        max_clock;
};

#endif /* __SDHCI_MT_HW_H */
