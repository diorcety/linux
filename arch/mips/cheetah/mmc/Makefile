# Cheetah Platform Support
ifeq ($(CONFIG_MMC_DEBUG),y)
	EXTRA_CFLAGS        += -DDEBUG
endif

obj-$(CONFIG_CHEETAH_SDHCI) += sdhci.o sdhci-mt.o
obj-$(CONFIG_CHEETAH_MMCSPI) += mmc_spi.o

