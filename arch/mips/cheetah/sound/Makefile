# Cheetah Platform Support

BACKPORT_PWD := $(shell pwd)/net/compat-wireless-2013-06-27
NOSTDINC_FLAGS := \
	-I$(BACKPORT_PWD)/backport-include/ \
	-I$(BACKPORT_PWD)/backport-include/uapi \
	-I$(BACKPORT_PWD)/include/ \
	-I$(BACKPORT_PWD)/include/uapi \
	-I$(BACKPORT_PWD)/include/drm \
	-include $(BACKPORT_PWD)/backport-include/backport/backport.h \
	$(CFLAGS)

obj-$(CONFIG_CHEETAH_SND) += cta-ai.o cta-i2s.o cta-pcm.o
obj-$(CONFIG_CHEETAH_SND_PCM0_NAU8822) += codec/nau8822.o
obj-$(CONFIG_CHEETAH_SND_PCM1_NAU8822) += codec/nau8822.o 
obj-$(CONFIG_CHEETAH_SND_PCM0_WM8988) += codec/wm8988.o
obj-$(CONFIG_CHEETAH_SND_PCM1_WM8988) += codec/wm8988.o
obj-$(CONFIG_CHEETAH_SND_PCM0_WM8750) += codec/wm8750.o
obj-$(CONFIG_CHEETAH_SND_PCM1_WM8750) += codec/wm8750.o
obj-$(CONFIG_CHEETAH_SND_PCM0_TAS5711) += codec/tas5711.o
obj-$(CONFIG_CHEETAH_SND_PCM1_TAS5711) += codec/tas5711.o
obj-$(CONFIG_CHEETAH_SND_PCM0_ES8316) += codec/es8316.o
obj-$(CONFIG_CHEETAH_SND_PCM1_ES8316) += codec/es8316.o
obj-$(CONFIG_CHEETAH_SND_PCM0_ES9023) += codec/es9023.o
obj-$(CONFIG_CHEETAH_SND_PCM1_ES9023) += codec/es9023.o
obj-$(CONFIG_CHEETAH_SND_PCM0_MP320) += codec/mp320.o
obj-$(CONFIG_CHEETAH_SND_PCM1_MP320) += codec/mp320.o
obj-$(CONFIG_CHEETAH_SND_PCM0_WM8737) += codec/wm8737.o
obj-$(CONFIG_CHEETAH_SND_PCM1_WM8737) += codec/wm8737.o
obj-$(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586) += codec/wm8737.o codec/ad83586.o
