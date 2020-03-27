#
#	Copyright (c) 2013	Montage Inc.	All rights reserved. 
#
EXTRA_CFLAGS += -Werror
NOSTDINC_FLAGS += -I$(src)/../wla/include/
obj-$(CONFIG_CHEETAH) += init.o setup.o reset.o prom.o time.o int.o gpio.o platform.o register.o mactbl.o clk.o pwr.o serial.o

obj-$(CONFIG_CHEETAH_INTERNAL_DEBUGGER)	+= idb.o

obj-$(CONFIG_CHEETAH_GSPI) += gspi_dev.o cta_gspi.o gspi.o 

obj-$(CONFIG_CHEETAH_BOOTLOADER_CDB) += boot_cdb.o

obj-$(CONFIG_CHEETAH_GDMA) += gdma.o

obj-$(CONFIG_CHEETAH_GVCOM) += gpiovcom.o

obj-$(CONFIG_CHEETAH_GI2C) += gi2c.o

obj-$(CONFIG_SRAM_ENABLE) += sram.o

obj-$(CONFIG_CHEETAH_KEYPAD) += cta_keypad.o

obj-$(CONFIG_CHEETAH_GPIO_KEYS_POLLED) += cta_gpio_hotplug.o

obj-$(CONFIG_CHEETAH_ANA_KEYPAD) += cta_ana_keypad.o

obj-$(CONFIG_CHEETAH_ST7565P_LCD) += st7565p_lcd.o

obj-$(CONFIG_CHEETAH_ST7588T_LCD) += st7588t_lcd.o

obj-$(CONFIG_CHEETAH_MADC) += cta_madc.o

obj-$(CONFIG_CHEETAH_PWM) += cta_pwm.o

obj-$(CONFIG_CHEETAH_GPIO_ROTARY_ENCODER) += rotary_encoder.o

obj-$(CONFIG_MTD_CHEETAH) += cta_flash.o

obj-$(CONFIG_LEDS_TRIGGER_CTAINTF) += ledtrig-ctaintf.o

#obj-$(CONFIG_EARLY_PRINTK)	+=

#EXTRA_CFLAGS += -Werror
#
# 	Uncomment to use Switch Port0 as WAN port
#
#EXTRA_CFLAGS += -DCML_WAN_P0

