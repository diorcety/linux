# for backports system

ifdef CONFIG_LINUX_WLA 
NOSTDINC_FLAGS += $(LINUXINCLUDE) -Werror
NOSTDINC_FLAGS += -I$(src)/include/
NOSTDINC_FLAGS += -include $(BACKPORT_PWD)/net/mac80211/ieee80211_i.h -include $(BACKPORT_PWD)/net/wireless/core.h 

NOSTDINC_FLAGS += -DCONFIG_CHEETAH_INTERNAL_DEBUGGER

MATH_OBJS = $(patsubst %, linux/math/%, e_asin.o e_log10.o e_log.o e_pow.o e_rem_pio2.o e_sqrt.o k_cos.o k_rem_pio2.o k_sin.o k_tan.o s_ceil.o s_copysign.o s_cos.o s_fabs.o s_finite.o \
			s_floor.o s_isnan.o s_scalbn.o s_sin.o s_tan.o)
endif

CORE_OBJS = $(patsubst %, core/%, bb.o beacon.o complex.o ip301.o mac_ctrl.o mac_init.o mac_sta.o mat.o rate.o rfac.o rfac_rfc_patch.o rf.o rfc_comm.o rfc_new.o \
			txrx.o util.o)

ifdef CONFIG_LINUX_WLA 
$(obj)/lib.a: $(src)/linux/libgcc.a.gz
	@gzip -d -c $< > $(obj)/lib.a
endif

ifdef CONFIG_LINUX_WLA 
cheetah_wla-y = $(patsubst %, linux/%, main.o resource.o smartcfg.o debugfs.o procfs.o wtfs.o)
cheetah_wla-y += $(MATH_OBJS) $(CORE_OBJS) lib.a
endif

obj-$(CONFIG_CHEETAH_WLA) = cheetah_wla.o
