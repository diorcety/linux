EXTRA_CFLAGS += -Werror
obj-$(CONFIG_CHEETAH_ETH) += cta_eth.o cta_switch.o cta_hnat.o  cta_hnat_sdmz.o
obj-$(CONFIG_NETFILTER) += nf_hnat.o

ifeq "" "$(wildcard $(srctree)/$(src)/cta_hnat.c)"
$(obj)/cta_hnat.o: $(src)/cta_hnat.o.gz
     gzip -d -c $< > $@
endif

