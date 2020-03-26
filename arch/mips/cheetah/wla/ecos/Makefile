OBJS = $(patsubst %, ../core/%, bb.o beacon.o complex.o ip301.o mac_ctrl.o mac_init.o mac_sta.o mat.o rate.o rfac.o rfac_rfc_patch.o rf.o rfc_comm.o rfc_new.o \
			txrx.o util.o)
 
OBJS += wla_main.o wla_dbg.o wla_test.o resource.o

EXTRA_CFLAGS = -Werror -I../include 
include $(TOPDIR)/include/rules.mak
all: $(MO)
clean:
	rm -f $(MO) $(OBJS)

