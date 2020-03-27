/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 *
 *  PROM library initialisation code.  
 */
#include <linux/version.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/traps.h>

#include <asm/mach-cheetah/cheetah.h>

/* CPU: 60.95, 64, 80, 120, 160, 202.11, 240, 295.38, 320, 384, 480, 512, 640 MHz */
static unsigned int cpu_div[] = {
        0x7fc, 0x7f0, 0x7c0, 0x780, 0x680, 0x398, 0x380, 0x1d0, 0x280, 0x1a0, 0x180, 0x0f0, 0x0c0 };
static unsigned int cpu_tbl[] = {
        60950000, 64000000, 80000000, 120000000, 160000000, 202110000, 240000000, 295380000, 320000000, 384000000, 480000000, 
		512000000, 640000000};
/* SYS: 60.95, 64, 80, 120, 150.59, 160 MHz */
static unsigned int bus_div[] = { 0x7fc, 0x7f0, 0x7c0, 0x780, 0x688, 0x3c0 };
static unsigned int bus_tbl[] = { 60950000, 64000000, 80000000, 120000000, 150590000, 160000000};

int prom_argc;
int *_prom_argv, *_prom_envp;

static void __init mips_nmi_setup(void)
{
	void *base;
	extern char except_vec_nmi;

	base = cpu_has_veic ?
		(void *)(CAC_BASE + 0xa80) :
		(void *)(CAC_BASE + 0x380);
	memcpy(base, &except_vec_nmi, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

static void __init mips_ejtag_setup(void)
{
	void *base;
	extern char except_vec_ejtag_debug;

	base = cpu_has_veic ?
		(void *)(CAC_BASE + 0xa00) :
		(void *)(CAC_BASE + 0x300);
	memcpy(base, &except_vec_ejtag_debug, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}


/* Note: It is obsoleted to hardcode the default kernel cmdline in here 

   Please configure it in menuconfig
	Kernel hacking  --->  Default kernel command string

   The routine will try to fetch the kernel cmdline from the ASBOOT boot loader.
   If there is no cmdline passed from the ASBOOT, the default kernel cmdline is used.


   Here are some samples for Camelot kernel cmdline

    - start and wait GDB connect (with initrd)  
      WARNING: wrong rd_start/rd_size pair will trap the system on bootup

      "mem=0x1000000@0 rd_start=0x80800000 rd_size=0x400000 kgdbwait kgdboc=ttyS0"


    - enable remote GDB support

      "mem=0x1000000@0 rd_start=0x80800000 rd_size=0x400000 kgdboc=ttyS0"

    - normal boot up cmdline with rootfs on /dev/mtd3

      "root=1f03 mem=0x1000000@0 console=ttyS0 kgdboc=ttyS0"

    - nfsroot cmdline

      "mem=0x1000000@0 console=ttyS0 kgdboc=ttyS0 root=/dev/nfs rw nfsroot=/home/nfsroot,rw,nolock ip=192.168.2.20:192.168.2.7:192.168.2.1:255.255.255.0:evb:eth0:none"

    - openwrt cmdline , rootfs on /dev/mtd3

      "root=1f03 mem=0x1000000@0 console=ttyS0 kgdboc=ttyS0 init=/etc/preinit"
*/

/* BOOT_CMDLINE_DRAM_ADDR must be identical with ASBOOT definition */
#define BOOT_CMDLINE_DRAM_ADDR  0xA0000300

void  __init prom_init_cmdline(void)
{
    /* enable this after ASBOOT is ready to pass cmdline */
#if 0
    unsigned char *asboot_cmdline = (unsigned char *) BOOT_CMDLINE_DRAM_ADDR;

    if(asboot_cmdline[0] != '\0')
#else
    if(0)
#endif
    {
	memcpy( arcs_cmdline, (void *) BOOT_CMDLINE_DRAM_ADDR, COMMAND_LINE_SIZE);
    }
    else
    {
	strcpy( arcs_cmdline, CONFIG_CMDLINE);
    }

    arcs_cmdline[COMMAND_LINE_SIZE-1] = 0;
}

unsigned int CPU_CLK = DEFAULT_CPU_CLK;
unsigned int SYS_CLK = DEFAULT_SYS_CLK;

void __init probe_system_speed(void)
{
    unsigned int spd_ctrl;
	unsigned int cpu, sys;
	unsigned int i;

    spd_ctrl = ANAREG(CLKDIV);
	
	sys = spd_ctrl & 0xfff;
	cpu = (spd_ctrl >> 16) & 0xfff;
	
	/*Please reference boot code cpu_div and bus_div*/
	for(i=0; i < sizeof(cpu_div)/sizeof(cpu_div[0]); i++)
		if(cpu == cpu_div[i])
			CPU_CLK = cpu_tbl[i];
	
	for(i=0; i < sizeof(bus_div)/sizeof(bus_div[0]); i++)
		if(sys == bus_div[i])
			SYS_CLK = bus_tbl[i];
}

extern struct plat_smp_ops msmtc_smp_ops;
#include <asm/mipsregs.h>
void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *) fw_arg1;
	_prom_envp = (int *) fw_arg2;

	set_io_port_base(KUSEG);

	board_nmi_handler_setup = mips_nmi_setup;
	board_ejtag_handler_setup = mips_ejtag_setup;

#if !defined(CONFIG_CHEETAH_FPGA)
	probe_system_speed();
#endif

	prom_init_cmdline();

#ifdef CONFIG_MIPS_CMP
#error CONFIG_MIPS_MT_SMTC not implement yet!
	register_smp_ops(&cmp_smp_ops);
#endif
#ifdef CONFIG_MIPS_MT_SMP
#error CONFIG_MIPS_MT_SMTC not implement yet!
	register_smp_ops(&vsmp_smp_ops);
#endif
#ifdef CONFIG_MIPS_MT_SMTC
#error CONFIG_MIPS_MT_SMTC not implement yet!
	register_smp_ops(&msmtc_smp_ops);
#endif
}

