/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 *
 *  Routines to reset the Cheetah platform
 *
 */
#include <linux/pm.h>
#include <asm/io.h>
#include <asm/reboot.h>
#include <asm/mach-cheetah/cheetah.h>

static void cheetah_machine_restart(char *command);
static void cheetah_machine_halt(void);
static void cheetah_machine_power_off(void);

#if defined(CONFIG_CHEETAH_WDT)
extern void cheetah_wdt_enable(void);
extern void cheetah_wdt_disable(void);
#endif

static void cheetah_machine_restart(char *command)
{
    volatile unsigned int *TimerReg = (unsigned int *) TM_BASE;

#if defined(CONFIG_CHEETAH_WDT)
    cheetah_wdt_disable();
#endif

    TimerReg[T2PR]=1;
    TimerReg[T2LR]=10;
    TimerReg[T2CR]=0;
    TimerReg[T2CN]=0xc;   /* b3: watch-dog , b2: start */
}

static void cheetah_machine_halt(void)
{
#if defined(CONFIG_CHEETAH_WDT)
    cheetah_wdt_disable();
#endif

    local_irq_disable();
    while(1);
}

static void cheetah_machine_power_off(void)
{
    printk("System halted. Please turn off power.\n");
    cheetah_machine_halt();
}

void cheetah_reboot_setup(void)
{
	_machine_restart = cheetah_machine_restart;
	_machine_halt = cheetah_machine_halt;
	pm_power_off = cheetah_machine_power_off;
}

