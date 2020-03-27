/* 
 *  Copyright (c) 2008, 2009    Montage Inc.    All rights reserved. 
 */
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel_stat.h>
#include <linux/kernel.h>
#include <linux/random.h>

#include <asm/traps.h>
#include <asm/irq_cpu.h>
#include <asm/irq_regs.h>
#include <asm/bitops.h>
#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/mach-cheetah/irq.h>
#include <asm/mach-cheetah/cheetah.h>

asmlinkage void plat_irq_dispatch(void)
{
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
	unsigned int pending = ic[ISTS];
    int i;
	
	pending &= 0xfffc;

#if 0		
    unsigned int pending = ( read_c0_cause() & read_c0_status())>>8;  //take b15-8
    u32 lx_ecause;

    __asm__ __volatile__ (
                ".word\t0x40680800\n"
                "nop\n"
                "move %0, $8\n"
                : "=r" (lx_ecause) /* output */
                : /* input */
                : "$8" /* clobbered register */ );

    pending =  (( pending & 0x00FF ) | (lx_ecause >> 8));
#endif
	// test each bit 
	for (i=15; i>1; i--)
		if (pending& (1<<i))
        	do_IRQ(i);
}

#ifdef CONFIG_MIPS_MT_SMP
    #error CONFIG_MIPS_MT_SMP not supported!
#endif

#define ALLINTS (IE_IRQ0 | IE_IRQ1 | IE_IRQ2 | IE_IRQ3 | IE_IRQ4 | IE_IRQ5)

static DEFINE_SPINLOCK(cheetah_irq_lock);

static inline void cheetah_mask_irq(struct irq_data *data)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    ic[IMSK] |= (1 << irq_nr);
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);
}

static inline void cheetah_ack_irq(struct irq_data *data)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    ic[ISTS] = (1 << irq_nr);
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);
}

static inline void cheetah_mask_ack_irq(struct irq_data *data)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    ic[IMSK] |= (1 << irq_nr);
    ic[ISTS] = (1 << irq_nr);
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);
}

static inline void cheetah_unmask_irq(struct irq_data *data)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    ic[IMSK] &= ~(1 << irq_nr);
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);

}

static inline void cheetah_unmask_ack_irq(struct irq_data *data)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    ic[ISTS] = (1 << irq_nr);
    ic[IMSK] &= ~(1 << irq_nr);
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);

}

static inline int cheetah_set_irq_type(struct irq_data *data, unsigned int flow_type)
{
    unsigned long flags;
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int irq_nr = data->irq;

    spin_lock_irqsave(&cheetah_irq_lock, flags);
    if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH) )
    	ic[IMSK] |= (1 << (irq_nr + NR_IRQS));
    else 
    	ic[IMSK] &= ~(1 << (irq_nr + NR_IRQS));
    spin_unlock_irqrestore(&cheetah_irq_lock, flags);
    return 0;
}

static struct irq_chip cheetah_irq_controller = {
    .name       = "Cheetah VIC",
    .irq_ack        = cheetah_ack_irq,
    .irq_mask       = cheetah_mask_irq,
    .irq_mask_ack   = cheetah_mask_ack_irq,
    .irq_unmask     = cheetah_unmask_irq,
    .irq_eoi        = cheetah_ack_irq,
    .irq_set_type   = cheetah_set_irq_type,
};

void __init init_cheetah_irqs(void)
{
    volatile unsigned int *ic = (unsigned int *) IC_BASE;
    unsigned int i;
#if 0
    u32 lx_estatus = 0x00FF0000UL;
    u32 lx_ebase = 0x80000400UL;
#endif
    /* Clear interrupts */
    /* Disable all CAMELOT interrupts. */
    ic[IMSK]=0xffffffff;          /* b31-16: pol , b15-0: mask */
    ic[ISTS]=0xffffffff;          /* b31-15: cause, b15-0: status */

    /* Disable all hardware interrupts */
    change_c0_status(ST0_IM, 0x00);

    /* re-routing the interrupt */
    ic[ISR1]=0xfeb56000;          /* tmr0=0xf, tmr1=0xe, sdio=0xb, i2s=0x5, gpiox=0x6 */
    ic[ISR2]=0x4a98d700;          /* mac=0xd, wifi=0x4, usb=0xa, gpio0=0x7,  */

#ifdef CONFIG_CHEETAH_GVCOM
#if CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM>0
    ic[IGPIOXSEL]=CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM;
#endif
#endif

#ifdef CONFIG_CHEETAH_GPIO_ROTARY_ENCODER
    ic[IGPIOXSEL]=CONFIG_CHEETAH_ROTARY_ENCODER_GPIO_A;
#endif

#ifdef CONFIG_CHEETAH_UART_RTSCTS
    ic[IGPIOXSEL]=CONFIG_CHEETAH_UART_CTS_GPIO_NUM;
#endif

    /* Initialize IRQ action handlers */
    for (i = 0; i < NR_IRQS; i++)
    {
        set_irq_chip_and_handler(i, &cheetah_irq_controller, handle_level_irq);
    }

    /* Enable all interrupts */
    change_c0_status(ST0_IM, ALLINTS);

    /* Enable interrupt 8~15 */
    /*
        Move From Lexra Coprocessor 0 Register
            MFLXC0 rT,LX0reg	(COP0 << 26) | (MFLX << 21) | (rT << 16) | (LX0reg << 11)

        Move To Lexra Coprocessor 0 Register
            MTLXC0 rT,LX0reg	(COP0 << 26) | (MTLX << 21) | (rT << 16) | (LX0reg << 11)

        COP0	0x10UL
        MFLX	0x03UL
        MTLX	0x07UL

        rT  r0 ~ r31
        LX0reg
            ESTATUS	0
            ECAUSE	1
            INTVEC	2

            MTLXC0 $8, INTVEC     ((0x10 << 26) | (0x07 << 21) | (8 < 16) | ( 2 << 11 ) )

            MTLXC0 $8, ESTATUS    ((0x10 << 26) | (0x07 << 21) | (8 < 16))
                
    */
#if 0
    __asm__ __volatile__ (
                "move $8, %1\n"
                ".word\t0x40E81000\n"
                "nop\n"
                "move $8, %0\n"
                ".word\t0x40E80000\n"
                "nop\n"
                : /* output */
                : "r" (lx_estatus), "r" (lx_ebase) /* input */
                : "$8" /* clobbered register */ );
#endif	
}

void __init arch_init_irq(void)
{
    init_cheetah_irqs();

#if defined(CONFIG_MIPS_MT_SMP)
#error CONFIG_MIPS_MT_SMP not supported!
#endif
}

