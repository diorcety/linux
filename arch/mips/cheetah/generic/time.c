/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>

#include <asm/time.h>
#include <asm/io.h>
#include <asm/mach-cheetah/cheetah.h>

/* To enable tickless system , please enable CONFIG_NO_HZ & CONFIG_HIGH_RES_TIMERS in kernel config */

/* 
   For Camelot 1.0, TIMER0 need to be used as continuous timer for tickless system, TIMER1 is selected for event timer  
   Hence, timer0 & timer1 are both used in IP3210 
   The system use TIMER1 for event timer (on both tick & tickless configuration).
   Also, the register width of both timer is 20bits.

   For Camelot 2.0~, TIMER0 is used as continuous timer and event timer
   The TIMER0 has register width as 32bits, while the width of TIMER1 remains unchanged (20bits).
 */

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
#define CHEETAH_TIMER_IRQ   IRQ_TMR0
#else
#define CHEETAH_TIMER_IRQ   IRQ_TMR1
#endif

#define TCN_INI         (1<<2)|(1<<1) //(1<<2)|(1<<1)|(1<<0)    /* b2: go, b1: ie, b0: auto-reload */

#define TCN_BIT_AUTO_RELOAD     0x01UL
#define TCN_BIT_INTR_ENABLE     0x02UL
#define TCN_BIT_GO              0x04UL

#if defined(CONFIG_NO_HZ)
#define ONE_MHZ     1000000
#define TIMER_PR_INI    ((SYS_CLK/ONE_MHZ) - 1)
#define TICKS_PER_SEC   (SYS_CLK/(TIMER_PR_INI+1))
#define TICKS_PER_JIFFY (TICKS_PER_SEC/(HZ))
#else
/* engineered to be workable for CPU up to 500Mhz for 20bits counter in Camelot 1.0 */
#define TIMER_PR_INI        4
#define TICKS_PER_SEC   (SYS_CLK/(TIMER_PR_INI+1))
#define TICKS_PER_JIFFY (TICKS_PER_SEC/(HZ))
#endif

#if (CHEETAH_TIMER_IRQ == IRQ_TMR1)
#define TIMER_CR    T1CR
#define TIMER_LR    T1LR
#define TIMER_PR    T1PR
#define TIMER_CN    T1CN
#define TIMER_IS    T1IS
#endif

#if (CHEETAH_TIMER_IRQ == IRQ_TMR0)
#define TIMER_CR    T0CR
#define TIMER_LR    T0LR
#define TIMER_PR    T0PR
#define TIMER_CN    T0CN
#define TIMER_IS    T0IS
#endif

#if defined(CONFIG_CHEETAH_HW_TIMER)
#define TIMER_HW	0xf
#endif

#if defined(CONFIG_NO_HZ) && defined(CONFIG_HIGH_RES_TIMERS)
unsigned long long notrace sched_clock(void)
{
    static unsigned long last_jiff = INITIAL_JIFFIES - (100 * HZ);
    static unsigned long last_counter = 0;
    static unsigned long long ts = 0;

    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    unsigned long cur_jiff;
    unsigned long cur_counter;

    unsigned long flags;

    raw_local_irq_save(flags);

    cur_jiff = jiffies;
    cur_counter = pt[T0CR];

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
    /*
        CAMELOT 2.0 has 32bit counter, it will rollover in about 14 seconds when operating in 300Mhz mode
        (on 300Mhz mode, the PR_INI value is 0, which is identical to CPU clock rate)
     */
    if(time_after(cur_jiff, last_jiff + 10 * HZ))
#else
    if(time_after(cur_jiff, last_jiff + HZ))
#endif
    {
        ts = (unsigned long long)(cur_jiff - INITIAL_JIFFIES) * (NSEC_PER_SEC / HZ);
    }
    else
    {
        if(cur_counter > last_counter)
        {
            ts += (cur_counter - last_counter) * 1000;
        }
        else
        {
            ts += (last_counter + 0x100000 - cur_counter) * 1000;
        }
    }

    last_jiff = cur_jiff;
    last_counter = cur_counter;

    raw_local_irq_restore(flags);

    //printk("%lld\n", ts);

    return ts;
}
#endif

static void camelot_set_state_shutdown(struct clock_event_device *evt)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[TIMER_CN] &= ~TCN_BIT_INTR_ENABLE;
}

static void camelot_set_state_periodic(struct clock_event_device *evt)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[TIMER_CN] |= (TCN_BIT_AUTO_RELOAD | TCN_BIT_INTR_ENABLE);
}

static void camelot_set_state_oneshot(struct clock_event_device *evt)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[TIMER_CN] &= ~TCN_BIT_AUTO_RELOAD;
    pt[TIMER_CN] |= TCN_BIT_INTR_ENABLE;
}

static void camelot_tick_resume(struct clock_event_device *evt)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[TIMER_CN] |= TCN_BIT_INTR_ENABLE;
}

#if defined(CONFIG_NO_HZ)

static int camelot_next_event(unsigned long delta, struct clock_event_device *evt)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
	#if defined(CONFIG_CHEETAH_HW_TIMER)
		pt[TIMER_HW] = (0x80000000 | (delta / 1000));
	#else
		pt[TIMER_LR] = (pt[TIMER_CR] + (delta / 1000));
	#endif	
#else
    pt[TIMER_CR] = 0;
    pt[TIMER_LR] = (delta / 1000);
    pt[TIMER_CN] |= TCN_BIT_INTR_ENABLE;
#endif

	return 0;
}
#endif

struct clock_event_device camelot_clockevent = {
    .name       = "camelot",
#if defined(CONFIG_NO_HZ)
    .features   = CLOCK_EVT_FEAT_ONESHOT,
#else
    .features   = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
#endif
    .rating     = 100,
    .irq        = CHEETAH_TIMER_IRQ,
    .set_state_periodic = camelot_set_state_periodic,
    .set_state_oneshot = camelot_set_state_oneshot,
    .set_state_shutdown = camelot_set_state_shutdown,
    .tick_resume = camelot_tick_resume,
    .mult       = 1,
#if defined(CONFIG_NO_HZ)
    .set_next_event	= camelot_next_event,
    .max_delta_ns = (ONE_MHZ / 2) * 1000,
    .min_delta_ns = 1000,
#endif
};

#if defined(CONFIG_NO_HZ)

__init void camelot_timer_init(void)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;
    unsigned int svn_version = *(volatile unsigned int*)0xaf0050dc;

    pt[TIMER_CN] = 0;
#if defined(CONFIG_EPHY_GLITCH_PATCH) 
#define NEW_TIMER_CLK 25000000 //25MHz
#else
#define NEW_TIMER_CLK 40000000 //40MHz
#endif
#define NEW_TIMER_CLK_VERSION 0x32813136
    if (svn_version >= NEW_TIMER_CLK_VERSION)
        pt[TIMER_PR] = ((NEW_TIMER_CLK/ONE_MHZ) - 1);
    else
    pt[TIMER_PR] = TIMER_PR_INI;

    pt[TIMER_CR] = 0;

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
    pt[TIMER_LR] = 0xffffffffUL;
#else
    pt[TIMER_LR] = 0xfffff;
#endif

    pt[TIMER_CN] = (TCN_BIT_INTR_ENABLE|TCN_BIT_GO);
}

static cycle_t camelot_timer_read(struct clocksource *cs)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
	return pt[TIMER_CR];
#else
	return pt[T0CR];
#endif
}

#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
static struct clocksource clocksource_camelot = {
	.name		= "CHEETAH",
	.read		= camelot_timer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};
#else
static struct clocksource clocksource_camelot = {
	.name		= "CHEETAH",
	.read		= camelot_timer_read,
	.mask		= CLOCKSOURCE_MASK(20),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};
#endif

int __init init_camelot_clocksource(void)
{
#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
    /* Camelot 2.0~ do not need 2nd timer to use as continuous counter */
#else
    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[T0CR] = 0;
    pt[T0LR] = 0x0fffff;
    pt[T0PR] = TIMER_PR_INI;
    pt[T0CN] = (TCN_BIT_AUTO_RELOAD | TCN_BIT_GO);
#endif

	/* Calculate a somewhat reasonable rating value */
	clocksource_camelot.rating = 200;

	clocksource_register_hz(&clocksource_camelot, ONE_MHZ);

	return 0;
}

#else

__init void camelot_timer_init(void)
{
    volatile unsigned int *pt = (unsigned int *) TM_BASE;
    unsigned int svn_version = *(volatile unsigned int*)0xaf0050dc;

    pt[TIMER_CR] = 0;
#define NEW_TIMER_CLK 40000000 //40MHz
#define NEW_TIMER_CLK_VERSION 0x32813136
    if (svn_version >= NEW_TIMER_CLK_VERSION)
        pt[TIMER_LR] = (NEW_TIMER_CLK/(TIMER_PR_INI+1)/(HZ));
    else
    pt[TIMER_LR] = TICKS_PER_JIFFY;
    pt[TIMER_PR] = TIMER_PR_INI;
    pt[TIMER_CN] = (TCN_BIT_AUTO_RELOAD  | TCN_BIT_INTR_ENABLE | TCN_BIT_GO);
}

#endif

static irqreturn_t camelot_timer_interrupt(int irq, void *dev_id)
{
    struct clock_event_device *cd = dev_id;

    volatile unsigned int *pt = (unsigned int *) TM_BASE;

    pt[TIMER_IS]=1;
    cd->event_handler(cd);

    return IRQ_HANDLED;
}

static struct irqaction camelot_timer_irqaction = {
    .handler    = camelot_timer_interrupt,
    .flags      = 0,
//  .mask       = CPU_MASK_NONE,
    .name       = "TIMER",
};

void __init plat_time_init(void)
{
    struct clock_event_device *cd = &camelot_clockevent;
    struct irqaction *action = &camelot_timer_irqaction;

#if !defined(CONFIG_NO_HZ)
    BUG_ON(HZ != 100);
#endif

    cd->cpumask = cpumask_of(0);

    action->dev_id = cd;
    setup_irq(CHEETAH_TIMER_IRQ, action);

    camelot_timer_init();

    clockevents_register_device(cd);

#if defined(CONFIG_NO_HZ)
    init_camelot_clocksource();
#endif
}

