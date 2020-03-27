/*
 *  Copyright (c) 2013	Montage Inc.	All rights reserved.
 *
 *  Serial driver for Cheetah
 *
 */

#if defined(CONFIG_KGDB)
#include <linux/kgdb.h>
#endif

#if defined(CONFIG_MAGIC_SYSRQ)
    #define SUPPORT_SYSRQ

/* use Ctrl-Break-h to invoke the SYSRQ help menu */
    #include <linux/sysrq.h>
#endif

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/circ_buf.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/console.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#ifdef CONFIG_CHEETAH_UART_STAT
#include <linux/seq_file.h>
#endif
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bitops.h>

#include <asm/irq.h>
#include <asm/mach-cheetah/cheetah.h>
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	#include <asm/mach-cheetah/gpio.h>
#endif


#define DRV_NAME "ttyS"
extern void idelay(unsigned int count);

struct serial_state {
       int     baud_base;
       unsigned long   port;
       int     irq;
       int     flags;
       int     type;
       int     line;
       int     xmit_fifo_size;
       int     custom_divisor;
       int     count;
       unsigned short  close_delay;
       unsigned short  closing_wait; /* time to wait before closing */
       struct async_icount     icount;
       struct async_struct *info;
};

#define URCS_CTRL_MASK   (~((unsigned int)(1<<URCS_BRSHFT)-1)|URCS_TIE|URCS_RIE|URCS_PE|URCS_EVEN)
#define URCS_ENABLE(bitmap)  {\
    cta_uart_control|=bitmap;\
    URREG(URCS) = cta_uart_control; }
#define URCS_DISABLE(bitmap)  {\
    cta_uart_control&=~bitmap;\
    URREG(URCS) = cta_uart_control; }
#define URCS_UPDATE_BR(br)  {\
    cta_uart_control=(cta_uart_control&((unsigned int)(1<<URCS_BRSHFT)-1))|(br<<URCS_BRSHFT);\
    URREG(URCS) = cta_uart_control; }

static unsigned char hfuart_halfdup;
static unsigned int cta_uart_control;
static unsigned int cta_uart_status;
static void cta_uart_deliver(unsigned long x);
DECLARE_TASKLET(cta_uart_tasklet, cta_uart_deliver, 0); /* tasklet for dsr */

#ifdef CONFIG_CHEETAH_UART_RTSCTS
static unsigned int idelay_before_rx;
static bool ctsrts_en = true;
#endif

#if defined(CONFIG_MAGIC_SYSRQ)
static unsigned long break_pressed = 0;
#endif

static struct tty_driver *serial_driver;

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

static void rs_disable_tx_interrupts (void)
{
    unsigned long flags;

    local_irq_save(flags);

    URCS_DISABLE(URCS_TIE);

    local_irq_restore(flags);
}

static void transmit_chars(void);

static void rs_enable_tx_interrupts (void)
{
    unsigned long flags;

    local_irq_save(flags);

    URCS_ENABLE(URCS_TIE);

	transmit_chars();

    local_irq_restore(flags);
}

static void rs_disable_rx_interrupts (void)
{
    unsigned long flags;

    local_irq_save(flags);

    URCS_DISABLE(URCS_RIE);

    local_irq_restore(flags);
}

static void rs_enable_rx_interrupts (void)
{
    unsigned long flags;

    local_irq_save(flags);

    URCS_ENABLE(URCS_RIE);

    local_irq_restore(flags);
}

static struct serial_state *IRQ_ports;

static void change_speed(struct serial_state *info,
                         struct ktermios *old_termios)
{
	int baud,brsr = 0;

	if (!info->tty || !info->tty->termios)
		return;
	/* Determine divisor based on baud rate */
	baud = tty_get_baud_rate(info->tty);
	if (baud > 0)
		brsr = SYS_CLK/baud;

	/* Write the control registers */
	URCS_UPDATE_BR(brsr);
}

void cta_baudrate_chng(void)
{
	if (IRQ_ports)
		change_speed(IRQ_ports, NULL);
}

static void rs_wait_until_sent(struct tty_struct *tty, int timeout);


static struct serial_state rs_table[1];

#define NR_PORTS ARRAY_SIZE(rs_table)

#include <asm/uaccess.h>

#define serial_isroot()	(capable(CAP_SYS_ADMIN))

/*
 * ------------------------------------------------------------
 * rs_stop() and rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void rs_stop(struct tty_struct *tty)
{
    rs_disable_tx_interrupts();
}

static void rs_start(struct tty_struct *tty)
{
    struct serial_state *info = tty->driver_data;
    unsigned long flags;

    local_irq_save(flags);
    if (info->xmit.head != info->xmit.tail
        && info->xmit.buf
	)
    {
        rs_enable_tx_interrupts();
    }
    local_irq_restore(flags);
}

static void rs_sched_event(struct serial_state *info)
{
    tasklet_schedule(&info->tlet);
}

#if defined(CONFIG_KGDB)
void trigger_kgdb_breakpoint(void);
#endif
static void receive_chars(struct serial_state *info)
{
	struct tty_struct *tty = info->tty;
	unsigned char ch, flag;
	volatile int *p = (int *) UR_BASE;
	struct  async_icount *icount;
	int rx_loop = 0;
	int ubr;
#if defined(CONFIG_MAGIC_SYSRQ)
	static int zero_received = 0;
#endif

	flag = TTY_NORMAL;
	icount = &info->info->icount;

	do
	{
		if (!(URBR_RDY&(ubr = p[URBR]) )) //if no input, skip
			break;
		ch = ubr >>URBR_DTSHFT;
		icount->rx++;

#if defined(CONFIG_CHEETAH_SND)
		if (!(GPREG(PINMUX) & (EN_UART_FNC | EN_UART_AUX_FNC)))
		    continue;
#endif

#if defined(CONFIG_MAGIC_SYSRQ)

		/* cheetah UART does not implement line-break detection, we use this workaround to detect line break */
		/* detect consecutive 8 zero received to hint it is a line break */
		#define UART_LB_CONSECUTIVE_ZERO_THRESHOLD  8

		if(ch == 0)
		{
			zero_received++;

			if(zero_received > UART_LB_CONSECUTIVE_ZERO_THRESHOLD)
				break_pressed = jiffies;

			continue;
		}
		else
		{
			/* we have to drop first bytes received after LB */
			if((zero_received) && (break_pressed))
			{
				zero_received = 0;
				continue;
			}
			zero_received = 0;
		}
		/* end of line-break detection routine */

#if defined(CONFIG_KGDB)
		if((kgdb_connected) && (ch == 0x3))
		{
			break_pressed = 0;
			trigger_kgdb_breakpoint();
			continue;
		}
#endif

		if ( break_pressed )
		{
			if (time_before(jiffies, break_pressed + HZ*5))
			{
				if (ch == 0)
					continue;
				handle_sysrq(ch, tty);
				break_pressed = 0;
				continue;
			}
			break_pressed = 0;
		}
#endif
		tty_insert_flip_char(tty, ch, flag);
	} while (rx_loop++ < 256) ;

	// if rx some data, push it
	if (rx_loop)
		tty_flip_buffer_push(tty);

	return;
}

static void transmit_chars(void)
{
	struct serial_state *info = IRQ_ports;
	volatile int *p = (int *) UR_BASE;
	int empty=0;
#if 1
	int start=0;
#endif

	for (;;)
	{
#ifdef CONFIG_CHEETAH_UART_RTSCTS
		if (ctsrts_en && info->tty->hw_stopped)
			break;
#endif
		/* XXX: check fifo full */
		if ((p[URCS>>2]&URCS_TF))
			break;
		else
			if (info->x_char)
			{
				p[URBR>>2] = ((unsigned int)info->x_char)<<URBR_DTSHFT;
				info->info->icount.tx++;
				info->x_char = 0;
			}
			else
				if (info->xmit.head == info->xmit.tail
				|| info->tty->stopped || info->tty->hw_stopped)
				{
					empty=1;
					break;
				}
				else
				{
#if 1
					if(hfuart_halfdup && !start){
						gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);
						start = 1;
					}
#endif
					p[URBR>>2] = ((unsigned int)info->xmit.buf[info->xmit.tail++]) <<URBR_DTSHFT;
					info->xmit.tail = info->xmit.tail & (SERIAL_XMIT_SIZE-1);
					info->info->icount.tx++;
					if (info->xmit.head == info->xmit.tail)
					{
						empty=1;
						break;
					}
				}
#ifdef CONFIG_CHEETAH_UART_RTSCTS
		if (ctsrts_en)
			break;
#endif
	}
#if 1
	if(hfuart_halfdup && start){
		while(URREG(URCS)&URCS_TB){
			;
		}
		gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 0);
	}
#endif
	if (empty)
		rs_disable_tx_interrupts();

	if (CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE) < WAKEUP_CHARS)
		rs_sched_event(info);
	return;
}

#ifdef CONFIG_CHEETAH_UART_RTSCTS
static irqreturn_t cta_cts(int irq, void *data)
{
	struct serial_state * info = IRQ_ports;
	//if (info && (info->flags & ASYNC_CTS_FLOW)) {
	if (info) {
		//update trigger level
		set_irq_type(IRQ_GPIOX, (info->tty->hw_stopped) ? IRQ_TYPE_LEVEL_HIGH : IRQ_TYPE_LEVEL_LOW);

		if (info->tty->hw_stopped) {
			printk("tx start...");
			info->tty->hw_stopped = 0;
			//enable TX interrupt
			rs_enable_tx_interrupts();
			tasklet_schedule(&info->tlet);
		} else {
			printk("tx stop...");
			info->tty->hw_stopped = 1;
			//disable TX interrupt
			rs_disable_tx_interrupts();
		}
	}
	return IRQ_HANDLED;
}
#endif

static irqreturn_t cta_uart_interrupt(int irq, void *data)
{
	unsigned long mask = 0;
	cta_uart_status = URREG(URCS);
	if (cta_uart_status&URCS_RR) {
		mask|=URCS_RIE;
#ifdef CONFIG_CHEETAH_UART_RTSCTS
		if (ctsrts_en)
			gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);
#endif
	}
	if (cta_uart_status&URCS_TE)
		mask|=URCS_TIE;
	//disable interrupt
	URCS_DISABLE(mask);
	tasklet_hi_schedule(&cta_uart_tasklet);
	return IRQ_HANDLED;
}

static void cta_uart_deliver(unsigned long x)
{
	unsigned long status = cta_uart_status;
	volatile int *p = (int *) UR_BASE;
	struct serial_state * info;
	int pass_counter = 256;

	/* XXX : Clear any interrupts we might be about to handle */
	info = IRQ_ports;
	if (!info || !info->tty)
	{
		// printk("cheetah_uart_interrupt: ignored\n");
		return ;
	}

	do
	{
		// only handle these event
		if (!(status & (URCS_RR|URCS_TE)))
			break;

		/* TX holding register empty - transmit a byte */
		if (status & URCS_TE)
		{
			URREG(URCS) = cta_uart_control|URCS_TE;
			transmit_chars();
			info->last_active = jiffies;
		}

		/* Byte received (which bit for RX timeout? NO) */
		if (status & URCS_RR)
		{
			/* RX Frame Error */
			if (status & URCS_FE)
				info->info->icount.frame++;

			/* RX Parity Error */
			if (status & URCS_PER)
				info->info->icount.parity++;

#ifdef CONFIG_CHEETAH_UART_RTSCTS
			if (ctsrts_en) {
				gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);
				idelay(idelay_before_rx);
			}
#endif
			receive_chars(info);
#ifdef CONFIG_CHEETAH_UART_RTSCTS
			if (ctsrts_en)
				gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 0);
#endif
			info->last_active = jiffies;
		}

		status = p[URCS>>2];
	} while (--pass_counter > 0) ;

	//re-enable interrupt
	URCS_ENABLE((URCS_TIE|URCS_RIE));
	return ;
}

/*
 * -------------------------------------------------------------------
 * Here ends the serial interrupt routines.
 * -------------------------------------------------------------------
 */

/*
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * rs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using rs_sched_event(), and they get done here.
 */

static void do_softint(unsigned long private_)
{
    struct serial_state *info = (struct serial_state *) private_;
    struct tty_struct   *tty;

    tty = info->tty;
    if (!tty)
        return;

    tty_wakeup(tty);
}

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */

static int startup(struct tty_struct *tty, struct serial_state *info)
{
    unsigned long flags;
    int retval=0;
    unsigned long page;

    if (info->flags & ASYNC_INITIALIZED)
    {
		return retval;
    }


    local_irq_save(flags);
    if ( !info->xmit.buf)
	{
    	page = get_zeroed_page(GFP_KERNEL);
    	if (!page)
		{
        	retval= -ENOMEM;
			goto errout;
		}
    	info->xmit.buf = (unsigned char *) page;
	}

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("starting up ttys%d ...", info->line);
#endif

    /* Clear anything in the input buffer */
    if (serial_isroot())
    {
        if (info->tty)
            set_bit(TTY_IO_ERROR,
                    &info->tty->flags);
        retval = 0;
    }

    IRQ_ports = info;

    rs_enable_tx_interrupts();
    rs_enable_rx_interrupts();
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en) {
		if (gpio_get_value(CONFIG_CHEETAH_UART_CTS_GPIO_NUM)) {
			info->tty->hw_stopped = 1;
		}
		else {
			info->tty->hw_stopped = 0;
		}
		set_irq_type(IRQ_GPIOX, (info->tty->hw_stopped) ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH);
		enable_irq(IRQ_GPIOX);
		gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 0);
	}
#endif

    if (info->tty)
        clear_bit(TTY_IO_ERROR, &info->tty->flags);
    info->xmit.head = info->xmit.tail = 0;

    /*
     * and set the speed of the serial port
     */
    change_speed(info, NULL);

    info->flags |= ASYNC_INITIALIZED;
    local_irq_restore(flags);
    return 0;

    errout:
    local_irq_restore(flags);
    return retval;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void shutdown(struct serial_state * info)
{
    unsigned long   flags;

    if (!(info->flags & ASYNC_INITIALIZED))
        return;

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("Shutting down serial port %d ....\n", info->line);
#endif

    local_irq_save(flags); /* Disable interrupts */

    /*
     * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
     * here so the queue might never be waken up
     */
    wake_up_interruptible(&info->delta_msr_wait);

    rs_disable_tx_interrupts();
    rs_disable_rx_interrupts();
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en) {
		disable_irq(IRQ_GPIOX);
		gpio_set_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);
	}
#endif

    if (info->xmit.buf)
    {
        free_page((unsigned long) info->xmit.buf);
        info->xmit.buf = NULL;
    }

    IRQ_ports = NULL;

    if (info->tty)
        set_bit(TTY_IO_ERROR, &info->tty->flags);

    info->flags &= ~ASYNC_INITIALIZED;
    local_irq_restore(flags);
}


static int rs_put_char(struct tty_struct *tty, unsigned char ch)
{
    struct serial_state *info;
    unsigned long flags;

    if (!tty)
        return 0;

    info = tty->driver_data;

    if (!info->xmit.buf)
        return 0;

    local_irq_save(flags);
    if (CIRC_SPACE(info->xmit.head,
                   info->xmit.tail,
                   SERIAL_XMIT_SIZE) == 0)
    {
        local_irq_restore(flags);
        return 0;
    }

    info->xmit.buf[info->xmit.head++] = ch;
    info->xmit.head &= SERIAL_XMIT_SIZE-1;
    local_irq_restore(flags);
    return 1;
}

static void rs_flush_chars(struct tty_struct *tty)
{
    struct serial_state *info = tty->driver_data;

    if (info->xmit.head == info->xmit.tail
        || tty->stopped
        || tty->hw_stopped
        || !info->xmit.buf)
        return;

    rs_enable_tx_interrupts();
}

static int rs_write(struct tty_struct * tty, const unsigned char *buf, int count)
{
    int c, ret = 0;
    struct serial_state *info;
    unsigned long flags;

    if (!tty)
        return 0;

    info = tty->driver_data;

    if (!info->xmit.buf)
        return 0;

    local_irq_save(flags);
    while (1)
    {
        c = CIRC_SPACE_TO_END(info->xmit.head,
                              info->xmit.tail,
                              SERIAL_XMIT_SIZE);
        if (count < c)
            c = count;
        if (c <= 0)
        {
            break;
        }
        memcpy(info->xmit.buf + info->xmit.head, buf, c);
        info->xmit.head = ((info->xmit.head + c) &
                           (SERIAL_XMIT_SIZE-1));
        buf += c;
        count -= c;
        ret += c;
    }
    local_irq_restore(flags);

    if (info->xmit.head != info->xmit.tail
        && !tty->stopped
        && !tty->hw_stopped
		)
    {
       rs_enable_tx_interrupts();
    }

    return ret;
}

static int rs_write_room(struct tty_struct *tty)
{
    struct serial_state *info = tty->driver_data;

    if (serial_paranoia_check(info, tty->name, "rs_write_room"))
            return 0;
    return CIRC_SPACE(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static int rs_chars_in_buffer(struct tty_struct *tty)
{
    struct serial_state *info = tty->driver_data;

    if (serial_paranoia_check(info, tty->name, "rs_chars_in_buffer"))
            return 0;
    return CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static void rs_flush_buffer(struct tty_struct *tty)
{
    struct serial_state *info = tty->driver_data;
    unsigned long flags;

    if (serial_paranoia_check(info, tty->name, "rs_flush_buffer"))
            return;
    local_irq_save(flags);
    info->xmit.head = info->xmit.tail = 0;
    local_irq_restore(flags);
    tty_wakeup(tty);
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void rs_send_xchar(struct tty_struct *tty, char ch)
{
    struct serial_state *info = tty->driver_data;

    info->x_char = ch;
    if (ch)
    {
        rs_enable_tx_interrupts();
    }
}

#if 0
/*
 * ------------------------------------------------------------
 * rs_throttle()
 *
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void rs_throttle(struct tty_struct * tty)
{
    struct serial_state *info = tty->driver_data;
    unsigned long flags;
#ifdef CHEETAH_UART_DEBUG_THROTTLE
    char    buf[64];

    printk("throttle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (serial_paranoia_check(info, tty->name, "rs_throttle"))
            return;

    if (I_IXOFF(tty))
        rs_send_xchar(tty, STOP_CHAR(tty));
}

static void rs_unthrottle(struct tty_struct * tty)
{
    struct serial_state *info = tty->driver_data;
    unsigned long flags;
#ifdef CHEETAH_UART_DEBUG_THROTTLE
    char    buf[64];

    printk("unthrottle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (serial_paranoia_check(info, tty->name, "rs_unthrottle"))
            return;

    if (I_IXOFF(tty))
    {
        if (info->x_char)
            info->x_char = 0;
        else
            rs_send_xchar(tty, START_CHAR(tty));
    }
}
#endif

/*
 * ------------------------------------------------------------
 * rs_ioctl() and friends
 * ------------------------------------------------------------
 */

static int get_serial_info(struct serial_state * info,
                           struct serial_struct __user * retinfo)
{
    struct serial_struct tmp;

    if (!retinfo)
        return -EFAULT;
    memset(&tmp, 0, sizeof(tmp));
    lock_kernel();
    tmp.type = info->type;
    tmp.line = info->line;
    tmp.port = info->port;
    tmp.irq = info->irq;
    tmp.flags = info->flags;
    tmp.xmit_fifo_size = info->xmit_fifo_size;
    tmp.baud_base = info->baud_base;
    tmp.close_delay = info->close_delay;
    tmp.closing_wait = info->closing_wait;
    tmp.custom_divisor = info->custom_divisor;
    unlock_kernel();
    if (copy_to_user(retinfo,&tmp,sizeof(*retinfo)))
        return -EFAULT;
    return 0;
}

static int set_serial_info(struct serial_state * info,
                           struct serial_struct __user * new_info)
{
    struct serial_struct new_serial;
    struct serial_state *state;
    unsigned int        change_irq,change_port;
    int             retval = 0;

    if (copy_from_user(&new_serial,new_info,sizeof(new_serial)))
        return -EFAULT;

    lock_kernel();
    state = info->state;

    change_irq = new_serial.irq != info->irq;
    change_port = (new_serial.port != info->port);
    if (change_irq || change_port || (new_serial.xmit_fifo_size != info->xmit_fifo_size))
    {
        unlock_kernel();
        return -EINVAL;
    }

    if (!serial_isroot())
    {
        if ((new_serial.baud_base != info->baud_base) ||
            (new_serial.close_delay != info->close_delay) ||
            (new_serial.xmit_fifo_size != info->xmit_fifo_size) ||
            ((new_serial.flags & ~ASYNC_USR_MASK) !=
             (info->flags & ~ASYNC_USR_MASK)))
            return -EPERM;
        info->flags = ((info->flags & ~ASYNC_USR_MASK) |
                        (new_serial.flags & ASYNC_USR_MASK));
        info->flags = ((info->flags & ~ASYNC_USR_MASK) |
                       (new_serial.flags & ASYNC_USR_MASK));
        info->custom_divisor = new_serial.custom_divisor;
        goto check_and_exit;
    }

    if (new_serial.baud_base < 9600)
    {
        unlock_kernel();
        return -EINVAL;
    }

    /*
     * OK, past this point, all the error checking has been done.
     * At this point, we start making changes.....
     */

    info->baud_base = new_serial.baud_base;
    info->flags = ((info->flags & ~ASYNC_FLAGS) |
                    (new_serial.flags & ASYNC_FLAGS));
    info->flags = ((info->flags & ~ASYNC_INTERNAL_FLAGS) |
                   (info->flags & ASYNC_INTERNAL_FLAGS));
    info->custom_divisor = new_serial.custom_divisor;
    info->close_delay = new_serial.close_delay * HZ/100;
    info->closing_wait = new_serial.closing_wait * HZ/100;
    info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    check_and_exit:
    if (!(info->flags & ASYNC_INITIALIZED))
        retval = startup(info);
    unlock_kernel();
    return retval;
}

#if 0
/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space.
 */
static int get_lsr_info(struct serial_state * info, unsigned int __user *value)
{
    unsigned char status;
    unsigned int result;
    unsigned long flags;

#if 0
    local_irq_save(flags);
    status = custom.serdatr;
    mb();
    local_irq_restore(flags);
    result = ((status & SDR_TSRE) ? TIOCSER_TEMT : 0);
#endif
    if (copy_to_user(value, &result, sizeof(int)))
        return -EFAULT;
    return 0;
}
#endif

/*
 * rs_break() --- routine which turns the break handling on or off
 */
static int rs_break(struct tty_struct *tty, int break_state)
{
    struct serial_state *info = tty->driver_data;
    unsigned long flags;

    if (serial_paranoia_check(info, tty->name, "rs_break"))
            return -EINVAL;

    local_irq_save(flags);
#if 0
    if (break_state == -1)
        custom.adkcon = AC_SETCLR | AC_UARTBRK;
    else
        custom.adkcon = AC_UARTBRK;
#endif
    mb();
    local_irq_restore(flags);
    return 0;
}

static int set_hfuart_halfdup(struct tty_struct *tty, char __user * new_info)
{
	struct serial_state * info = tty->driver_data;
	unsigned char halfdup;
	int err;
	
    if (copy_from_user(&halfdup,new_info,1))
        return -EFAULT;
	
	if(halfdup!=0 && halfdup!=1)
		return -EFAULT;
		
	if(halfdup == hfuart_halfdup)
		return 0;
		
	if(halfdup){
		printk("set half dup mode!\n");
		
		rs_stop(tty);
		
		ctsrts_en = 0;
		info->tty->hw_stopped = 0;
		free_irq(IRQ_GPIOX, NULL);
		gpio_free(CONFIG_CHEETAH_UART_CTS_GPIO_NUM);
		gpio_free(CONFIG_CHEETAH_UART_RTS_GPIO_NUM);
		
		err = gpio_request(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, DRV_NAME);
		if (err)
			panic("Couldn't request GPIO for UART RTS\n");
		gpio_direction_output(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 0);
		
		rs_start(tty);
	}
	
	hfuart_halfdup = halfdup;
	return 0;
}

#define TIOCSETHALFDUP	0x5436
static int rs_ioctl(struct tty_struct *tty,
                    unsigned int cmd, unsigned long arg)
{
    struct serial_state * info = tty->driver_data;
    struct async_icount cprev, cnow;    /* kernel counter temps */
    struct serial_icounter_struct icount;
    void __user *argp = (void __user *)arg;
    unsigned long flags;
    DEFINE_WAIT(wait);

    if (serial_paranoia_check(info, tty->name, "rs_ioctl"))
            return -ENODEV;

    if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
        (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) &&
        (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT) && (cmd != TIOCSETHALFDUP))
    {
        if (tty->flags & (1 << TTY_IO_ERROR))
            return -EIO;
    }
    switch (cmd)
    {
    case TIOCGSERIAL:
        return get_serial_info(info, argp);
    case TIOCSSERIAL:
        return set_serial_info(info, argp);
    case TIOCSERCONFIG:
        return 0;
#if 0
    case TIOCSERGETLSR: /* Get line status register */
        return get_lsr_info(info, argp);
#endif
    case TIOCSERGSTRUCT:
        if (copy_to_user(argp,
                         info, sizeof(struct serial_state)))
            return -EFAULT;
        return 0;

        /*
         * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
         * - mask passed in arg for lines of interest
         *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
         * Caller should use TIOCGICOUNT to see which one it was
         */
    case TIOCMIWAIT:
        local_irq_save(flags);
        /* note the counters on entry */
        cprev = info->info->icount;
        local_irq_restore(flags);
        while (1)
        {
            prepare_to_wait(&info->tport.delta_msr_wait,
                            &wait, TASK_INTERRUPTIBLE);
            local_irq_save(flags);
            cnow = info->info->icount; /* atomic copy */
            local_irq_restore(flags);
            if (cnow.cts == cprev.cts)
                return -EIO; /* no change => error */
            if ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))
            {
                return 0;
            }
            cprev = cnow;
        }
        /* NOTREACHED */

        /*
         * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
         * Return: write counters to the user passed counter struct
         * NB: both 1->0 and 0->1 transitions are counted except for
         *     RI where only 0->1 is counted.
         */
    case TIOCGICOUNT:
        local_irq_save(flags);
        cnow = info->info->icount;
        local_irq_restore(flags);
        icount.cts = cnow.cts;
        icount.dsr = 0;
        icount.rng = 0;
        icount.dcd = 0;
        icount.rx = cnow.rx;
        icount.tx = cnow.tx;
        icount.frame = cnow.frame;
        icount.overrun = cnow.overrun;
        icount.parity = cnow.parity;
        icount.brk = cnow.brk;
        icount.buf_overrun = cnow.buf_overrun;

        if (copy_to_user(argp, &icount, sizeof(icount)))
            return -EFAULT;
        return 0;
    case TIOCSERGWILD:
    case TIOCSERSWILD:
        /* "setserial -W" is called in Debian boot */
        printk ("TIOCSER?WILD ioctl obsolete, ignored.\n");
        return 0;
	case TIOCSETHALFDUP:
		return set_hfuart_halfdup(tty, argp);
    default:
        return -ENOIOCTLCMD;
    }
    return 0;
}

static void rs_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
    struct serial_state *info = tty->driver_data;
    unsigned int cflag = tty->termios->c_cflag;
#ifdef CONFIG_CHEETAH_UART_RTSCTS
    unsigned int frame_len = 10; //start(1) + data(8) + stop(1)
	int err;
	unsigned int irqflags = 0;

    /* update delay for CTS */
#endif

    /* check para whether changed */
    if (old_termios && (cflag == old_termios->c_cflag)){
        //printk("paras don't changed\n");
        return;
    }

    /* change stop bit setting */
    if (cflag & CSTOPB)
        cta_uart_control|=URCS_SP2;
    else
        cta_uart_control&=~URCS_SP2;

    /* change parity enable/disable setting */
    if (cflag & PARENB)
        cta_uart_control|=URCS_PE;
    else
        cta_uart_control&=~URCS_PE;

    /* change parity odd/even setting */
    if (cflag & PARODD)
        cta_uart_control&=~URCS_EVEN;
    else
        cta_uart_control|=URCS_EVEN;

    /* change baud rate setting */
    if (cflag & CBAUD) {
        cta_uart_control&=((unsigned int)(1<<URCS_BRSHFT)-1);
        cta_uart_control|=((SYS_CLK/tty_termios_baud_rate(tty->termios))<<URCS_BRSHFT);
    }
    URREG(URCS) = cta_uart_control;
#ifdef CONFIG_CHEETAH_UART_RTSCTS
    if (cflag & CSTOPB)
        frame_len++;
    if (cflag & PARENB)
        frame_len++;
    idelay_before_rx = (CPU_CLK/tty_termios_baud_rate(tty->termios))*(frame_len<<1);
#endif

#ifdef CONFIG_CHEETAH_UART_RTSCTS
    /* Handle turning on/off CRTSCTS */
    /* turn off */
    if ((old_termios->c_cflag & CRTSCTS) &&
        !(tty->termios->c_cflag & CRTSCTS))
    {
        rs_stop(tty);
        ctsrts_en = false;
		info->tty->hw_stopped = 0;
		free_irq(IRQ_GPIOX, NULL);
		gpio_free(CONFIG_CHEETAH_UART_CTS_GPIO_NUM);
		gpio_free(CONFIG_CHEETAH_UART_RTS_GPIO_NUM);
        rs_start(tty);
    }
    /* turn on */
    else if (!(old_termios->c_cflag & CRTSCTS) &&
        (tty->termios->c_cflag & CRTSCTS))
    {
        rs_stop(tty);
        ctsrts_en = true;
		err = gpio_request(CONFIG_CHEETAH_UART_CTS_GPIO_NUM, DRV_NAME);
		if (err)
			panic("Couldn't request GPIO for UART CTS\n");
		gpio_direction_input(CONFIG_CHEETAH_UART_CTS_GPIO_NUM);
		err = gpio_request(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, DRV_NAME);
		if (err)
			panic("Couldn't request GPIO for UART RTS\n");
		gpio_direction_output(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);

		if (gpio_get_value(CONFIG_CHEETAH_UART_CTS_GPIO_NUM)) {
			irqflags |= IRQF_TRIGGER_LOW;
			info->tty->hw_stopped = 1;
		}
		else {
			irqflags |= IRQF_TRIGGER_HIGH;
			info->tty->hw_stopped = 0;
		}
		if (0 > request_irq(IRQ_GPIOX, cta_cts, irqflags, "UART CTS", NULL))
			panic("Couldn't request IRQ for UART CTS\n");
        rs_start(tty);
    }
#endif

#if 1
    /*
     * No need to wake up processes in open wait, since they
     * sample the CLOCAL flag once, and don't recheck it.
     * XXX  It's not clear whether the current behavior is correct
     * or not.  Hence, this may change.....
     */
    if (!(old_termios->c_cflag & CLOCAL) &&
        (tty->termios->c_cflag & CLOCAL))
        wake_up_interruptible(&info->open_wait);
#endif
}

/*
* ------------------------------------------------------------
* rs_close()
*
* This routine is called when the serial port gets closed.  First, we
* wait for the last remaining data to be sent.  Then, we unlink its
* async structure from the interrupt chain if necessary, and we free
* that IRQ if nothing is left in the chain.
* ------------------------------------------------------------
*/
static void rs_close(struct tty_struct *tty, struct file * filp)
{
    struct serial_state *state = tty->driver_data;
    unsigned long flags;

    if (!state || serial_paranoia_check(state, tty->name, "rs_close"))
            return;

    local_irq_save(flags);

    if (tty_hung_up_p(filp))
    {
        local_irq_restore(flags);
        return;
    }

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("rs_close ttys%d, count = %d\n", info->line, info->count);
#endif
    if ((tty->count == 1) && (info->count != 1))
    {
        /*
         * Uh, oh.  tty->count is 1, which means that the tty
         * structure will be freed.  info->count should always
         * be one in these conditions.  If it's greater than
         * one, we've got real problems, since it means the
         * serial port won't be shutdown.
         */
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("rs_close: bad serial port count; tty->count is 1, "
               "info->count is %d\n", info->count);
#endif
        info->count = 1;
    }
    if (--info->count < 0)
    {
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("rs_close: bad serial port count for ttys%d: %d\n",
               info->line, info->count);
#endif
        info->count = 0;
    }
    if (info->count)
    {
        local_irq_restore(flags);
        return;
    }
    info->flags |= ASYNC_CLOSING;
    /*
     * Now we wait for the transmit buffer to clear; and we notify
     * the line discipline to only process XON/XOFF characters.
     */
    tty->closing = 1;
    if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
        tty_wait_until_sent(tty, info->closing_wait);
    if (info->flags & ASYNC_INITIALIZED)
    {
        /*
         * Before we drop DTR, make sure the UART transmitter
         * has completely drained; this is especially
         * important if there is a transmit FIFO!
         */
        rs_wait_until_sent(tty, info->timeout);
    }
    shutdown(info);
    rs_flush_buffer(tty);

    tty_ldisc_flush(tty);
    tty->closing = 0;
    info->tty = NULL;
    if (info->blocked_open)
    {
        if (info->close_delay)
        {
            msleep_interruptible(jiffies_to_msecs(info->close_delay));
        }
        wake_up_interruptible(&info->open_wait);
    }
    info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CLOSING);
    wake_up_interruptible(&info->close_wait);
    local_irq_restore(flags);
}

/*
 * rs_wait_until_sent() --- wait until the transmitter is empty
 */
static void rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
#if 1
    return;
#else
    struct serial_state * info = tty->driver_data;
    unsigned long orig_jiffies, char_time;
    int lsr;

    if (serial_paranoia_check(info, tty->name, "rs_wait_until_sent"))
            return;

    if (info->xmit_fifo_size == 0)
        return; /* Just in case.... */

    orig_jiffies = jiffies;

    lock_kernel();
    /*
     * Set the check interval to be 1/5 of the estimated time to
     * send a single character, and make it at least 1.  The check
     * interval should also be less than the timeout.
     *
     * Note: we have to use pretty tight timings here to satisfy
     * the NIST-PCTS.
     */
    char_time = (info->timeout - HZ/50) / info->xmit_fifo_size;
    char_time = char_time / 5;
    if (char_time == 0)
        char_time = 1;
    if (timeout)
        char_time = min_t(unsigned long, char_time, timeout);
    /*
     * If the transmitter hasn't cleared in twice the approximate
     * amount of time to send the entire FIFO, it probably won't
     * ever clear.  This assumes the UART isn't doing flow
     * control, which is currently the case.  Hence, if it ever
     * takes longer than info->timeout, this is probably due to a
     * UART bug of some kind.  So, we clamp the timeout parameter at
     * 2*info->timeout.
     */
    if (!timeout || timeout > 2*info->timeout)
        timeout = 2*info->timeout;
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
    printk("In rs_wait_until_sent(%d) check=%lu...", timeout, char_time);
    printk("jiff=%lu...", jiffies);
#endif
#if 0
    while (!((lsr = custom.serdatr) & SDR_TSRE))
    {
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
        printk("serdatr = %d (jiff=%lu)...", lsr, jiffies);
#endif
        msleep_interruptible(jiffies_to_msecs(char_time));
        if (signal_pending(current))
            break;
        if (timeout && time_after(jiffies, orig_jiffies + timeout))
            break;
    }
#endif
    __set_current_state(TASK_RUNNING);
    unlock_kernel();
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
    printk("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
#endif
}

/*
 * rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void rs_hangup(struct tty_struct *tty)
{
    struct serial_state * info = tty->driver_data;

    if (serial_paranoia_check(info, tty->name, "rs_hangup"))
            return;

    rs_flush_buffer(tty);
    shutdown(info);
    info->count = 0;
    info->flags &= ~ASYNC_NORMAL_ACTIVE;
    info->tty = NULL;
    wake_up_interruptible(&info->open_wait);
}

static int get_serial_state(int line, struct serial_state **ret_info)
{
    struct serial_state *info;
    struct serial_state *sstate;

    sinfo->count++;
    if (sinfo->info)
    {
        *ret_info = sinfo->info;
        return 0;
    }
    info = kzalloc(sizeof(struct serial_state), GFP_KERNEL);
    if (!info)
    {
        sinfo->count--;
        return -ENOMEM;
    }
#ifdef DECLARE_WAITQUEUE
    init_waitqueue_head(&info->open_wait);
    init_waitqueue_head(&info->close_wait);
    init_waitqueue_head(&info->delta_msr_wait);
#endif
    info->port = sinfo->port;
    info->flags = sinfo->flags;
    info->xmit_fifo_size = sinfo->xmit_fifo_size;
    info->line = line;
    tasklet_init(&info->tlet, do_softint, (unsigned long)info);
    info->state = sstate;
    if (sinfo->info)
    {
        kfree(info);
        *ret_info = sinfo->info;
        return 0;
    }
    *ret_info = sinfo->info = info;
    return 0;
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int rs_open(struct tty_struct *tty, struct file * filp)
{
    struct serial_state *info = rs_table + tty->index;
    struct tty_port *port = &info->tport;
    int retval;

    port->count++;
    port->tty = tty;
    tty->driver_data = info;
    tty->port = port;
    if (serial_paranoia_check(info, tty->name, "rs_open"))
            return -ENODEV;

    port->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    retval = startup(tty, info);
    if (retval) {
        return retval;
    }

    return tty_port_block_til_ready(port, tty, filp);
}

#ifdef CONFIG_CHEETAH_UART_STAT
/*
 * /proc fs routines....
 */

static inline void line_info(struct seq_file *m, struct serial_state *state)
{
	struct serial_state *info = info->info, scr_info;
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	char	stat_buf[10];
#endif

	/*
	 * Figure out the current RS-232 lines
	 */
	if (!info) {
		info = &scr_info;	/* This is just for serial_{in,out} */

		info->flags = info->flags;
		info->quot = 0;
		info->tty = NULL;
	}

#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en) {
		stat_buf[0] = stat_buf[1] = 0;
		if (gpio_get_value(CONFIG_CHEETAH_UART_RTS_GPIO_NUM))
			strcat(stat_buf, "|RTS");
		if (gpio_get_value(CONFIG_CHEETAH_UART_CTS_GPIO_NUM))
			strcat(stat_buf, "|CTS");
	}
#endif

	if (info->quot) {
		seq_printf(m, " baud:%d", info->baud_base / info->quot);
	}

	seq_printf(m, " tx:%d rx:%d", info->icount.tx, info->icount.rx);

	if (info->icount.frame)
		seq_printf(m, " fe:%d", info->icount.frame);

	if (info->icount.parity)
		seq_printf(m, " pe:%d", info->icount.parity);

	if (info->icount.brk)
		seq_printf(m, " brk:%d", info->icount.brk);

	if (info->icount.overrun)
		seq_printf(m, " oe:%d", info->icount.overrun);

#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en)
		seq_printf(m, " ibr:%d", idelay_before_rx);
#endif

	seq_printf(m, "\n");
	/*
	 * Last thing is the RS-232 status lines
	 */
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en)
		seq_printf(m, " %s\n", stat_buf+1);
#endif
}

static int rs_proc_show(struct seq_file *m, void *v)
{
	line_info(m, &rs_table[0]);
	return 0;
}

static int rs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rs_proc_show, NULL);
}

static const struct file_operations rs_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rs_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
int cheetah_uart_poll_init(struct tty_driver *driver, int line, char *options)
{
	return 0;
}

int cheetah_uart_poll_get_char(struct tty_driver *driver, int line)
{
	volatile int *p = (int *) UR_BASE;
	int ch;

	while(1)
	{
		if (URBR_RDY & (ch = p[URBR]) ) // rx flag on, break
			break;
	}
	return ch>>URBR_DTSHFT;
}

void cheetah_uart_poll_put_char(struct tty_driver *driver, int line, char ch)
{
    int i;
    volatile int *p = (int *) UR_BASE;

    /* Wait for UARTA_TX register to empty */
    i = 1000000;
    while ((p[URCS>>2] & URCS_TF) && i--);
    /* Send the character */
    p[URBR>>2] = (int)ch<<URBR_DTSHFT;
}
#endif

static const struct tty_operations serial_ops = {
    .open = rs_open,
    .close = rs_close,
    .write = rs_write,
    .put_char = rs_put_char,
    .flush_chars = rs_flush_chars,
    .write_room = rs_write_room,
    .chars_in_buffer = rs_chars_in_buffer,
    .flush_buffer = rs_flush_buffer,
    .ioctl = rs_ioctl,
    //.throttle = rs_throttle,
    //.unthrottle = rs_unthrottle,
    .set_termios = rs_set_termios,
    .stop = rs_stop,
    .start = rs_start,
    .hangup = rs_hangup,
    .break_ctl = rs_break,
    .send_xchar = rs_send_xchar,
    .wait_until_sent = rs_wait_until_sent,
#ifdef CONFIG_CHEETAH_UART_STAT
    .proc_fops = &rs_proc_fops,
#endif
#ifdef CONFIG_CONSOLE_POLL
    .poll_init  = cheetah_uart_poll_init,
    .poll_get_char  = cheetah_uart_poll_get_char,
    .poll_put_char  = cheetah_uart_poll_put_char,
#endif
};

static int __init cheetah_rs_init(void)
{
#ifdef CONFIG_CHEETAH_UART_RTSCTS
	int err;
#endif
    unsigned long flags;
    struct serial_state * state;

    serial_driver = alloc_tty_driver(1);
    if (!serial_driver)
        return -ENOMEM;

    IRQ_ports = NULL;
    cta_uart_control = ((CONFIG_CHEETAH_UART_RX_THRESHOLD-1)<<URCS_RTH_S) | (URREG(URCS)&URCS_CTRL_MASK);
	hfuart_halfdup = 0;
    /* Initialize the tty_driver structure */

    serial_driver->owner = THIS_MODULE;
    serial_driver->driver_name = "cheetah_uart";
    serial_driver->name = "ttyS";
#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
    /* assume only 1 GVCOM */
    serial_driver->name_base = 1;
#endif
    serial_driver->major = TTY_MAJOR;
    serial_driver->minor_start = 64;
    serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
    serial_driver->subtype = SERIAL_TYPE_NORMAL;
    serial_driver->init_termios = tty_std_termios;
    serial_driver->init_termios.c_cflag =
#ifdef CONFIG_CHEETAH_UART_RTSCTS
    CRTSCTS |
#endif
#if CONFIG_CHEETAH_UART_BAUDRATE == 57600
    B57600 | CS8 | CREAD | HUPCL | CLOCAL;
#elif CONFIG_CHEETAH_UART_BAUDRATE == 115200
    B115200 | CS8 | CREAD | HUPCL | CLOCAL;
#elif CONFIG_CHEETAH_UART_BAUDRATE == 230400
    B230400 | CS8 | CREAD | HUPCL | CLOCAL;
#elif CONFIG_CHEETAH_UART_BAUDRATE == 460800
    B460800 | CS8 | CREAD | HUPCL | CLOCAL;
#else
#error "unsupported baudrate"
#endif
    serial_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
    tty_set_operations(serial_driver, &serial_ops);

    if (tty_register_driver(serial_driver))
        panic("Couldn't register serial driver\n");

    tty_register_device(serial_driver, 0, NULL);

    state = rs_table;
#if 0
    state->port = (int)&custom.serdatr; /* Just to give it a value */
#endif
    state->line = 0;
    state->custom_divisor = 0;
    state->close_delay = 5*HZ/10;
    state->closing_wait = 30*HZ;
    state->icount.cts = state->icount.dsr =
     state->icount.rng = state->icount.dcd = 0;
    state->icount.rx = state->icount.tx = 0;
    state->icount.frame = state->icount.parity = 0;
    state->icount.overrun = state->icount.brk = 0;

    printk(KERN_INFO "ttyS%d is enabled\n",
           state->line);

    state->xmit_fifo_size = 16;
    local_irq_save(flags);

    if (0 > request_irq( IRQ_UR, cta_uart_interrupt, 0, "UART", state))
        panic("Couldn't request IRQ for UART device\n");

#ifdef CONFIG_CHEETAH_UART_RTSCTS
	err = gpio_request(CONFIG_CHEETAH_UART_CTS_GPIO_NUM, DRV_NAME);
	if (err)
		panic("Couldn't request GPIO for UART CTS\n");
	gpio_direction_input(CONFIG_CHEETAH_UART_CTS_GPIO_NUM);
	err = gpio_request(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, DRV_NAME);
	if (err)
		panic("Couldn't request GPIO for UART RTS\n");
	gpio_direction_output(CONFIG_CHEETAH_UART_RTS_GPIO_NUM, 1);

	if (0 > request_irq(IRQ_GPIOX, cta_cts, 0, "UART CTS", NULL))
		panic("Couldn't request IRQ for UART CTS\n");
	disable_irq(IRQ_GPIOX);
#endif
    local_irq_restore(flags);

    return 0;
}

static __exit void cheetah_rs_exit(void)
{
    int error;
    struct serial_state *info = rs_table[0].info;

#ifdef CONFIG_CHEETAH_UART_RTSCTS
	if (ctsrts_en) {
		free_irq(IRQ_GPIOX, NULL);
		gpio_free(CONFIG_CHEETAH_UART_CTS_GPIO_NUM);
		gpio_free(CONFIG_CHEETAH_UART_RTS_GPIO_NUM);
	}
#endif

	if(hfuart_halfdup)
		gpio_free(CONFIG_CHEETAH_UART_RTS_GPIO_NUM);

    /* printk("Unloading %s: version %s\n", serial_name, serial_version); */
    tasklet_kill(&info->tlet);
    if ((error = tty_unregister_driver(serial_driver)))
        printk("SERIAL: failed to unregister serial driver (%d)\n",
               error);
    put_tty_driver(serial_driver);

    if (info)
    {
        rs_table[0].info = NULL;
        kfree(info);
    }
}

module_init(cheetah_rs_init)
module_exit(cheetah_rs_exit)

void cheetah_serial_outc(unsigned char c)
{
    int i;
    volatile int *p = (int *) UR_BASE;

    /* Disable UARTA_TX interrupts */
    URCS_DISABLE(URCS_TIE);

    /* Wait for UARTA_TX register to empty */
    i = 10000;
    while ((p[URCS>>2] & URCS_TF) && i--);

    /* Send the character */
    p[URBR>>2] = (int)c<<URBR_DTSHFT;

    /* Enable UARTA_TX interrupts */
    URCS_ENABLE(URCS_TIE);
}

static __init int serial_console_setup(struct console *co, char *options)
{
    int baud = CONFIG_CHEETAH_UART_BAUDRATE;
    //int bits = 8;
    int parity = 'n';
    char *s;
    int brsr = 0;

    if (options)
    {
        baud = simple_strtoul(options, NULL, 10);
        s = options;
        while (*s >= '0' && *s <= '9')
            s++;
        if (*s) parity = *s++;
        //if (*s) bits   = *s++ - '0';
    }

    if (baud >0)
        brsr = SYS_CLK/baud;
#if 0
    switch (bits)
    {
    case 7:
        break;
    default:
        break;
    }
#endif
    switch (parity)
    {
    case 'o':
    case 'O':
        cta_uart_control|=(URCS_PE);
        cta_uart_control&=~URCS_EVEN;
        break;
    case 'e':
    case 'E':
        cta_uart_control|=(URCS_PE|URCS_EVEN);
        break;
    default:
        break;
    }

    /* Write the control registers */
    URCS_UPDATE_BR(brsr);

    return 0;
}

static void serial_console_write(struct console *co, const char *s,
                                 unsigned count)
{

    while (count--)
    {
        if (*s == '\n')
            cheetah_serial_outc('\r');
        cheetah_serial_outc(*s++);
    }
}

static struct tty_driver *serial_console_device(struct console *c, int *index)
{
    *index = 0;
    return serial_driver;
}

static struct console cta_console = {
    .name =     DRV_NAME,
    .write =    serial_console_write,
    .device =   serial_console_device,
    .setup =    serial_console_setup,
    .flags =    CON_PRINTBUFFER,
    .index =    -1,
};

/*
 *	Register console.
 */
static int __init cheetah_serial_console_init(void)
{
#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
    (void)cta_console;
#else
    register_console(&cta_console);
#endif
    return 0;
}
console_initcall(cheetah_serial_console_init);
