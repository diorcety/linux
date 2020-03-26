#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/termios.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>

#if CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM==0
#define GPIO_COM_IRQ IRQ_GPIO0
#else
#define GPIO_COM_IRQ IRQ_GPIOX
#endif
#define GPIO_VCOM_TTY_NR 1
#define GPIO_VCOM_MAJOR TTY_MAJOR

#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
#define GPIO_VCOM_TTY_NAME_BASE 0
#else
#define GPIO_VCOM_TTY_NAME_BASE 1
#endif

#define GPIO_OUT_PIPE_NUM CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM
#define GPIO_IN_PIPE_NUM CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM
#define GPIO_VCOM_TX (1 << GPIO_OUT_PIPE_NUM)
#define GPIO_VCOM_RX (1 << GPIO_IN_PIPE_NUM)

/*config accept char number by every interrupt*/
#define READ_BUF_SIZE 4 

#ifdef CONFIG_CHEETAH_GVCOM_B9600
    #define DEFAULT_BAUD_RATE B9600 
    #define DEFAULT_BAUD_RATE_NUM 9600 
#elif defined(CONFIG_CHEETAH_GVCOM_B14400)
    #define DEFAULT_BAUD_RATE B14400 
    #define DEFAULT_BAUD_RATE_NUM 14400 
#elif defined(CONFIG_CHEETAH_GVCOM_B19200)
    #define DEFAULT_BAUD_RATE B19200 
    #define DEFAULT_BAUD_RATE_NUM 19200 
#elif defined(CONFIG_CHEETAH_GVCOM_B38400)
    #define DEFAULT_BAUD_RATE B38400 
    #define DEFAULT_BAUD_RATE_NUM 38400 
#elif defined(CONFIG_CHEETAH_GVCOM_B57600)
    #define DEFAULT_BAUD_RATE B57600 
    #define DEFAULT_BAUD_RATE_NUM 57600 
#elif defined(CONFIG_CHEETAH_GVCOM_B115200)
    #define DEFAULT_BAUD_RATE B1152000 
    #define DEFAULT_BAUD_RATE_NUM 1152000 
#elif defined(CONFIG_CHEETAH_GVCOM_CONSOLE)
    #define DEFAULT_BAUD_RATE B9600 
    #define DEFAULT_BAUD_RATE_NUM 9600 
#else
#error "UART Baud Rate Not Defined"
#endif

#define VCOM_DBUG_LEVEL 0
#define VCOM_DEBUG(level, s, params...) do{ if(level >= VCOM_DBUG_LEVEL)\
                        printk(KERN_INFO "[%s, %d]: " s "\n", __FUNCTION__, __LINE__, params);\
                    }while(0)
            
#define RELEVANT_IFLAG(iflag) (iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))


#ifdef CONFIG_CHEETAH_GVCOM_DUMMY
static int hi_gpio_vcom_open(struct tty_struct *tty, struct file *filp)
{
    return 0;
}
static void hi_gpio_vcom_close(struct tty_struct *tty, struct file *filp)
{
    return;
}
static int hi_gpio_vcom_write(struct tty_struct * tty, const unsigned char *buf, int count)
{
    return count;
}
static int gpio_vcom_write_room(struct tty_struct *tty)
{
    return 512;
}
static void hi_gpio_vcom_set_termios(struct tty_struct * tty, struct ktermios *old_termios)
{
    return;
}
static int gpio_vcom_chars_in_buffer(struct tty_struct *tty)
{
    return 512;
}
#else

struct gpio_vcom_serial{
    int open_cnt;
    struct semaphore sem;
    unsigned char *cir_buff;
    int head;
    int tail;
    
    struct tty_struct *p_tty;   
    spinlock_t in_lock;
    unsigned char *in_buff;
    int in_head;
    int in_tail;
};

struct serial_comm_para {
    int in_baud;
    int out_baud;
    int check_flag; /* 1--odd,2--even,3-none*/
    int stop_num;       /*stop bits length*/
    int data_num;       /*data bits length*/
};

static struct gpio_vcom_serial *gpio_serial = 0;
static unsigned int rcv_cycle = 0;
static unsigned int send_cycle = 0;

/*initialize the communication parameter*/
static struct serial_comm_para g_comm_para = {
    DEFAULT_BAUD_RATE_NUM,
    DEFAULT_BAUD_RATE_NUM,  
    3,      /*none check bit*/
    1,      /*stop bits length*/
    8       /*data bits length*/
};

void rcv_data_push_tasklet(unsigned long data);

#if (CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM!=-1)
static DECLARE_TASKLET(rcv_data_tasklet, rcv_data_push_tasklet, 0);
#endif

int gpio_vcom_init_pinmux(void)
{
    static int first = 0;
    int ret_tx = 0, ret_rx = 0;

    if (first == 0) {
        /*initialize the in and out port value and */
#if CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM == CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM
#error "GVCOM TX/RX GPIO NUM are the same!!"
#endif
#if CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM < 0 || CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "GVCOM TX GPIO NUM is out of range!!"
#else
        ret_tx = gpio_request(CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM, "gvcom");
#endif
#if CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM < -1 || CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "GVCOM RX GPIO NUM is out of range!!"
#elif CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM == -1
#else
        ret_rx = gpio_request(CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM, "gvcom");
#endif
        if (ret_tx || ret_rx) {
            if (ret_tx)
                gpio_free(CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM);
            if (ret_rx)
                gpio_free(CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM);
            return -EINVAL;
        }

        /* set high value to out line */
        if (ret_tx)
            gpio_direction_output(CONFIG_CHEETAH_GVCOM_TX_GPIO_NUM, 1);
        if (ret_rx)
            gpio_direction_input(CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM);

        first = 1;
    }

    return 0;
}
void gpio_vcom_set_comm_params(void)
{
    send_cycle = 1000 * 1000 / g_comm_para.out_baud;   /*unit:us*/
    rcv_cycle = (int)(1000 * 1000 / g_comm_para.in_baud);   /* unit:us */
//    rcv_cycle = (int)((1000 * 1000 + (g_comm_para.in_baud / 2)) / g_comm_para.in_baud);   /* unit:us */
}

void rcv_data_push_tasklet(unsigned long data)
{
    if (!gpio_serial)
        return;
    while(1){
        if (gpio_serial->in_head == gpio_serial->in_tail){
            break;
        }else{
            tty_insert_flip_char(gpio_serial->p_tty, gpio_serial->in_buff[gpio_serial->in_tail], TTY_NORMAL);
            gpio_serial->in_tail = (gpio_serial->in_tail + 1) % 4096;
        }
    }   
    
    tty_flip_buffer_push(gpio_serial->p_tty);

}

#if (CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM!=-1)
static irqreturn_t gpio_com_interrupt(int irq, void *dev_id)
{
    unsigned int rcv_bit;
    unsigned int rcv_data;
    unsigned int check_data = 0;
    int i,k;

    if (rcv_cycle == 0)
        gpio_vcom_set_comm_params();

    local_irq_disable();

    for(k=0; k<READ_BUF_SIZE; k++){
        rcv_data = 0;
        check_data = 0;
        rcv_bit = GPREG(GPVAL) & GPIO_VCOM_RX;
        if (rcv_bit == GPIO_VCOM_RX){
            goto out;
        }

        /*get all data bits by timer cycle method*/
        for(i=0; i< g_comm_para.data_num; i++){
            udelay(rcv_cycle);
            rcv_bit = GPREG(GPVAL) & GPIO_VCOM_RX;
            if (rcv_bit == GPIO_VCOM_RX){
                check_data += 1;
                rcv_data |= (1 << i);    /*received data from low bit to high bit*/
            }
        }
        
        /*check the check bit*/
        if (g_comm_para.check_flag != 3){
            udelay(rcv_cycle);
            rcv_bit = GPREG(GPVAL) & GPIO_VCOM_RX;
            if (rcv_bit == GPIO_VCOM_RX)
                rcv_bit = 1;
            else
                rcv_bit = 0;
            check_data += rcv_bit;
            if (g_comm_para.check_flag == 1){
                if (check_data % 2 != 1){
                    goto out;
                }
            }
            else{
                if (check_data % 2 != 0){
                    goto out;
                }
            }
        
        }
        
        /*check the stop bits*/
        udelay(rcv_cycle);
        rcv_bit = GPREG(GPVAL) & GPIO_VCOM_RX;
        if (rcv_bit != GPIO_VCOM_RX){
            goto out;
        }
        if (g_comm_para.stop_num == 2){
            udelay(rcv_cycle);
            rcv_bit = GPREG(GPVAL) & GPIO_VCOM_RX;
            if (rcv_bit != GPIO_VCOM_RX){
                goto out;
            }   
        }
        
        gpio_serial->in_buff[gpio_serial->in_head] = rcv_data;
        gpio_serial->in_head = (gpio_serial->in_head + 1) % 4096;
        
        if (k != (READ_BUF_SIZE - 1))
            udelay(rcv_cycle);
    }

out:
    tasklet_schedule(&rcv_data_tasklet);
    local_irq_enable();
    return IRQ_HANDLED;
}
#endif

static void hi_gpio_vcom_set_termios(struct tty_struct * tty, struct ktermios *old_termios)
{
    unsigned int cflag;
    cflag = tty->termios->c_cflag;
    /*check para if or not changed*/
    if (old_termios){
        if ((cflag == old_termios->c_cflag) && 
            (RELEVANT_IFLAG(tty->termios->c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag))){
            return;
        }else{
            printk("com parameter be changed,%d\n", 1);
        }
    }
    
    /*get the data bits length*/
    switch(cflag & CSIZE){
        case CS5:
            printk("data bits num = %d\n", 5);
            g_comm_para.data_num = 5;
            break;
        case CS6:
            printk("data bits num = %d\n", 6);
            g_comm_para.data_num = 6;
            break;
        case CS7:
            printk("data bits num = %d\n", 7);
            g_comm_para.data_num = 7;
            break;
        case CS8:
            printk("data bits num = %d\n", 8);
            g_comm_para.data_num = 8;
            break;
        default:
            printk("data bits num = %d\n", 8);  
            g_comm_para.data_num = 8;
            break;      
    }
    
    /*get the check bit value*/
    if (cflag & PARENB){
        if (cflag & PARODD){
            printk("Check flag = %d\n", 1);
            g_comm_para.check_flag = 1;
        }
        else{
            printk("Check flag = %d\n", 2);
            g_comm_para.check_flag = 2;     
        }
    }
    else    {
        printk("Check flag = %d\n", 3);
        g_comm_para.check_flag = 3;
    }
    
    /*get the stop bits length*/
    if (cflag & CSTOPB){
        printk("stop bits length  = %d\n", 2);
        g_comm_para.stop_num = 2;
    }
    else{
        printk("stop bits length = %d\n", 1);
        g_comm_para.stop_num = 1;
    }
    
    g_comm_para.in_baud = tty_get_baud_rate(tty);
    g_comm_para.out_baud = tty_get_baud_rate(tty);      
    printk("communication baud = %d\n", g_comm_para.in_baud);

    gpio_vcom_set_comm_params();
}

void gpiovcom_send_data(unsigned int send_data)
{
    int k = 0;
    unsigned int check_bit = 0;
    unsigned int send_bit;
    
    if (gpio_vcom_init_pinmux())
        return;
    if (send_cycle == 0)
        gpio_vcom_set_comm_params();

    local_irq_disable();
    /*send start bit*/
    udelay(send_cycle);
    GPREG(GPCLR) = GPIO_VCOM_TX;

    for(k= 0; k < g_comm_para.data_num; k++){
        send_bit = send_data & 0x01;
        udelay(send_cycle);
        if (send_bit == 0x01)
            GPREG(GPSET) = GPIO_VCOM_TX;
        else
            GPREG(GPCLR) = GPIO_VCOM_TX;
        send_data = send_data >> 1;
    }
    
    /*send the check bit*/
    if (g_comm_para.check_flag != 3){
        if (g_comm_para.check_flag == 1)     /*odd check*/
            check_bit = (check_bit + 1) % 2;
        else        /*even check*/
            check_bit = check_bit % 2;
            
        udelay(send_cycle);
        if (check_bit == 0x01)
            GPREG(GPSET) = GPIO_VCOM_TX;
        else
            GPREG(GPCLR) = GPIO_VCOM_TX;
    }
    
    /*send the stop bit*/
    udelay(send_cycle);
    GPREG(GPSET) = GPIO_VCOM_TX;
    /*send the second stop bit*/
    if (g_comm_para.stop_num == 2){
        udelay(send_cycle);
        GPREG(GPSET) = GPIO_VCOM_TX;
    }
    local_irq_enable();
}


static int hi_gpio_vcom_write(struct tty_struct * tty, const unsigned char *buf, int count)
{
    unsigned int send_data;
    int retval = 0;
    
    if (gpio_serial == NULL)
        return -EINVAL;
    down(&gpio_serial->sem);
    if (gpio_serial->open_cnt <= 0){
        up(&gpio_serial->sem);
        return -EINVAL;  
    }
    
    if (gpio_serial->head + count >= 512){
        memcpy(&(gpio_serial->cir_buff[gpio_serial->head]), buf, 512 - gpio_serial->head);
        memcpy(&(gpio_serial->cir_buff[0]), (buf+512-gpio_serial->head), count+gpio_serial->head-512);
        gpio_serial->head = count+gpio_serial->head-512;
    }else{
        memcpy(&(gpio_serial->cir_buff[gpio_serial->head]), buf, count);
        gpio_serial->head += count;
    }
    up(&gpio_serial->sem);
    
    while(1){
        down(&gpio_serial->sem);
        if (gpio_serial->head == gpio_serial->tail){/*no data in circle buffer*/
            up(&gpio_serial->sem);
            break;
        }
        else {
            send_data = gpio_serial->cir_buff[gpio_serial->tail];
            retval++;
            gpio_serial->tail = (gpio_serial->tail+1) % 512;
            up(&gpio_serial->sem);
            gpiovcom_send_data(send_data);
        }
    }
    
    return retval;
}

static int gpio_vcom_write_room(struct tty_struct *tty)
{
    int room = -EINVAL;
    if (gpio_serial == NULL)
        return -ENODEV;
    down(&gpio_serial->sem);
    if (gpio_serial->open_cnt <= 0)
        goto exit;
    if (gpio_serial->head >= gpio_serial->tail)
        room = 512 - gpio_serial->head + gpio_serial->tail;
    else if (gpio_serial->head < gpio_serial->tail)
        room = gpio_serial->tail - gpio_serial->head;
exit:
    up(&gpio_serial->sem);
    return room;
}

static int hi_gpio_vcom_open(struct tty_struct *tty, struct file *filp)
{
    int ret = 0;

    if (!gpio_serial){
        /*the first time open this device*/
        gpio_serial = (struct gpio_vcom_serial *)kmalloc(sizeof(*gpio_serial), GFP_KERNEL);
        if (!gpio_serial)
            return -ENOMEM;
        init_MUTEX(&gpio_serial->sem);
        gpio_serial->open_cnt = 0;
    }
    
    GPREG(GPDIR) |= GPIO_VCOM_TX;  // as tx output pin
#if (CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM!=-1)
    GPREG(GPDIR) &= ~GPIO_VCOM_RX; // as rx input pin
#endif
    /* set high value to out line */
    GPREG(GPSET) = GPIO_VCOM_TX;
    
    down(&gpio_serial->sem);

    ++gpio_serial->open_cnt;
    if (gpio_serial->open_cnt == 1){
        /*the first time open this device need alloc buff*/
        gpio_serial->head = 0; 
        gpio_serial->tail = 0;
        gpio_serial->cir_buff = (unsigned char *)vmalloc(512*sizeof(unsigned char));
        gpio_serial->p_tty = tty;
        gpio_serial->in_lock = SPIN_LOCK_UNLOCKED;
        gpio_serial->in_head = 0; 
        gpio_serial->in_tail = 0;
        gpio_serial->in_buff = (unsigned char *)vmalloc(4096*sizeof(unsigned char));

        /*the first time open this device need request IRQ*/
#if (CONFIG_CHEETAH_GVCOM_RX_GPIO_NUM!=-1)
        ret = request_irq(GPIO_COM_IRQ, gpio_com_interrupt, IRQF_TRIGGER_LOW, "gpio_com", tty);
        if (ret) 
        {
            printk("gpio_com" " can't request irq(%d)\n", GPIO_COM_IRQ);
            return ret;
        }
#endif
    }
    up(&gpio_serial->sem); 
    
    return ret;
}

static void hi_gpio_vcom_close(struct tty_struct *tty, struct file *filp)
{
    if (gpio_serial == NULL)
        return;
    down(&gpio_serial->sem);
    if (gpio_serial->open_cnt == 0){
        goto exit;
    }
    
    --gpio_serial->open_cnt;
    if (gpio_serial->open_cnt == 0){
        free_irq(GPIO_COM_IRQ, tty);
        vfree(gpio_serial->cir_buff);
        vfree(gpio_serial->in_buff);
    }
exit:
    up(&gpio_serial->sem);
}

static int gpio_vcom_chars_in_buffer(struct tty_struct *tty)
{
    int room = -EINVAL;
    if (gpio_serial == NULL)
        return -ENODEV;
    down(&gpio_serial->sem);
    if (gpio_serial->open_cnt <= 0)
        goto exit;
    room = (512 + gpio_serial->head-gpio_serial->tail) % 512;
exit:
    up(&gpio_serial->sem);
    return room;
}
#endif

static struct tty_operations gpio_vcom_ops = {
    .open   = hi_gpio_vcom_open,
    .close  = hi_gpio_vcom_close,
    .write  = hi_gpio_vcom_write,
    .write_room      = gpio_vcom_write_room,
    .set_termios     = hi_gpio_vcom_set_termios,
    .chars_in_buffer = gpio_vcom_chars_in_buffer
};

static struct tty_driver *gpio_vcom_tty_drv = NULL;

static int __init vcom_probe(struct tty_driver *tty_drv)
{
    int ret = 0;
    int i;

    for(i=0; i<GPIO_VCOM_TTY_NR; i++) {
        VCOM_DEBUG(1, "vcom add %s%d", tty_drv->name, (tty_drv->name_base + i));
        tty_register_device(tty_drv, i, NULL);
    }

    return ret;
}

static int __init hi_gpio_tty_init(void)
{
    int ret = 0;

#ifdef CONFIG_CHEETAH_GVCOM_DUMMY
#else
    if ((ret = gpio_vcom_init_pinmux()))
        return ret;
#endif
    /* create tty driver object */
    gpio_vcom_tty_drv = alloc_tty_driver(GPIO_VCOM_TTY_NR);
    if (!gpio_vcom_tty_drv)
        return -ENOMEM;
    gpio_vcom_tty_drv->magic = TTY_DRIVER_MAGIC;
    gpio_vcom_tty_drv->owner = THIS_MODULE;
    gpio_vcom_tty_drv->driver_name = "gpio_tty";
    gpio_vcom_tty_drv->name = "ttyS";
    gpio_vcom_tty_drv->name_base = GPIO_VCOM_TTY_NAME_BASE;
    gpio_vcom_tty_drv->major = GPIO_VCOM_MAJOR;
    gpio_vcom_tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
    gpio_vcom_tty_drv->subtype = SERIAL_TYPE_NORMAL;
    gpio_vcom_tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
    gpio_vcom_tty_drv->init_termios = tty_std_termios;
    gpio_vcom_tty_drv->init_termios.c_cflag = DEFAULT_BAUD_RATE | CS8 | CREAD | HUPCL |CLOCAL;
    
    tty_set_operations(gpio_vcom_tty_drv, &gpio_vcom_ops);
    
    ret = tty_register_driver(gpio_vcom_tty_drv);
    if (ret) {
        tty_unregister_driver(gpio_vcom_tty_drv);
        put_tty_driver(gpio_vcom_tty_drv);
        panic("Couldn't register serial driver\n");
    }
    vcom_probe(gpio_vcom_tty_drv);

    return ret;
}   

static void __exit hi_gpio_tty_exit(void)
{
    tty_unregister_driver(gpio_vcom_tty_drv);
    put_tty_driver(gpio_vcom_tty_drv);
}

module_init(hi_gpio_tty_init);
module_exit(hi_gpio_tty_exit);









#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
#ifdef CONFIG_CHEETAH_GVCOM_DUMMY
static __init int serial_console_setup(struct console *co, char *options)
{
    return 0;
}
static void serial_console_write(struct console *co, const char *s,
                                 unsigned count)
{
    return;
}
static struct tty_driver *serial_console_device(struct console *c, int *index)
{
    *index = 0;
    return gpio_vcom_tty_drv;
}
#else
static __init int serial_console_setup(struct console *co, char *options)
{
    int baud = 9600;
    int bits = 8;
    int parity = 'n';
    char *s;

    if (options)
    {
        baud = simple_strtoul(options, NULL, 10);
        s = options;
        while (*s >= '0' && *s <= '9')
            s++;
        if (*s) parity = *s++;
        if (*s) bits   = *s++ - '0';
    }

    switch(baud) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
            break;
        default:
            baud = 9600;
            break;
    }
    g_comm_para.in_baud = baud;
    g_comm_para.out_baud = baud;
    if ((bits != 7) && (bits != 8))
        bits = 8;
    g_comm_para.data_num = bits;
    switch (parity) {
        case 'o': case 'O':
            g_comm_para.check_flag = 1;
            /*fall through*/
        case 'e': case 'E': 
            g_comm_para.check_flag = 2;
            break;
        default:
            g_comm_para.check_flag = 3;
            break;
    }
    g_comm_para.stop_num = 1;

    gpio_vcom_set_comm_params();

    return 0;
}

static void serial_console_write(struct console *co, const char *s,
                                 unsigned count)
{
    while (count--)
    {
        if (*s == '\n')
            gpiovcom_send_data('\r');
        gpiovcom_send_data(*s++);
    }
}

static struct tty_driver *serial_console_device(struct console *c, int *index)
{
    *index = 0;
    return gpio_vcom_tty_drv;
}
#endif

static struct console cta_console = {
    .name =     "ttyS",
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
    register_console(&cta_console);
    return 0;
}
console_initcall(cheetah_serial_console_init);
#endif

MODULE_AUTHOR("weimaoxi");
MODULE_DESCRIPTION("Hisilicon virtual serial port over gpio");
MODULE_LICENSE("GPL");

