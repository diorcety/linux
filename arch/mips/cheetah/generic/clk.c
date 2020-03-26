/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#ifdef CONFIG_PROC_FS
//#include <linux/clockchips.h>
//#include <linux/init.h>
//#include <linux/interrupt.h>
#include <linux/kernel.h>
//#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h> //copy_from_user
#include <linux/string.h>
//#include <linux/delay.h>
#include <asm/time.h>
#include <asm/mach-cheetah/cheetah.h>
//#include <asm/bugs.h>

#include <asm/cpu.h>
#include <asm/cpu-info.h>

#define CHEETAH_MODULE_NAME "soc"
#define CLK_PROC_ENTRY_NAME "clk"

#define MHZ 1000000
#define MAX_FREQ 0x1E000 //122880
#define CPU_CLK_MODE_NUM 13
#define SYS_CLK_MODE_NUM 6
#define HVER_SYSCLK_BITMAP 0xf
#define HVER_CPUCLK_BITMAP 0xf
#define HVER_SYSCLK_SHIFT 16
#define HVER_CPUCLK_SHIFT 20
#define MIN_HIGH_PERIOD 2
#define MAX_USE_FREQ 640000000
#define MIN_USE_FREQ 60950000

static struct proc_dir_entry *cta_dir;

/* CPU: 60.95, 64, 80, 120, 160, 202.11, 240, 295.38, 320, 384, 480, 512, 640 MHz */
static char cpu_clk_tb[CPU_CLK_MODE_NUM + 1][7] = {"60","64","80","120","160","202","240","295","320","384","480","512","640","0"};
static unsigned int cpu_div[CPU_CLK_MODE_NUM] = { 0x7fc, 0x7f0, 0x7c0, 0x780, 0x680, 0x398, 0x380, 0x1d0, 0x280, 0x1a0, 0x180, 0x0f0, 0x0c0 };

static unsigned int cpu_clk_hz[CPU_CLK_MODE_NUM] = { 60950000, 64000000, 80000000, 120000000, 160000000, 202110000, 240000000, 295380000, 320000000, 384000000, 480000000, 512000000, 640000000};

/* SYS: 60.95, 64, 80, 120, 150.59, 160 MHz */
static char bus_clk_tb[SYS_CLK_MODE_NUM + 1][7] = {"60","64","80","120","150","160","0"};
static unsigned int bus_div[SYS_CLK_MODE_NUM] = { 0x7fc, 0x7f0, 0x7c0, 0x780, 0x688, 0x3c0 };
static unsigned int bus_clk_hz[SYS_CLK_MODE_NUM] = { 60000000, 64000000, 80000000, 120000000, 150590000, 160000000};

extern unsigned long loops_per_jiffy; /* init/main.c */
extern void __delay(int loops);
static unsigned long jiffy_per_Mhz;
static unsigned long cpu_clk_first;
extern void cta_baudrate_chng(void);

/*!
 * function: idelay
 *
 *  \brief delay specific instructions
 *  \param instruction count
 *  \return none
 */
void idelay(unsigned int count)
{
    __asm__ __volatile__(".set noreorder\n"
                         "1:\n"
                         "bnez    %0, 1b\n"
                         "subu    %0, 1\n"
                         ".set reorder\n":"=r"(count):"0"(count));
}

static int proc_cpuclk_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;

	p += sprintf(p, "%u %u\n", CPU_CLK/MHZ, SYS_CLK/MHZ);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

int cheetah_cpuclk(unsigned int cpu, unsigned int sys)
{
	unsigned int div, div_old, sys_clk_old = SYS_CLK;
	unsigned int n = smp_processor_id();

	if(!jiffy_per_Mhz)
	{
		cpu_clk_first = CPU_CLK/10000;
		jiffy_per_Mhz = (loops_per_jiffy*100)/cpu_clk_first;
	}
	if((cpu>=CPU_CLK_MODE_NUM+1) || (sys>=SYS_CLK_MODE_NUM+1)) //skip wrong mode setting
		return -1;
	if(cpu==CPU_CLK_MODE_NUM || sys==SYS_CLK_MODE_NUM)
	{
		if(cpu==CPU_CLK_MODE_NUM && sys==SYS_CLK_MODE_NUM)
		{
			div = ANAREG(CLKDIV);
			goto write_reg;
		}
		if(cpu==CPU_CLK_MODE_NUM)
		{
			div_old = ANAREG(CLKDIV);
			div = (div_old & 0xffff0000) | bus_div[sys];
			SYS_CLK = bus_clk_hz[sys];
		}
		if(sys==SYS_CLK_MODE_NUM)
		{
			div_old = ANAREG(CLKDIV);
			div = (cpu_div[cpu]<<CLKDIV_CPUFFST) | (div_old & 0x0000ffff);
			CPU_CLK = cpu_clk_hz[cpu];
		}
	}
	else
	{
		div = (cpu_div[cpu]<<CLKDIV_CPUFFST) | bus_div[sys];
		CPU_CLK = cpu_clk_hz[cpu];
		SYS_CLK = bus_clk_hz[sys];
	}

write_reg:
	//printk("div = %x\n",div);
	//sysclk = (MAX_FREQ/postdiv[(div&SYS_CLK_POSTDIV)>>CLKDIV_CPUSHFT]/(div&SYS_CLK_PREDIV))*MHZ;
	//printf("mode=0x%x\n", mode);
	//printf("div=0x%08x\n", div);
	//printf("sysclk=0x%08x\n", sysclk);
	ANAREG(CLKDIV) = div;
	ANAREG(CLKDIV) |= (CPU_CLK_UPDATE|SYS_CLK_UPDATE);
	ANAREG(CLKDIV); //read-back
	/*
	* 1. the minimum high period is 2 cycle for update signal
	* 2. increase the high period because lower bus frequency exist possibly
	*/
	idelay(MIN_HIGH_PERIOD*(MAX_USE_FREQ/MIN_USE_FREQ+1));
	ANAREG(CLKDIV) = div;

	// re-init uart baudrate
	if(SYS_CLK!=sys_clk_old)
		cta_baudrate_chng();

	printk("CPU_CLK = %u\n",CPU_CLK);
	printk("SYS_CLK = %u\n",SYS_CLK);

	if(cpu!=CPU_CLK_MODE_NUM)
		cpu_data[n].udelay_val = (jiffy_per_Mhz*(cpu_clk_hz[cpu]/10000))/100;
	
	return 0;
}

static int proc_cpuclk_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[300];
	int rc;
	int argc;
	unsigned int i, cpu_match, sys_match;
	char * argv[8] ;
	extern int get_args (const char *string, char *argvs[]);

	if (count > 0 && count < 299) 
	{
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
		argc = get_args( (const char *)buf , argv );
		//printk("%d, %s, %s\n",argc ,argv[0],argv[1]);
		if(argc!=2)
			goto err;
	}

	for(i = 0, cpu_match = 0; i < (CPU_CLK_MODE_NUM+1); i++)
	{
		if(!strcmp(cpu_clk_tb[i],argv[0]))
		{
			cpu_match = i;
			goto match1;
		}
	}
	goto err;
match1:
	for(i = 0, sys_match = 0; i < (SYS_CLK_MODE_NUM+1); i++)
	{
		if(!strcmp(bus_clk_tb[i],argv[1]))
		{
			sys_match = i;
			goto match2;
		}
	}
	goto err;
match2:
	//printk("cpu_match=%u\n",cpu_match);
	//printk("sys_match=%u\n",sys_match);
#if 1
	rc = cheetah_cpuclk(cpu_match, sys_match);
	if(rc == -1)
		goto err;
#endif
	return count;

err:
	printk("Wrong clk parameter! \n");
	return count;
}

static int __init plat_clk_init(void)
{
	struct proc_dir_entry *res;

	/* create directory */
	cta_dir = proc_mkdir(CHEETAH_MODULE_NAME, NULL);
	if(cta_dir == NULL) {
		return -ENOMEM;
	}

	res = create_proc_entry(CLK_PROC_ENTRY_NAME, S_IWUSR | S_IRUGO, cta_dir);
	if (!res) {
		remove_proc_entry(CHEETAH_MODULE_NAME, NULL);
		return -ENOMEM;
	}

	res->read_proc = proc_cpuclk_read;
	res->write_proc = proc_cpuclk_write;

	return 0;
}

static void __exit plat_clk_fini(void)
{
	remove_proc_entry(CLK_PROC_ENTRY_NAME, cta_dir);
	remove_proc_entry(CHEETAH_MODULE_NAME, NULL);
}

subsys_initcall(plat_clk_init);
module_exit(plat_clk_fini);
#endif
