/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#ifdef CONFIG_PROC_FS
/* #define DEBUG */
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h> //copy_from_user
#include <linux/string.h>
#include <asm/time.h>
#include <asm/mach-cheetah/cheetah.h>

#include <linux/synclink.h>
#include <asm/mach-cheetah/common.h>

#include <asm/cpu.h>
#include <asm/cpu-info.h>

#include <ip301.h>

#define PWR_PROC_ENTRY_NAME	"soc/power"
#define CMD_STRING_LEN_MAX	50
#define CMD_ARGUMENT_MAX	2

extern unsigned int rf_read(char reg);
extern void rf_write(char reg, int val);

typedef enum CHEETAH_POWER_DOMAIN {
	PD_WIFI = 0,
	PD_MADC,
	PD_USB,
	PD_MAX
} CHEETAH_PD;

typedef enum CHEETAH_POWER_DOMAIN_CONTROL {
	PD_CTRL_OFF = 0,
	PD_CTRL_ON,
	PD_CTRL_INFO,
	PD_CTRL_MAX
} CHEETAH_PD_CTRL;

typedef enum CHEETAH_POWER_DOMAIN_STATE {
	PD_OFF = 0,
	PD_ON
} CHEETAH_PD_STATE;

static CHEETAH_PD_STATE pd_state[PD_MAX];

#ifdef DEBUG
static void cheetah_power_state_dump(void)
{
	int i;

	for (i = 0; i < PD_MAX; i++)
		pr_info("power domain state[%d] %d\n", i, pd_state[i]);
}
#else
#define cheetah_power_state_dump(x)	do {} while(0);
#endif

struct proc_dir_entry *res;

static void cheetah_power_control(CHEETAH_PD pd, CHEETAH_PD_CTRL ctrl)
{
#ifdef CONFIG_CHEETAH_WLA
	static unsigned int reg0;
	static unsigned int reg6;
	static unsigned int reg15;
	static unsigned int reg17;
#endif

	if ((ctrl >= PD_CTRL_MAX) || (ctrl < 0)) {
		pr_err("%s() invalid control command %d\n", __func__, __LINE__);
		return;
	}

	cheetah_power_state_dump();

	switch (pd) {
#ifdef CONFIG_CHEETAH_WLA
	case PD_WIFI:
		if (ctrl == PD_CTRL_OFF) {
			if (pd_state[PD_WIFI] == PD_OFF) break;

			pr_debug("WiFi Power off\n");

			reg0 = rf_read(0);
			reg6 = rf_read(0x6);
			reg15 = rf_read(0x15);
			reg17 = rf_read(0x17);

			rf_write(0,0x0);
			rf_write(0x6,0x10ba);
			rf_write(0x15,0x1000);
			rf_write(0x17,0x3fff);

			/*WiFi ADC & DAC power down*/
			ANAREG(W_ADC) |= (W_ADC_PD);
			ANAREG(W_DAC) |= (W_DAC_PD);

			pd_state[PD_WIFI] = PD_OFF;
		} else if (ctrl == PD_CTRL_ON) {
			if (pd_state[PD_WIFI] == PD_ON) break;

			pr_debug("WiFi Power on\n");

			rf_write(0, reg0);
			rf_write(0x6, reg6);
			rf_write(0x15, reg15);
			rf_write(0x17, reg17);

			/*WiFi ADC & DAC power up*/
			ANAREG(W_ADC) &= ~(W_ADC_PD);
			ANAREG(W_DAC) &= ~(W_DAC_PD);

			pd_state[PD_WIFI] = PD_ON;
		} else if (ctrl == PD_CTRL_INFO) {
			pr_info("Rf 0x0:0x%x\n", rf_read(0));
			pr_info("Rf 0x6:0x%x\n", rf_read(0x6));
			pr_info("Rf 0x15:0x%x\n", rf_read(0x15));
			pr_info("Rf 0x17:0x%x\n", rf_read(0x17));
			pr_info("WiFi ADC:0x%.8x\n", ANAREG(W_ADC));
			pr_info("WiFi DAC:0x%.8x\n", ANAREG(W_DAC));
			pr_info("WiFi power state:0x%x\n", pd_state[PD_WIFI]);
		}
		break;
#endif
	case PD_MADC:
		if (ctrl == PD_CTRL_OFF) {
			if (pd_state[PD_MADC] == PD_OFF) break;

			pr_debug("Monitor ADC Power off\n");

			/*Monitor ADC power down. Need to cancel work queue*/
			ANAREG_UPDATE(MADCCTL, BIT0, BIT0);
			pd_state[PD_MADC] = PD_OFF;
		} else if (ctrl == PD_CTRL_ON) {
			if (pd_state[PD_MADC] == PD_ON) break;

			pr_debug("Monitor DAC Power on\n");

			/*Monitor ADC power up*/
			ANAREG_UPDATE(MADCCTL, 0, BIT0);
			pd_state[PD_MADC] = PD_ON;
		} else if (ctrl == PD_CTRL_INFO) {
			pr_info("Monitor ADC:0x%.8x\n", ANAREG(MADCCTL));
			pr_info("Monitor ADC power state:0x%x\n", pd_state[PD_MADC]);
		}
		break;
	case PD_USB:
		if (ctrl == PD_CTRL_OFF) {
			if (pd_state[PD_USB] == PD_OFF) break;

			pr_debug("USB Power off\n");

			/*USB PHY power down*/
			USBREG(USB_A0_REG) |= (USB_DOWN);
			pd_state[PD_USB] = PD_OFF;
		} else if (ctrl == PD_CTRL_ON) {
			if (pd_state[PD_USB] == PD_ON) break;

			pr_debug("USB Power on\n");

			/*USB PHY power up, Need to enable wq*/
			USBREG(USB_A0_REG) &= ~(USB_DOWN);
			pd_state[PD_USB] = PD_ON;
		} else if (ctrl == PD_CTRL_INFO) {
			pr_info("USB PHY:0x%.8x\n", USBREG(USB_A0_REG));
			pr_info("USB power state:0x%x\n", pd_state[PD_USB]);
		}
		break;
	default:
		pr_err("%s() invalid power domain %d\n", __func__, __LINE__);
		break;
	}
}

static int proc_cpupwr_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;

	p += sprintf(p, "power control entry\n");

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int proc_cpupwr_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[CMD_STRING_LEN_MAX];
	int argc;
	char * argv[8];
	extern int get_args (const char *string, char *argvs[]);
	CHEETAH_PD_CTRL pd_cmd = PD_CTRL_ON;

	pr_debug("%s() start %s %s\n", __func__, __DATE__, __TIME__);

	if ((count == 0) || (count > CMD_STRING_LEN_MAX)) {
		pr_err("%s() err %d\n", __func__, __LINE__);
		goto err;
	}

	if (copy_from_user(buf, buffer, count)) {
		pr_err("%s() err %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	buf[count - 1] = '\0';
	argc = get_args((const char *)buf, argv);
	//printk("%d, %s, %s\n",argc ,argv[0],argv[1]);

	if ((argc > CMD_ARGUMENT_MAX) || (argc < 1)) {
		pr_err("%s() err %d\n", __func__, __LINE__);
		goto err;
	}

	if (argc == 2) {
		pd_cmd = !!simple_strtoul(argv[1], &argv[1], 10);
		if (!strcmp("all", argv[0])) {
#ifdef CONFIG_CHEETAH_WLA
			cheetah_power_control(PD_WIFI, pd_cmd);
#endif
			cheetah_power_control(PD_MADC, pd_cmd);
			cheetah_power_control(PD_USB, pd_cmd);
#ifdef CONFIG_CHEETAH_WLA
		} else if (!strcmp("wifi", argv[0])) {
			cheetah_power_control(PD_WIFI, pd_cmd);
#endif
		} else if (!strcmp("madc", argv[0])) {
			cheetah_power_control(PD_MADC, pd_cmd);
		} else if (!strcmp("usb", argv[0])) {
			cheetah_power_control(PD_USB, pd_cmd);
		} else {
			pr_err("%s() err %d\n", __func__, __LINE__);
			goto err;
		}
	} else {
		if (!strcmp("info", argv[0])  ) {
#ifdef CONFIG_CHEETAH_WLA
			cheetah_power_control(PD_WIFI, PD_CTRL_INFO);
#endif
			cheetah_power_control(PD_MADC, PD_CTRL_INFO);
			cheetah_power_control(PD_USB, PD_CTRL_INFO);
		} else {
			pr_err("%s() err %d\n", __func__, __LINE__);
			goto err;
		}
	}

	pr_debug("%s() end %s %s\n", __func__, __DATE__, __TIME__);
	return count;

err:
	pr_err("Wrong parameter! \n");
	return count;
}

static int __init plat_pwr_init(void)
{
	int i;

	/* synchronize cheetah power domain state with hardware */
	for (i = 0; i < PD_MAX; i++)
		pd_state[i] = PD_ON;

	res = create_proc_entry(PWR_PROC_ENTRY_NAME, S_IRUGO, NULL);
	if (!res) {
		pr_err("create_proc_entry() failed [%s]\n", PWR_PROC_ENTRY_NAME);
		return -ENOMEM;
	}
	res->read_proc = proc_cpupwr_read;
	res->write_proc = proc_cpupwr_write;

	pr_info("%s() successful\n", __func__);
	return 0;
}

static void __exit plat_pwr_fini(void)
{
	remove_proc_entry(PWR_PROC_ENTRY_NAME, res);
}

module_init(plat_pwr_init);
module_exit(plat_pwr_fini);
#endif
