 /*
 * Copyright (c) 2012   Montage Inc.,  All rights reserved.
 * 
 * pwm control
 */
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/idb.h>

#include "cta_pwm.h"

struct pwm_dev *pwm;

static int proc_pwm_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int i;

    idb_print("PWM CMD:\n");
    idb_print("\ten   (0~1)\n");
    idb_print("\tmode (0~1)\n");
    idb_print("\tduty (0~31)\n");
    idb_print("\tfreq (0~7)\n");
    idb_print("\tpol  (0~1)\n");
    if (!pwm) {
    	idb_print("PWM not init\n");
        return 0;
    }
    for (i = PWM_A; i < PWM_NUM; i++) {
    	idb_print("PWM_%c\n", (i + 'A'));
        idb_print("\tEnable     :%d\n", pwm->enable[i]);
        idb_print("\tAuto Mode  :%d\n", pwm->mode[i]);
        idb_print("\tDuty Cycle :%d\n", pwm->duty[i]);
        idb_print("\tSweep Freq :%d\n", pwm->sweep[i]);
        idb_print("\tPolarity   :%d\n", pwm->polarity[i]);
    }

    return 0;
}

void pwm_update(void)
{
    static int init = 0;
    unsigned int val = 0;

    // do the settings update
    val |= ((pwm->polarity[PWM_B] << 21) | (pwm->polarity[PWM_A] << 20));
    val |= ((pwm->sweep[PWM_B]    << 17) | (pwm->sweep[PWM_A]    << 14));
    val |= ((pwm->duty[PWM_B]     << 9 ) | (pwm->duty[PWM_A]     << 4 ));
    val |= ((pwm->mode[PWM_B]     << 3 ) | (pwm->mode[PWM_A]     << 2 ));
    val |= ((pwm->enable[PWM_B]   << 1 ) | (pwm->enable[PWM_A]   << 0 ));
    GPREG(PWM) = val;

    if (!init) {
        // switch PINMUX
        val = GPREG(PINMUX);
        if (GPREG(PINMUX) & EN_SIP_FNC) // DDR-QFN88-PWM
        {
            val &= ~(EN_PWM_AUX_FNC | EN_ADC_OUT_FNC);
            val |= EN_PWM_FNC;
        }
        else                            // SDR-QFN128-PWM_AUX
        {
            val &= ~(EN_PWM_FNC | EN_AGC_FNC);
            val |= EN_PWM_AUX_FNC;
        }
        GPREG(PINMUX) = val;
        init = 1;
    }
}

static void pwm_func(int argc, char *argv[])
{
    static int idx = PWM_A;
    unsigned int val = 0;

    if (argc < 2)
        return;

    sscanf(argv[1],"%d",&val);
	if (!strcmp(argv[0], "set")) {
        if ( (!strcmp(argv[1], "A")) || (!strcmp(argv[1], "a")) || (!strcmp(argv[1], "0")) )
            idx = PWM_A;
        else if ( (!strcmp(argv[1], "B")) || (!strcmp(argv[1], "b")) || (!strcmp(argv[1], "1")) )
            idx = PWM_B;
        else {
            idb_print("PWM out of range!!\n");
        }
        return;
	}
	else if (!strcmp(argv[0], "en")) {
        if (val > 1)
            return;
        pwm->enable[idx] = val;
	}
	else if (!strcmp(argv[0], "mode")) {
        if (val > 1)
            return;
        pwm->mode[idx] = val;
	}
	else if (!strcmp(argv[0], "duty")) {
        if (val > 31)
            return;
        pwm->duty[idx] = val;
	}
	else if (!strcmp(argv[0], "freq")) {
        if (val > 7)
            return;
        pwm->sweep[idx] = val;
	}
	else if (!strcmp(argv[0], "pol")) {
        if (val > 1)
            return;
        pwm->polarity[idx] = val;
	}
    else {
        return;
    }

    pwm_update();
}

extern int get_args (const char *string, char *argvs[]);
static int proc_pwm_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf[300];
    int argc ;
    char * argv[8] ;

    if (count > 0 && count < 299) {
        if (copy_from_user(buf, buffer, count))
            return -EFAULT;
        buf[count-1]='\0';
        argc = get_args( (const char *)buf , argv );
        pwm_func(argc, argv);
    }
    return count;
    
}

static int __init cta_pwm_init(void)
{
	struct proc_dir_entry *res;

	res = create_proc_entry("pwm", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->read_proc = proc_pwm_read;
	res->write_proc = proc_pwm_write;

	if(!pwm) {
		pwm = kzalloc(sizeof(struct pwm_dev), GFP_ATOMIC);
    }

	return 0;
}

static void __exit cta_pwm_exit(void)
{
	if(!pwm)
        kfree(pwm);
}

module_init(cta_pwm_init);
module_exit(cta_pwm_exit);
