#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/synclink.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/mach-cheetah/cta_keypad.h>
#include <asm/mach-cheetah/cheetah.h>

#define RAKEY                   "/lib/wdk/rakey"
#define RAKEY_PATH              "PATH=/usr/bin:/bin:/sbin:/lib/wdk"

#define MADC_CHANNEL	0
#define MADC_TICK_THD	0x50
#define MADC_MAXADC	28

unsigned short *keycodes;
//unsigned int keycodes_size;
struct ana_keypad {	
	const struct cta_ana_keypad_platform_data *pdata;
	unsigned int last_key_state;	
	struct delayed_work work;
};

static inline void
cta_keypad_build_keymap(const struct cta_keymap_data *keymap_data)
{
	int i;

	//keycodes_size = keymap_data->keymap_size;
	for (i=0; i<MADC_MAXADC+1; i++)
		keycodes[i] = 0;

	for (i=0; i<keymap_data->keymap_size; i++)
	{
		unsigned int key = keymap_data->keymap[i];
		unsigned int row = KEY_ROW(key);
		unsigned int adc = KEY_VAL(key);
		if (adc <= MADC_MAXADC)
			keycodes[adc] = row;
	}
}

static unsigned int get_madc_value(int ch)
{
	unsigned int val = 0;

	ANAREG_UPDATE(MADCCTL, BIT0, BIT0);
	udelay(32);
	//mdelay(1);
	ANAREG_UPDATE(MADCCTL, (ch << 4), (BIT4 | BIT0));
	udelay(32);
	//mdelay(1);
	GPREG(MADCSAMP) = (MADC_TICK_THD << 16) | BIT0;
	while((GPREG(MADCSAMP) & BIT0))
	{	
		;
	}	
	val = (GPREG(MADCSAMP) >> 8) & 0xFF;
	
	return val;
}

/*
static void keypad_create_event(char *action)
{
	char *argv[3];
	char *envp[2];
	int ret;
       
	argv[0] = RAKEY;
	argv[1] = action;
	argv[2] = NULL;

	//env
	envp[0] = RAKEY_PATH; 
	envp[1] = NULL; 
	//printk("!!! argv[0]:%s argv[1]:%s envp[0]:%s\n",argv[0], argv[1], envp[0]);   
       
	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
	return;

}
*/

static void ana_keypad_scan(struct work_struct *work)
{
	struct ana_keypad *keypad = container_of(work, struct ana_keypad, work.work);
	//struct cta_ana_keypad_platform_data *pdata = keypad->pdata;
	const int delay = 10;
	unsigned int adc_value = 0;
	/*
	char *keyaction[12] =
	{
		"1", "2", "3", "4",
		"5", "6", "7", "8",
		"9", "10", "11", "12"
	};
	*/
	adc_value = get_madc_value(0);	
	if (adc_value != 0) {
		printk("read adc value %d ", adc_value);	
		
		if (keycodes[adc_value])
		{
			printk("key %d\n", keycodes[adc_value]);
			keypad->last_key_state = adc_value;
		}
	}	

	schedule_delayed_work(&keypad->work,delay);	
}

static int __devinit cta_ana_keypad_probe(struct platform_device *pdev)
{
	const struct cta_ana_keypad_platform_data *pdata;
	const struct cta_keymap_data *keymap_data;
	struct ana_keypad *keypad;
	int err;

	pdata = pdev->dev.platform_data;
	if (!pdata) {	
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}
	
	keymap_data = pdata->keymap_data;
	if (!keymap_data) {
		dev_err(&pdev->dev, "no keymap data defined\n");
		return -EINVAL;
	}

	keypad = kzalloc(sizeof(struct ana_keypad), GFP_KERNEL);
	
	if (!keypad) {
		err = -ENOMEM;
		goto err_free_mem;
	}


	keycodes = (unsigned short *)kzalloc((MADC_MAXADC+1) * sizeof(unsigned short), GFP_KERNEL);

	if (!keycodes) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	cta_keypad_build_keymap(keymap_data);

	keypad->pdata = pdata;
	
	keypad->last_key_state = 99;

	INIT_DELAYED_WORK(&keypad->work, ana_keypad_scan);
	        
	device_init_wakeup(&pdev->dev, pdata->wakeup);

	platform_set_drvdata(pdev, keypad);
	
	schedule_delayed_work(&keypad->work, 0);

	return 0;

	err_free_mem:
	        kfree(keypad);
	        return err;
}

static int __devexit cta_ana_keypad_remove(struct platform_device *pdev)   
{
	struct ana_keypad *keypad = platform_get_drvdata(pdev);
	
	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);
	kfree(keycodes);
	kfree(keypad);
	
	return 0;
}

static struct platform_driver cta_ana_keypad_driver = {
	.probe          = cta_ana_keypad_probe,
	.remove         = __devexit_p(cta_ana_keypad_remove),
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name   = "cta-ana-keypad",
		.owner  = THIS_MODULE,
	},
};

#ifdef CONFIG_PROC_FS
static int get_args(const char *string, char *argvs[])
{
	const int MAX_ARGV = 3;
	char *p;
	int n;

	argvs[0] = 0;
	n = 0;

	p = (char*) string;
	while (*p == ' ')
		p++;
	while (*p)
	{
		argvs[n] = p;
		while (*p != ' ' && *p)
			p++;
		if (*p == 0)
			goto out;
		*p++ = '\0';
		while (*p == ' ' && *p)
			p++;
out:
		n++;
		if (n == MAX_ARGV)
			break;
	}
	return n;
}


static int strtoul(char *str, void *v)
{   
	return(sscanf(str,"%d", (unsigned *)v)==1);
}


static int proc_ana_keypad_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int i = 0;
	int len = 0;
	int btn = 0;

	if (!keycodes) 
		return 0;

	for (i=0; i<MADC_MAXADC+1; i++)
	{
		if (keycodes[i])
		{
			if (keycodes[i] == btn)
				p += sprintf(p, " %d", i);
			else
				p += sprintf(p, "\nbtn %d = adc %d", keycodes[i], i);
			btn = keycodes[i];
		}
	} 
	p += sprintf(p, "\n");

	len = (p - page) - off;
	if (len < 0)
		len = 0;
	*eof = (len <= count)? 1 : 0;
	*start = page + off;

	return len;
}


static int proc_ana_keypad_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[64] = {0};
	int argc = 0;
	char *argv[3];
	unsigned int btn = 0;
	unsigned int adc = 0;
	
	if (count > 0 && count < 64 )
	{
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1] = '\0';
		argc = get_args((const char *)buf, argv);
		
		if (argc == 3)
		{
			if (strcmp(argv[0], "btn"))
				goto err;
			if (!strtoul(argv[1], &btn))
				goto err;
			if (!strtoul(argv[2], &adc))
				goto err;
			if (adc > MADC_MAXADC || adc < 0)
				goto err;
			keycodes[adc] = btn;
		}
		else
			goto err;

	}
	return count;
err:
	printk(KERN_ERR "btn <btn num> <adc>\n");
	return -EFAULT;
}
#endif

static int __init cta_ana_keypad_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *res;

	res = create_proc_entry("anakeypad", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->read_proc = proc_ana_keypad_read;
	res->write_proc = proc_ana_keypad_write;
#endif

	return platform_driver_register(&cta_ana_keypad_driver);
}
	
static void __exit cta_ana_keypad_exit(void)
{
	platform_driver_unregister(&cta_ana_keypad_driver);
}

	
module_init(cta_ana_keypad_init);
module_exit(cta_ana_keypad_exit);
