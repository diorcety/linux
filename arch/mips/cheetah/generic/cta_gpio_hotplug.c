/*
 *  Copyright (c) 2013	Montage Inc.	All rights reserved.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>

#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <asm/mach-cheetah/idb.h>

#define hotplug_path	uevent_helper
#define DRV_NAME	"cta-gpio-buttons"

#define BH_SKB_SIZE	2048

#define PFX	DRV_NAME ": "

#undef BH_DEBUG

#ifdef BH_DEBUG
#define BH_DBG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, DRV_NAME, ##args )
#else
#define BH_DBG(fmt, args...) do {} while (0)
#endif

#define BH_ERR(fmt, args...) printk(KERN_ERR "%s: " fmt, DRV_NAME, ##args )
#define EV_KEY2 0x0002

const struct gpio_keys_button *cta_buttons;
int polling=1;

struct bh_map {
	unsigned int	code;
	const char	*name;
};

struct gpio_keys_button_data {
	char *action;
	char *name;
	unsigned int type;
	int last_state;
	int sw_state;
	int count;
	int press_cnt;
	int threshold;
	int can_sleep;
	unsigned long diff;
	struct delayed_work work;
};

struct gpio_keys_polled_dev {
	struct delayed_work work;

	struct device *dev;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button_data data[0];
};

extern u64 uevent_next_seqnum(void);

#define BH_MAP(_code, _name)		\
	{				\
		.code = (_code),	\
		.name = (_name),	\
	}

static struct bh_map button_map[] = {
	BH_MAP(BTN_0,		"BTN_0"),
	BH_MAP(BTN_1,		"BTN_1"),
	BH_MAP(BTN_2,		"BTN_2"),
	BH_MAP(BTN_3,		"BTN_3"),
	BH_MAP(BTN_4,		"BTN_4"),
	BH_MAP(BTN_5,		"BTN_5"),
	BH_MAP(BTN_6,		"BTN_6"),
	BH_MAP(BTN_7,		"BTN_7"),
	BH_MAP(BTN_8,		"BTN_8"),
	BH_MAP(BTN_9,		"BTN_9"),
	BH_MAP(KEY_RESTART,	"reset"),
#ifdef KEY_WPS_BUTTON
	BH_MAP(KEY_WPS_BUTTON,	"wps"),
#endif /* KEY_WPS_BUTTON */
};

/* -------------------------------------------------------------------------*/

static void button_hotplug_work(struct work_struct *work)
{
	struct gpio_keys_button_data *bdata; 
	char *argv[3];
	char *envp[5];

	bdata = (struct gpio_keys_button_data *)container_of(work, struct gpio_keys_button_data, work.work);
	
	argv[0] = hotplug_path;
	if(bdata->type == EV_KEY || bdata->type == EV_KEY2)
		argv[1] = "button";
	else if(bdata->type == EV_SW)
		argv[1] = "switch";
	
	argv[2]=NULL;

	/*env*/
	envp[0] = kmalloc(sizeof("BUTTON=")+ sizeof(bdata->name), GFP_ATOMIC);
	sprintf(envp[0],"BUTTON=%s",bdata->name);

	envp[1] = kmalloc(sizeof("SEEN=") + 4, GFP_ATOMIC);
	sprintf(envp[1],"SEEN=%ld",bdata->diff/HZ);

	envp[2] = kmalloc(sizeof("ACTION=")+ sizeof(bdata->action), GFP_ATOMIC);
	sprintf(envp[2],"ACTION=%s",bdata->action);

	envp[3] = kmalloc(sizeof("COUNT=")+ 4, GFP_ATOMIC);
	sprintf(envp[3],"COUNT=%d",bdata->press_cnt);

	envp[4]	= NULL;  
	
	call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
	
	if(bdata->type == EV_KEY)
		bdata->press_cnt = 0;

	kfree(envp[0]);
	kfree(envp[1]);
	kfree(envp[2]);
	kfree(envp[3]);

}

static int button_get_index(unsigned int code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(button_map); i++)
		if (button_map[i].code == code)
			return i;

	return -1;
}

static void button_hotplug_event(struct gpio_keys_button_data *bdata,
			   unsigned int type, unsigned int code, int state)
{
	int btn;
	
	btn = button_get_index(code);
	if (btn < 0)
		return;

	bdata->type = type;
	bdata->action = state ? "pressed" : "released";
	bdata->name=(char *)button_map[btn].name;

	schedule_delayed_work(&bdata->work, 30);
}


static void gpio_keys_polled_check_state(const struct gpio_keys_button *button,
					 struct gpio_keys_button_data *bdata)
{
	int state;
	unsigned int type = button->type;
	if(button->gpio < 0)
		return;

	if (bdata->can_sleep)
		state = !!gpio_get_value_cansleep(button->gpio);
	else
		state = !!gpio_get_value(button->gpio);

	state = !!(state ^ button->active_low);
	if(type == EV_KEY) {
		if (state != bdata->last_state) {
			if (state == 1) { //pressed
				int ret;
				ret = cancel_delayed_work(&bdata->work);
				if(!ret) {
					cancel_work_sync(&(bdata->work.work));
				}
				bdata->diff = jiffies;
				bdata->press_cnt++;
			}
			else {
				bdata->diff = (long)jiffies - (long)bdata->diff;
				button_hotplug_event(bdata, type, button->code, state);
				bdata->count = 0;
			}
			bdata->last_state = state;
		}
	}
	else if(type == EV_KEY2){
		if (state != bdata->last_state) {
			if (state == 1) { //pressed
				int ret;
				ret = cancel_delayed_work(&bdata->work);
				if(!ret) {
					cancel_work_sync(&(bdata->work.work));
				}
				button_hotplug_event(bdata, type, button->code, state);
				bdata->diff = jiffies;
				bdata->press_cnt++;
			}
			else {//released
				bdata->diff = (long)jiffies - (long)bdata->diff;
				button_hotplug_event(bdata, type, button->code, state);
				bdata->count = 0;
			}
			bdata->last_state = state;
		}
	}
	else if (type == EV_SW) {
		if (state != bdata->last_state) {
			if(bdata->diff == 0)
				bdata->diff = jiffies;
			if( (long)jiffies - (long)bdata->diff > 10) {
				bdata->last_state = state;
				button_hotplug_event(bdata, type, button->code, state);
				bdata->diff = 0;
			}
			bdata->sw_state = state;
		}
		else if(bdata->sw_state != state) {
			bdata->diff = 0;
			bdata->sw_state = state;
		}
	}
}

static void gpio_keys_polled_queue_work(struct gpio_keys_polled_dev *bdev)
{
	struct gpio_keys_platform_data *pdata = bdev->pdata;
	unsigned long delay = msecs_to_jiffies(pdata->poll_interval);

	if (delay >= HZ)
	    delay = round_jiffies_relative(delay);
	schedule_delayed_work(&bdev->work, delay);
}

static void gpio_keys_polled_poll(struct work_struct *work)
{
	struct gpio_keys_polled_dev *bdev =
		container_of(work, struct gpio_keys_polled_dev, work.work);
	struct gpio_keys_platform_data *pdata = bdev->pdata;
	int i;

	if(!polling)
		goto schedule;

	for (i = 0; i < bdev->pdata->nbuttons; i++) {
		struct gpio_keys_button_data *bdata = &bdev->data[i];

		if (bdata->count < bdata->threshold)
			bdata->count++;
		else
			gpio_keys_polled_check_state(&pdata->buttons[i], bdata);
	}

schedule:	
	gpio_keys_polled_queue_work(bdev);
}

static void gpio_keys_polled_open(struct gpio_keys_polled_dev *bdev)
{
	struct gpio_keys_platform_data *pdata = bdev->pdata;
	int i;

#if 0
	if (pdata->enable)
		pdata->enable(bdev->dev);
#endif

	/* report initial state of the buttons */
	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_polled_check_state(&pdata->buttons[i], &bdev->data[i]);

	gpio_keys_polled_queue_work(bdev);
}

static void gpio_keys_polled_close(struct gpio_keys_polled_dev *bdev)
{
#if 0
	struct gpio_keys_platform_data *pdata = bdev->pdata;
#endif

	cancel_delayed_work_sync(&bdev->work);

#if 0
	if (pdata->disable)
		pdata->disable(bdev->dev);
#endif
}

static void gpio_func(int argc, char *argv[])
{
	static char buf[5][32];	
	if (!strcmp(argv[0], "start"))
		polling = 1;
	else if (!strcmp(argv[0], "stop"))
		polling = 0;
	else if (!strcmp(argv[0], "set")) {
		int num, gpio, low_trig, type;
		struct gpio_keys_button *gpiop;
		if(argc != 6) {
			return;
		}
		sscanf(argv[1],"%d",&num);
		sscanf(argv[2],"%d",&gpio);
		sscanf(argv[3],"%d",&low_trig);
		sscanf(argv[4],"%d",&type);
		sscanf(argv[5],"%s",&buf[num][0]);
		gpiop = (struct gpio_keys_button *)&cta_buttons[num];
		gpiop->gpio = gpio;
		gpiop->active_low=low_trig;
		gpiop->type=type;
	}
	else if (!strcmp(argv[0], "show")) {
		int i;
		const struct gpio_keys_button *gpiop;
		for(i=0; i<5; i++) {
			gpiop = &cta_buttons[i];
			printk("BTN_%d gpio:%d type:%d desc:%s\n",i, gpiop->gpio, gpiop->type, &buf[i][0]);
		}
	}
}

extern int get_args (const char *string, char *argvs[]);
static int gpio_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
    char buf[300];
    int argc ;
    char * argv[8] ;

    if (count > 0 && count < 299) {
        if (copy_from_user(buf, buffer, count))
            return -EFAULT;
        buf[count-1]='\0';
        argc = get_args( (const char *)buf , argv );
        gpio_func(argc, argv);
    }
    return count;
}

static const struct file_operations gpio_proc_fops = {
	.owner          = THIS_MODULE,
	.write          = gpio_write,
	.llseek         = noop_llseek,
};

static int gpio_keys_polled_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct gpio_keys_polled_dev *bdev;
	int error;
	int i;

	if (!pdata || !pdata->poll_interval)
		return -EINVAL;

	bdev = kzalloc(sizeof(struct gpio_keys_polled_dev) +
		       pdata->nbuttons * sizeof(struct gpio_keys_button_data),
		       GFP_KERNEL);
	if (!bdev) {
		dev_err(dev, "no memory for private data\n");
		return -ENOMEM;
	}

	cta_buttons = &pdata->buttons[0];

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_keys_button_data *bdata = &bdev->data[i];

		if (button->wakeup) {
			dev_err(dev, DRV_NAME " does not support wakeup\n");
			error = -EINVAL;
			goto err_free_gpio;
		}

		bdata->last_state = 0;
		bdata->sw_state = 0;
		bdata->threshold = DIV_ROUND_UP(button->debounce_interval,
						pdata->poll_interval);
		
		INIT_DELAYED_WORK(&bdata->work, (work_func_t)button_hotplug_work);

	}

	bdev->dev = &pdev->dev;
	bdev->pdata = pdata;
	platform_set_drvdata(pdev, bdev);

	INIT_DELAYED_WORK(&bdev->work, gpio_keys_polled_poll);

	gpio_keys_polled_open(bdev);

	{
		struct proc_dir_entry *res;

		res = proc_create(DRV_NAME, S_IWUSR | S_IRUGO, NULL, &gpio_proc_fops);

		if (!res) {
			error = -ENOMEM;
			goto err_free_gpio;
		}
	}

	return 0;

err_free_gpio:
	while (--i >= 0)
		gpio_free(pdata->buttons[i].gpio);

	kfree(bdev);
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int gpio_keys_polled_remove(struct platform_device *pdev)
{
	struct gpio_keys_polled_dev *bdev = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = bdev->pdata;
	int i = pdata->nbuttons;

	gpio_keys_polled_close(bdev);

	while (--i >= 0)
		gpio_free(pdata->buttons[i].gpio);

	kfree(bdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver gpio_keys_polled_driver = {
	.probe	= gpio_keys_polled_probe,
	.remove	= gpio_keys_polled_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_keys_polled_init(void)
{
	return platform_driver_register(&gpio_keys_polled_driver);
}

static void __exit gpio_keys_polled_exit(void)
{
	platform_driver_unregister(&gpio_keys_polled_driver);
}

module_init(gpio_keys_polled_init);
module_exit(gpio_keys_polled_exit);

MODULE_AUTHOR("Sky Wang <sky.wang@montage-tech.com>");
MODULE_DESCRIPTION("Montage Cheetah Polled GPIO Buttons hotplug driver");
MODULE_LICENSE("GPL v2");
