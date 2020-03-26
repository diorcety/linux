/*
 * LED Kernel Network Interface Trigger for cheetah
 *
 * Toggles the LED to reflect the link and traffic state of a named net device
 *
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <asm/mach-cheetah/cheetah.h>

#include "../../../drivers/leds/leds.h"

#define SW_BASE				0xaf008000UL
#define PHY_LINK (1<<4)
#define P0_OFS		0x000
#define P1_OFS		0x100
#define SWREG_READ32(x)  (*(volatile unsigned int*)(SW_BASE+(x)))
#define EMAC_STATISTIC_3		0x7C
	#define RX_ENQ_CNT				0x3FFFFFFFUL
#define EMAC_STATISTIC_6		0x88
	#define TX_DEQ_CNT				0x3FFFFFFFUL
#include <mac_ctrl.h>
#define WIFI_RX		0xbe4
	#define RX2TX_CNT				0x0000FFFFUL
	#define RX2ETH_CNT				0xFFFF0000UL
#define WIFI_TX1	0xac4
	#define TX_SOFT_CNT				0x0000FFFFUL
	#define TX_ETH_CNT				0xFFFF0000UL
#define WIFI_TX2	0xac8
	#define TX_WIFI_CNT				0xFFFF0000UL
#define WIFI_CNT_SHIFTBIT 16

#define MODE_NONE 0
#define MODE_LINK 1
#define MODE_ACTV 2

#define MODESTR_BUFSIZE 64
/*
 * Configurable sysfs attributes:
 *
 * interface - interface number to monitor (0:eth0, 1:eth1, 2:WiFi)
 *
 * interval - duration of LED blink, in milliseconds
 *
 * Some suggestions:
 *
 *  Ethernet port0 link+activity LED:
 *  $ echo cta_if >someled/trigger
 *  $ echo 0 >someled/interface
 *  $ echo "link activity" >someled/mode
 *
 *  Ethernet port1 link+activity LED:
 *  $ echo cta_if >someled/trigger
 *  $ echo 1 >someled/interface
 *  $ echo "link activity" >someled/mode
 *
 *  WiFi link+activity LED:
 *  $ echo cta_if >someled/trigger
 *  $ echo 2 >someled/interface
 *  $ echo "link activity" >someled/mode
 *
 *  Seperate Ethernet port1 link LED and Ethernet port1 activity LED:
 *  $ echo cta_if >led_link/trigger
 *  $ echo 1 >led_link/interface
 *  $ echo "link" >led_link/mode
 *  $ echo cta_if >led_activity/trigger
 *  $ echo 1 >led_activity/interface
 *  $ echo "activity" >led_activity/mode
 *
 */

extern int link_status[2];
extern MAC_INFO *info;

struct led_cta_if_data {
	rwlock_t lock;

	struct timer_list timer;

	struct led_classdev *led_cdev;

	int interface;
	unsigned interval;
	unsigned mode;
	unsigned last_activity;
};

static bool cta_if_link(struct led_cta_if_data *trigger_data)
{
	switch(trigger_data->interface) {
#ifdef CONFIG_CHEETAH_ETH
		case 0:
		    return (link_status[0] & PHY_LINK);
		    break;
		case 1:
		    return (link_status[1] & PHY_LINK);
		    break;
#endif
#ifdef CONFIG_CHEETAH_WLA
		case 2:
		    return true;
		    break;
#endif
	}
	return false;
}

static unsigned cta_if_count(struct led_cta_if_data *trigger_data)
{
#ifdef CONFIG_CHEETAH_WLA
	unsigned int wifi_rx, wifi_tx;
#endif
	switch(trigger_data->interface) {
#ifdef CONFIG_CHEETAH_ETH
		case 0:
		    return (SWREG_READ32(P0_OFS|EMAC_STATISTIC_3)&RX_ENQ_CNT)+(SWREG_READ32(P0_OFS|EMAC_STATISTIC_6)&TX_DEQ_CNT);
		    break;
		case 1:
		    return (SWREG_READ32(P1_OFS|EMAC_STATISTIC_3)&RX_ENQ_CNT)+(SWREG_READ32(P1_OFS|EMAC_STATISTIC_6)&TX_DEQ_CNT);
		    break;
#endif
#ifdef CONFIG_CHEETAH_WLA
		case 2:
		    wifi_rx = MACREG_READ32(WIFI_RX);
		    wifi_tx = MACREG_READ32(WIFI_TX1);
		    return info->wl_rxq.pkt_count + (wifi_rx&RX2TX_CNT) + ((wifi_rx&RX2ETH_CNT)>>WIFI_CNT_SHIFTBIT) + (wifi_tx&TX_SOFT_CNT) + ((wifi_tx&TX_ETH_CNT)>>WIFI_CNT_SHIFTBIT) + ((MACREG_READ32(WIFI_TX2)&TX_WIFI_CNT)>>WIFI_CNT_SHIFTBIT);
		    break;
#endif
	}
	return 0;
}

static void set_baseline_state(struct led_cta_if_data *trigger_data)
{
	mod_timer(&trigger_data->timer, jiffies + trigger_data->interval);
}

static ssize_t led_interface_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;

	read_lock(&trigger_data->lock);
	sprintf(buf, "%d\n", trigger_data->interface);
	read_unlock(&trigger_data->lock);

	return strlen(buf) + 1;
}

static ssize_t led_interface_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;
	int num = -1;

	num = simple_strtol(buf, NULL, 10);
	if (num > 2 || num < 0)
		return -EINVAL;

	write_lock(&trigger_data->lock);

	trigger_data->interface = num;

	if (trigger_data->interface >= 0) {
		set_baseline_state(trigger_data); /* start timers */
	}

	write_unlock(&trigger_data->lock);
	return size;
}

static DEVICE_ATTR(interface, 0644, led_interface_show, led_interface_store);

static ssize_t led_mode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;

	read_lock(&trigger_data->lock);

	if (trigger_data->mode == MODE_NONE) {
		strcpy(buf, "none\n");
	} else {
		if (trigger_data->mode & MODE_LINK)
			strcat(buf, "link ");
		if (trigger_data->mode & MODE_ACTV)
			strcat(buf, "activity ");
		strcat(buf, "\n");
	}

	read_unlock(&trigger_data->lock);

	return strlen(buf)+1;
}

static ssize_t led_mode_store(struct device *dev,
			      struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;
	char copybuf[MODESTR_BUFSIZE];
	int new_mode = -1;
	char *p, *token;

	/* take a copy since we don't want to trash the inbound buffer when using strsep */
	strncpy(copybuf, buf, sizeof(copybuf));
	copybuf[MODESTR_BUFSIZE-1] = 0;
	p = copybuf;

	while ((token = strsep(&p, " \t\n")) != NULL) {
		if (!*token)
			continue;

		if (new_mode == -1)
			new_mode = MODE_NONE;

		if (!strcmp(token, "none"))
			new_mode = MODE_NONE;
		else if (!strcmp(token, "activity"))
			new_mode |= MODE_ACTV;
		else if (!strcmp(token, "link"))
			new_mode |= MODE_LINK;
		else
			return -EINVAL;
	}

	if (new_mode == -1)
		return -EINVAL;

	write_lock(&trigger_data->lock);
	trigger_data->mode = new_mode;
	set_baseline_state(trigger_data);
	write_unlock(&trigger_data->lock);

	return size;
}

static DEVICE_ATTR(mode, 0644, led_mode_show, led_mode_store);

static ssize_t led_interval_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;

	read_lock(&trigger_data->lock);
	sprintf(buf, "%u\n", jiffies_to_msecs(trigger_data->interval));
	read_unlock(&trigger_data->lock);

	return strlen(buf) + 1;
}

static ssize_t led_interval_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	/* impose some basic bounds on the timer interval */
	if (count == size && value >= 5 && value <= 10000) {
		write_lock(&trigger_data->lock);
		trigger_data->interval = msecs_to_jiffies(value);
		set_baseline_state(trigger_data); // resets timer
		write_unlock(&trigger_data->lock);
		ret = count;
	}

	return ret;
}

static DEVICE_ATTR(interval, 0644, led_interval_show, led_interval_store);

/* here's the real work! */
static void cta_if_trig_timer(unsigned long arg)
{
	struct led_cta_if_data *trigger_data = (struct led_cta_if_data *)arg;
	unsigned new_activity;

	write_lock(&trigger_data->lock);

	new_activity = (trigger_data->mode & MODE_ACTV) ? cta_if_count(trigger_data) : 0;

	if (cta_if_link(trigger_data) && (trigger_data->mode & MODE_LINK)) {
		/* base state is ON (link present) */
		/* if there's no link, we don't get this far and the LED is off */

		/* OFF -> ON always */
		/* ON -> OFF on activity */
		if (trigger_data->led_cdev->brightness == LED_OFF) {
			led_set_brightness(trigger_data->led_cdev, LED_FULL);
		} else if (trigger_data->last_activity != new_activity) {
			led_set_brightness(trigger_data->led_cdev, LED_OFF);
		}
	} else {
		/* base state is OFF */
		/* ON -> OFF always */
		/* OFF -> ON on activity */
		if (trigger_data->led_cdev->brightness == LED_FULL) {
			led_set_brightness(trigger_data->led_cdev, LED_OFF);
		} else if (trigger_data->last_activity != new_activity) {
			led_set_brightness(trigger_data->led_cdev, LED_FULL);
		}
	}

	trigger_data->last_activity = new_activity;
	mod_timer(&trigger_data->timer, jiffies + trigger_data->interval);

	write_unlock(&trigger_data->lock);
}

static void cta_if_trig_activate(struct led_classdev *led_cdev)
{
	struct led_cta_if_data *trigger_data;
	int rc;

	trigger_data = kzalloc(sizeof(struct led_cta_if_data), GFP_KERNEL);
	if (!trigger_data)
		return;

	rwlock_init(&trigger_data->lock);

	setup_timer(&trigger_data->timer, cta_if_trig_timer, (unsigned long) trigger_data);

	trigger_data->led_cdev = led_cdev;
	trigger_data->interface = 0;

	trigger_data->mode = MODE_NONE;
	trigger_data->interval = msecs_to_jiffies(50);
	trigger_data->last_activity = 0;

	led_cdev->trigger_data = trigger_data;

	rc = device_create_file(led_cdev->dev, &dev_attr_interface);
	if (rc)
		goto err_out;
	rc = device_create_file(led_cdev->dev, &dev_attr_mode);
	if (rc)
		goto err_out_interface;
	rc = device_create_file(led_cdev->dev, &dev_attr_interval);
	if (rc)
		goto err_out_mode;

	return;

err_out_mode:
	device_remove_file(led_cdev->dev, &dev_attr_mode);
err_out_interface:
	device_remove_file(led_cdev->dev, &dev_attr_interface);
err_out:
	led_cdev->trigger_data = NULL;
	kfree(trigger_data);
}

static void cta_if_trig_deactivate(struct led_classdev *led_cdev)
{
	struct led_cta_if_data *trigger_data = led_cdev->trigger_data;

	if (trigger_data) {
		device_remove_file(led_cdev->dev, &dev_attr_interface);
		device_remove_file(led_cdev->dev, &dev_attr_mode);
		device_remove_file(led_cdev->dev, &dev_attr_interval);

		write_lock(&trigger_data->lock);

		write_unlock(&trigger_data->lock);

		del_timer_sync(&trigger_data->timer);

		kfree(trigger_data);
	}
}

static struct led_trigger cta_if_led_trigger = {
	.name     = "cta_if",
	.activate = cta_if_trig_activate,
	.deactivate = cta_if_trig_deactivate,
};

static int __init cta_if_trig_init(void)
{
	return led_trigger_register(&cta_if_led_trigger);
}

static void __exit cta_if_trig_exit(void)
{
	led_trigger_unregister(&cta_if_led_trigger);
}

module_init(cta_if_trig_init);
module_exit(cta_if_trig_exit);
