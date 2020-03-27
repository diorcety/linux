/*
 *	Generic parts
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/llc.h>
#include <net/llc.h>
#include <net/stp.h>

#include "br_private.h"

//int (*br_should_route_hook)(struct sk_buff *skb);

void dump_skb(struct sk_buff *skb ,char *name)
{
	printk("!!!! %s start\n",name);
	{	
        int j;
        int dlen = (skb->len > 64)? 64 : skb->len;

        printk(">>> %s skb->data=%x , len=%x", name, (int)skb->data, skb->len);
        for(j=0;j<dlen;j++)
        {
            if (0==(j&0x1f))
                printk("\n%03x:", j);
            printk(" %02x",skb->data[j]&0xff);
        }
        printk("\n");
	}
	printk("!!!! %s End\n\n",name);

}
void dump_buf(char *buf, int len)
{
	printk("!!!! Dump buf\n");
	{	
        int j;

        for(j=0;j<len;j++)
        {
            if (0==(j&0x1f))
                printk("\n%03x:", j);
            printk(" %02x",buf[j]&0xff);
        }
        printk("\n");
	}
	printk("!!!! Dump buf end\n");
}

static int __init cta_br_init(void)
{
	brioctl_set(br_ioctl_deviceless_stub); //add/del bridge. add api for wireless use.

	return 0;
}

static void __exit cta_br_deinit(void)
{
	brioctl_set(NULL);

	rcu_barrier(); /* Wait for completion of call_rcu()'s */

	br_handle_frame_hook = NULL;
}

//EXPORT_SYMBOL(br_should_route_hook);

module_init(cta_br_init)
module_exit(cta_br_deinit)
MODULE_LICENSE("GPL");
MODULE_VERSION(BR_VERSION);
