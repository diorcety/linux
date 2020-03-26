/*
 *	Handle incoming frames
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

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netfilter_bridge.h>
#include <asm/mach-cheetah/common.h>
#include "br_private.h"

/* Bridge group multicast address 802.1d (pg 51). */
const u8 br_group_address[ETH_ALEN] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x00 };

static void br_pass_frame_up(struct net_bridge *br, struct sk_buff *skb)
{
#ifdef CONFIG_NETFILTER
	struct net_device *indev;
#endif
	struct net_device *brdev = br->dev;

	brdev->stats.rx_packets++;
	brdev->stats.rx_bytes += skb->len;

#ifdef CONFIG_NETFILTER
	indev = skb->dev;
#endif
	skb->dev = brdev;

#ifdef CONFIG_NETFILTER
	NF_HOOK(PF_BRIDGE, NF_BR_LOCAL_IN, skb, indev, NULL,
		netif_receive_skb);
#else
	NF_HOOK(0,0,skb,0, NULL,
		netif_receive_skb);
#endif
}

/* note: already called with rcu_read_lock (preempt_disabled) */
int br_handle_frame_finish(struct sk_buff *skb)
{
	const unsigned char *dest = eth_hdr(skb)->h_dest;
	struct net_bridge_port *p = rcu_dereference(skb->dev->br_port);
	struct net_bridge *br;
	struct sk_buff *skb2;
	int flood = 0;
	
	/* insert into forwarding database after filtering to avoid spoofing */
	br = p->br;

	/* The packet skb2 goes to the local host (NULL to skip). */
	skb2 = NULL;

	if (br->dev->flags & IFF_PROMISC)
		skb2 = skb;

	if (is_multicast_ether_addr(dest)) {
		br->dev->stats.multicast++;
		skb2 = skb;
		
		if(!(br->dev->priv_flags & IFF_NO_FLOOD_MC))
			flood = 1;
		else	/* don't flood to wan port */
			skb = NULL;
	} else if (skb->pkt_type == PACKET_HOST
			|| !p->path_cost) {
		skb2 = skb;
		
		/* Do not forward the packet since it's local. */
		skb = NULL;
	}

	if (skb2 == skb)
		skb2 = skb_clone(skb, GFP_ATOMIC);

	if (skb2)
	{
#if defined(BR_DBG)
		printk("skb2 case -------------------------------------\n");
		printk("from: (%s)\n", skb2->dev->name);
		printk("to: host (br=0x%s)\n", br->dev->name);
		dump_buf(skb2->mac_header, 14);
		dump_buf(skb2->data, skb2->len);
#endif
		br_pass_frame_up(br, skb2);
	}

	if(flood)
	{
#if defined(BR_DBG)
		printk("flood case -------------------------------------\n");
		printk("from mcast: (%s)\n", skb->dev->name);
		printk("to: host (br=0x%s)\n", br->dev->name);
		dump_buf(skb->mac_header, 14);
		dump_buf(skb->data, skb->len);
#endif
		br_flood_forward(br, skb);
		return 1;
	}
	if (skb) {

#if defined(BR_DBG)
		printk("skb case -------------------------------------\n");
		printk("from: (%s)\n", skb->dev->name);
		printk("to: host (br=0x%s)\n", br->dev->name);
		dump_buf(skb->mac_header, 14);
		dump_buf(skb->data, skb->len);
#endif
		skb_forward_csum(skb);
		/* Call cta_br_dev_xmit */
		br->dev->netdev_ops->ndo_start_xmit(skb, br->dev);
	}
	return 1;
}
#if 0
/* note: already called with rcu_read_lock (preempt_disabled) */
static int br_handle_local_finish(struct sk_buff *skb)
{
	struct net_bridge_port *p = rcu_dereference(skb->dev->br_port);

	if (p)
		br_fdb_update(p->br, p, eth_hdr(skb)->h_source);
	return 0;	 /* process further */
}
#endif

/* Does address match the link local multicast address.
 * 01:80:c2:00:00:0X
 */
static inline int is_link_local(const unsigned char *dest)
{
	__be16 *a = (__be16 *)dest;
	static const __be16 *b = (const __be16 *)br_group_address;
	static const __be16 m = cpu_to_be16(0xfff0);

	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | ((a[2] ^ b[2]) & m)) == 0;
}

/*
 * Called via cta_br_handle_frame_hook.
 * Return NULL if skb is handled
 * note: already called with rcu_read_lock (preempt_disabled)
 */
struct sk_buff *cta_br_handle_frame(struct net_bridge_port *p, struct sk_buff *skb)
{
	const unsigned char *dest = eth_hdr(skb)->h_dest;

#if 0
	dump_skb(skb,__func__);
#endif

#if defined(CONFIG_WLA_WAPI)
	if((eth_hdr(skb)->h_proto == ETH_P_PAE) || (eth_hdr(skb)->h_proto == ETH_P_WAPI_PAE))
#else
	if(eth_hdr(skb)->h_proto == ETH_P_PAE)
#endif
	{	
		return skb;
	}

	if (!is_valid_ether_addr(eth_hdr(skb)->h_source))
		goto drop;

	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		return NULL;

	if (unlikely(is_link_local(dest))) {
		/* Pause frames shouldn't be passed up by driver anyway */
		if (skb->protocol == htons(ETH_P_PAUSE))
			goto drop;
	}

	if (!compare_ether_addr(p->br->dev->dev_addr, dest)) //br0, eth0, wlan0 same mac 
		skb->pkt_type = PACKET_HOST;
	if (NF_HOOK(PF_BRIDGE, NF_BR_PRE_ROUTING, skb, skb->dev, NULL,
		br_handle_frame_finish))
		return NULL;
	else
		return skb;

drop:
	kfree_skb(skb);
	return NULL;
}
