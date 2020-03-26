/*
 *	Device handling code
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
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/udp.h>

#include <asm/uaccess.h>
#include "br_private.h"
#include <linux/netfilter_bridge.h>
#include <asm/mach-cheetah/common.h>

u16 (*br_forward_lookup_hook)(char *addr) = NULL;
EXPORT_SYMBOL_GPL(br_forward_lookup_hook);

static int cta_br_software_forward(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct udphdr *udph;

	if ((eth_hdr(skb)->h_proto == htons(ETH_P_ARP)) ||
		(eth_hdr(skb)->h_proto == htons(ETH_P_WAPI_PAE)) || 
		(eth_hdr(skb)->h_proto == htons(ETH_P_PAE)))
	{
		return 1;
	}
	iph = (struct iphdr*)skb_network_header(skb);
	udph = udp_hdr(skb);
	if ((iph->protocol == IPPROTO_UDP)&&
	   ((udph->dest==htons(67))||(udph->dest==htons(68))))
	{
		return 1;
	}
	return 0;
}

static int cta_br_forward_xmit(struct sk_buff *skb, struct net_device *dev, __u16 id, char bss)
{
	struct net_bridge *br = netdev_priv(dev);
	struct net_bridge_port *pt;
	char bss_desc = 0;

	list_for_each_entry_rcu(pt, &br->port_list, list)
	{
		if(pt->port_id == WLAID)
			bss_desc = ((pt->dev->priv_flags & IFF_BSS_MASK) >> IFF_BSS_SHIFT);
		if(pt->port_id != id)	
			continue;
		if((pt->port_id == WLAID) && ((bss_desc != bss) && !(dev->priv_flags & IFF_FLOOD_UNKNOW_UC)))
			continue;
		skb->dev = (struct net_device *)pt->dev;
#if defined(BR_DBG)
		printk("forward: (%s)\n", skb->dev->name);
		dump_buf(skb->mac_header, 14);
		dump_buf(skb->data, skb->len);
#endif
		br_dev_queue_push_xmit(skb);
		return 1;
	}
	return 0;
}

//extern int mac_addr_lookup_engine_search_addr(u8 *addr, int *hash_index, u32* basic_cap);
/* net device transmit always called with no BH (preempt_disabled) */
netdev_tx_t cta_br_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	const unsigned char *dest;
	struct net_bridge_port *p = rcu_dereference(skb->dev->br_port);
#if 0
	dump_skb(skb,__func__);
#endif

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	if(!p)
	{	
		skb_reset_mac_header(skb);
		skb_pull(skb, ETH_HLEN);
	}
	dest = skb->mac_header;

	if (dest[0] & 1)
	{
#if defined(BR_DBG)
		printk("forward mcast: (%s)\n", skb->dev->name);
		dump_buf(skb->mac_header, 14);
		dump_buf(skb->data, skb->len);
#endif
		br_flood_deliver(br, skb);
	}
	else
	{
		unsigned short val=0;
		char fwd = 0, bss = 0;
		int hit = 0;
		if(br_forward_lookup_hook)
		{
			val = br_forward_lookup_hook((char *)dest);
			fwd = (val & STA_FORWARD_MASK) >> STA_FORWARD_SHIFT;
			bss = (val & STA_BSSDESC_MASK) >> STA_BSSDESC_SHIFT;
		}
		hit = cta_br_software_forward(skb);
		if(p)
		{
			if((fwd == SOFTWARE_FORWARD) ||
			  ((fwd == HARDWARE_FORWARD) && hit))
			{
				/* wifi */
#if defined(BR_DBG)
				printk("forward by bss index: (%s)\n", skb->dev->name);
				dump_buf(skb->mac_header, 14);
				dump_buf(skb->data, skb->len);
#endif
				cta_br_forward_xmit(skb, dev, WLAID, bss);
			}
			else if((p->port_id==WLAID) && (fwd==0))
			{
				/* To partner's interface */
				skb->dev = (struct net_device *)p->path_cost;
#if defined(BR_DBG)
				printk("forward to partner: (%s)\n", skb->dev->name);
				dump_buf(skb->mac_header, 14);
				dump_buf(skb->data, skb->len);
#endif
				br_dev_queue_push_xmit(skb);
			}
			else
			{
				/* To self interface */
#if defined(BR_DBG)
				printk("forward to itself: (%s)\n", skb->dev->name);
				dump_buf(skb->mac_header, 14);
				dump_buf(skb->data, skb->len);
#endif
				br_dev_queue_push_xmit(skb);
			}
		}
		else
		{
			/* Local out */
			if((fwd == SOFTWARE_FORWARD) ||
			  ((fwd == HARDWARE_FORWARD) && hit) || 
			  (!hit && (br->dev->priv_flags & IFF_FLOOD_UNKNOW_UC)))
			{
				cta_br_forward_xmit(skb, dev, WLAID, bss);
			}
			else
			{
				cta_br_forward_xmit(skb, dev, ETHID, 0);
			}
		}
	}
	return NETDEV_TX_OK;
}

static int cta_br_dev_open(struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);

	br_features_recompute(br);
	netif_start_queue(dev);

	return 0;
}
#if 0
static void br_dev_set_multicast_list(struct net_device *dev)
{
}
#endif
static int cta_br_dev_stop(struct net_device *dev)
{
	netif_stop_queue(dev);

	return 0;
}
#if 0
static int br_change_mtu(struct net_device *dev, int new_mtu)
{
	struct net_bridge *br = netdev_priv(dev);
	if (new_mtu < 68 || new_mtu > br_min_mtu(br))
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}
#endif
/* Allow setting mac address to any valid ethernet address. */
static int cta_br_set_mac_address(struct net_device *dev, void *p)
{
	struct net_bridge *br = netdev_priv(dev);
	struct sockaddr *addr = p;
	struct net_bridge_port *pt;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EINVAL;

	spin_lock_bh(&br->lock);
	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);

	list_for_each_entry_rcu(pt, &br->port_list, list)
	{
		pt->dev->netdev_ops->ndo_set_mac_address(pt->dev,p);
	}

	br->flags |= BR_SET_MAC_ADDR;
	spin_unlock_bh(&br->lock);

	return 0;
}

#if 0
static void br_getinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy(info->driver, "bridge");
	strcpy(info->version, BR_VERSION);
	strcpy(info->fw_version, "N/A");
	strcpy(info->bus_info, "N/A");
}
static int br_set_sg(struct net_device *dev, u32 data)
{
	struct net_bridge *br = netdev_priv(dev);

	if (data)
		br->feature_mask |= NETIF_F_SG;
	else
		br->feature_mask &= ~NETIF_F_SG;

	br_features_recompute(br);
	return 0;
}

static int br_set_tso(struct net_device *dev, u32 data)
{
	struct net_bridge *br = netdev_priv(dev);

	if (data)
		br->feature_mask |= NETIF_F_TSO;
	else
		br->feature_mask &= ~NETIF_F_TSO;

	br_features_recompute(br);
	return 0;
}

static int br_set_tx_csum(struct net_device *dev, u32 data)
{
	struct net_bridge *br = netdev_priv(dev);

	if (data)
		br->feature_mask |= NETIF_F_NO_CSUM;
	else
		br->feature_mask &= ~NETIF_F_ALL_CSUM;

	br_features_recompute(br);
	return 0;
}
#endif
static const struct ethtool_ops br_ethtool_ops = {
//	.get_drvinfo    = br_getinfo,
	.get_link		= ethtool_op_get_link,
	.get_tx_csum	= ethtool_op_get_tx_csum,
//	.set_tx_csum 	= br_set_tx_csum,
	.get_sg			= ethtool_op_get_sg,
//	.set_sg			= br_set_sg,
	.get_tso		= ethtool_op_get_tso,
//	.set_tso		= br_set_tso,
	.get_ufo		= ethtool_op_get_ufo,
	.get_flags		= ethtool_op_get_flags,
};

static struct net_device_ops br_netdev_ops = {
	.ndo_open				= cta_br_dev_open,
	.ndo_stop				= cta_br_dev_stop,
	.ndo_start_xmit			= cta_br_dev_xmit,
	.ndo_set_mac_address	= cta_br_set_mac_address,
//	.ndo_set_multicast_list	= br_dev_set_multicast_list,
//	.ndo_change_mtu			= br_change_mtu,
	.ndo_do_ioctl			= cta_br_dev_ioctl,
};

void br_dev_setup(struct net_device *dev)
{
	random_ether_addr(dev->dev_addr);
	ether_setup(dev);

	dev->netdev_ops = &br_netdev_ops;
	dev->destructor = free_netdev;
	SET_ETHTOOL_OPS(dev, &br_ethtool_ops);
	dev->tx_queue_len = 0;
	dev->priv_flags = IFF_EBRIDGE;

	dev->features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_HIGHDMA |
			NETIF_F_GSO_MASK | NETIF_F_NO_CSUM | NETIF_F_LLTX |
			NETIF_F_NETNS_LOCAL | NETIF_F_GSO;
}
