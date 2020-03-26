/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file nf_hnat.c
*   \brief  netfilter hook for cheetah hnat
*   \author Montage
*/

#include <linux/types.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <net/checksum.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <net/dst.h>
#include <asm/mach-cheetah/cta_hnat.h>
#include <asm/mach-cheetah/cta_eth.h>

#include <net/netfilter/nf_conntrack_acct.h>
MODULE_AUTHOR("Montage");
MODULE_DESCRIPTION("HNAT for netfilter module");
MODULE_LICENSE("GPL");

#if LINUX_VERSION_CODE >=KERNEL_VERSION(2,6,17)
#define LINUX_2_6
#endif

#define ASSERT_READ_LOCK(x) MUST_BE_READ_LOCKED(&ip_nat_lock)
#define ASSERT_WRITE_LOCK(x) MUST_BE_WRITE_LOCKED(&ip_nat_lock)
#ifndef	LINUX_2_6 
#include <linux/netfilter_ipv4/ip_conntrack.h>
#include <linux/netfilter_ipv4/ip_conntrack_tuple.h>
#include <linux/netfilter_ipv4/ip_conntrack_core.h>
#include <linux/netfilter_ipv4/ip_conntrack_protocol.h>
#include <linux/netfilter_ipv4/ip_nat.h>
#include <linux/netfilter_ipv4/ip_nat_rule.h>
#include <linux/netfilter_ipv4/ip_nat_protocol.h>
#include <linux/netfilter_ipv4/ip_nat_core.h>
#include <linux/netfilter_ipv4/ip_nat_helper.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/netfilter_ipv4/listhelp.h>
#define nfct_help(ct) (ct->helper)
#define skb_network_header(skb) skb->nh.iph;
#else

#include <linux/netfilter_ipv4.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <net/netfilter/nf_nat_helper.h>

#define ip_conntrack nf_conn
#define ip_conntrack_tuple_hash nf_conntrack_tuple_hash 
#define ip_conntrack_get nf_ct_get
#endif

#define SIP2LSB(v)			((unsigned short)((v << 16) >>16))
#define SIP2MSB(v)			((unsigned short)(v >> 16))
#define LSB2SIP(v, m)		((unsigned int)(v | (m << 16)))
#define REFRESH_INTERVAL   	HZ*5
#define HNAT_CHECK			5
struct module *ip_conntrack_fastnat_module = THIS_MODULE;
struct timer_list rf_timer;
extern void (*hnat_detach_function)(struct ip_conntrack *ct);
extern if_cta_eth_t * if_cta_eth_g[MAX_IF_NUM];
#define DBG(a...) //printk
//#define DBG printk
#define IPA(a,b,c,d) ((a<<24)|(b<<16)|(c<<8)|d)
#define SIPNET(ip)	(nf_hnat_lan_msk&(ip))
unsigned int nf_hnat_lan_ip=IPA(192,168,0,1);
unsigned int nf_hnat_wan_ip=IPA(10,20,0,4);
unsigned int nf_hnat_lan_msk=IPA(255,255,255,0);
int wan_mode=0;
int hnat_mode=0;
int hnat_tcp_timeout=1800;
int hnat_udp_timeout=180;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
struct hlist_nulls_head *nf_conntrack_hash = NULL;
#endif
extern unsigned int cta_hnat_sdmz_fakeip;
extern char cta_hnat_sdmz_mac[6];

int hnat_fast_send(unsigned long key)
{
	struct sk_buff *skb = (struct sk_buff *)key;
	struct iphdr *iph;
	semi_cb_t *semi_cb = SKB2SEMICB(skb);
	int err = 1;

	/* HW NAT has changed IP header, TCP/UDP port and TTL */
	if(IS_HWMOD(semi_cb))
	{
		if (!pskb_may_pull(skb, sizeof(struct iphdr)))
			goto out;		
		iph = (struct iphdr *)skb_network_header(skb);

		cta_hnat_fast_send((char*)iph, skb->len, (unsigned long)skb, semi_cb);
		err = 0;
	}
out:
	return err;
}

void hnat_detach(struct ip_conntrack *ct)
{
	if(ct->hnat_cb!= 0)
	{
		natdes *ns;
		ns = ct->hnat_cb;
		ns->used=0;
		cta_hnat_free(ct->hnat_cb);
		ct->hnat_cb=0;
	}
}
	
#if 1 /* User have to add a timer to maitain the hnat */
#ifndef LINUX_2_6
static int iterate_all(const struct ip_conntrack_tuple_hash *i)
{
	ct = i->ctrack;
	iterate_ct(ct);
	return 0;
}
#else
#define ip_ct_refresh(ct,delta) __nf_ct_refresh_acct(ct,0,0,delta,0)
static int ip_conntrack_htable_size=0;
#endif
static int iterate_ct(struct ip_conntrack *ct)
{
	natdes *ns;
	if(ct->hnat_cb!= 0)
	{
		ns = ct->hnat_cb;
		if(ns->hnat_acc && ns->counter != 0 )
		{
			ns->counter=0;
				/* we have to study the refresh for the ct, below is a simple sample */
			switch(ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple.dst.protonum)
			{
					case IPPROTO_TCP:
						if(ct->proto.tcp.state == TCP_CONNTRACK_ESTABLISHED)
							ip_ct_refresh(ct, hnat_tcp_timeout*HZ);
						break;
					case IPPROTO_UDP:
						ip_ct_refresh(ct, hnat_udp_timeout*HZ);
						break;
					default:
						break;	
			}	
		}
	}
	
	return 0;
}
#endif
/* The link for get_next_corpse may be a problem */
void ip_natexpire(unsigned long id)
{
	int i;
	/* Please refer to the ip_nat.c */
	/* search all the ct, refer to the ip_conntrack_core.c */
	//cta_hnat_dump(0, 0 );
#ifndef LINUX_2_6
	READ_LOCK(&ip_conntrack_lock);
	for (i = 0; i < ip_conntrack_htable_size; i++) {
		if (LIST_FIND(&ip_conntrack_hash[i], iterate_all, struct ip_conntrack_tuple_hash *))
			goto finished;
	}
	finished:
	READ_UNLOCK(&ip_conntrack_lock);
#else
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
	struct hlist_nulls_node *n;
#else
	struct hlist_node *n;
#endif

	if(nf_conntrack_hash)
	{
		spin_lock_bh(&nf_conntrack_lock);
		for (i = 0; i < ip_conntrack_htable_size; i++) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
			hlist_nulls_for_each_entry(h, n, &nf_conntrack_hash[i], hnnode) {
#else
			hlist_for_each_entry(h, n, &nf_conntrack_hash[i], hnode) {
#endif
				ct = nf_ct_tuplehash_to_ctrack(h);
				iterate_ct(ct);
			}
		}	
		spin_unlock_bh(&nf_conntrack_lock);

	}
#endif //test
	rf_timer.expires = jiffies + REFRESH_INTERVAL;
	add_timer(&rf_timer);
}

static unsigned int
ip_fastnat_fwd(unsigned int hooknum,
#ifndef LINUX_2_6
	  struct sk_buff **pskb,
#else
	  struct sk_buff *skb,
#endif
	  const struct net_device *in,
	  const struct net_device *out,
	  int (*okfn)(struct sk_buff *))
{
		//printk("ip_fastnat_fwd\n");
#ifndef LINUX_2_6
	    struct sk_buff *skb = *pskb;
#endif
		struct iphdr *iph = (struct iphdr*)skb_network_header(skb);
		struct ip_conntrack *ct;
		enum ip_conntrack_info ctinfo;
		struct nf_conn_counter *acct;
		
		/* Support only TCP and UDP */
		if ((iph->protocol != IPPROTO_TCP)&&(iph->protocol != IPPROTO_UDP))
			return NF_ACCEPT;
		
		/* Skip fragement */	
		if (iph->frag_off & htons(IP_MF|IP_OFFSET))
			return NF_ACCEPT;
		
		ct = ip_conntrack_get(skb, &ctinfo);
		if(!ct)
			return NF_ACCEPT;

		if (nfct_help(ct))
			return NF_ACCEPT;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
		nf_conntrack_hash = dev_net(in)->ct.hash;
		ip_conntrack_htable_size = dev_net(in)->ct.htable_size;
#endif
		{
			/* Here you can setup the hardware NAT by ct information */
			if(iph->protocol == IPPROTO_TCP)
			{
				if(ct->proto.tcp.state != TCP_CONNTRACK_ESTABLISHED)
					return NF_ACCEPT;
			}	
			/*We attach pkt when total pkt larger than HNAT_CHECK*/
			acct = nf_conn_acct_find(ct);
			if (!acct)
				return NF_ACCEPT;
			else if ((acct[IP_CT_DIR_ORIGINAL].packets + acct[IP_CT_DIR_REPLY].packets) <= HNAT_CHECK)
				return NF_ACCEPT;

			/* Here we have add a menber hnat_cb to the ct data structure */
			if(!ct->hnat_cb)  /* only add once */
			{
				int dir, i;
				natdes *ns;
				DBG("iph->daddr=%x, saddr=%x dir=%d\n", iph->daddr, iph->saddr, CTINFO2DIR(ctinfo));
				if((SIPNET(nf_hnat_lan_ip)==SIPNET(iph->saddr) && CTINFO2DIR(ctinfo)==IP_CT_DIR_ORIGINAL) || /* LAN to WAN */
					(SIPNET(nf_hnat_lan_ip)==SIPNET(iph->daddr) && CTINFO2DIR(ctinfo)==IP_CT_DIR_REPLY)) /* WAN to LAN */
				{
						dir = IP_CT_DIR_ORIGINAL;
				}
				else
				{
						dir = IP_CT_DIR_REPLY;
				}
#ifndef LINUX_2_6
				if(( i = find_mipidx((unsigned int)ct->tuplehash[!dir].tuple.dst.ip)) < 0)
#else
				if(( i = find_mipidx((unsigned int)ct->tuplehash[!dir].tuple.dst.u3.ip)) < 0)
#endif
				{
					return NF_ACCEPT;
				}
				/*
				DBG("ct->tuplehash[0].tuple.dst.ip=%x\n",ct->tuplehash[0].tuple.dst.ip);
				DBG("ct->tuplehash[0].tuple.src.ip=%x\n",ct->tuplehash[0].tuple.src.ip);
				DBG("ct->tuplehash[0].tuple.src.u.tcp.port=%x\n",ct->tuplehash[0].tuple.src.u.tcp.port);
				DBG("ct->tuplehash[0].tuple.dst.u.tcp.port=%x\n",ct->tuplehash[0].tuple.dst.u.tcp.port);
				DBG("ct->tuplehash[1].tuple.dst.ip=%x\n",ct->tuplehash[1].tuple.dst.ip);
				DBG("ct->tuplehash[1].tuple.src.ip=%x\n",ct->tuplehash[1].tuple.src.ip);
				DBG("ct->tuplehash[1].tuple.src.u.tcp.port=%x\n",ct->tuplehash[1].tuple.src.u.tcp.port);
				DBG("ct->tuplehash[1].tuple.dst.u.tcp.port=%x\n",ct->tuplehash[1].tuple.dst.u.tcp.port);
				*/
				if (hnat_mode == 0)
					return NF_ACCEPT;

				if (0==(ns = (void *) cta_hnat_alloc()))
				{
					return NF_ACCEPT;
				}
				
				memset(ns, 0, sizeof(natdes));
				ns->mipidx = i;
				
				ns->sp = ct->tuplehash[dir].tuple.src.u.tcp.port;

#ifndef LINUX_2_6
				ns->dip = ct->tuplehash[dir].tuple.dst.ip;
#else
				ns->dip = ct->tuplehash[dir].tuple.dst.u3.ip;
#endif
				ns->dp = ct->tuplehash[dir].tuple.dst.u.tcp.port;
				ns->mp = ct->tuplehash[!dir].tuple.dst.u.tcp.port;
				ns->prot = iph->protocol;
#if 1				
#ifndef LINUX_2_6
				if(ct->tuplehash[dir].tuple.src.ip == cta_hnat_sdmz_fakeip)
#else
				if(ct->tuplehash[dir].tuple.src.u3.ip == cta_hnat_sdmz_fakeip)
#endif
				{
					ns->siplsb = SIP2LSB(nf_hnat_wan_ip);
					ns->sipmsb = SIP2MSB(nf_hnat_wan_ip);
					cta_hnat_manual_cfgmac(ns, cta_hnat_sdmz_mac, 0, 0);
				}
				else
#endif				 
				{
#ifndef LINUX_2_6
					ns->siplsb = SIP2LSB(ct->tuplehash[dir].tuple.src.ip);
					ns->sipmsb = SIP2MSB(ct->tuplehash[dir].tuple.src.ip);
#else
					ns->siplsb = SIP2LSB(ct->tuplehash[dir].tuple.src.u3.ip);
					ns->sipmsb = SIP2MSB(ct->tuplehash[dir].tuple.src.u3.ip);
#endif
				}
				ns->used = 1;
				cta_hnat_attach(ns);
#if 1				
#ifndef LINUX_2_6
				if(ct->tuplehash[dir].tuple.src.ip == cta_hnat_sdmz_fakeip)
#else
				if(ct->tuplehash[dir].tuple.src.u3.ip == cta_hnat_sdmz_fakeip)
#endif
				{
					ns->siplsb = 0;
				}
#endif				
				ct->hnat_cb=ns;	
			}	
#ifdef LINUX_2_6	//skip tcp_packet window check
			if(iph->protocol == IPPROTO_TCP)
			{
				ct->proto.tcp.seen[0].flags|=IP_CT_TCP_FLAG_BE_LIBERAL;
				ct->proto.tcp.seen[1].flags|=IP_CT_TCP_FLAG_BE_LIBERAL;
			}
#endif
		}
		return NF_ACCEPT;
}	

static struct nf_hook_ops ip_fastnat_fwd_ops
#ifndef LINUX_2_6
= { { NULL, NULL }, ip_fastnat_fwd, PF_INET, NF_IP_FORWARD, NF_IP_PRI_FILTER+1};
#else
= { { NULL, NULL }, ip_fastnat_fwd, 0, PF_INET, NF_INET_FORWARD, NF_IP_PRI_FILTER+1};
#endif

static int init_or_cleanup(int init)
{
	int ret = 0;	

	if (!init) goto cleanup;	

	ret = nf_register_hook(&ip_fastnat_fwd_ops);
	if (ret < 0) {
		printk("ip_fastnat_init: can't register in hook.\n");
		goto cleanup1;
	}	
	return ret;

cleanup1:	
	nf_unregister_hook(&ip_fastnat_fwd_ops);
cleanup:	

return ret;	
	
}
#define MAX_ARGV 8
static inline int get_args (const char *string, char *argvs[])
{
	char *p;
	int n;

	argvs[0]=0;
	n = 0;
//  memset ((void*)argvs, 0, MAX_ARGV * sizeof (char *));
  p = (char *) string;
  while (*p == ' ')
	p++;
  while (*p)
    {
      argvs[n] = p;
      while (*p != ' ' && *p)
		p++;
	if (0==*p)
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

char *
inet_ntoa (void *ipp)
{
  static char buf[16];
  char *ip = (char *) ipp;
  sprintf (buf, "%d.%d.%d.%d",
	   ip[0] & 0xff, ip[1] & 0xff, ip[2] & 0xff, ip[3] & 0xff);
  return buf;
}

int
inet_aton (char *buf, void *p)
{
  int d[6];
  int i;
  char *ip = (char *) p;
  if ((sscanf (buf, "%d.%d.%d.%d", d, d + 1, d + 2, d + 3)) != 4)
    return -1;
  for (i = 0; i < 4; i++)
    ip[i] = d[i] & 0xff;
  return 0;
}

int
ether_aton (char *s, char *mac)
{
  int i;
  char *e;
  for (i=0; i <6; i++)
  {
  	if (s)
	{
		mac[i]=simple_strtoul(s, &e, 16);
		s=(*e) ? e+1 : e;
	}
    else
    	return-1;
  }
  return 0;
}

static char *
ether_ntoa (char *mac)
{
  static char buf[18];
  sprintf (buf, "%02x:%02x:%02x:%02x:%02x:%02x",
	   mac[0] & 0xff, mac[1] & 0xff, mac[2] & 0xff,
	   mac[3] & 0xff, mac[4] & 0xff, mac[5] & 0xff);
  return buf;
}

int vaddr_check(unsigned int addr)
{
    if (addr < 0x8000000)
        return -1;
    return 0;
}

int hextoul (char *str, void *v)
{
    return(sscanf(str,"%x", (unsigned *)v)==1);
}

int hnat_cmd(int argc, char *argv[])
{
	char *val;
	int i;
	int rc;

	if (argc < 1)
	{	
		goto err;
	}
	if (!strcmp(argv[0], "dump"))
	{
		int cmd=0;
		if (argc > 1)
		{
			if  (!strcmp(argv[1], "all"))
				cmd=-1;	
			else
			sscanf(argv[1], "%x", &cmd);
		}
		cta_hnat_dump(cmd, 0);
		goto err;
	}
	if (!strcmp(argv[0], "reset"))
	{
		cta_hnat_reset();
		goto done;
	}
	if (0==(val=strchr(argv[0],'=')))
	{
		printk("%s(%d) no = found\n",__func__,__LINE__);
		goto err;
	}
	*val++=0;
	if (!strcmp(argv[0], "lan_ip"))
	{
		if ((rc=inet_aton(val, &i)))
		{
			printk("%s(%d) incorrect lan_ip:%s\n",__func__,__LINE__,val);
			goto err;
		}
		nf_hnat_lan_ip=i;
		cta_hnat_cfgip(CTA_HNAT_PRIVATE_IF, nf_hnat_lan_ip, 0);
	}
	if (!strcmp(argv[0], "lan_msk"))
	{
		if ((rc=inet_aton(val, &i)))
		{
			printk("%s(%d) incorrect lan_msk:%s\n",__func__,__LINE__,val);
			goto err;
		}
		nf_hnat_lan_msk=i;
	}
	if (!strcmp(argv[0], "wan_ip"))
	{
		if ((rc=inet_aton(val, &i)))
		{
			printk("%s(%d) incorrect wan_ip:%s\n",__func__,__LINE__,val);
			goto err;
		}
		nf_hnat_wan_ip=i;
		cta_hnat_cfgip(CTA_HNAT_PUBLIC_IF, nf_hnat_wan_ip, ((wan_mode==3)? 1: 0));
	}
	if (!strcmp(argv[0], "sdmz_mac"))
	{
		char mac[6];
		if ((rc=ether_aton(val, &mac[0])))
		{
			printk("%s(%d) incorrect sdmz_mac:%s\n",__func__,__LINE__,val);
			goto err;
		}
		memcpy(cta_hnat_sdmz_mac, mac, 6);
	}
	if (!strcmp(argv[0], "sdmz_lan_ip"))
	{
		if ((rc=inet_aton(val, &i)))
		{
			printk("%s(%d) incorrect sdmz_lan_ip:%s\n",__func__,__LINE__,val);
			goto err;
		}
		cta_hnat_sdmz_fakeip=i;
	}
	if (!strcmp(argv[0], "wan_mode"))
	{
		if (!sscanf(val,"%d",&i))
		{
			printk("%s(%d) incorrect mode:%s\n",__func__,__LINE__,val);
			goto err;
		}
		wan_mode=i;
	}
	if (!strcmp(argv[0], "hnat_mode"))
	{
		if (!sscanf(val,"%d",&i))
		{
			printk("%s(%d) incorrect mode:%s\n",__func__,__LINE__,val);
			goto err;
		}
		if (i<=2)
		{
		hnat_mode=i;
		cta_hnat_cfgmode(hnat_mode);
		}
	}
    if(!strcmp(argv[0], "tcp_timo"))
    {
        if (!sscanf(val,"%d",&i))
		{
			printk("%s(%d) incorrect mode:%s\n",__func__,__LINE__,val);
			goto err;
		}
        hnat_tcp_timeout=i;
		printk("hnat_tcp_timeout=%d\n", hnat_tcp_timeout);
    }
    if(!strcmp(argv[0], "udp_timo"))
    {
        if (!sscanf(val,"%d",&i))
		{
			printk("%s(%d) incorrect mode:%s\n",__func__,__LINE__,val);
			goto err;
		}
        hnat_udp_timeout=i;
		printk("hnat_udp_timeout=%d\n", hnat_udp_timeout);
    }
	if (!strcmp(argv[0], "debug"))
	{
		if (!sscanf(val,"%x",&i))
		{
			printk("%s(%d) debug level:%s\n",__func__,__LINE__,val);
			goto err;
		}
		cta_eth_debug_lvl=i;
		printk("debug=%x\n", cta_eth_debug_lvl);
	}
done:
	return 0;

err:
	printk("lan_ip=%s\n", inet_ntoa(&nf_hnat_lan_ip));
	printk("lan_msk=%s\n", inet_ntoa(&nf_hnat_lan_msk));
	printk("wan_ip=%s\n", inet_ntoa(&nf_hnat_wan_ip));
	printk("wan_mode=%d\n", wan_mode);
	printk("hnat_mode=%d\n", hnat_mode);
	printk("sdmz_mac=%s\n", ether_ntoa(&cta_hnat_sdmz_mac[0]));
	printk("sdmz_lan_ip=%s\n", inet_ntoa(&cta_hnat_sdmz_fakeip));
	return 0;
}
static int proc_hnat_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;

	p += sprintf(p, "use dump command instead\n");
#if 0
	pa=(volatile unsigned int *) 0x80000000;
	va= 0xc0000000;

    	for (i=0 ; i<16; i++)
	{
	  	p += sprintf(p, "%08x = %08x\n", va+4*i, *pa++);
	}
#endif
	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int proc_hnat_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[300];
	int	argc ;
	char * argv[MAX_ARGV] ;

	if (count > 0 && count < 299) {
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
		argc = get_args( (const char *)buf , argv );
		hnat_cmd(argc, argv);
	}
	return count;
}


static int __init hnat_init(void)
{
#ifdef	CONFIG_PROC_FS
	struct proc_dir_entry *res;

	res = create_proc_entry("hnat", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->read_proc = proc_hnat_read;
	res->write_proc = proc_hnat_write;
#endif

	hnat_detach_function = hnat_detach;	
#if 1	
	init_timer(&rf_timer);
	rf_timer.function = ip_natexpire;
	rf_timer.expires = jiffies + REFRESH_INTERVAL;
	add_timer(&rf_timer);
#endif	

	return init_or_cleanup(1);
}

static void __exit hnat_exit(void)
{
#ifdef	CONFIG_PROC_FS
	remove_proc_entry("hnat", NULL);
#endif
	del_timer(&rf_timer);
	init_or_cleanup(0);
}

module_init(hnat_init);
module_exit(hnat_exit);
