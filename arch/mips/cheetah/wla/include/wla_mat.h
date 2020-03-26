/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_mat.h
*   \brief  For MAC translation use.
*   \author Montage
*/

#ifndef __WLA_MAT_H__
#define __WLA_MAT_H__

#include <wbuf.h>
#include <mac_common.h>

#define MAT_INSERT 0
#define MAT_LOOKUP 1

#define NETADDR_LEN 8
//#define NETADDR_LEN 12	/* for IPX */
//#define NETADDR_LEN 16  	/* for IPv6 */
#define MAT_TIMEOUT 300
#define MAT_CHECK_TIME 10000
#define MAT_HASH_TABLE_SIZE 32

//#define MAT_TYPE_IP		0
#define MAT_TYPE_ARP_IP	1
#define MAT_TYPE_PPP	2

/* Define the flags in the MAT entry  */
#define MAT_FLAGS_PPPOE_REMOVE 	0x1		/* the original frame don't contain the Host-Uniq tag */
#define MAT_FLAGS_NO_TIMEOUT 	0x2		/* the MAt entry can't be timeout */


#define MAT_MSG_EN 0
#if MAT_MSG_EN
#define MAT_MSG WLA_MSG
#else
#define MAT_MSG(...)
#endif

void mat_init(void);
void mat_flush(void);
int mat_handle_lookup(struct wbuf *wb, ehdr *ef);
int mat_handle_insert(struct wbuf *wb, ehdr *ef, u8 *ap_mac);
void mat_timeout_handle(void);
#ifdef CONFIG_MAT_NO_FLUSH_ENTRY_FROM_DHCP
void mat_insert_by_dhcp(ehdr *ef);
#endif


//FIXME:
#define ETHER_ADDR_LEN 6
#define PADS_CODE   0x65
#define PADT_CODE   0xa7

#define ETHERTYPE_PUP       0x0200  /* PUP protocol */
#define ETHERTYPE_IP        0x0800  /* IP protocol */
#define ETHERTYPE_ARP       0x0806  /* address resolution protocol */
#define ETHERTYPE_REVARP    0x8035  /* reverse addr resolution protocol */
#define ETHERTYPE_IPV6      0x86DD  /* IPv6 protocol */
#define ETHERTYPE_PPP_DISC  0x8863  /* PPPoE discovery message */
#define ETHERTYPE_PPP_SES   0x8864  /* PPPoE session message */
#define PTT_RELAY_SID		0x0110
#if defined(CONFIG_WLA_WAPI) || defined(CONFIG_WAPI)
#define ETHERTYPE_WAPI		0x88B4	/* WAPI WAI */
#endif
#define ETHERTYPE_EAPOL		0x888E	/* Port Access Entity (IEEE 802.1X) */

#define MAT_FLAGS_NO_FLUSH	BIT(0)

struct mat_entry {
	struct mat_entry *next;

	time_t ref_time;
	u8 addr_type;
	u8 net_addr[NETADDR_LEN];
	u8 mac_addr[ETHER_ADDR_LEN];
	u8 flags;
};

struct pppoe_tag {
    unsigned short tag_type;
    unsigned short tag_len;
    char tag_data[0];
}__attribute ((packed));

struct pppoe_hdr{
    unsigned char ver:4;
    unsigned char type:4;
    unsigned char code;
    unsigned short sid;
    unsigned short length;
    struct pppoe_tag tag[0];
}__attribute__ ((packed));


struct	arphdr {
	unsigned short ar_hrd;	/* format of hardware address */
#define ARPHRD_ETHER 	1	/* ethernet hardware format */
#define ARPHRD_IEEE802 	6	/* IEEE 802 hardware format */
#define ARPHRD_FRELAY 	15	/* frame relay hardware format */
	unsigned short ar_pro;	/* format of protocol address */
	unsigned char  ar_hln;	/* length of hardware address */
	unsigned char  ar_pln;	/* length of protocol address */
	unsigned short ar_op;	/* one of: */
#define	ARPOP_REQUEST	1	/* request to resolve address */
#define	ARPOP_REPLY	2	/* response to previous request */
#define	ARPOP_REVREQUEST 3	/* request protocol address given hardware */
#define	ARPOP_REVREPLY	4	/* response giving protocol address */
#define	ARPOP_INVREQUEST 8 	/* request to identify peer */
#define	ARPOP_INVREPLY	9	/* response identifying peer */
};

struct  ether_arp {
    struct   arphdr ea_hdr;         /* fixed-size header */
    unsigned char arp_sha[ETHER_ADDR_LEN];   /* sender hardware address */
    unsigned char arp_spa[4];            /* sender protocol address */
    unsigned char arp_tha[ETHER_ADDR_LEN];   /* target hardware address */
    unsigned char arp_tpa[4];            /* target protocol address */
};

struct udphdr {
    unsigned short uh_sport;     /* source port */
    unsigned short uh_dport;     /* destination port */
    unsigned short uh_ulen;      /* udp length */
    unsigned short uh_sum;       /* udp checksum */
};
#define OPTIONS_TOTAL_LEN   308
struct dhcpMessage {
    unsigned char op;
    unsigned char htype;
    unsigned char hlen;
    unsigned char hops;
    unsigned int xid;
    unsigned short secs;
    unsigned short flags;
    unsigned int ciaddr;
    unsigned int yiaddr;
    unsigned int siaddr;
    unsigned int giaddr;
    unsigned char chaddr[16];
    unsigned char sname[64];
    unsigned char file[128];
    unsigned int cookie;
    unsigned char options[OPTIONS_TOTAL_LEN]; /* 312 - cookie */
};
struct ip {
    unsigned char  ip_v:4,       /* version */
          ip_hl:4;      /* header length */
    unsigned char  ip_tos;       /* type of service */
    unsigned short ip_len;       /* total length */
    unsigned short ip_id;        /* identification */
    unsigned short ip_off;       /* fragment offset field */
#define IP_RF 0x8000            /* reserved fragment flag */
#define IP_DF 0x4000            /* dont fragment flag */
#define IP_MF 0x2000            /* more fragments flag */
#define IP_OFFMASK 0x1fff       /* mask for fragmenting bits */
     unsigned char  ip_ttl;       /* time to live */
     unsigned char  ip_p;         /* protocol */
     unsigned short ip_sum;       /* checksum */
     unsigned int ip_src, ip_dst; /* source and dest address */
};


#endif   // __WLA_MAT_H__

