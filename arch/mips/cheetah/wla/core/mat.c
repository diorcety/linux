/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_mat.c
*   \brief  For MAC translation use.
*   \author Montage
*/

//#ifdef CONFIG_WLA_UNIVERSAL_REPEATER

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <wla_mat.h>
#include <wla_debug.h>

#ifndef HZ
#define HZ				100
#endif
#ifndef IPPROTO_UDP
#define IPPROTO_UDP		17
#endif

struct mat_entry *(mat_hash_tbl[MAT_HASH_TABLE_SIZE]);

/*!-----------------------------------------------------------------------------
 * function: hash_idx_gen()
 *
 *      \brief	generate the mat hash table index 
 *		\param	net_addr: 	the key for hash function
 *              	prot:	the frame protocol type
 *      \return	hash index
 +----------------------------------------------------------------------------*/
int hash_idx_gen(u8 *net_addr, u16 prot)
{
	int idx=0;

	switch(prot)
	{
		case ETHERTYPE_IP:
		case ETHERTYPE_ARP:
			idx = net_addr[0] ^ net_addr[1] ^ net_addr[2] ^ net_addr[3];
			break;
		case ETHERTYPE_PPP_DISC:
		case ETHERTYPE_PPP_SES:
			idx = net_addr[0] ^ net_addr[1] ^ net_addr[2] ^ net_addr[3] ^ net_addr[4] ^ 
				net_addr[5] ^ net_addr[6] ^ net_addr[7];
			break;
	}
	return (idx & (MAT_HASH_TABLE_SIZE - 1));
}

/*!-----------------------------------------------------------------------------
 * function: pppoe_insert_relay_sid()
 *
 *      \brief	add relay session id tag to the pppoe payload
 *		\param 	pppoe:	pppoe header pointer
 *				  mac:	frame source mac address
 *      \return	the pointer of the relay session id tag
 +----------------------------------------------------------------------------*/
struct pppoe_tag* pppoe_insert_relay_sid(struct pppoe_hdr *pppoe, u8 *mac)
{
	struct pppoe_tag *tag = pppoe->tag;
	//u16 sid_pendding = rand() & 0xffff;
	/* pppoe->sid is always 0 in ETHERTYPE_PPP_DISC. use time to random generate sid */
	u16 sid_pendding = ((*(u32*)mac) ^ WLA_CURRENT_TIME) & 0xffff;

	tag =(struct pppoe_tag *) (((int) tag) + pppoe->length); // move pointer to the end of the old tags
	tag->tag_type = PTT_RELAY_SID;
	tag->tag_len = 8; 		// relay sid length = 8 bytes
	pppoe->length += 12; // relay sid tag length = 12

	/* relay sid = {mac, sid_pending} */
	memcpy(tag->tag_data, mac, 6);
	memcpy(tag->tag_data+6, &sid_pendding, 2);
	
	return tag;
}

/*!-----------------------------------------------------------------------------
 * function: pppoe_find_relay_sid()
 *
 *      \brief	find the relay session id in the pppoe payload
 *		\param 	pppoe:	pppoe header pointer
 *      \return	the pointer of the relay session id tag
 +----------------------------------------------------------------------------*/
struct pppoe_tag* pppoe_find_relay_sid(struct pppoe_hdr *pppoe)
{
	int tags_len=pppoe->length, tag_len=0;
	struct pppoe_tag *tag = &pppoe->tag[0];
	
	MAT_MSG("In Function %s\n", __FUNCTION__);

	while(tags_len > 0)
	{
		if(tag->tag_type == PTT_RELAY_SID)
		{
			WLA_DUMP_BUF(tag->tag_data, tag->tag_len);
			break;
		}
		else
		{
			tag_len = (4 + tag->tag_len);
			tags_len -= tag_len;
			tag = (struct pppoe_tag *) ((int)tag + tag_len);
		}
	}

	if(tags_len <= 0)
		tag = NULL;
			
	return tag;
}

/*!-----------------------------------------------------------------------------
 * function: mat_int()
 *
 *      \brief	set the mat_has_tbl to NULL
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/

void mat_init(void)
{
	int i=0;

	for(i=0; i<MAT_HASH_TABLE_SIZE; i++)
	{
		mat_hash_tbl[i] = NULL;
	}
}

/*!-----------------------------------------------------------------------------
 * function: mat_flush()
 *
 *      \brief	flush the mat hash table
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/

void mat_flush(void)
{
	int i=0;
	struct mat_entry *mat_next=NULL, *mat_ptr=NULL;

	for(i = 0; i < MAT_HASH_TABLE_SIZE; i++)
	{
		mat_ptr = mat_hash_tbl[i];
		while(mat_ptr)
		{
			mat_next = mat_ptr->next;
			WLA_MFREE(mat_ptr);
			mat_ptr = mat_next;
		}
		mat_hash_tbl[i] = NULL;
	}
}

/*!-----------------------------------------------------------------------------
 * function: mat_int()
 *
 *      \brief	check the mat hash entry has timeout or not
 *		\param 	
 *      \return	void
 +----------------------------------------------------------------------------*/

void mat_timeout_handle(void)
{
	int i=0;
	struct mat_entry *mat_next=NULL, *mat_ptr=NULL, *mat_prev=NULL;
	time_t old, now = WLA_CURRENT_TIME/HZ;
	

	//if(now > MAT_TIMEOUT)
	{
		old = now - MAT_TIMEOUT;
	
		for(i = 0; i < MAT_HASH_TABLE_SIZE; i++)
		{
			mat_ptr = mat_hash_tbl[i];
			mat_prev = NULL;
			while(mat_ptr)
			{
				mat_next = mat_ptr->next;

				printk("[MAT_DBG] i=%d, ref_time=%d, flags=0x%x, diff=%d\n", i, (int)mat_ptr->ref_time, mat_ptr->flags, (mat_ptr->ref_time <= old));

				if((mat_ptr->ref_time <= old) && (!(mat_ptr->flags & MAT_FLAGS_NO_FLUSH)))
				{
					printk("free mat entry\n");
					if(mat_prev)
						mat_prev->next = mat_next;
					else
						mat_hash_tbl[i] = mat_next;
					WLA_MFREE(mat_ptr);
				}
				else
				{
					mat_prev = mat_ptr;
				}
				
				mat_ptr = mat_next;
			}
		}
	}
}

/*!-----------------------------------------------------------------------------
 * function: mat_lookup()
 *
 *      \brief	search the matching mat entry
 *		\param  	 idx:	hash key (index)
 *				net_addr:	hash data
 *					prot:	frame protocol type
 *      \return	the matched mat entry
 +----------------------------------------------------------------------------*/

struct mat_entry * mat_lookup(int idx, u8 *net_addr, u8 prot)
{
	struct mat_entry *result = mat_hash_tbl[idx];
	
	MAT_MSG("Function: %s, idx = %d\n", __FUNCTION__, idx);

	while(result != NULL)
	{
		if((memcmp(result->net_addr, net_addr, NETADDR_LEN) == 0) && (prot == result->addr_type))
		{
			result->ref_time = WLA_CURRENT_TIME/HZ;
			break;
		}
		else
		{
			result = result->next;
		}
	}

	return result;
}

/*!-----------------------------------------------------------------------------
 * function: mat_insert()
 *
 *      \brief	add mat entry
 *		\param  	 idx:	hash key (index)
 *				net_addr:	hash data
 *				mac_addr:	source mac address
 *					prot:	frame protocol type
 *      \return	void
 +----------------------------------------------------------------------------*/

void mat_insert(int hash_idx, u8 *net_addr, u8 *mac_addr, u8 prot)
{
	struct mat_entry *mat_ent = mat_hash_tbl[hash_idx], *mat_ptr=NULL;

	MAT_MSG("FUNCTION: %s, hash_idx = %d\n", __FUNCTION__, hash_idx);
	
	if((mat_ent != NULL) && (mat_ptr =  mat_lookup(hash_idx, net_addr, prot)))
	{
		mat_ptr->ref_time = WLA_CURRENT_TIME/HZ;
	}
	else
	{
		mat_ptr = WLA_MALLOC(sizeof(struct mat_entry), DMA_MALLOC_BZERO);
		mat_ptr->ref_time = WLA_CURRENT_TIME/HZ;
		mat_ptr->addr_type = prot;
		memcpy(mat_ptr->net_addr, net_addr, NETADDR_LEN);
		memcpy(mat_ptr->mac_addr, mac_addr, ETHER_ADDR_LEN);
		mat_ptr->next = mat_hash_tbl[hash_idx];
		mat_hash_tbl[hash_idx] = mat_ptr; 
	}
}

/*!-----------------------------------------------------------------------------
 * function: mat_insert_by_dhcp()
 *
 *      \brief	add mat entry
 *		\param  	 idx:	hash key (index)
 *				net_addr:	hash data
 *				mac_addr:	source mac address
 *					prot:	frame protocol type
 *      \return	void
 +----------------------------------------------------------------------------*/
#ifdef CONFIG_MAT_NO_FLUSH_ENTRY_FROM_DHCP
extern struct macaddr_pool maddr_tables;
void mat_insert_by_dhcp(ehdr *ef)
{
	struct ip *ip_header = (struct ip *)ef->data;
	struct udphdr *udp;
	struct dhcpMessage *dhcp;
	struct mat_entry *mat_ent, *mat_ptr=NULL;
	int idx;
	int i;
	u8 addr[NETADDR_LEN];

	if(ip_header->ip_p == IPPROTO_UDP)
	{
		udp = (struct udphdr *)((int)&ef->data + ip_header->ip_hl*4);

		if(udp->uh_sport == 67) //67: bootps
		{
			dhcp = (struct dhcpMessage *) ((int)udp + sizeof(struct udphdr));

			for(i=0; i<MACADDR_TABLE_MAX; i++)
			{
				if(maddr_tables.m[i].type == 0)
					continue;

				/* don't add DUT's address entry to MAT tables */
				if(memcmp(maddr_tables.m[i].addr, dhcp->chaddr, 6) == 0)
					return;
			}
			
			memset(addr, 0, NETADDR_LEN);
			memcpy(addr, (u8 *)&dhcp->yiaddr, 4);
			idx = hash_idx_gen(addr, ef->type);
			mat_ent = mat_hash_tbl[idx];
	
			if((mat_ent != NULL) && (mat_ptr = mat_lookup(idx, addr, MAT_TYPE_ARP_IP)))
			{
				mat_ptr->ref_time = WLA_CURRENT_TIME/HZ;
				mat_ptr->flags |= MAT_FLAGS_NO_FLUSH;
			}
			else
			{
				mat_ptr = WLA_MALLOC(sizeof(struct mat_entry), DMA_MALLOC_BZERO);
				if(mat_ptr)
				{
					mat_ptr->ref_time = WLA_CURRENT_TIME/HZ;
					mat_ptr->addr_type = MAT_TYPE_ARP_IP;
					memcpy(mat_ptr->net_addr, addr, NETADDR_LEN);
					memcpy(mat_ptr->mac_addr, dhcp->chaddr, ETHER_ADDR_LEN);
					mat_ptr->next = mat_hash_tbl[idx];
					mat_hash_tbl[idx] = mat_ptr; 
					mat_ptr->flags |= MAT_FLAGS_NO_FLUSH;
				}
			}
		}
	}
}
#endif // CONFIG_MAT_NO_FLUSH_ENTRY_FROM_DHCP

/*!-----------------------------------------------------------------------------
 * function: mat_handle_lookup()
 *
 *      \brief	handle ingress flow
 *		\param  	wb: wbuf	
 *					sf:	frame header pointer
 *				ap_mac:	repeater's mac address
 *      \return	lookup result (success/fail)
 +----------------------------------------------------------------------------*/

int mat_handle_lookup(struct wbuf *wb, ehdr *ef)
{
	int idx=0;
	int ret=1;
	u8 addr[NETADDR_LEN];
	struct mat_entry *mat_ptr=NULL;
	
	MAT_MSG("MAT_MSG: fucntion %s\n", __FUNCTION__);
	MAT_MSG("ef->da = %02x:%02x:%02x:%02x:%02x:%02x\n", ef->da[0], ef->da[1], ef->da[2], ef->da[3], ef->da[4], ef->da[5]);
	MAT_MSG("ef->sa = %02x:%02x:%02x:%02x:%02x:%02x\n", ef->sa[0], ef->sa[1], ef->sa[2], ef->sa[3], ef->sa[4], ef->sa[5]);
	MAT_MSG("ef->type = %04x\n", ef->type);
	
	memset(addr, 0, NETADDR_LEN);
	
	switch(ef->type)
	{
		case ETHERTYPE_IP:
		{
			struct ip *ip_header = (struct ip *)ef->data;
			memcpy(addr, &ip_header->ip_dst, 4);
			idx = hash_idx_gen(addr, ef->type);
			if((mat_ptr = mat_lookup(idx, addr, MAT_TYPE_ARP_IP)))
				ret=0;
			break;
		}
		case ETHERTYPE_ARP:
		{
			struct	ether_arp *ea = (struct ether_arp *)ef->data;
			memcpy(addr, ea->arp_tpa, 4);
			idx = hash_idx_gen(addr, ef->type);
			if((mat_ptr = mat_lookup(idx, addr, MAT_TYPE_ARP_IP)))
			{
				memcpy(ea->arp_tha, mat_ptr->mac_addr, ETHER_ADDR_LEN);
				ret=0;
			}
			break;
		}
		case ETHERTYPE_PPP_DISC:
		{
			struct pppoe_hdr *pppoe =(struct pppoe_hdr *)ef->data; 
			struct pppoe_tag *tag=NULL;
			if((tag = pppoe_find_relay_sid(pppoe)) != NULL)
			{
				memcpy(addr, tag->tag_data, 8);
				idx = hash_idx_gen(addr, ef->type);
				if((mat_ptr = mat_lookup(idx, addr, MAT_TYPE_PPP)))
					ret=0;
			}
			break;
		}
		case ETHERTYPE_PPP_SES:
		{
			struct pppoe_hdr *pppoe =(struct pppoe_hdr *)ef->data; 
			memcpy(addr, &pppoe->sid, 2);
			//memcpy(addr+2, ef->sa, ETHER_ADDR_LEN);
			idx = hash_idx_gen(addr, ef->type);
			if((mat_ptr = mat_lookup(idx, addr, MAT_TYPE_PPP)))
				ret=0;
			break;
		}
		case ETHERTYPE_IPV6:
		{
			break;
		}
		default:
		{
			MAT_MSG("MAT_MSG: Not support type (0x%x) in mat()\n", ef->type);
			break;
		}
	}

	if(ret == 0)
		memcpy(ef->da, mat_ptr->mac_addr, ETHER_ADDR_LEN);
	
	return ret;	
}

/*!-----------------------------------------------------------------------------
 * function: mat_handle_insert()
 *
 *      \brief	handle egress flow
 *		\param  	wb: wbuf	
 *					ef:	frame header pointer
 *				ap_mac:	repeater's mac address
 *      \return	insert result (success/fail)
 +----------------------------------------------------------------------------*/

/* reference from napt.h */
#define FIX_CHECKSUM(diff, cksum) { \
    diff += (cksum); \
    if (diff < 0) \
    { \
        diff = -diff; \
        diff = (diff >> 16) + (diff & 0xffff); \
        diff += diff >> 16; \
        (cksum) = (u_short) ~diff; \
    } \
    else \
    { \
        diff = (diff >> 16) + (diff & 0xffff); \
        diff += diff >> 16; \
        (cksum) = (u_short) diff; \
    } \
}

int mat_handle_insert(struct wbuf *wb, ehdr *ef, u8 *ap_mac)
{
	int idx=0, ret=1;
	u8 addr[NETADDR_LEN];
	
	MAT_MSG("MAT_MSG: fucntion %s\n", __FUNCTION__);
	MAT_MSG("ap_mac =  %02x:%02x:%02x:%02x:%02x:%02x\n", ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);
	MAT_MSG("ef->da = %02x:%02x:%02x:%02x:%02x:%02x\n", ef->da[0], ef->da[1], ef->da[2], ef->da[3], ef->da[4], ef->da[5]);
	MAT_MSG("ef->sa = %02x:%02x:%02x:%02x:%02x:%02x\n", ef->sa[0], ef->sa[1], ef->sa[2], ef->sa[3], ef->sa[4], ef->sa[5]);
	MAT_MSG("ef->type = %04x\n", ef->type);
	
	memset(addr, 0, NETADDR_LEN);
	
	switch(ef->type)
	{
		case ETHERTYPE_IP:
		{
			struct ip *ip_header = (struct ip *)ef->data;
			if(ip_header->ip_src)
			{
				memcpy(addr, &ip_header->ip_src, 4);
				idx = hash_idx_gen(addr, ef->type);
				mat_insert(idx, addr,ef->sa, MAT_TYPE_ARP_IP);
				ret = 0;
			}
			else
			{
				/* handle dhcp frame (udp) */
				if(ip_header->ip_p == IPPROTO_UDP)
				{
					struct udphdr *udp = (struct udphdr *)((int)&ef->data + ip_header->ip_hl*4);
					if(udp->uh_dport == 67) //67: bootps
					{
						struct dhcpMessage * dhcp = (struct dhcpMessage *) ((int)udp + sizeof(struct udphdr));
						if(!(dhcp->flags & 0x8000))
						{
							int diff = -(0x8000);
							dhcp->flags |= 0x8000;  // set flags as broadcast
							FIX_CHECKSUM(diff, udp->uh_sum);
						}
						ret = 0;
					}
				}
			}
			break;
		}
		case ETHERTYPE_ARP:
		{
			struct	ether_arp *ea = (struct ether_arp *)ef->data;
			memcpy(addr, ea->arp_spa, 4);
			idx = hash_idx_gen(addr, ef->type);
			mat_insert(idx, addr, ea->arp_sha, MAT_TYPE_ARP_IP);
			memcpy(ea->arp_sha, ap_mac, ETHER_ADDR_LEN);
			ret = 0;
			break;
		}
		case ETHERTYPE_PPP_DISC:
		{
			struct pppoe_hdr *pppoe =(struct pppoe_hdr *)ef->data; 
			
			/* bypass PADS & PADT frame to create mat entry with the session id  */
			if((pppoe->code != PADS_CODE) && (pppoe->code != PADT_CODE))
			{
				struct pppoe_tag *tag=NULL;
				int len=0;
				if((tag = pppoe_find_relay_sid(pppoe)) == NULL)
				{
					tag = pppoe_insert_relay_sid(pppoe, ef->sa);
					wb->pkt_len += 12; /* add the relay sid tag len to the wb pkt_len */
				}
				if(tag->tag_len > 8)
					len = 8;
				else
					len = tag->tag_len;
				memcpy(addr, tag->tag_data, len);
				idx = hash_idx_gen(addr, ef->type);
				mat_insert(idx, addr, ef->sa, MAT_TYPE_PPP);
				ret = 0;
				break;
			}
		}
		case ETHERTYPE_PPP_SES:
		{
			struct pppoe_hdr *pppoe2 =(struct pppoe_hdr *)ef->data; 
			memcpy(addr, &pppoe2->sid, 2);
			//memcpy(addr+2, ef->da, ETHER_ADDR_LEN);
			idx = hash_idx_gen(addr, ef->type);
			mat_insert(idx, addr, ef->sa, MAT_TYPE_PPP);
			ret = 0;
			break;
		}
		case ETHERTYPE_IPV6:
		{
			break;
		}
		default:
		{
			MAT_MSG("MAT_MSG: Not support type (0x%x) in mat()\n", ef->type);
			break;
		}
	}
	
	if(ret == 0)
		memcpy(ef->sa, ap_mac, ETHER_ADDR_LEN);

	return ret;
}

#if MAT_MSG_EN

#include <cli_api.h>

int mat_cmd(int argc, char* argv[])
{
	int i=0;
	struct mat_entry *mat_ent;
	
	if(argc == 2)
	{
		switch(atoi(argv[1]))
		{
			case 0:
			{
				for(i = 0; i < MAT_HASH_TABLE_SIZE; i++)
				{
					mat_ent = mat_hash_tbl[i];
					printd("***************** MAT TABLE: hash_idx = %d ***************\n", i);
					while(mat_ent)
					{
						printd("addr_type = %x\n", mat_ent->addr_type);	
						printd("net_addr = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", 
								mat_ent->net_addr[0], mat_ent->net_addr[1], 
								mat_ent->net_addr[2], mat_ent->net_addr[3],
								mat_ent->net_addr[4], mat_ent->net_addr[5], 
								mat_ent->net_addr[6], mat_ent->net_addr[7]);
						printd("mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\n", 
								mat_ent->mac_addr[0], mat_ent->mac_addr[1], 
								mat_ent->mac_addr[2], mat_ent->mac_addr[3],
								mat_ent->mac_addr[4], mat_ent->mac_addr[5]);	
						printd("ref_time = %d\n", mat_ent->ref_time);
						printd("flags = 0x%x\n", mat_ent->flags);
						mat_ent = mat_ent->next;
					}
				}
				break;
			}
			case 1:
				mat_flush();
				break;
		}
	}
	else
	{
		MAT_MSG("mat <sel>\n");
		MAT_MSG("sel = 0: dump mat_table\n");
		MAT_MSG("sel = 1: flush mat_table\n");
	}
	return CLI_OK;
}

CLI_CMD(mat, mat_cmd, "mat", 0);
#endif // MAT_MSG_EN

//#endif // CONFIG_UNIVERSAL_REPEATER

