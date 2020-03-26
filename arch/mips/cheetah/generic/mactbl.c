/*=============================================================================+
|                                                                              |
| Copyright 2013                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mactbl.c
*   \brief
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#ifdef __ECOS
#include <stdlib.h>
#include <string.h>
#include <common.h>

#define DBG(...)				diag_printf(__VA_ARGS__)
#else
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>

#include "common.h"
#define DBG(...)				printk(__VA_ARGS__)
#endif

struct macaddr_pool maddr_tables = { 0 };

/*=============================================================================+
| Functions
+=============================================================================*/
void program_macaddr_table(struct macaddr_pool *mp)
{	
	int i;
	unsigned char en_bitmap, client_bitmap, ap_bitmap, p2p_bitmap, hnat_bitmap;

	en_bitmap = client_bitmap = ap_bitmap = p2p_bitmap = hnat_bitmap = 0;
	for(i=0; i<MACADDR_TABLE_MAX; i++)
	{
		if(mp->m[i].type == 0)
			continue;
		en_bitmap |= (1 << i);
		if(mp->m[i].type & PHYSICAL_MACADDR)
		{
			hnat_bitmap |= (1 << i);
		}
		if(mp->m[i].type & (STA_MACADDR|IBSS_MACADDR))
		{
			client_bitmap |= (1 << i);
		}
		if(mp->m[i].type & AP_MACADDR)
		{
			ap_bitmap |= (1 << i);
		}
		if(mp->m[i].type & P2P_MACADDR)
		{
			p2p_bitmap |= (1 << i);
		}
	
		/* default interface */
		if(i == 0)
		{
			CREG_WRITE32(MAC_BSSID0, (mp->m[i].addr[0] << 8) | mp->m[i].addr[1]);
    		CREG_WRITE32(MAC_BSSID1, (mp->m[i].addr[2] << 24) | ((mp->m[i].addr[3] << 16)&0xff0000) | ((mp->m[i].addr[4] << 8)&0xff00) |  (mp->m[i].addr[5] & 0xff));
		}
		else
		{
			CREG_WRITE32(MAC_BSSID_LOW_1 + (i-1)*4, ((mp->m[i].addr[3] << 16)&0xff0000) | ((mp->m[i].addr[4] <<8)&0xff00) |  (mp->m[i].addr[5]&0xff));
		}
	}

    CREG_UPDATE32(MAC_BSSID0, (en_bitmap << 16), 0x00FF0000UL);
    CREG_WRITE32(MAC_BSSID_FUNC_MAP, (hnat_bitmap << 24) | (ap_bitmap << 16) | (client_bitmap << 8) | (p2p_bitmap));

}

s16 register_macaddr(u8 type, u8 *macaddr)
{
	short i, clone;
	struct macaddr_pool *mp = &maddr_tables;

	clone = 0;

	if(mp->count && (memcmp(mp->m[0].addr, macaddr, 3) != 0))
	{
		DBG("MSB of macaddr is non-match!\n");
		return -1;
	}

	for(i=0; i<MACADDR_TABLE_MAX; i++)
	{
		/* LMAC will find first matched address to check mode. 
		   If we have differnet mode execute on same MAC address, first matched entry 
		   need to support all mode. */
		if(memcmp(mp->m[i].addr, macaddr, 6) == 0)
		{
			if(type == PHYSICAL_MACADDR)
			{
				mp->m[i].type |= type;
				break;
			}
			/* first BSS was reserved for AP/IBSS mode */
			if((i == 0) && (type == STA_MACADDR))
				clone = 1;
			else if(mp->m[i].type & WIF_USED)
				clone = 1;

			mp->m[i].type |= type;
			if(clone == 0)
				mp->m[i].type |= WIF_USED;
			break;
		}
	}

	if(clone || (i >= MACADDR_TABLE_MAX))
	{
		for(i=0; i<MACADDR_TABLE_MAX; i++)
		{
			/* first BSS was reserved for AP/IBSS mode */
			if((i == 0) && (type == STA_MACADDR))
				continue;

			if(mp->m[i].type == 0)
			{
				mp->m[i].type = type|WIF_USED;
				memcpy(mp->m[i].addr, macaddr, 6);
				mp->count++;
				break;
			}
		}
	}

	if(i >= MACADDR_TABLE_MAX)
	{
		DBG("???macaddr table is not enough\n");
		return -1;
	}

	program_macaddr_table(mp);

	return i;
}

void clean_macaddr(char *addr, char type)
{
	short i;
	struct macaddr_pool *mp = &maddr_tables;

	for(i=0; i<MACADDR_TABLE_MAX; i++)
	{
		if(addr)
			if(memcmp(mp->m[i].addr, addr ,6))
				continue;
		if(type != 0)
		{		
			if((mp->m[i].type & type)==0)
				continue;
			mp->m[i].type &= ~type;
			
			if((mp->m[i].type & (AP_MACADDR|STA_MACADDR|P2P_MACADDR|IBSS_MACADDR|WIF_USED)) == WIF_USED)
				mp->m[i].type &= ~WIF_USED;
		}
		else
			mp->m[i].type &= PHYSICAL_MACADDR;

		
		if(mp->m[i].type == 0)
		{
			memset(mp->m[i].addr, 0, 6);
			mp->count--;
		}
	}
	program_macaddr_table(mp);
}

void clean_all_macaddr(void)
{
	short i;
	struct macaddr_pool *mp = &maddr_tables;

	for(i=0; i<MACADDR_TABLE_MAX; i++)
	{
		if(mp->m[i].type & PHYSICAL_MACADDR)
		{
			mp->m[i].type = PHYSICAL_MACADDR;
		}
		else if(mp->m[i].type)
		{
			clean_macaddr(mp->m[i].addr, mp->m[i].type);
		}
	}
}

void lookup_table_done(void)
{
	if(STA_DS_TABLE_CFG_DONE & CREG_READ32(STA_DS_TABLE_CFG))
		return;
	CREG_WRITE32(STA_DS_TABLE_CFG, STA_DS_TABLE_CFG_DONE | STA_DS_TABLE_CACHE_CFG | STA_TABLE_HASH_MASK);
}

#ifndef __ECOS
EXPORT_SYMBOL(maddr_tables);
EXPORT_SYMBOL(program_macaddr_table);
EXPORT_SYMBOL(register_macaddr);
EXPORT_SYMBOL(clean_macaddr);
EXPORT_SYMBOL(clean_all_macaddr);
EXPORT_SYMBOL(lookup_table_done);
#endif
