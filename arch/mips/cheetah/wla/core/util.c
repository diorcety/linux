/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file util.c
*   \brief  wla utility of driver.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <wla_util.h>
#include <wla_debug.h>

/*=============================================================================+
| Functions
+=============================================================================*/
int bhdr_to_idx(MAC_INFO* info, buf_header *bhdr)
{
	return (bhdr - &info->buf_headers[0]);
}

buf_header *idx_to_bhdr(MAC_INFO* info, int index)
{
	return (buf_header *)&info->buf_headers[index];
}

buf_header *bhdr_find_tail(MAC_INFO* info, buf_header *bhdr)
{
	BHDR_LOCK();

	for( ; bhdr; bhdr = (buf_header *)&info->buf_headers[bhdr->next_index])
		if(bhdr->ep)
			break;
	BHDR_UNLOCK();

	return bhdr;
}

struct wbuf *dptr_to_wb(char *dptr)
{
#if defined(WBUF_IS_CACHED)
	return (struct wbuf *)(CACHED_ADDR(dptr) - WLAN_TX_MIN_RESERVED_LEADIND_SPACE);
#else
	return (struct wbuf *)(NONCACHED_ADDR(dptr) - WLAN_TX_MIN_RESERVED_LEADIND_SPACE);
#endif
}

char *wb_to_dptr(struct wbuf *wb)
{
	return (char *)PHYSICAL_ADDR((int)wb + WLAN_TX_MIN_RESERVED_LEADIND_SPACE);
}

void fill_bhdr(buf_header *bhdr, struct wbuf *wb, unsigned short index, unsigned short ep)
{
	unsigned int *bhdr_ptr, data_off;

	data_off = wb->data_off - WLAN_TX_MIN_RESERVED_LEADIND_SPACE; 
	bhdr_ptr = (unsigned int *)bhdr;
	bhdr_ptr[0] = (index << 16)|(ep << 15)|((data_off >> 6) << 13)|wb->pkt_len;
	bhdr_ptr[1] = ((data_off & 0x3f) << 26)|(int)wb_to_dptr(wb);
	wb->data_off = data_off;
}

void fill_bhdr_offset(buf_header *bhdr, unsigned short offset)
{
	offset -= WLAN_TX_MIN_RESERVED_LEADIND_SPACE; 
	bhdr->offset_h = offset >> 6;
	bhdr->offset = offset & 0x3f;
}

unsigned char get_bhdr_offset(buf_header *bhdr)
{
	return (((bhdr->offset_h << 6) | bhdr->offset ) + WLAN_TX_MIN_RESERVED_LEADIND_SPACE); 
}

char *wb_to_llc(struct wbuf *wb)
{
	return (char *)((int)wb + WLAN_RX_LLC_OFFSET + WLAN_TX_MIN_RESERVED_LEADIND_SPACE);
}

buf_header* bhdr_get_first(MAC_INFO* info)
{
    volatile buf_header *bhdr;

    BHDR_LOCK();

    if(info->sw_tx_bhdr_head >= 0) 
    {
        bhdr = idx_to_bhdr(info, info->sw_tx_bhdr_head); 
        if(info->sw_tx_bhdr_head == info->sw_tx_bhdr_tail)
        {
            info->sw_tx_bhdr_head = -1;   /* the freelist is now empty */
            info->sw_tx_bhdr_tail = -1;
        }
        else
        {
            info->sw_tx_bhdr_head = bhdr->next_index;
        }
#if defined(SW_BUF_TRACE)
		/* trace sw buffer usage */
		{
			int idx = bhdr_to_idx(info, (buf_header *)bhdr);
			if(info->sw_buf_trace[idx - info->sw_rx_bhdr_start])
			{
				WLA_DBG(WLAERROR, "%s(), fail to get bhdr!!!, info->sw_buf_headers_head_index=%d\n", __func__, idx);
				do {
					int i;
					WLA_DBG(WLAERROR, "dump sw_buf_trace:\n");	
					for(i=0; i<info->sw_buf_headers_count; i++)
					{
						WLA_DBG(WLAERROR, "[%d]=%d\n", i, info->sw_buf_trace[i]); 	
					}
				}while(0);
				//panic("fail to get bhdr!!!\n");
			}
			info->sw_buf_trace[idx - info->sw_rx_bhdr_start] = 1;
		}
#endif 
		bhdr->ep = 0;
        bhdr->next_index = 0;

        BHDR_UNLOCK();

        return (buf_header*) bhdr;
    }
    else
    {
		BHDR_UNLOCK();
		return NULL;
    }
}

void bhdr_insert_tail(MAC_INFO *info, int head, int tail)
{
    volatile buf_header *bhdr;
	int idx;

    BHDR_LOCK();
    
    if(info->sw_tx_bhdr_tail >= 0) 
    {
        bhdr = idx_to_bhdr(info, info->sw_tx_bhdr_tail);
        bhdr->next_index = head;
		bhdr->ep = 0;
    }
    else
    {
        info->sw_tx_bhdr_head = head;
    }

	for(idx = head;; idx = bhdr->next_index)
	{
		/* check software TX buffer header should in last region */
		if((idx < info->sw_tx_bhdr_start) || 
			(idx >= info->sw_tx_bhdr_end))
		{
			WLA_DBG(WLAERROR, "%s(), free wrong bhdr!!!, head=%d, tail=%d, now=%d\n", __func__, head, tail, idx);
			panic("free wrong bhdr!!!\n");
		}
#if defined(SW_BUF_TRACE)
		/* trace sw buffer usage */
		if(info->sw_buf_trace[idx - info->sw_rx_bhdr_start] == 0)
		{
			WLA_DBG(WLAERROR, "%s(), fail to free bhdr!!!, head=%d, tail=%d, now=%d\n", __func__, head, tail, idx);
			do{
				int i;
				WLA_DBG(WLAERROR, "dump sw_buf_trace:\n");	
				for(i=0; i<info->sw_buf_headers_count; i++)
				{
					WLA_DBG(WLAERROR, "[%d]=%d\n", i, info->sw_buf_trace[i]); 	
				}
	
			}while(0);

			panic("fail to free bhdr!!!\n");
		}
		info->sw_buf_trace[idx - info->sw_rx_bhdr_start] = 0;
#endif
		bhdr = &info->buf_headers[idx];
		bhdr->dptr = 0;

		if((idx == tail) || bhdr->ep)
			break;
		bhdr->ep = 0;
	}

	bhdr->ep = 1;
	info->sw_tx_bhdr_tail = bhdr_to_idx(info, (buf_header *)bhdr);

    BHDR_UNLOCK();
}

struct wbuf *copy_bhdr_to_wb(MAC_INFO* info, unsigned short head, unsigned short tail)
{
	struct wbuf *wb;
	buf_header *bhdr;
	short tlen, offset;
	char *dptr, *dptr1;
	
	if((wb=(struct wbuf *)WBUF_ALLOC(0)) == 0)
	{
		WLA_DBG(WLAERROR, "%s: wbuf_alloc failed\n", __func__);
		return NULL;
	}

	bhdr = idx_to_bhdr(info, head);
	offset = ((bhdr->offset_h << 6) | bhdr->offset );
#if !defined(WBUF_IS_CACHED)
	memcpy((char *)((int)wb + WLAN_TX_MIN_RESERVED_LEADIND_SPACE), (char *)NONCACHED_ADDR(bhdr->dptr), offset);
#else
	memcpy((char *)((int)wb + WLAN_TX_MIN_RESERVED_LEADIND_SPACE), (char *)CACHED_ADDR(bhdr->dptr), offset);
#endif
	tlen = 0;
	dptr1 = (char *)((int)wb + WLAN_TX_MIN_RESERVED_LEADIND_SPACE + offset);

	/* copy multiple buffers to one */
	while(1)
	{
#if !defined(WBUF_IS_CACHED)
		dptr = (char *)(NONCACHED_ADDR(bhdr->dptr) + offset);
#else
		dptr = (char *)(CACHED_ADDR(bhdr->dptr) + offset);
#endif
#ifndef CONFIG_SRAM_WLA
		WLA_DBG(WLADEBUG, "dptr=%p, bhdr->len=%d\n", dptr, bhdr->len);
#endif
		memcpy(dptr1+tlen, dptr, bhdr->len);
		tlen += bhdr->len;

		if(tail == bhdr_to_idx(info, bhdr))
			break;
		bhdr = idx_to_bhdr(info, bhdr->next_index);
		offset = ((bhdr->offset_h << 6) | bhdr->offset );
	}

	wb->data_off = (int)dptr1 - (int)wb;

	return wb;
}

