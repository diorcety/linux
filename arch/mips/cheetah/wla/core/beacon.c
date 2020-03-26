/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_beacon.c
*   \brief  wla beacon functions.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
//#include <wlan_def.h>
#include <wla_debug.h>
#include <wla_util.h>

char wla_add_beacon(MAC_INFO *info, struct wbuf *wb1,  struct wbuf *wb2,  struct wbuf *wb3) 
{
	struct beacon_desc *b_desc, *last, *old_desc, *prev;
	buf_header *bhdr, *bhdr1=0, *bhdr2=0, *bhdr3=0;
	u16 idx, next_idx=0, is_ep=1, pkt_len=0;
	struct wbuf *old_wb;

	if(!(b_desc = (struct beacon_desc *)WLA_MALLOC(sizeof(struct beacon_desc), DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN)))
	{
		return 0;
	}

	/* third block: country IE ... */
	if(wb3 != NULL)
	{
		DCACHE_STORE((unsigned int)wb3, wb3->data_off + wb3->pkt_len);
		if((bhdr3 = bhdr_get_first(info)) == NULL)
			goto err;
		fill_bhdr(bhdr3, wb3, 0, 1);
		next_idx = bhdr_to_idx(info, bhdr3);
		b_desc->desc.tx_descr.bhdr_tail_index = next_idx;
		is_ep = 0;	/* assume only wb1 or existing wb1,wb2,wb3 */
		pkt_len = wb3->pkt_len;
	}

	/* second block: TIM IE */
	if(wb2 != NULL)
	{
		DCACHE_STORE((unsigned int)wb2, wb2->data_off + wb2->pkt_len);
		if((bhdr2 = bhdr_get_first(info)) == NULL)
			goto err;
		fill_bhdr(bhdr2, wb2, next_idx, 0);
		next_idx = bhdr_to_idx(info, bhdr2);
		pkt_len += wb2->pkt_len;
	}
 
	/* first block: WLAN header ... DS IE */
	/* flush cache and setting macframe header */
	DCACHE_STORE((unsigned int)wb1, wb1->data_off + wb1->pkt_len);
	if((bhdr1 = bhdr_get_first(info)) == NULL)
		goto err;
	fill_bhdr(bhdr1, wb1, next_idx, is_ep);
	next_idx = bhdr_to_idx(info, bhdr1);
	pkt_len += wb1->pkt_len;

	if(is_ep)
		b_desc->desc.tx_descr.bhdr_tail_index = next_idx;
	
	/* setting BEACON TX descr */
	b_desc->desc.tx_descr.bssid = wb1->bss_desc;		/* the bssid of skbs are input in-order */
	//b_desc->tx_descr.ifs = MAC_IFS_DIFS;
	b_desc->desc.tx_descr.ack_policy = MAC_ACK_POLICY_NOACK;
	b_desc->desc.tx_descr.dis_duration = 0;  /* enable duration */
	b_desc->desc.tx_descr.ins_ts = 1;
	b_desc->desc.tx_descr.bc = 1;            /* groupcast */

	b_desc->desc.tx_descr.pkt_length = pkt_len;
	b_desc->total_len = b_desc->desc.tx_descr.pkt_length; /* remember original length without unicast'TIM */
	b_desc->desc.tx_descr.bhdr_head_index = next_idx;
	b_desc->desc.next_pointer = 0;

	b_desc->desc.tx_descr.u.beacon.format = get_hw_bitrate_format(BEACON_BITRATE);
	b_desc->desc.tx_descr.u.beacon.ch_offset = primary_ch_offset(info->bandwidth_type);
	
	b_desc->desc.tx_descr.eor = 1;

	wla_lock();

	for(old_desc = (struct beacon_desc *)info->beacon_update_list, prev=0; 
			PHYSICAL_ADDR(old_desc); prev = old_desc, 
			old_desc = (struct beacon_desc *) VIRTUAL_ADDR(old_desc->desc.next_pointer))
	{
		bhdr = idx_to_bhdr(info, old_desc->desc.tx_descr.bhdr_head_index);
		old_wb = dptr_to_wb((char*)(unsigned int)bhdr->dptr);

		if(old_wb->bss_desc == wb1->bss_desc)
		{
			if(prev)
			{
				prev->desc.next_pointer = old_desc->desc.next_pointer;
				prev->desc.tx_descr.eor = old_desc->desc.tx_descr.eor;
			}
			else
			{
				info->beacon_update_list = 
						((ssq_tx_descr *)(unsigned int)old_desc->desc.next_pointer ? 
						(ssq_tx_descr *) VIRTUAL_ADDR(old_desc->desc.next_pointer): 0);
			}

			while(bhdr)
			{
				WBUF_FREE( dptr_to_wb((char*)(unsigned int)bhdr->dptr));
				if(bhdr->ep)
					break;
				bhdr = idx_to_bhdr(info, bhdr->next_index);
			}

			bhdr_insert_tail(info, old_desc->desc.tx_descr.bhdr_head_index, 
					old_desc->desc.tx_descr.bhdr_tail_index);

			WLA_MFREE((void *)old_desc);

			break;
		}
	}

	/* find last descriptor of list */
	for(last = (struct beacon_desc *)info->beacon_update_list; 
				PHYSICAL_ADDR(last) && last->desc.next_pointer; 
				last = (struct beacon_desc *)VIRTUAL_ADDR(last->desc.next_pointer));

	if(last)
	{
		last->desc.tx_descr.eor = 0;
		last->desc.next_pointer = PHYSICAL_ADDR(b_desc);
	}
	else
		info->beacon_update_list = (ssq_tx_descr *)b_desc;

	wla_unlock();
	if(!(MACREG_READ32(BEACON_CONTROL) & BEACON_CONTROL_ENABLE))
	{
		wla_beacon_start(info);

		/* XXX: We should wait beacon isr to kick HW, otherwise beacon queue may die */
		/* write 1 clear to IDLE bit, set HW to busy , trigger the HW */
		//MACREG_WRITE32(BEACON_CONTROL, MACREG_READ32(BEACON_CONTROL));
	}

	return 1;

err:
	if(bhdr1)
	{
		idx = bhdr_to_idx(info, bhdr1);
		bhdr_insert_tail(info, idx, idx);
	}
	
	if(bhdr2)
	{
		idx = bhdr_to_idx(info, bhdr2);
		bhdr_insert_tail(info, idx, idx);
	}

	if(bhdr3)
	{
		idx = bhdr_to_idx(info, bhdr3);
		bhdr_insert_tail(info, idx, idx);
	}

	if(wb1)
		WBUF_FREE(wb1);
	if(wb2)
		WBUF_FREE(wb2);
	if(wb3)
		WBUF_FREE(wb3);

	WLA_MFREE((void *)b_desc);
	return 0;
}

#if defined(CONFIG_WMAC_RECOVER)
void save_beacon_list(MAC_INFO* info, struct beacon_desc *b_desc)
{
	buf_header* bhdr;
	unsigned int bhdr_idx, desc_idx;
	struct wbuf *wb;

	desc_idx = 0;
	while(PHYSICAL_ADDR(b_desc))
	{
		bhdr = idx_to_bhdr(info, b_desc->desc.tx_descr.bhdr_head_index);
		bhdr_idx = 0;

		#if 0
		printd("%s: desc(%x)\n", __func__, b_desc);
		#endif
		while(bhdr)
		{
			wb = (struct wbuf *)VIRTUAL_ADDR(dptr_to_wb((char*)(u32)bhdr->dptr));
			wb->data_off += WLAN_TX_MIN_RESERVED_LEADIND_SPACE;
			info->beacon_wb_addr[desc_idx][bhdr_idx] = wb;
			bhdr->dptr = 0;
			#if 0
			printd("%s: beacon_wb_addr[%d][%d]=%x wb(%x) flags=(%x) (%d) (%d)\n",
											__func__, desc_idx, bhdr_idx, 
											info->beacon_wb_addr[desc_idx][bhdr_idx], 
											wb, wb->flags, wb->data_off, wb->pkt_len);
			#endif
			if(bhdr->ep)
				break;
			bhdr = idx_to_bhdr(info, bhdr->next_index);
			bhdr_idx++;
		}
		b_desc = (struct beacon_desc *) VIRTUAL_ADDR(b_desc->desc.next_pointer);
		desc_idx++;
	}
}

void update_beacon_list(MAC_INFO* info)
{
	unsigned int desc_idx;
	struct wbuf *wb1, *wb2, *wb3;
	unsigned int *wb_ptr;

	for(desc_idx = 0; desc_idx < MAC_MAX_BSSIDS; desc_idx++)
	{
		wb_ptr = (unsigned int *)&info->beacon_wb_addr[desc_idx];
		wb1 = (struct wbuf *)wb_ptr[0];
		wb2 = (struct wbuf *)wb_ptr[1];
		wb3 = (struct wbuf *)wb_ptr[2];
		if(!wb1)
			continue;
		#if 0
		printd("%s: desc_idx(%d) wb1(%x) wb2(%x) wb3(%x)\n", __func__, desc_idx, wb1, wb2, wb3);
		#endif
		wla_add_beacon(info, wb1, wb2, wb3);
	}
	/* Reset wb address */
	memset(&info->beacon_wb_addr[0], 0, sizeof(info->beacon_wb_addr));
}
#endif

void clean_beacon_list(MAC_INFO* info, struct beacon_desc *b_desc)
{
	struct beacon_desc *prev;
	buf_header* bhdr;

	while(PHYSICAL_ADDR(b_desc))
	{
		bhdr = idx_to_bhdr(info, b_desc->desc.tx_descr.bhdr_head_index);
		
		while(bhdr)
		{
			if(bhdr->dptr)
				WBUF_FREE(dptr_to_wb((char*)(unsigned int)bhdr->dptr));
			if(bhdr->ep)
				break;
			bhdr = idx_to_bhdr(info, bhdr->next_index);
		}
		bhdr_insert_tail(info, b_desc->desc.tx_descr.bhdr_head_index, b_desc->desc.tx_descr.bhdr_tail_index);
		prev = b_desc;
		b_desc = (struct beacon_desc *) VIRTUAL_ADDR(b_desc->desc.next_pointer);
		WLA_MFREE((void *)CACHED_ADDR(prev));
	}
}

void wla_beacon_flush(MAC_INFO* info)
{
	clean_beacon_list(info, (struct beacon_desc *)info->beacon_descriptors_list);
	info->beacon_descriptors_list = 0;
	clean_beacon_list(info, (struct beacon_desc *)info->beacon_update_list);
	info->beacon_update_list = 0;
}

inline void wla_beacon_setup(unsigned short beacon_interval, unsigned char dtim_period)
{
#define LMAC_DEFAULT_SLOT_TIME          1024
#define LMAC_DEFAULT_BEACON_TX_TIMEOUT  (30 *1000) /* the unit is us */
#define PRE_HW_TBTT_DEFAULT_TIME				0x10

	MACREG_WRITE32(TS0_DTIM, dtim_period);

    /*	
        ---|----------------|-----------------|-------------------|------------------|--- 
       HW pre-TBTT    SW pre-TBTT           TBTT               timeout          HW pre-TBTT   
                                                                                    
		Under AdHoc mode, beacon listen and sync window is from HW pre-TBTT to timeout. 
		To speed timestamp synchronization of beacon, we shall enlarge listen 
		window as possiable as we can.
	*/
	/*the pre-tbtt setting is 16bits width, beware of overflow problem */
	MACREG_WRITE32(GLOBAL_PRE_TBTT, PRE_TBTT_TIME(beacon_interval)|((((beacon_interval)/2)&0xffff) << 16));
	/* JH modify beacon sync window to entire TBTT. So we do not extend duration of timeout */
	//MACREG_WRITE32(TS0_BE, (beacon_interval << 16)|((((beacon_interval*1000)/2)&0xffff) - 1));
	MACREG_WRITE32(TS0_BE, (beacon_interval << 16)|((((beacon_interval)/3)&0xffff) - 1));

}

void wla_beacon_start(MAC_INFO* info)
{
    /* enable LMAC beacon timer */
	/* set BEACON_CONTROL_TX_IDLE is 0 to prevent beacon engine start to transmit beacon */
    MACREG_UPDATE32(LMAC_CNTL, LMAC_CNTL_BEACON_ENABLE, LMAC_CNTL_BEACON_ENABLE);
	MACREG_UPDATE32(BEACON_CONTROL, (get_hw_bitrate_code(BEACON_BITRATE) << 8)|BEACON_CONTROL_ENABLE,
					BEACON_CONTROL_RATE|BEACON_CONTROL_ENABLE|BEACON_CONTROL_TX_IDLE);

	/* enable TS timer */
	MACREG_WRITE32(TS0_CTRL, TS_ENABLE);
}

inline void wla_beacon_stop(MAC_INFO* info)
{
	MACREG_UPDATE32(TS_INT_MASK, 0x1, 0x1);
#ifdef TBTT_EXCEPTION_HANDLE
	/* FIXME: may not disable all bss interrupt? */
	//MACREG_WRITE32(TS_AP_INT_MASK, 0xFFFF0000);
	MACREG_WRITE32(TS_AP_INT_MASK, 0xFFFFFFFF);
#endif  // TBTT_EXCEPTION_HANDLE
	MACREG_WRITE32(TS0_CTRL, 0);

	info->int_mask |= MAC_INT_PRETBTT;
	MACREG_UPDATE32(MAC_INT_MASK_REG, MAC_INT_PRETBTT, MAC_INT_PRETBTT);
	MACREG_WRITE32(GLOBAL_PRE_TBTT, PRE_TBTT_INTERVAL_S|(PRE_HW_TBTT_DEFAULT_TIME << 16));
	MACREG_UPDATE32(BEACON_CONTROL, 0, BEACON_CONTROL_ENABLE|BEACON_CONTROL_TX_IDLE);
	MACREG_UPDATE32(LMAC_CNTL, 0x0, LMAC_CNTL_BEACON_ENABLE);
}

#if (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
extern struct wbuf *ibss_u_wb;
extern struct wbuf *ibss_b_wb;
extern u32 ibss_sta_idx;

u32 wla_add_atim(MAC_INFO *info, struct wbuf *wb1, u8 sta_idx, u8 bc)
{
	static struct beacon_desc *unicast_desc=NULL, *bc_desc=NULL;
	struct beacon_desc *b_desc=NULL;
	struct beacon_desc *last;
	struct wbuf *wb=NULL;
	buf_header *bhdr=NULL;
	u16 idx;

	if(bc)
		b_desc = bc_desc;
	else
		b_desc = unicast_desc;
	
	if(b_desc)
		goto attach;

	wb = wbuf_alloc(wb1->pkt_len + wb1->data_off);
	if(wb == NULL)
		return 0;
	else
		memcpy(wb, wb1, wb1->pkt_len + wb1->data_off);

	if(!(b_desc = (struct beacon_desc *)WLA_MALLOC(sizeof(struct beacon_desc), DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN)))
	{
		goto err;
	}

	DCACHE_STORE((unsigned int)wb, wb->data_off + wb->pkt_len);
	if((bhdr = bhdr_get_first(info)) == NULL)
		goto err;
	fill_bhdr(bhdr, wb, 0, 1);

	b_desc->desc.is_atim = 1;
	/* setting BEACON TX descr */
	if(bc)
	{
		b_desc->desc.tx_descr.bc = 1;            /* groupcast */
		b_desc->desc.tx_descr.ack_policy = MAC_ACK_POLICY_NOACK;
	}
	else
	{
		b_desc->desc.tx_descr.bc = 0;          
		b_desc->desc.tx_descr.ack_policy = MAC_ACK_POLICY_NORMAL_ACK;
	}
		
	b_desc->desc.tx_descr.addr_index = sta_idx;
	b_desc->desc.tx_descr.bssid = wb->bss_desc;		/* the bssid of skbs are input in-order */
	//b_desc->tx_descr.ifs = MAC_IFS_DIFS;
	b_desc->desc.tx_descr.dis_duration = 0;  /* enable duration */
	b_desc->desc.tx_descr.ins_ts = 1;

	b_desc->desc.tx_descr.pkt_length = wb->pkt_len;
	b_desc->total_len = b_desc->desc.tx_descr.pkt_length; /* remember original length without unicast'TIM */
	b_desc->desc.tx_descr.bhdr_head_index = bhdr_to_idx(info, bhdr);
	b_desc->desc.tx_descr.bhdr_tail_index = bhdr_to_idx(info, bhdr);

	b_desc->desc.tx_descr.u.atim.format = get_hw_bitrate_format(BEACON_BITRATE);
	b_desc->desc.tx_descr.u.atim.ch_offset = primary_ch_offset(info->bandwidth_type);
	/* FIXME: wait bit-file ready */
	b_desc->desc.tx_descr.u.atim.backoff_9 = 1; // indicate ATIM frame

	if(bc)
		bc_desc = b_desc;
	else
		unicast_desc = b_desc;

attach:
	b_desc->desc.tx_descr.eor = 1;
	b_desc->desc.next_pointer = 0;
	
	/* find last descriptor of list */
	for(last = (struct beacon_desc *)info->beacon_update_list; 
				PHYSICAL_ADDR(last) && last->desc.next_pointer; 
				last = (struct beacon_desc *)VIRTUAL_ADDR(last->desc.next_pointer));

	if(last)
	{
		last->desc.tx_descr.eor = 0;
		last->desc.next_pointer = PHYSICAL_ADDR(b_desc);
	}
	else
		info->beacon_update_list = (ssq_tx_descr *)b_desc;
#if 0
	for(last = (struct beacon_desc *)info->beacon_update_list; 
				PHYSICAL_ADDR(last) && last->desc.next_pointer; 
				last = (struct beacon_desc *)VIRTUAL_ADDR(last->desc.next_pointer));
	{
		buf_header *bhdr = idx_to_bhdr(info, last->desc.tx_descr.bhdr_head_index);
		struct wbuf *wb = dptr_to_wb((char*) bhdr->dptr);
		printd("%s(): update list =>\n", __FUNCTION__);
		diag_dump_buf(last, sizeof(struct beacon_desc));
		printd("wb : \n");
		diag_dump_buf(wb, wb->data_off + wb->pkt_len);
		printd("\n");
	}
#endif
	return 1;

err:
	if(wb)
		WBUF_FREE(wb);

	if(bhdr)
	{
		idx = bhdr_to_idx(info, bhdr);
		bhdr_insert_tail(info, idx, idx);
	}
	
	if(b_desc)
		WLA_MFREE((void *)b_desc);
	return 0;

}
#endif //(defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))

void wla_beacon(MAC_INFO *info)
{
	volatile struct beacon_desc *b_desc;
	buf_header *bhdr;
	char *tim, dtim_count, bss_idx, all_aid0;
	unsigned int i, tx_miss, beacon_timestamp;
	struct beacon_desc *prev, *last, *new_desc, *old_desc;

	/* only updated if beacon queue is enable already */
	if(!(MACREG_READ32(BEACON_CONTROL) & BEACON_CONTROL_ENABLE))
	{
		WLA_DBG(WLADEBUG, "NO beacon queue!!\n");
		return;
	}

	beacon_timestamp = WLA_CURRENT_TIME;
	if(info->beacon_timestamp && (beacon_timestamp > info->beacon_timestamp))
	{
		i = beacon_timestamp - info->beacon_timestamp;

		if(i < 3)
		{
			WLA_DBG(WLADEBUG, "delta=%d beacon_timestamp:%d info->beacon_timestamp:%d\n", i,beacon_timestamp,info->beacon_timestamp);
		}
	}


	if(!(MACREG_READ32(BEACON_CONTROL) & BEACON_CONTROL_TX_IDLE))
	{
		WLA_DBG(WLADEBUG, "Beacon is busy, skip this TBTT??? delta:%d\n",beacon_timestamp - info->beacon_timestamp);
		return;
	}
	info->beacon_timestamp = beacon_timestamp;

	wla_lock();

	if(info->mac_is_ready == 0)
	{
		wla_unlock();
		return;
	}

#if (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
	/* ATIM TEST : check broadcsat queue */
	u32 reg = MACREG_READ32(PSBA_PS_AID0_TIM_CR) & PS_AID0_TIM_DQ;
	if((reg & 0x1) && (ibss_b_wb))
	{
		all_aid0 = 0x1;	// FIXME: ibss bss id always is 0?
		wla_add_atim(info, ibss_b_wb, 0, 1);
#if 0
		if(MACREG_READ32(PSBA_PS_AID0_TIM_CR) & PS_AID0_TIM_DQ_STATUS) 
			info->bc_ps_queue_pending++;
		else
			MACREG_WRITE32(PSBA_PS_AID0_TIM_CR, (0x1 << 8)|PS_AID0_TIM_DQ_STATUS);
#endif
	}
	/* ATIM TEST : check unicast queue */
	for(b_desc = (struct beacon_desc *)info->beacon_descriptors_list; PHYSICAL_ADDR(b_desc); 
				b_desc = (struct beacon_desc *) VIRTUAL_ADDR(b_desc->desc.next_pointer))
	{
		if(b_desc->desc.is_atim)
			continue;
		bss_idx = b_desc->desc.tx_descr.bssid;
		if(info->bcast_qinfos[(int)bss_idx].sta_cnt)
		{	
			for(i=0; i<info->sta_cap_tbl_count; i++)
			{
				struct wsta_cb *wcb;
				wcb = info->wcb_tbls[i];
				if(wcb && (wcb->ibss_bssid)) // for ibss test
				{
					MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | i);
					while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD);
					reg = MACREG_READ32(PSBA_PS_UCAST_TIM_SR);
					if((reg & 0x8) && ibss_u_wb)
					{
						wla_add_atim(info, ibss_u_wb, ibss_sta_idx, 0);
					}
				}
			}
		}
	}

	/* ATIM TEST : free atim desc (only test & not really free resource) */
	for(old_desc = (struct beacon_desc *)info->beacon_descriptors_list, prev=0; 
		PHYSICAL_ADDR(old_desc);
		old_desc = (struct beacon_desc *) VIRTUAL_ADDR(old_desc->desc.next_pointer))
	{
		if(old_desc->desc.is_atim)
		{
			if(prev)
			{
				prev->desc.next_pointer = old_desc->desc.next_pointer;
				prev->desc.tx_descr.eor = old_desc->desc.tx_descr.eor;
			}
			else
			{
				info->beacon_descriptors_list = 
						((ssq_tx_descr *)old_desc->desc.next_pointer ? 
						(ssq_tx_descr *) VIRTUAL_ADDR(old_desc->desc.next_pointer): 0);
			}
		}
		else
		{
			prev = old_desc;
		}
	}
		
#endif //(defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))

	/* update beacons */
	if(info->beacon_update_list)
	{
		struct wbuf *old_wb, *new_wb;
		buf_header *new_bhdr;
		
		/* remove matched beacon */
		for(new_desc = (struct beacon_desc *)info->beacon_update_list; PHYSICAL_ADDR(new_desc); 
				new_desc = (struct beacon_desc *) VIRTUAL_ADDR(new_desc->desc.next_pointer))
		{
#if (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
			if(new_desc->desc.is_atim)
				continue;
#endif //(defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
			new_bhdr = idx_to_bhdr(info, new_desc->desc.tx_descr.bhdr_head_index);
			new_wb = dptr_to_wb((char *)(u32) new_bhdr->dptr);
			for(old_desc = (struct beacon_desc *)info->beacon_descriptors_list, prev=0; 
				PHYSICAL_ADDR(old_desc); prev = old_desc, 
				old_desc = (struct beacon_desc *) VIRTUAL_ADDR(old_desc->desc.next_pointer))
			{
				bhdr = idx_to_bhdr(info, old_desc->desc.tx_descr.bhdr_head_index);
				old_wb = dptr_to_wb((char *)(u32) bhdr->dptr);

				if(old_wb->bss_desc == new_wb->bss_desc)
				{
					if(prev)
					{
						prev->desc.next_pointer = old_desc->desc.next_pointer;
						prev->desc.tx_descr.eor = old_desc->desc.tx_descr.eor;
					}
					else
					{
						info->beacon_descriptors_list = 
								((ssq_tx_descr *)(u32)old_desc->desc.next_pointer ? 
								(ssq_tx_descr *) VIRTUAL_ADDR(old_desc->desc.next_pointer): 0);
					}

					while(bhdr)
					{
						WBUF_FREE( dptr_to_wb((char*)(u32) bhdr->dptr));
						if(bhdr->ep)
							break;
						bhdr = idx_to_bhdr(info, bhdr->next_index);
					}

					bhdr_insert_tail(info, old_desc->desc.tx_descr.bhdr_head_index, 
							old_desc->desc.tx_descr.bhdr_tail_index);

					WLA_MFREE((void *)old_desc);

					break;
				}
			}
		}

		/* find last descriptor of list */
		for(last = (struct beacon_desc *)info->beacon_descriptors_list; 
				PHYSICAL_ADDR(last) && last->desc.next_pointer; 
				last = (struct beacon_desc *)VIRTUAL_ADDR(last->desc.next_pointer));

		if(last)
		{
			last->desc.tx_descr.eor = 0;
			last->desc.next_pointer = PHYSICAL_ADDR(info->beacon_update_list);
		}
		else
			info->beacon_descriptors_list = info->beacon_update_list;

		info->beacon_update_list = 0;
		MACREG_WRITE32(BEACON_TXDSC_ADDR, PHYSICAL_ADDR(info->beacon_descriptors_list));
	}
	tx_miss = info->beacon_tx_bitmap ^ (MACREG_READ32(BEACON_TX_STATUS) & BEACON_TX_STATUS_RESULT);
	if(tx_miss)
	{
		for(i=0; i<info->bss_num; i++)
		{
			if((tx_miss >> i) & 0x1)
				info->beacon_tx_miss++;
		}
	}
	info->beacon_tx_bitmap = 0;	
	all_aid0 = 0;
	dtim_count = mac_get_dtim_counter();
	for(b_desc = (struct beacon_desc *)info->beacon_descriptors_list; PHYSICAL_ADDR(b_desc); 
				b_desc = (struct beacon_desc *) VIRTUAL_ADDR(b_desc->desc.next_pointer))
	{
#if (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
		if(b_desc->desc.is_atim)		// not handle IBSS power saving here
			continue;
#endif //(defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))

		/*	We expected one beacon frame should be divided three blocks: 
			1. WLAN header + Timestamp + ... 
			2. TIM IE
			3. country IE and others
		*/
		bss_idx = b_desc->desc.tx_descr.bssid;
		bhdr = idx_to_bhdr(info, b_desc->desc.tx_descr.bhdr_head_index);
		/* find second block of frame and update periodically */
		bhdr = idx_to_bhdr(info, bhdr->next_index);
		tim = (char *)VIRTUAL_ADDR(bhdr->dptr) + ((bhdr->offset_h << 6) | bhdr->offset);
		tim[2] = dtim_count;
#if defined(ENABLE_POWER_SAVING)
		{
			u32 n1, n2, regval;
			u16 aid;
			u32 have_bits = 0;
			u8 aid0, bitmap[WLAN_MAX_TIM_LEN];
			u32 delivery_mask;
	
		aid0 = 0;
		regval = 0;
		if(dtim_count == 0)
		{
			regval = MACREG_READ32(PSBA_PS_AID0_TIM_CR) & PS_AID0_TIM_DQ;
			//if(regval)
			//	WLA_DBG(WLADEBUG, "regval=%x, bss_idx=%d\n", regval, bss_idx); 
			if(regval & (0x1UL << bss_idx))
			{
				aid0 = 1; 
				all_aid0 = 1 << bss_idx;
			}
		}

		if(info->bcast_qinfos[(int)bss_idx].sta_cnt)
		{	
			for(i=0; i<info->sta_cap_tbl_count; i++)
			{
				struct wsta_cb *wcb;

				wcb = info->wcb_tbls[i];
				if(wcb && ((aid = wcb->aid) != 0))
				{
					MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | i);
					while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD);
					regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR);

					delivery_mask = 0x88888888UL;
					if(((wcb->apsd_trigger_deliver & APSD_AC_ALL_DELIVERY) == 0) ||
						((wcb->apsd_trigger_deliver & APSD_AC_ALL_DELIVERY) == APSD_AC_ALL_DELIVERY))
						/* all ACs are nondelivery-enabled OR all ACs are delivery-enabled
						   keep the delivery_mask as 0x88888888UL 
					     */
						;
					else
					{
						if(wcb->apsd_trigger_deliver & APSD_AC_BE_DELIVERY)
							delivery_mask &= ~0x00008008UL;
						if(wcb->apsd_trigger_deliver & APSD_AC_BK_DELIVERY)
							delivery_mask &= ~0x00000880UL;
						if(wcb->apsd_trigger_deliver & APSD_AC_VI_DELIVERY)
							delivery_mask &= ~0x00880000UL;
						if(wcb->apsd_trigger_deliver & APSD_AC_VO_DELIVERY)
							delivery_mask &= ~0x88000000UL;  
					}

					//if(regval)
					//	WLA_DBG(WLADEBUG, "regval=%x, delivery_mask=%x\n", regval, delivery_mask); 

					if(regval & delivery_mask)
					{
						/* toggle TIM bit if there is data in the STA unicast queue */
						if(0==have_bits)
						{
							/* lazy clear on TIM bitmap only if we start using it */
							memset(bitmap, 0, sizeof(bitmap));
							have_bits = 1;
						}
						bitmap[aid / 8] |= (1 << (aid % 8));

					}
				}
			}
		}

		if(have_bits)
		{
			/* Find largest even number N1 so that bits numbered 1 through
		 	 * (N1 x 8) - 1 in the bitmap are 0 and number N2 so that bits
		 	 * (N2 + 1) x 8 through 2007 are 0. */
			n1 = 0;
			for (i = 0; i < WLAN_MAX_TIM_LEN; i++) 
			{
				if (bitmap[i]) 
				{
					n1 = i & 0xfe;
					break;
				}
			}
			n2 = n1;
			for (i = WLAN_MAX_TIM_LEN - 1; i >= n1; i--) 
			{
				if (bitmap[i]) 
				{
					n2 = i;
					break;
				}
			}

			/* Bitmap control */
			tim[4] = n1 | aid0;
			/* Part Virt Bitmap */
			memcpy(&tim[5], bitmap + n1, n2 - n1 + 1);

			tim[1] = n2 - n1 + 4;
			bhdr->len = 6 + n2 - n1;
			b_desc->desc.tx_descr.pkt_length = b_desc->total_len + n2 -n1; 
		}
		else
		{
			tim[1] = 4;
			tim[4] = aid0;
			tim[5] = 0;
			bhdr->len = 6;
			b_desc->desc.tx_descr.pkt_length = b_desc->total_len;
		}
		}
#endif 	// ENABLE_POWER_SAVING
		info->beacon_tx_bitmap |= (0x1 << bss_idx);
	}

	if(all_aid0)
	{
		if(MACREG_READ32(PSBA_PS_AID0_TIM_CR) & PS_AID0_TIM_DQ_STATUS) 
		{
			//WLA_DBG(WLADEBUG, "??? BC PS queue pending????reg=%x\n", MACREG_READ32(PSBA_PS_AID0_TIM_CR));
			info->bc_ps_queue_pending++;
		}
		else
			MACREG_WRITE32(PSBA_PS_AID0_TIM_CR, (all_aid0 << 8)|PS_AID0_TIM_DQ_STATUS);


	}

#ifdef CONFIG_WLAN_IBSS_MODE
	/* setup IBSS beacon backoff */
	if(info->ibss_exist)
	{
		/* setup backoff = CWmin*2 */
		unsigned long backoff;
		get_random_bytes(&backoff, sizeof(unsigned long));
		backoff = (backoff & BEACON_BACKOFF) % IBSS_BACKOFF_TIMER;
		MACREG_WRITE32(GLOBAL_BEACON, backoff);

#if 0	
		/* afjust the timing delta (new adjust = (pre adjust + new delta)/2) */
		static s16 adjust = 0;
		s16 delta = (s16) (MACREG_READ32(TS0_ADJUST) & TS_INACCURACY);
		if((delta < 50) && (delta > -50))	/* only fine tune the |delta|<50 */
		{
			adjust = ((delta + adjust) >> 1);
			MACREG_WRITE32(TS0_ADJUST, adjust << 16);
		}
#endif
	}
#endif	//	CONFIG_WLAN_IBSS_MODE

	if(info->beacon_tx_bitmap)
	{
		/* write 1 clear to IDLE bit, set HW to busy , trigger the HW */
		MACREG_WRITE32(BEACON_CONTROL, MACREG_READ32(BEACON_CONTROL));
	}
	
	wla_unlock();

}

#ifdef TBTT_EXCEPTION_HANDLE
void wla_update_next_tbtt(MAC_INFO *info, u32 intr_mask)
{
	int i;
	u16 dtim_interval=0;
	u32 er_status=0;
	u32 bss_idx, offset;
	u32 beacon_interval, beacon_interval_us;
	struct wsta_cb *wcb=NULL;
	union {
		long long val_64;
		u32	val_32[2];
	} ts, q, r, final, dtim;

	er_status = (intr_mask >> 16);
	er_status = ((er_status ) | (er_status >> 8)) & 0xFF; // mix ERROR 0 & ERROR 1 info

	/* FIXME: should use macro to define the hardware BSS number (now is 8) */
	for(bss_idx=0; bss_idx<8; bss_idx++)
	{
		if((1 << bss_idx) & er_status)
			break;
	}

	if(bss_idx == 8)
	{
		return;
	}

	/* update Next TBTT */
	offset = bss_idx * 0x10;
	beacon_interval = (MACREG_READ32(TS0_BE+offset) >> 16) & 0xffff;
	beacon_interval_us = beacon_interval * 1024;

	ts.val_32[0] = MACREG_READ32(TS_REG_HI(bss_idx));
	ts.val_32[1] = MACREG_READ32(TS_REG_LO(bss_idx));
	q.val_64 = ts.val_64 / beacon_interval_us;
	r.val_64 = ts.val_64 % beacon_interval_us;
	final.val_64 = ts.val_64 - r.val_64 + 2*beacon_interval_us;
	
	MACREG_WRITE32(TS0_NEXT_TBTT_L + offset, final.val_32[1]);
	MACREG_WRITE32(TS0_NEXT_TBTT_H + offset, final.val_32[0]);

	/* update DTIM counter */
	if((dtim_interval = (MACREG_READ32(TS0_DTIM + offset) & 0xFF)))
	{
		dtim.val_64 = (q.val_64 + 2)%dtim_interval;
		if(dtim.val_32[1])	// dtim counter is a count down counter 
			dtim.val_32[1] = dtim_interval - dtim.val_32[1];
		MACREG_WRITE32(TS0_DTIM + offset, ((dtim.val_32[1] & 0xFF) << 8) | dtim_interval);
	}
#if 0
	/* set timeout & pretbtt for test */
	MACREG_UPDATE32(GLOBAL_PRE_TBTT, (10 << 16), 0xffff0000);
	MACREG_UPDATE32(TS0_BE+offset, 20, 0xffff);
#endif
	/* only set ts_synced when ts is greater than 1 sec. avoid the not stable state */
	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		wcb = info->wcb_tbls[i];
		if(wcb && (wcb->bss_desc == bss_idx))
		{
			if(ts.val_64 > 1000000)		
				wcb->ts_synced = TS_STATE_SYNCED;
			else
				wcb->ts_synced = TS_STATE_INIT;
			break;
		}
	}

	//WLA_DBG(WARN, "%s(): timestamp=(0x%08x:0x%08x), Next TBTT=(0x%08x:0x%08x), dtim=%d, ts_synced=%d, time=%d\n", __FUNCTION__, ts.val_32[0], ts.val_32[1], final.val_32[0], final.val_32[1], dtim.val_32[1], wcb->ts_synced, time(0));
}
#endif //TBTT_EXCEPTION_HANDLE

#ifdef CONFIG_WLAN_CLIENT_PS
void client_tbtt_handle(MAC_INFO *info, u32 bss_desc)
{
	struct mbss_t *mbss=&info->mbss[bss_desc];

	if(mbss->ps_rx_state == PS_STATE_SLEEP)
		wla_ps(bss_desc, PS_STATE_DOZE, 0, 1);
	if(mbss->ps_rx_state > PS_STATE_AWAKE)
		wla_cfg(DEL_RX_FILTER, RX_FILTER_BEACON);
}
#endif // CONFIG_WLAN_CLIENT_PS
