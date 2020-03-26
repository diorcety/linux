/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_sta.c
*   \brief  wla wlan mac sta function.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
#include <wla_util.h>
#include <wla_bb.h>
#include <rf.h>
#include <wla_debug.h>

#ifdef CONFIG_SRAM_WLA
#define CLA					aligned(32)
#if (defined(CONFIG_SRAM_SIZE) && (CONFIG_SRAM_SIZE >= 128))
#define IN_SRAM				section(".sram")
#else
#define IN_SRAM	
#endif
sta_tx_status_tbl all_tx_tbls[QOS_TX_STATUS_TBLS*MAX_STA_CAP_TBL_COUNT] __attribute__((CLA, IN_SRAM));
sta_rx_status_tbl all_rx_tbls[QOS_RX_STATUS_TBLS*MAX_STA_CAP_TBL_COUNT] __attribute__((CLA, IN_SRAM));
ucast_qinfo all_qinfos[QOS_TX_STATUS_TBLS*MAX_STA_CAP_TBL_COUNT] __attribute__((CLA, IN_SRAM));
#endif

/* Non reentrant function */
int wla_release_sta(struct wsta_cb *wcb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	sta_basic_info bcap;
	int flag, captbl_idx;
	sta_tx_status_tbl *tx_tbls;
	int val;
	unsigned char addr[6];	

	if(wcb == 0)
		return 0;

	if(wcb->hw_cap_ready == 0)
		return 0;
	
	captbl_idx = wcb->captbl_idx;
	captbl = &info->sta_cap_tbls[captbl_idx];
	if(!captbl->valid)
	{
		WLA_DBG(WLADEBUG, "???%s(%d), captbl->valid=%d\n", __func__, __LINE__, captbl->valid);
		return 0;
	}

	flag = BY_ADDR_IDX;

	if(ds_table(wcb->mode) == IN_DS_TBL)
	{
		/* flush related MAC address first */
		find_all_addrs_and_update(info, captbl_idx, 0);
		flag = IN_DS_TBL|BY_ADDR_IDX;
	}
#ifdef CONFIG_WLAN_IBSS_MODE
	else if(wcb->mode == IBSS_STA)
	{
		mac_addr_lookup_engine_update(info, wcb->ibss_ds_idx, 0, IN_DS_TBL|BY_ADDR_IDX);
	}
#endif

	bcap.val = 0;
	mac_addr_lookup_engine_find(addr, wcb->addr_idx, (int *)&bcap, flag);
	if(bcap.bit.vld == 0)
	{
		WLA_DBG(WLADEBUG, "???%s(), No address table to find???\n", __func__);

	}
	else
		mac_addr_lookup_engine_update(info, wcb->addr_idx, 0, flag|BY_ADDR_IDX);

	/* reset the peer_cache when the sta release */
	if(memcmp(info->peer_cache, addr, 6) == 0)
		memset(info->peer_cache, 0, 6);

#if defined(WLA_PSBA_QUEUE)
	//psbaq_invalidate(captbl->bssid, captbl_idx);
		WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before trigger invalidate\n", info->psbaq_cmd_waiting_max_cnt);

	/* invalidate PSBAQ, given the sta index and bssid index */
	MACREG_WRITE32(PSBAQ_QINFO_CSR0, (captbl->bssid));
	MACREG_WRITE32(PSBAQ_QINFO_CSR2, (captbl_idx << 8) | PSBAQ_QINFO_CSR2_CMD_STA_INV | PSBAQ_QINFO_CSR2_CMD_TRIGGER);

	WMAC_WAIT_AND_SLEEP(((val = MACREG_READ32(PSBAQ_QINFO_CSR2)) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), "trigger invalidate and waiting on PSBAQ command idle");
#endif
	//DCACHE_FLUSH((unsigned int)captbl, sizeof(sta_cap_tbl));  /* captbl should be located non-cached memory */
	captbl = (sta_cap_tbl *)NONCACHED_ADDR(captbl);

	captbl->valid = 0;
	tx_tbls = (sta_tx_status_tbl *)CACHED_ADDR(captbl->TX_tbl_addr);

	if(wcb->linked_ap)
	{
		struct peer_ap *pap = (struct peer_ap *)wcb->linked_ap;
		pap->type = 0;
		wcb->linked_ap = 0;

		if(info->linked_ap_count > 0)
			info->linked_ap_count--;
	}
	info->mbss[wcb->bss_desc].ln_ap = 0;
	
	/* write back TX and RX part of stacap */
	mac_stacap_cache_sync(captbl_idx, true);
	mac_rx_linkinfo_cache_sync(); 

	if(tx_tbls)
	{
		WMAC_WAIT_AND_SLEEP(((val = MACREG_READ32(DISASSOC_CTRL)) & DISASSOC_CTRL_IDLE) != DISASSOC_CTRL_IDLE, "mac_disassociate(1): waiting to idle");
	
        MACREG_WRITE32(DISASSOC_CTRL, (DISASSOC_STA_CAP_INDEX & captbl_idx) | DISASSOC_CTRL_SECURITY_AND_RX_KICK | DISASSOC_CTRL_TX_KICK);
	}

	wla_lock();
	wcb->hw_cap_ready = 0;
	wcb->sta = 0;
	wla_unlock();

	return 0;
}

/* free memory of sta_cap, wcb, status table */
int wla_post_release_sta(struct wsta_cb *wcb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	int i, captbl_idx;
	sta_tx_status_tbl *tx_tbls;
	sta_rx_status_tbl *rx_tbls;

	/* MAC disassociation may be blocked by software buffer valid signal. 
		Under this state, we need to receive RXQ to release this pending. */
	if((MACREG_READ32(DISASSOC_CTRL) & DISASSOC_CTRL_IDLE) 
			!= DISASSOC_CTRL_IDLE)
	{	
		return -1;
	}

	captbl_idx = wcb->captbl_idx;
	captbl = &info->sta_cap_tbls[captbl_idx];

	tx_tbls = (sta_tx_status_tbl *)CACHED_ADDR(captbl->TX_tbl_addr);
	rx_tbls = (sta_rx_status_tbl *)CACHED_ADDR(captbl->RX_tbl_addr);
		
	mac_stacap_cache_sync(captbl_idx, true); /* XXX: Need? */ 
	if(rx_tbls)
	{
		int tbl_num;
		//unsigned int cnt = 0;

		if(captbl->NE)
			tbl_num = QOS_RX_STATUS_TBLS;
		else
			tbl_num = 1;
 
		for(i=0; i< tbl_num; i++)
		{
#ifdef CONFIG_SRAM_WLA
			u32 ps_trig_en = rx_tbls[i].ps_trig_en;
			memset(&rx_tbls[i], 0, sizeof(sta_rx_status_tbl));
			rx_tbls[i].ps_trig_en = ps_trig_en;
#else
			if(rx_tbls[i].vld && rx_tbls[i].htvld)
			{
				mac_free_buffer_hdr(rx_tbls[i].frame_head_index, rx_tbls[i].frame_tail_index, !(rx_tbls[i].swb));
			}
#endif
		}
#if defined(WLA_PSBA_QUEUE)
		mac_detach_psbaq_ucast_qinfo(info, captbl_idx);
#endif
	}

	memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)); 
#ifdef CONFIG_SRAM_WLA
	if(tx_tbls)
		memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
#else
	WLA_MFREE(tx_tbls);
#endif
	DCACHE_STORE((unsigned int)tx_tbls, sizeof(sta_tx_status_tbl));

	WLA_DBG(WLADEBUG, "%s(), free tx_tbls=%p, size=0x%x, rx_tbls=%p, size=0x%x\n", __func__, tx_tbls, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS, rx_tbls, sizeof(sta_rx_status_tbl) * QOS_RX_STATUS_TBLS + sizeof(ucast_qinfo) * QOS_TX_STATUS_TBLS);

	info->wcb_tbls[wcb->captbl_idx] = 0;
	if(wcb->mode == 0)
		info->mbss[captbl->bssid].sta_count--;
	
	WLA_MFREE(wcb);
	
	return 0;
}

struct wsta_cb *wla_get_wcb(void)
{
	return (struct wsta_cb *)WLA_MALLOC(sizeof(struct wsta_cb), DMA_MALLOC_BZERO);
}

char wla_get_sta(char *sta_addr, char bss_desc, struct wsta_cb *wcb, char mode)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl* captbl;
	sta_cap_tbl* rate_captbl;
	int index, i;
	int addr_index;
	sta_basic_info bcap;
	sta_tx_status_tbl *tx_tbls;
    sta_rx_status_tbl *rx_tbls;
	ucast_qinfo *qinfos;
	struct peer_ap *ap;

	/* always allocate QOS table size, even for non-QOS STA */
	int tx_tbl_num = QOS_TX_STATUS_TBLS;
	int rx_tbl_num = QOS_RX_STATUS_TBLS;
	int mpdu_win_num = DEFAULT_LEGACY_RX_MPDU_WINDOW;
	char flag;
	
	if(!wcb)
	{
		panic("forget to assign wcb!!\n");
	}

	flag = ds_table(mode);	
	bcap.val = 0;
	/* find duplicated address and release it */
	index = mac_addr_lookup_engine_find(sta_addr, (int)&addr_index, 0, flag);
	if(index >= 0)
	{
		WLA_DBG(WLADEBUG, "duplicated addr and link to captbl[%d], release it!\n", index);
		mac_addr_lookup_engine_update(info, addr_index, 0, flag|BY_ADDR_IDX);
		if(flag & IN_DS_TBL) 
			find_all_addrs_and_update(info, index, 0);
	}

	if(0 > (index = sta_cap_get_free(info)))
	{
		return -1;
	}

	
	captbl = &info->sta_cap_tbls[index];

	
    /*
        BG & NE field is not actually used by HW currently

        BG=1, NE=0  -> legacy BG
        BG=1, NE=1  -> 11e
        BG=0, NE=1  -> 11n
     */

    /* 
        WARNING: we use .NE & .BG bit field to judge if the entry is allocated
                 Hence, for a allocated entry, either NE or BG shall be 1.
     */

	captbl->BG = 1;
	captbl->gsn = 1;
	captbl->QOS = 0;
	captbl->nsd = 1;	// not sounding

	if(wcb->qos)
	{
    	captbl->NE = 1;
		captbl->gsn = 0;
		captbl->QOS = 1;

		tx_tbl_num = QOS_TX_STATUS_TBLS;
		rx_tbl_num = QOS_RX_STATUS_TBLS;
		mpdu_win_num = DEFAULT_QOS_RX_MPDU_WINDOW;
	}

#ifdef DEBUG_MEM_CORRUPT
	if(info->tx_tbl_free_memory)
	{
		WLA_DBG(WLADEBUG, "tx_tbl_free_memory=%x, master=%x\n", *(int *)NONCACHED_ADDR(info->tx_tbl_free_memory), REG_READ32(0xaf0050d8));
		info->tx_tbl_free_memory = 0;
	}
#endif


#ifdef DEBUG_TRACE_RX_TBL_HISTORY
	tx_tbls = (sta_tx_status_tbl *)WLA_MALLOC(sizeof(sta_tx_status_tbl) * tx_tbl_num, 
							DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);
	rx_tbls = (sta_rx_status_tbl *)WLA_MALLOC(sizeof(sta_rx_status_tbl) * rx_tbl_num + 
							mpdu_win_num*sizeof(rx_defrag_tbl)*rx_tbl_num, 
							DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);

#else
	/*	
		| tx_tbl |
		| rx_tbl |
		| tx_baps_qinfos |
	*/
#ifdef CONFIG_SRAM_WLA
	tx_tbls = &all_tx_tbls[index*QOS_TX_STATUS_TBLS];
	rx_tbls = &all_rx_tbls[index*QOS_RX_STATUS_TBLS];
	qinfos =  &all_qinfos[index*QOS_TX_STATUS_TBLS];
	
	memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
	memset(rx_tbls, 0, sizeof(sta_rx_status_tbl)*QOS_RX_STATUS_TBLS);
#else
	tx_tbls = (sta_tx_status_tbl *)WLA_MALLOC(sizeof(sta_tx_status_tbl) * tx_tbl_num +
							sizeof(sta_rx_status_tbl) * rx_tbl_num +
							sizeof(ucast_qinfo) * tx_tbl_num,
							DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);
	rx_tbls = (sta_rx_status_tbl*)(&tx_tbls[tx_tbl_num]);
#endif	// CONFIG_SRAM_WLA
#endif
#ifdef DEBUG_MEM_CORRUPT
	{
		extern void *memory_district;
		memory_district = 0;
	}
#endif
	for(i=0; i<tx_tbl_num; i++)
	{
		tx_tbls[i].long_retry = HT_LONG_RETRY_DEFAULT;
		tx_tbls[i].short_retry = HT_SHORT_RETRY_DEFAULT;
		tx_tbls[i].ack_policy = MAC_ACK_POLICY_NORMAL_ACK;
	}

#if defined(ENABLE_POWER_SAVING)
	if(wcb->apsd_trigger_deliver & APSD_AC_BE_TRIGGER)
	{	
		rx_tbls[1].ps_trig_en = 1;
		rx_tbls[4].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_BK_TRIGGER)
	{	
		rx_tbls[2].ps_trig_en = 1;
		rx_tbls[3].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_VI_TRIGGER)
	{	
		rx_tbls[5].ps_trig_en = 1;
		rx_tbls[6].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_VO_TRIGGER)
	{	
		rx_tbls[7].ps_trig_en = 1;
		rx_tbls[8].ps_trig_en = 1;
	}
#endif

	captbl->TX_tbl_addr = (u32) PHYSICAL_ADDR(tx_tbls);
    captbl->RX_tbl_addr = (u32) PHYSICAL_ADDR(rx_tbls);

	/* these settings will be used when toggling gsn in TX descr */
	captbl->gack_policy = MAC_ACK_POLICY_NORMAL_ACK;
	captbl->glong_retry = LEGACY_LONG_RETRY_DEFAULT;
	captbl->gshort_retry = LEGACY_SHORT_RETRY_DEFAULT;

	captbl->bssid =	bss_desc; /* why set it in cap-table ? */

	captbl->MPDU_spacing = wcb->min_mpdu_start_spacing;
	/* AP manager assign wcb */
	wcb->captbl_idx = index;
	info->wcb_tbls[index] = wcb;
	wla_rc_init(info, wcb); 

	bcap.bit.bssid = bss_desc;
	bcap.bit.vld = 1;
#if defined(CONFIG_CHEETAH)
/* disable this func due to multicast under station mode */	
	bcap.bit.eth_header = 1; /* 1: convert WLAN header to ethernet header */
#endif

	bcap.bit.tosw = 1;
	/* The cached cap-table should write back before inserting addr-table */
	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
	/* 
        flush bitmap cache associated with this STA, full TID range
        This can avoid the problem when the same STA does not dis-associated properly
        (without tear-down BA-session)
     */
    for(i=0;i<MAX_QOS_TIDS;i++)
    {
        mac_invalidate_ampdu_scoreboard_cache(index, i);
    }

	/* write back TX and RX part of stacap */
	mac_stacap_cache_sync(index, true);
	mac_rx_linkinfo_cache_sync();
	
	/* [31:8] is basic capability, [7:0] is lookup index */
	
	addr_index = mac_addr_lookup_engine_update(info, (int)sta_addr, (bcap.val | index), flag);
	WLA_DBG(WLADEBUG,"add: addr_index=%d, table index=%d, sta_addr=%s\n", addr_index, index, wmaddr_ntoa((char *)sta_addr));

#ifdef CONFIG_WLAN_IBSS_MODE
	/* create ibss group bssid info into ds table */
	int tmp=0;
	if(wcb->ibss_bssid != NULL) /* if mode == IBSS_STA, then wcb->ibss_bssid != NULL */
	{
		/* FIXME: Should free the created sta cap when the ds tbl entry can't be created ? */
		if((tmp = mac_addr_lookup_engine_update(info, (int)wcb->ibss_bssid, (bcap.val | index), IN_DS_TBL)) < 0)
			WLA_DBG(WARN, "### IBSS WARN: can't add ibss group into ds table\n");
		else
			wcb->ibss_ds_idx = tmp;
	}
#endif

	wcb->addr_idx = addr_index;
	wcb->hw_cap_ready = 1;
#if defined(INTEL_SETUP_BA_PATCH)
	if(memcmp(sta_addr, intel_mac, 3) == 0)
	{
		wcb->chip_vendor = VENDOR_INTEL;
		WLA_DBG(WLADEBUG, "..Intel STA\n");
	}
#endif
	ap = 0;
	switch(mode)
	{
		case LINKED_AP:
		case MAT_LINKED_AP:
			captbl->ess_idx = addr_index; 
			captbl->parent_ap = 1;
			captbl->AP = 1;

			ap = new_peer_ap(info, mode);
			ap->sta_captbl = index;
			ap->bss_desc = bss_desc;
			info->mbss[(u32)bss_desc].ln_ap = ap;

			/* FIXED ME: update BASIC_RX_RATE only */
			i = info->mbss[(u32)bss_desc].tx_rate[BASIC_TX_RATE].rate_captbl;
			rate_captbl = &info->rate_tbls[i];
			rate_captbl->ess_idx = addr_index; 
			rate_captbl->parent_ap = 1;
			rate_captbl->AP = 1;
			break;
		case WDS_LINKED_AP:
			captbl->ess_idx = addr_index; 
			captbl->parent_ap = 0;
			captbl->AP = 1;

			ap = new_peer_ap(info, WDS_LINKED_AP);
			ap->sta_captbl = index;
			ap->bss_desc = bss_desc;
			info->mbss[(u32)bss_desc].ln_ap = ap;
			break;
		case IBSS_STA:
			captbl->parent_ap = 1;
			captbl->AP = 0;
			info->mbss[(u32)bss_desc].sta_count++;
			break;
		default:
			/* STA MODE */
			captbl->parent_ap = 0;
			captbl->AP = 0;
			info->mbss[(u32)bss_desc].sta_count++;
			break; 
	}
	wcb->linked_ap = ap;
	wcb->mode = mode;
	wcb->bss_desc = bss_desc;

	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
	mac_stacap_cache_sync(index, true);

#if defined(WLA_PSBA_QUEUE)
	/* initialize psbaq for BA-noagg/PS/AMPDU */
#ifndef CONFIG_SRAM_WLA
	qinfos = (ucast_qinfo *)&rx_tbls[rx_tbl_num];
#endif

	for(i=0; i<tx_tbl_num; i++)
	{
		memset(&qinfos[i], 0, sizeof(ucast_qinfo));
		qinfos[i].tx_head_ptr = 0xffff;
		qinfos[i].tx_tail_ptr = 0xffff;
		qinfos[i].tx_queue_len = MAX_PS_QUEUE_LENGTH;
	}

	info->sta_qinfo_tbls[index].qinfo_addr = PHYSICAL_ADDR((char *)qinfos);
	if(flag & IN_DS_TBL)
		info->sta_qinfo_tbls[index].is_ap = 1;
	info->sta_qinfo_tbls[index].sta_idx = addr_index;
	info->sta_qinfo_tbls[index].qos = wcb->qos; 
	info->sta_qinfo_tbls[index].vld = 1;

#ifdef TX_NOAGG_BY_PSBAQ
	if(info->txq_limit)
	{
		for(i=0; i<tx_tbl_num; i++)
		{
			WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
					"waiting on PSBAQ command idle before nogg-ba setup\n", info->psbaq_cmd_waiting_max_cnt);
			MACREG_WRITE32(PSBAQ_QINFO_CSR0, PSBAQ_QINFO_CSR0_FORCE_NOAGG);
			MACREG_WRITE32(PSBAQ_QINFO_CSR2, (index << 8) | (i << 4) | PSBAQ_QINFO_CSR2_CMD_SETUP | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
		}

		//bcap.bit.tx_ampdu = 0xff;
		//mac_addr_lookup_engine_update(info, addr_index, (bcap.val | index), flag|BY_ADDR_IDX);
	}
#endif
#if defined(ENABLE_POWER_SAVING)
	i = 0;
	if(wcb->apsd_trigger_deliver & APSD_AC_BE_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 0, i);
	psbaq_set_tid_delivery_status(info, index, 3, i);

	i = 0;	
	if(wcb->apsd_trigger_deliver & APSD_AC_BK_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 1, i);
	psbaq_set_tid_delivery_status(info, index, 2, i);

	i = 0;	
	if(wcb->apsd_trigger_deliver & APSD_AC_VI_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 4, i);
	psbaq_set_tid_delivery_status(info, index, 5, i);	

	i = 0;
	if(wcb->apsd_trigger_deliver & APSD_AC_VO_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 6, i);
	psbaq_set_tid_delivery_status(info, index, 7, i);	
#endif
#endif

	wla_add_sta_timer(wcb, HW_SYNC_TIME);

	/* Delete any exist key */
	wla_cipher_key(wcb, AUTH_CIPHER_NONE, KEY_TYPE_PAIRWISE_KEY, 0, 0, 0); 
	return index;
}
/*FIXME: Need to refine*/
char wla_chg_sta(char *sta_addr, char bss_desc, struct wsta_cb *wcb, char mode)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl* captbl;
	int addr_index;
	int index, i;
	sta_rx_status_tbl *rx_tbls;

	int tx_tbl_num = QOS_TX_STATUS_TBLS;
	int rx_tbl_num = QOS_RX_STATUS_TBLS;
	int mpdu_win_num = DEFAULT_LEGACY_RX_MPDU_WINDOW;
	char flag;


	flag = ds_table(mode);	
	index = mac_addr_lookup_engine_find(sta_addr, (int)&addr_index, 0, flag);
	if(index < 0)
	{
		WLA_DBG(WLAERROR, "We did not find the exist sta fot change\n");
		return 0; 
	}

	captbl = &info->sta_cap_tbls[wcb->captbl_idx];
	
	if(wcb->qos)
	{
    	captbl->NE = 1;
		captbl->gsn = 0;
		captbl->QOS = 1;

		tx_tbl_num = QOS_TX_STATUS_TBLS;
		rx_tbl_num = QOS_RX_STATUS_TBLS;
		mpdu_win_num = DEFAULT_QOS_RX_MPDU_WINDOW;
	}
	
	rx_tbls = (sta_rx_status_tbl *)CACHED_ADDR(captbl->RX_tbl_addr);
#if defined(ENABLE_POWER_SAVING)
	if(wcb->apsd_trigger_deliver & APSD_AC_BE_TRIGGER)
	{	
		rx_tbls[1].ps_trig_en = 1;
		rx_tbls[4].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_BK_TRIGGER)
	{	
		rx_tbls[2].ps_trig_en = 1;
		rx_tbls[3].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_VI_TRIGGER)
	{	
		rx_tbls[5].ps_trig_en = 1;
		rx_tbls[6].ps_trig_en = 1;
	}
	if(wcb->apsd_trigger_deliver & APSD_AC_VO_TRIGGER)
	{	
		rx_tbls[7].ps_trig_en = 1;
		rx_tbls[8].ps_trig_en = 1;
	}
#endif
	info->sta_qinfo_tbls[wcb->captbl_idx].qos = wcb->qos;

	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
#if defined(ENABLE_POWER_SAVING)
	i = 0;
	if(wcb->apsd_trigger_deliver & APSD_AC_BE_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 0, i);
	psbaq_set_tid_delivery_status(info, index, 3, i);

	i = 0;	
	if(wcb->apsd_trigger_deliver & APSD_AC_BK_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 1, i);
	psbaq_set_tid_delivery_status(info, index, 2, i);

	i = 0;	
	if(wcb->apsd_trigger_deliver & APSD_AC_VI_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 4, i);
	psbaq_set_tid_delivery_status(info, index, 5, i);	

	i = 0;
	if(wcb->apsd_trigger_deliver & APSD_AC_VO_DELIVERY)
		i = 1;
	psbaq_set_tid_delivery_status(info, index, 6, i);
	psbaq_set_tid_delivery_status(info, index, 7, i);	
#endif
	return 0;
}
void wla_add_wif(u32 idx, u8 role, u8 vif_id)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct mbss_t *wif;

	wif = &info->mbss[idx];

	wif->role = role;
	wif->vif_id = vif_id;

	if(info->bss_num == 0)
		info->first_bss_desc = idx;

	info->bss_num++;

	if(role == WIF_IBSS_ROLE)
		info->ibss_exist = 1;
	else if((role == WIF_AP_ROLE) || (role == WIF_P2P_GO_ROLE))
		info->ap_exist = 1;
#ifdef WLA_MULTICAST_PATCH
	else if(role == WIF_STA_ROLE)
		wla_add_multicast_addr_tbl(idx);
#endif // WLA_MULTICAST_PATCH


	/* decide TSF mode of TS0 */
	if(info->ibss_exist)
		MACREG_UPDATE32(LMAC_CNTL, 0x4, LMAC_CNTL_STAMODE);
	else if(info->ap_exist)
		MACREG_UPDATE32(LMAC_CNTL, 0x0, LMAC_CNTL_STAMODE);
	else
		MACREG_UPDATE32(LMAC_CNTL, 0x2, LMAC_CNTL_STAMODE);

	return; 
}

u32 wla_update_sta_status(struct wsta_cb *wcb, u32 current_time)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	unsigned short rx_cnt_2b;
	u8 captbl_idx;
	u32 next_update; /* 1sec by default */

	if(!info->mac_is_ready)
		return 0;
	if((wcb->hw_cap_ready == 0)) {
		WLA_DBG(WLADEBUG, "!!! hw_cap_ready:%d sta:%p\n",wcb->hw_cap_ready, (char *)wcb->sta);
		if(wcb->sta == 0)
			return 0;

	}

	captbl_idx = wcb->captbl_idx;
	captbl = &info->sta_cap_tbls[captbl_idx];
	
	/* write back TX and RX part of stacap to get latest statistic */
	mac_stacap_cache_sync(captbl_idx, true);
	mac_rx_linkinfo_cache_sync();

	rx_cnt_2b = wcb->rx_cnt & 0xffff;
	if(rx_cnt_2b != captbl->rx_pkt_cnt)
	{
		wcb->activity = 1;

		if(rx_cnt_2b > captbl->rx_pkt_cnt)
		{
			wcb->rx_cnt += 0x10000;
		}
		
		wcb->rx_cnt = (wcb->rx_cnt & 0xffff0000) | captbl->rx_pkt_cnt; 
		wcb->rx_bytes = captbl->rx_byte_cnt; 
		wcb->last_rx = jiffies;

	}
	wcb->rx_rate = captbl->rx_rate;
	wcb->rssi = captbl->rssi;
	/* RC adapation */
	next_update = wla_rc_update(info, wcb);

#if defined(WLA_PSBA_QUEUE)
	/* Check zombie frame in power saving queue */
	if(info->sta_qinfo_tbls[captbl_idx].vld)
	{
		sta_tx_status_tbl *tx_tbls;
		ucast_qinfo *qinfos;
		int i;
		u32 qstatus;
		
		qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[captbl_idx].qinfo_addr);
		tx_tbls = (sta_tx_status_tbl *)NONCACHED_ADDR(captbl->TX_tbl_addr);

		/* AP aging function: drop idle PS frames for 2 second */
		qstatus = psbaq_all_status(captbl_idx);
		if((QMODE_PS_BIT & qstatus) && (qstatus & PSBAQ_HAS_DATA) )
		{
			u16 all_dq_cnt, dq_cnt;

			all_dq_cnt = 0; 
			for(i=0; i<QOS_TX_STATUS_TBLS; i++)
			{
				COUNTER_DELTA(dq_cnt, wcb->psbaq_dq_cnt[i], qinfos[i].dq_agg_cnt);
				all_dq_cnt += dq_cnt;
			}

			if(0 == all_dq_cnt)
			{
				if(wcb->psbaq_freeze_time == 0)
					wcb->psbaq_freeze_time = current_time;
				WLA_MSG("freeze PSBAQ, freeze_time=%d, current_time=%d, qstatus=%x\n", wcb->psbaq_freeze_time, current_time, qstatus);

				if((current_time - wcb->psbaq_freeze_time) > 200)
				{
					i = 0;

					WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), "Waiting PSQ idle\n", info->psbaq_cmd_waiting_max_cnt);
					/* Drop packet until all queues are empty */
					do{
						MACREG_WRITE32(PSBAQ_QINFO_CSR0, wcb->bss_desc); /* XXX: Need ?? */
						MACREG_WRITE32(PSBAQ_QINFO_CSR2, (captbl_idx << 8) | PSBAQ_QINFO_CSR2_CMD_STA_AGING | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
						i++;

						WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), "Waiting PSQ drop\n", info->psbaq_cmd_waiting_max_cnt);

						/*  When some but not all ACs are trigger and delivery 
							enabled, BRCM may forget to trigger ACs. 
							It may cause data of legacy ACs was drop together. */
						if(wcb->apsd_trigger_deliver && 
							((wcb->apsd_trigger_deliver & APSD_AC_ALL_DELIVERY) 
							!= APSD_AC_ALL_DELIVERY))
							break;

						qstatus = psbaq_all_status(captbl_idx);
					}
					while((QMODE_PS_BIT & qstatus) && (qstatus & PSBAQ_HAS_DATA));

					wcb->psbaq_freeze_time = 0;
					WLA_MSG("Drop %d freeze PS frames!!!\n", i);
				}
			}
			else
				/* record last checking time */
				wcb->psbaq_freeze_time = 0;
		}
		else
			wcb->psbaq_freeze_time = 0;

		/* accumulate all TID queue's TX bytes */
		for(i=0; i<QOS_TX_STATUS_TBLS; i++)
		{
			u32 tx_byte_cnt = tx_tbls[i].tx_byte_cnt;
			tx_tbls[i].tx_byte_cnt = 0;
			
			if(tx_byte_cnt > 0)
				wcb->activity = 1;

			wcb->tx_bytes += tx_byte_cnt;
#ifdef CONFIG_WLAN_CLIENT_PS
			if(tx_byte_cnt)
				wla_busy_timestamp(wcb->bss_desc, current_time, 1);
#endif //CONFIG_WLAN_CLIENT_PS
			
			/* Check BA session status */
			/* simple traffic rate checking, tx_byte_cnt > 1k Bytes */
			if((tx_byte_cnt>>10) && (wcb->rc_allow_ampdu > 0) && (wcb->ba_tx[i].state == BA_IDLE_STATE))
			{
				if(!psbaq_qmode_is_busy(info, wcb->captbl_idx, i))
				{
					wm_ba_event(BA_SESSION_INITIATION, i,(u32)wcb->sta);
				}
				else
				{
					WLA_DBG(WLADEBUG, "Try to setup BA, but qmode is busy\n");
				}
			}
			else if(wcb->ba_tx[i].state == BA_ESTABLISHED_STATE)
			{
				/* Rate adapation suggested tear-down AMPDU. */
				if(wcb->rc_allow_ampdu <= 0)
				{
					wm_ba_event(BA_SESSION_TEARDOWN, i,(u32)wcb->sta);
				}
                else if(((qinfos[i].qmode & QMODE_PS_BIT) == 0) && qinfos[i].pkt_cnt && qinfos[i].bar)
				{
					/* In PS state, PSBAQ only transmits MPDUs and BAR should not be sent. */
					WLA_DBG(DEBUG, "BAR is not accepted, pkt_cnt:%d\n", qinfos[i].pkt_cnt);
					wm_ba_event(BA_SESSION_TEARDOWN, i,(u32)wcb->sta);
				}
			}
		}
	}
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	while((current_time - wcb->reorder_buffer_check_time) > 50)
	{
		ampdu_reorder_buf *reorder_buf;
		u16 seq_num, offset;
		u8 m;
		u64 bitmap;
		u32 bitmap32[2];
		int i, win_sz, rb_count = 0;

		wcb->reorder_buffer_check_time = current_time;
		for(i = 0; i < MAX_QOS_TIDS; i++)
		{
			reorder_buf = (ampdu_reorder_buf *)info->reorder_buf_mapping_tbl[(captbl_idx * MAX_QOS_TIDS) + i];
			if(reorder_buf == 0)
				continue;
		
			reorder_buf->dbg_dirty = 1; /* Let table is dirty. If cache write back, it shall become clean. */
			rb_count++;;
		}

		/* bypass below if no reorder exist */
		if(rb_count == 0)
			break;

		wla_lock();
		if((MACREG_READ32(AMPDU_BMG_CTRL) & (REORDER_FLUSH|BMG_CLEAR)) || (info->stop_ampdu_rx))
		{
			wla_unlock();
			WLA_DBG(WLADEBUG, "Fail to flush AMPDU_BMG_CTRL!!\n");
			break;
		}

		MACREG_WRITE32(AMPDU_BMG_CTRL, VARIABLE_TABLE_1_CACHE_FLUSH | VARIABLE_TABLE_0_CACHE_FLUSH);
	wla_unlock();
	
		/* kick one frame out from reorder buffer to avoid sticky window */
		for(i = 0; (i < MAX_QOS_TIDS) && rb_count; i++)
		{
			reorder_buf = (ampdu_reorder_buf *)info->reorder_buf_mapping_tbl[(captbl_idx * MAX_QOS_TIDS) + i];
			if(reorder_buf == 0)
				continue;

			rb_count--;	
			if(wla_ampdu(GET_AMPDU_RX_STATUS, wcb, i, 0, 0) == 0)	
				continue;
	
			if(reorder_buf->bitmap[0] ||reorder_buf->bitmap[1])
			{
				bitmap32[0] = reorder_buf->bitmap[0];
				bitmap32[1] = reorder_buf->bitmap[1];

				bitmap = bitmap32[0];
				bitmap = (bitmap << 32) | ((u64)(bitmap32[1]) & 0xffffffff);

				win_sz = 1 << reorder_buf->buf_size;
				offset = WLAN_SEQ_SUB(reorder_buf->head_seq_num, reorder_buf->ssn) % win_sz;
			
				if(offset)
				{
					bitmap = ((bitmap << offset) | (bitmap >> (win_sz - offset)));
				}

				m = BITS_PER_LONG - fls((int)(bitmap >> 32));
				if(m == BITS_PER_LONG)
				{
					m = BITS_PER_LONG - fls((int)bitmap);
					seq_num = WLAN_SEQ_ADD(reorder_buf->head_seq_num, m + BITS_PER_LONG);
				}
				else
				{
					seq_num = WLAN_SEQ_ADD(reorder_buf->head_seq_num, m);
				}

				if(wcb->ampdu_rx_last_seq_num[i] == seq_num)
				{
					WLA_DBG(WLADEBUG, "++++++1, reorder_buf->head_seq_num=%d, reorder_buf->ssn=%d, dbg_dirty=%d\n", reorder_buf->head_seq_num, reorder_buf->ssn, reorder_buf->dbg_dirty);
					WLA_DUMP_BUF_16BIT((char *)&reorder_buf->bitmap[0], 8);

					WLA_DBG(WLADEBUG, "~~~~after alignment, offset=%d\n", offset);
					WLA_DUMP_BUF_16BIT((char *)&bitmap, 8);
			
					if(info->no_kick_reorder_buf == 0)
					{
						WLA_DBG(WLADEBUG, "kick reorder buffer, sta=%d, seq=%d, last_seq=%d\n", captbl_idx, seq_num, wcb->ampdu_rx_last_seq_num[i]);
						mac_flush_one_ampdu_reorder_buf(captbl_idx, i);
					}
				}
				else
					wcb->ampdu_rx_last_seq_num[i] = seq_num;
			}
		}
	}
#endif

	return next_update;	
}

#if defined(CONFIG_WMAC_RECOVER)
void wla_notify_no_ampdu(void)
{
	MAC_INFO *info = WLA_MAC_INFO;
	int i;
	struct wsta_cb *wcb;

	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		if(info->sta_cap_tbls[i].valid)
		{
			wcb = info->wcb_tbls[i];
			if (!wcb)
				continue;
			wm_ba_event(BA_SESSION_TEARDOWN_ALL, 0,(u32)wcb->sta);
		}
	}
}
#endif

