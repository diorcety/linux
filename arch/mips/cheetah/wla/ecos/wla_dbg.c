/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file arthur_debug.c
*   \brief  arthur debug function.
*   \author Montage
*/
#include <os_compat.h>
#include <sys/types.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/drv_api.h>
#include <netdev.h>

#include <cli_api.h>

#include <mac_ctrl.h>
#include <ecos_wla.h>
#include <wbuf.h>
#include <wlan_def.h>
#include <wla_debug.h>
#include <rfc.h>
#include <rfac.h>
#include <rfac_rfc_patch.h>
#include <rf.h>
#include <wla_bb.h>
#include <ether.h>
#include <ip301.h>

static int get_parameters(unsigned char *str, int* param)
{
    int i = 0;
    int j = 0;
    unsigned char value;

    while(str[i]!=0) 
    {
	if(str[i]=='-')
	{
	    j++;
	}
	else
	{
	    value = ((str[i] >= '0') && (str[i] <= '9')) ? (str[i] - '0') : 0xff;
	    value = ((str[i] >= 'A') && (str[i] <= 'Z')) ? ((str[i] - 'A') + 10) : value;
	    value = ((str[i] >= 'a') && (str[i] <= 'z')) ? ((str[i] - 'a') + 10) : value;

	    if(value == 0xff) 
            {
		i++;
		continue;
	    }

	    param[j] = param[j] * 16 + value;
	}

	i++;
    }
    return 0;
}

void reg_dump(unsigned char *addr,int size)
{
	unsigned char *caddr;

	caddr = addr;
	while (caddr < (addr + size))
	{
		if ((((int)caddr % 16)== 0) || caddr == addr)
		{
			diag_printf("\n%08x: ", caddr);
		}

		diag_printf("%08x ", *((volatile unsigned long *) caddr));
		caddr += 4;
	}
}

void dump_bcap(MAC_INFO* info, int index, char is_ds)
{
	char addr[6];
	int bcap = 0;
	
	memset(addr, 0, 6);
	mac_addr_lookup_engine_find(addr, index, &bcap, BY_ADDR_IDX|(is_ds ? IN_DS_TBL:0));

	printf("[%d] addr=%s, bcap=%08x\n", index, ether_ntoa((struct ether_addr *)addr), bcap);
}

void dump_addr_cache(MAC_INFO* info)
{
	int idx, val;
	for(idx = 0; idx < 32; idx++)
	{
		MACREG_WRITE32(0x86c, (idx << 2)|0x1);
		val = MACREG_READ32(0x86c);
		printd("[%d], val=%x\n", idx, val);
	}
}

void dump_sta(MAC_INFO* info, int sta_idx, char rate_tbl)
{
	int i;
	sta_cap_tbl *captbl;
	sta_rx_status_tbl *rx;
	sta_tx_status_tbl *tx;

	if(!rate_tbl)
		captbl = &info->sta_cap_tbls[sta_idx];
	else
		captbl = &info->rate_tbls[sta_idx];
	
	printf("sta_idx=%d, 0x%x\n", sta_idx, captbl);
	mac_stacap_cache_sync(sta_idx, true);
	mac_rx_linkinfo_cache_sync();

	printf("\tV:%d NE:%d BG:%d PS:%d AP:%d AMSDU:%d Short-Preamble:%d QOS:%d PSMP:%d gsn:%d, def_key:%d cipher_mode:%d essid:%d bssid:%d parent_ap:%d, force_rts:%d\n", captbl->valid, captbl->NE,  captbl->BG, captbl->power_saving, captbl->AP, captbl->AMSDU, captbl->short_preamble, 	captbl->QOS, captbl->PSMP, captbl->gsn, captbl->wep_defkey, captbl->cipher_mode,  captbl->ess_idx, captbl->bssid, captbl->parent_ap, captbl->force_rts);
	printf("\tRX_tbl_addr:%x, TX_tbl_addr:%x\n", captbl->RX_tbl_addr, captbl->TX_tbl_addr);
	printf("\tglong_retry:%d, gshort_retry:%d, gack_policy:%d\n", captbl->glong_retry, captbl->gshort_retry, captbl->gack_policy);
	printf("\trate=0x%x, 0x%x, 0x%x, 0x%x\n", captbl->rate[0].val, captbl->rate[1].val, captbl->rate[2].val, captbl->rate[3].val);
	printf("\ttx_ok_rate=%d, %d, %d, %d\n", captbl->tx_ok_rate[0], captbl->tx_ok_rate[1], captbl->tx_ok_rate[2], captbl->tx_ok_rate[3]);	
	printf("\tx_try_cnt0=%d, tx_try_cnt1=%d, tx_fail_cnt=%d, tx_try_cnt2=%d\n",  captbl->tx_try_cnt0,  captbl->tx_try_cnt1, captbl->tx_fail_cnt,  captbl->tx_try_cnt2);
	printf("\trx_byte_cnt=%d, rx_rate=0x%x, rssi=0x%x, rx_pkt_cnt=%d\n",  captbl->rx_byte_cnt, captbl->rx_rate, captbl->rssi, captbl->rx_pkt_cnt);

	if(captbl->RX_tbl_addr)
	{
	rx = (sta_rx_status_tbl *)NONCACHED_ADDR(captbl->RX_tbl_addr);
	
	if(!captbl->NE)
	{
		printf("###RX_tbl_addr###, size=%d\n", sizeof(sta_rx_status_tbl));
		diag_dump_buf(rx, sizeof(sta_rx_status_tbl));
	}
	else
	{
		sta_rx_status_tbl *rx = (sta_rx_status_tbl *)NONCACHED_ADDR(captbl->RX_tbl_addr);
		for(i=0; i<QOS_RX_STATUS_TBLS; i++)
		{
			printf("###RX_tbl_addr [%d]###, size=%d\n", i, sizeof(sta_rx_status_tbl));
			diag_dump_buf(&rx[i], sizeof(sta_rx_status_tbl));
		}	
	}
	}

	if(captbl->TX_tbl_addr)
	{
	if(!captbl->QOS)
	{
		printf("###TX_tbl_addr###, size=%d\n", sizeof(sta_tx_status_tbl));
		diag_dump_buf( (void*)NONCACHED_ADDR(captbl->TX_tbl_addr), sizeof(sta_tx_status_tbl));
	}
	else
	{
		tx = (sta_tx_status_tbl *)NONCACHED_ADDR(captbl->TX_tbl_addr);

		for(i=0; i<QOS_TX_STATUS_TBLS; i++)
		{
			printf("###TX_tbl_addr [%d]###, size=%d\n", i, sizeof(sta_tx_status_tbl));
			diag_dump_buf(tx+i, sizeof(sta_tx_status_tbl));
		}	
	}
	}
}

void dump_wcb(MAC_INFO* info, int sta_idx)
{
	struct wsta_cb *cb;

	cb = info->wcb_tbls[sta_idx];

	printf("WCB:0x%x\n", cb);
	if(cb)
	{
		printf("next:0x%x\nsync_time:%d\naid:%d\naddr_idx:%d\ncaptbl_idx:%d\nbss_desc:%d\nmode=%d\nsta:0x%x\n", cb->next,cb->sync_time,cb->aid, cb->addr_idx,cb->captbl_idx,cb->bss_desc,cb->mode,cb->sta);
		printf("rate_flags=0x%x\nsupported_rates:0x%x\nlinked_ap:0x%x\n", cb->rate_flags, cb->supported_rates,cb->linked_ap);
		printf("reorder_buffer_check_time:%d\ntx_ampdu_teardown_time:%d\npsbaq_freeze_time:%d\n", cb->reorder_buffer_check_time,cb->tx_ampdu_teardown_time,cb->psbaq_freeze_time);
		printf("wds_sa_count:%d\nmax_sp_len:%d\napsd_trigger_deliver:0x%x\n", cb->wds_sa_count, cb->max_sp_len,cb->apsd_trigger_deliver);
		printf("rc_allow_ampdu:%d\nrx_rate:0x%x\nrssi:0x%x\ntx_ampdu_bitmap:0x%x\n", cb->rc_allow_ampdu,cb->rx_rate,cb->rssi,cb->tx_ampdu_bitmap);
		printf("basic_rc_count:%d\nbasic_rc_flag=%d\n", cb->basic_rc_count,cb->basic_rc_flag);
	} 

}

void dump_ba(MAC_INFO* info, int sta_idx)
{
#if defined(WLA_AMPDU_RX_SW_REORDER)
	int j;
	struct wsta_cb *cb;
	struct rx_ba_session *ba;

	cb = info->wcb_tbls[sta_idx];
	if(cb == 0)
		return;
	printf("RX:\n");	
	for(j=0; j<8; j++)
	{
		ba = (struct rx_ba_session *)cb->ba_rx[j];
		if(ba)
		{
			printf("   [%d]ba=%x, size=%d, start=%d, num=%d, head=%x, tail=%x\n", j, ba, ba->win_size, ba->win_start, ba->stored_num, ba->reorder_q_head, ba->reorder_q_tail);
		}
	}
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	int regval, i;
	MACREG_WRITE32(AMPDU_BMG_CTRL, VARIABLE_TABLE_1_CACHE_FLUSH | VARIABLE_TABLE_0_CACHE_FLUSH);	
	for(i = 0; i < MAX_QOS_TIDS; i++)
	{
		if(info->reorder_buf_mapping_tbl[(sta_idx * MAX_QOS_TIDS) + i] != 0)
		{
			printf("TID:%d\n", i);
			diag_dump_buf_16bit((char *)info->reorder_buf_mapping_tbl[(sta_idx * MAX_QOS_TIDS) + i], sizeof(ampdu_reorder_buf));
		}
	}

	printf("TX:\n");
	MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | sta_idx);	
	while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD)
		;
    regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR); 
	printf("state=%08x\n",  regval);
	printf("sta_qinfo_tbls:\n");
	diag_dump_buf_16bit(&info->sta_qinfo_tbls[sta_idx], sizeof(sta_qinfo_tbl));
	if(info->sta_qinfo_tbls[sta_idx].qinfo_addr)
	{
		ucast_qinfo *qinfos = (ucast_qinfo *)NONCACHED_ADDR(info->sta_qinfo_tbls[sta_idx].qinfo_addr);
		for(i=0; i<8; i++)
		{
			printf("TID:%d\n", i);
			diag_dump_buf_16bit(&qinfos[i], sizeof(ucast_qinfo));
		}		
	}
#endif
}


void dump_bc_qinfo(MAC_INFO* info)
{
	bcast_qinfo *qinfos = info->bcast_qinfos;
	int i;

	for(i=0; i<MAC_MAX_BSSIDS; i++)
	{
		printf("BSS:%d\n", i);
		diag_dump_buf_16bit(&qinfos[i], sizeof(bcast_qinfo));
	}
}

void dump_list(MAC_INFO* info, short head, short tail, short low, short high)
{
	u32 i, j;
	buf_header *bhdr;
	u8 free[FASTPATH_BUFFER_HEADER_POOL_SIZE+SW_BUFFER_HEADER_POOL_SIZE];

	memset(free, 0, FASTPATH_BUFFER_HEADER_POOL_SIZE+SW_BUFFER_HEADER_POOL_SIZE);	
	printd("******freelist:%d, %d\n", head, tail);
	
	while(head != tail)
	{
		bhdr = idx_to_bhdr(info, head);
		printd("->[%d]\n", head);
			diag_dump_buf_32bit(bhdr, 8);
		if((head < low) || (head >=high))
			printd("!!!!!!!!!!!!!!illegal Range!!!!!!!!!!!!\n");
		free[head] = 1;
		head = bhdr->next_index; 
	}

	printd("******used:\n");
	for(i=low, j=0; i<high; i++)
	{
		if(!free[i])
		{
			j++;
			printd("??[%d]\n", i);
			diag_dump_buf_32bit(idx_to_bhdr(info, i), 8);	
		}
	}
	printd("used number:%d\n", j); 
}

#if defined(WLAN_RC_BTS)
void dump_bts(MAC_INFO* info, int sta_idx)
{
	struct wsta_cb *cb;
	struct bts_cb *bcb;
	struct bts_rate *br;
	u32 i;

	cb = info->wcb_tbls[sta_idx];
	if(cb ==0)
		return;
	bcb = &cb->bts;

	printf("\naux=%d, cur_tx_rate:", bcb->aux);
	for(i=0; i<4; i++)
	{
		printf("[%d|%d] ", cb->cur_tx_rate[i].rate_idx, cb->cur_tx_rate[i].bts_rates);
	}
	printf("\n[ridx]      TP Prob        TS   success/attemp\n");
	for(i=0; i<bcb->rates_len; i++)
	{
		br = &bcb->rates[i];
		printf("[%2d] %9d  %3d %10d  %4d/%-4d\n", br->rate_idx, br->tp, br->avg_prob, br->timestamp, br->success, br->attempts);
	}

}
#endif

static int wla_debug_cmd(int argc, char *argv[])
{
	MAC_INFO *info = WLA_MAC_INFO;
	int i;
	extern int wbuf_mc_used, wbuf_mem_used;
 
	if (2 > argc)
		return CLI_SHOW_USAGE;

	if (!strcmp(argv[1], "info"))
	{
		printf("sta_cap_tbl_count=%d\nrate_tbl_count=%d\n", 
				info->sta_cap_tbl_count, info->rate_tbl_count);
		printf("beacon_descriptors_list=%x\ntx_descriptors=%x\nsw_returned_tx_descriptors=%x\n", info->beacon_descriptors_list, info->wl_txq.desc_hdr, info->wl_sw_retq.desc_hdr);
		printf("hw_tx_buffer=%x\nfastpath_buf=%x\nfastpath_bufsize=%d\n", info->hw_tx_buffer, info->fastpath_buf, info->fastpath_bufsize);
		printf("ac_vo=%d, ac_vi=%d, ac_be=%d, ac_bk=%d\n", info->acq_len[ACQ_VO_IDX], info->acq_len[ACQ_VI_IDX], info->acq_len[ACQ_BE_IDX], info->acq_len[ACQ_BK_IDX]);
		printd("w_rx_dump=%d, w_tx_dump=%d, w2e_rx_dump=%d, w2e_tx_dump=%d\n", info->w_rx_dump, info->w_tx_dump, info->w2e_rx_dump, info->w2e_tx_dump);
		printd("data_forward_mode=%d, erp_protect=%d, non_gf_exist=%d, bw40mhz_intolerant=%d\n", info->data_forward_mode, info->erp_protect, info->non_gf_exist, info->bw40mhz_intolerant);
		printd("no_kick_reorder_buf=%d, txq_limit=%d, fix_acq_limit=%d\n", info->no_kick_reorder_buf, info->txq_limit, info->fix_ac_queue_limit);
#if defined(CONFIG_CHEETAH)
		printd("sta_tbl_count=%d, ds_tbl_count=%d\n", info->sta_tbl_count, info->ds_tbl_count);
#endif
		printd("wmac_polling=%x\n", info->wmac_polling);
		printd("sn_gap=%d\n", info->ampdu_sn_gap);
		printf("**********statistic***********\n");
		printf("pre_tbtt_int=%d, ts0_int=%d, beacon_tx_miss=%d\n", info->pre_tbtt_int, info->ts0_int, info->beacon_tx_miss);
		printf("sw_wrx=%d, sw_wrx_drop=%d\nsw_wtx=%d, sw_wret=%d\n", info->wl_rxq.pkt_count, info->sw_wrx_drop_pkt_cnt, info->wl_txq.pkt_count, info->wl_sw_retq.pkt_count);
		printf("sw_erx=%d\nsw_etx=%d\n", info->eth_rxq.pkt_count, info->eth_txq.pkt_count);
		printf("sw_w2e_unicast=%d, sw_w2e_bcast=%d, sw_e2w_unicast=%d, sw_e2w_bcast=%d\n", info->sw_w2e_unicast_pkt_cnt, info->sw_w2e_bcast_pkt_cnt, info->sw_e2w_unicast_pkt_cnt, info->sw_e2w_bcast_pkt_cnt);
		printf("sw_wtx_no_bhdr=%d, sw_wtx_no_desc=%d, sw_wtx_bcast_pkt_waiting=%d\n", info->sw_wtx_no_bhdr, info->sw_wtx_no_desc, info->sw_wtx_bcast_pkt_waiting);
		printf("hw_buf_full=%d, rx_desc_full=%d, rx_fifo_full=%d, sw_retq_full=%d\n", info->hw_buf_full, info->rx_desc_full, info->rx_fifo_full, info->sw_retq_full);
		printf("eth_hw_desc_full=%d, eth_sw_desc_full=%d\n", info->eth_hw_desc_full, info->eth_sw_desc_full);
		printf("int_mask=0x%x, rxfilter=0x%x\n", info->int_mask, info->filter);
		printf("bc_ps_queue_pending=%d\n", info->bc_ps_queue_pending);
		printf("wbuf_mc_used=%d, wbuf_mem_used=%d\n", wbuf_mc_used, wbuf_mem_used);
		printf("psbaq_cmd_waiting_max_cnt=%d\n", info->psbaq_cmd_waiting_max_cnt);
		printf("mac_recover_mode=%d, mac_recover=%d, mac_recover_cnt=%d\n", info->mac_recover_mode, info->mac_recover, info->mac_recover_cnt);
		printf("t_closest(%x) t_current(%x) t_delta(%x) t_recovery(%x)\n", info->dbg_closest_time, info->dbg_current_time, info->dbg_delta_time, info->dbg_recover_time);
		return CLI_OK;
	}
	else if (!strcmp(argv[1], "umac"))
	{
		reg_dump((unsigned char *)0xaf003800, 0x100);
		diag_printf("\nTX:");
		reg_dump((unsigned char *)0xaf003a00, 0x100);
		diag_printf("\nRX:");
		reg_dump((unsigned char *)0xaf003b00, 0x100);
		diag_printf("\nETH:");
		reg_dump((unsigned char *)0xaf009800, 0x100);
		diag_printf("\nSEC:");
		reg_dump((unsigned char *)0xaf009c00, 0x34);
	}
	else if (!strcmp(argv[1], "lmac"))
	{
		reg_dump((unsigned char *)0xaf009000, 0x100);
	}
	else if (!strcmp(argv[1], "txq"))
	{
		diag_printf("descr_count=%d\n", info->wl_txq.max);
		diag_dump_buf_32bit(info->wl_txq.desc_hdr, info->wl_txq.max*16);
	}
	else if (!strcmp(argv[1], "wrxq"))
	{
		diag_printf("descr_count=%d\n", info->wl_rxq.max);
		diag_dump_buf_32bit(info->wl_rxq.desc_hdr, info->wl_rxq.max*16);
	}
	else if (!strcmp(argv[1], "retq"))
	{
		diag_printf("descr_count=%d\n", info->wl_sw_retq.max);
		diag_dump_buf_32bit(info->wl_sw_retq.desc_hdr, info->wl_sw_retq.max*16);
	}
	else if (!strcmp(argv[1], "fretq"))
	{
		diag_printf("descr_count=%d\n", info->fp_retq.max);
		diag_dump_buf_32bit(info->fp_retq.desc_hdr, info->fp_retq.max*16);
	}
	else if (!strcmp(argv[1], "swbuf"))
	{
		int val;
		short head, tail;

		val = MACREG_READ32(SWBUFF_CURR_HT);
		head = val >> 16;
		tail = val & 0xffff;

		printf("sw_rx_bhdr_start=%d, sw_tx_bhdr_start=%d, sw_tx_bhdr_head=%d, sw_tx_bhdr_tail=%d\n", info->sw_rx_bhdr_start, info->sw_tx_bhdr_start, info->sw_tx_bhdr_head, info->sw_tx_bhdr_tail);
		dump_list(info, head, tail, info->sw_rx_bhdr_start, info->sw_tx_bhdr_start);

		if(info->sw_tx_bhdr_head > 0)
			dump_list(info, info->sw_tx_bhdr_head, info->sw_tx_bhdr_tail, info->sw_tx_bhdr_start, info->sw_tx_bhdr_end);
	}
	else if (!strcmp(argv[1], "hwbuf"))
	{
		int val;
		short head, tail;

		val = MACREG_READ32(HWBUFF_CURR_HT);
		head = val >> 16;
		tail = val & 0xffff;

		dump_list(info, head, tail, 0, info->sw_rx_bhdr_start);
	}
#if defined(ARTHUR_WIFI2ETH)
	else if (!strcmp(argv[1], "etxq"))
	{
		diag_printf("eth_tx_descr_count=%d, free=%d\n", info->eth_txq.max, info->eth_txq.free);
		diag_dump_buf_32bit(info->eth_txq.desc_hdr, info->eth_txq.max*16);
	}
	else if (!strcmp(argv[1], "erxq"))
	{
		diag_printf("eth_rx_descr_count=%d\n", info->eth_rxq.max);
		diag_dump_buf_32bit(info->eth_rxq.desc_hdr, info->eth_rxq.max*16);
	}
	else if(!strcmp(argv[1], "stat"))
	{
		unsigned int statistic1, statistic2, e2w, w2e, jam, acbusy, err;
		statistic1 = MACREG_READ32(ETH_STATISTIC_1);
		statistic2 = MACREG_READ32(ETH_STATISTIC_2);
		e2w = (statistic1 & ETH_TOTAL_E2W) >> 16; 
		w2e = statistic1 & ETH_TOTAL_W2E; 
		jam = (statistic2 & ETH_TOTAL_JAM) >> 21;
		acbusy = (statistic2 & ETH_TOTAL_ACBUSY) >> 10;
		err = statistic2 & ETH_TOTAL_FCS;

		/* Reset W2E statistics */
		MACREG_WRITE32(ETH_STATISTIC_1, ETH_TOTAL_E2W_CLR|ETH_TOTAL_W2E_CLR);
		MACREG_WRITE32(ETH_STATISTIC_2, ETH_TOTAL_JAM_CLR|ETH_TOTAL_ACBUSY_CLR|ETH_TOTAL_FCS_CLR);

		printd("====== packet statistics ======\n");
		printd("e2w=(%d)\n", e2w);
		printd("w2e=(%d)\n", w2e);
		printd("jam=(%d)\n", jam);
		printd("acbusy=(%d)\n", acbusy);
		printd("err=(%d)\n", err);
	}
#endif
	else if(!strcmp(argv[1], "gkey"))
	{	
		int tbl_idx = 0;
		if(argc == 3)
			tbl_idx = atoi(argv[2]); 

		diag_printf("group_keys[%d]\n", tbl_idx);
		diag_dump_buf_32bit(&info->group_keys[tbl_idx], sizeof(group_cipher_key));
	}
	else if(!strcmp(argv[1], "pkey"))
	{
		int tbl_idx = 0;
		if(argc == 3)
			tbl_idx = atoi(argv[2]); 
		diag_printf("private_keys[%d]\n", tbl_idx);
		diag_dump_buf_32bit(&info->private_keys[tbl_idx], sizeof(cipher_key));
	}
#if 0
	else if(!strcmp(argv[1], "sta_gkey"))
	{
		//int tbl_idx = 0;
		diag_printf("sta_group_keys\n");
		diag_dump_buf_16bit(&info->sta_group_keys->group_key, sizeof(cipher_key));
	}
#endif
	else if(!strcmp(argv[1], "sta"))
	{
		int tbl_idx = 0;
		if(argc == 3)
		{
			tbl_idx = atoi(argv[2]); 
			dump_sta(info, tbl_idx, 0);
		}
		else
		{
			for(; tbl_idx < info->sta_cap_tbl_count; tbl_idx++)
			{
				if(1 == info->sta_cap_tbls[tbl_idx].valid)
				{
					dump_sta(info, tbl_idx, 0);
				}
			}
		}
	}
	else if(!strcmp(argv[1], "rate"))
	{
		int tbl_idx = 0;
		if(argc == 3)
		{
			tbl_idx = atoi(argv[2]); 
			dump_sta(info, tbl_idx, 1);
		}
		else
		{
			for(; tbl_idx < info->rate_tbl_count; tbl_idx++)
			{
				if(1 == info->rate_tbls[tbl_idx].valid)
				{
					dump_sta(info, tbl_idx, 1);
				}
			}
		}	
	}
	else if(!strcmp(argv[1], "wcb"))
	{
		int tbl_idx = 0;
		if(argc == 3)
			tbl_idx = atoi(argv[2]); 
		dump_wcb(info, tbl_idx);	
	}
	else if(!strcmp(argv[1], "bcap"))
	{
		int tbl_idx = 0;
		if(argc >= 3)
		{
			tbl_idx = atoi(argv[2]);
			if(argc == 4)
			{
				int bcap_val;
				sscanf(argv[3], "%x", &bcap_val);
				mac_addr_lookup_engine_update(info, tbl_idx, bcap_val, BY_ADDR_IDX);
			}
			dump_bcap(info, tbl_idx, 0);
		}
		else
		{	 
			/* dump all */
			for(i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
			{
				dump_bcap(info, i, 0);
				diag_dump_buf_16bit(&((sta_addr_entry *)info->ext_sta_tbls)[i],
					sizeof(sta_addr_entry));
			}
		}	
	}
	else if(!strcmp(argv[1], "ds"))
	{
		int tbl_idx = 0;
		if(argc >= 3)
		{
			tbl_idx = atoi(argv[2]);
			if(argc == 4)
			{
				int bcap_val;
				sscanf(argv[3], "%x", &bcap_val);
				mac_addr_lookup_engine_update(info, tbl_idx, bcap_val, BY_ADDR_IDX|IN_DS_TBL);
			}
			dump_bcap(info, tbl_idx, 1);
		}
		else
		{
			/* dump all */
			for(i=0; i < MAX_DS_TABLE_ENTRIES; i++)
			{
				dump_bcap(info, i, 1);
				diag_dump_buf_16bit(&((sta_addr_entry *)info->ext_ds_tbls)[i],
					sizeof(sta_addr_entry));
			}
		}
	}
	else if(!strcmp(argv[1], "acache"))
	{
		dump_addr_cache(info);
	}
	else if(!strcmp(argv[1], "meter"))
	{
		if(!strcmp(argv[2], "erx"))
		{
			unsigned int rate;
			unsigned int now = os_current_time();

			rate = (info->sw_e2w_unicast_pkt_bytes * 8)/(now - info->erx_last_sample_time)*100/1000;
			info->sw_e2w_unicast_pkt_bytes = 0;
			info->erx_last_sample_time = now;
			printd("%d kbps\n", rate);
		}
	}
	else if(!strcmp(argv[1], "ba"))
	{
		int tbl_idx = 0;
		if(argc == 3)
			tbl_idx = atoi(argv[2]); 
		dump_ba(info, tbl_idx);	
	}
	else if(!strcmp(argv[1], "bcq"))
	{
		dump_bc_qinfo(info);	
	}
	else if(!strcmp(argv[1], "mmacd"))
	{
		printf("Total number: %d\n",  info->mmac_rx_descr_count);
		for(i=0; i<info->mmac_rx_descr_count; i++)
		{
			printf("###[%d]###\n", i);
			if(info->mmac_rx_descriptors[i].next_index == 0)
				printf("????????????????????wrong next_index\n");
			diag_dump_buf_32bit(&info->mmac_rx_descriptors[i], sizeof(mmac_rx_descriptor));
		}
	}
	else if(!strcmp(argv[1], "psbaq"))
	{
		printf("Total number: %d\n",  info->psbaq_descr_count);
		for(i=0; i<info->psbaq_descr_count; i++)
		{
			printf("###[%d]###\n", i);
			diag_dump_buf_32bit(&info->psbaq_descriptors[i], sizeof(psbaq_descriptor));
		}
	}
	else if(!strcmp(argv[1], "dump"))
	{
		int val = 0;

		if(argc != 4)
			return CLI_ERROR;

		val = atoi(argv[3]); 

		if(!strcmp(argv[2], "wrx"))
			info->w_rx_dump = val;
		else if(!strcmp(argv[2], "wtx"))
			info->w_tx_dump = val;
		else if(!strcmp(argv[2], "erx"))
			info->w2e_rx_dump = val;
		else if(!strcmp(argv[2], "etx"))
			info->w2e_tx_dump = val;
	}
	else if(!strcmp(argv[1], "rxfilter"))
	{
		info->filter = atoi(argv[2]); 
	}
	else if(!strcmp(argv[1], "no_kick"))
	{
		info->no_kick_reorder_buf = atoi(argv[2]); 
	}
	else if(!strcmp(argv[1], "bb_rst"))
	{
		bb_reset();
	}
	else if(!strcmp(argv[1], "acq"))
	{
		if(argc == 4)
		{
			info->acq_len[atoi(argv[2])] = atoi(argv[3]); 
			info->fix_ac_queue_limit = 1;
			MACREG_WRITE32(FC_ACQ_THRESHOLD, ((info->acq_len[ACQ_BK_IDX] << 24) | (info->acq_len[ACQ_BE_IDX] << 16) | (info->acq_len[ACQ_VI_IDX] << 8) | info->acq_len[ACQ_VO_IDX]));
		}
	}
	else if(!strcmp(argv[1], "txrate"))
	{
		short j;
		for(i=0; i<MAX_BSSIDS; i++)
		{
			diag_printf("MBSS[%d]\n", i);
			for(j=0;j<3;j++)
			{
				diag_printf("    TX RATE[%d] idx:%d, rate idx=%x\n", j, info->mbss[i].tx_rate[j].rate_captbl, info->mbss[i].tx_rate[j].rate_idx);
			}
		}
	}
	else if(!strcmp(argv[1], "polling"))
	{
		info->wmac_polling = atoi(argv[2]); 
	}
	else if(!strcmp(argv[1], "sn_gap"))
	{
		info->ampdu_sn_gap = atoi(argv[2]); 
	}
#if defined(WLAN_RC_BTS)
	else if(!strcmp(argv[1], "rc"))
	{
		int tbl_idx = 0;
		if((argc == 4) && !strcmp(argv[2], "dbg"))
			info->rc_debug = atoi(argv[3]);
		else if(argc == 3) 
			tbl_idx = atoi(argv[2]);
		dump_bts(info, tbl_idx);	
	}
#endif
	else if(!strcmp(argv[1], "pm"))
	{
		sta_cap_tbl *captbl;
		int tbl_idx, pm;
		
		if(argc == 4)
		{
			tbl_idx = atoi(argv[2]); 
			pm = atoi(argv[3]);
			
			captbl = &info->sta_cap_tbls[tbl_idx];
			captbl->power_saving = pm;
			mac_stacap_cache_sync(tbl_idx, true);
		}
	
	}
	else if(!strcmp(argv[1], "rts"))
	{
		sta_cap_tbl *captbl;
		int tbl_idx, rts;
		
		if(argc == 4)
		{
			tbl_idx = atoi(argv[2]); 
			rts = atoi(argv[3]);

			captbl = &info->sta_cap_tbls[tbl_idx];
			captbl->force_rts = rts;
			mac_stacap_cache_sync(tbl_idx, true);
		}
	
	}
	else if(!strcmp(argv[1], "t_psq"))
	{
		int tbl_idx, type, cmd;
		
		if(argc == 4)
		{
			tbl_idx = atoi(argv[2]); 
			type = atoi(argv[3]);
			
			if(type == 2)
			{
				printf("sta aging..\n");
				MACREG_WRITE32(PSBAQ_QINFO_CSR0, 0);
	MACREG_WRITE32(PSBAQ_QINFO_CSR2, (tbl_idx << 8) | PSBAQ_QINFO_CSR2_CMD_STA_AGING | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
			}
			else
			{
				if(type == 1)
					cmd = (tbl_idx&0xff) | TRIG_TYPE | TRIG_REQUEST;
				else
					cmd = (tbl_idx&0xff) | TRIG_REQUEST;
				printf("cmd=%x\n");
				MACREG_WRITE32(PSQ_TRIGGER, cmd);
			}
		}
	
	}
#if defined(SW_BUF_TRACE)
	else if(!strcmp(argv[1], "sw_buf_trace"))
	{
		for(i=0; i<info->sw_buf_headers_count; i++)
			printf("[%d] = %d\n", i, info->sw_buf_trace[i]);
	}
#endif
#if defined(CONFIG_WMAC_RECOVER)
	else if(!strcmp(argv[1], "recover"))
	{
		info->mac_recover = 1;
	}
#endif

	return CLI_OK;
}

CLI_CMD(wd, wla_debug_cmd, "wd(wlan debug)\tinfo : all info\n"
						"\tumac : umac reg\n"
						"\tlmac : lmac reg\n"
						"\ttxq : tx desc\n"
						"\twrxq : rx desc\n"
						"\tretq : return-q desc\n"
						"\tfretq : fast-path return-q desc\n"
						"\tswbuf : software buffer\n"
						"\thwbuf : hardware buffer\n"
#if defined(ARTHUR_WIFI2ETH)
						"\tetxq :  eth tx desc\n"
						"\terxq : eth rx desc\n"
						"\tstat : packet statistics(hw)"
#endif
						"\tgkey idx] : group key\n"
						"\tpkey [idx] : pairwise key\n"
						"\tsta [idx] : sta capbility\n"
						"\trate [idx] : rate table\n"
						"\tbcap [idx] : basic capbility\n"
						"\tds [idx] : basic ds capbility\n"
						"\tba [idx] : ba session\n"
						"\tbcq : broadcast queue\n"
						"\twcb [idx] : wcb\n"
						"\tmeter erx\n"
						"\tmmacd : mmac rx descriptors\n"
						"\tpsbaq : psba queue descriptors\n"
						"\tdump [wrx/wtx/erx/etx/arfc/dc] [0/1]\n"
						"\trxfilter : set wlan rx filter\n"
						"\tno_kick : no kick reorder buffer\n"
						"\tbb_rst : reset BB\n"
						"\ttxrate : show MBSS tx rate\n"
						"\tacq [idx] [len] : static AC queue length\n"
#if defined(WLAN_RC_BTS)
						"\trc [idx]/dbg [level] : BTS RC\n"
#endif
						"\tt_psq [idx] [type]: Trigger PSQ\n"
						"\tsw_buf_trace\n"
#if defined(CONFIG_WMAC_RECOVER)
						"\trecover : recover mac\n"
#endif
						, 0);

static int bb_cmd(int argc, char* argv[])
{
    unsigned char buf[256];
    int param[] = { 0, 0, 0, 0, 0, 0};
    unsigned char value74,value75,value76;
    unsigned long value;
    int i;

    if(argc >= 2) 
    {
	if(!strcmp("rxpeon", argv[1]))
	{
	    bb_rxpe_on();
	}
	else if(!strcmp("rxpeoff", argv[1]))
	{
	    bb_rxpe_off();
	}
	else if(!strcmp("txpeon", argv[1]))
	{
	    bb_txpe_on();
	}
	else if(!strcmp("txpeoff", argv[1]))
	{
	    bb_txpe_off();
	}
    else if(!strcmp("memdump", argv[1])) 
    {
        printf("Waiting BR71 trigger finish...");
        while(1)
        {    
            value = bb_register_read(0x71);

            if((value & 0x2)) 
                break;
        }
        printf("Done\n");

        printf("--------------------------DUMP START--------------------------\n");

        for(i=0;i<65536;i++) 
        {
            value74 = bb_register_read(0x74);
            //printf(" %02x", value);
            value75 = bb_register_read(0x75);
            //printf(" %02x", value);
            value76 = bb_register_read(0x76);
            //printf(" %02x\n", value);

            value = ((value74 << 16) | (value75 << 8) | value76);
            printf("%d\n", value & 0x00ffffff);
        }

        printf("--------------------------DUMP  DONE--------------------------\n");
        return CLI_OK;
    }
	else if(!strcmp("alldump", argv[1])) 
	{
		for(i=0; i<0xff; i++)
		{
			value = bb_register_read(i);
			printf("1-%02x-%02x\n", i, (unsigned long) value);
		}
	}
	else if(!strcmp("agcdump", argv[1])) 
	{
		for(i=0; i<17; i++)
		{
			bb_register_write(0x10, i);
			value = bb_register_read(0x11);
			printf("1-10-%02x\n", i);
			printf("1-11-%02x\n", value);
		}
	}
	else if(!strcmp("spitest", argv[1])) 
	{
		int loop = 10000;
		char reg = 0x2;
		char pattern, val;
		int count = 0;

		if(argc == 3)
			reg = atoi(argv[2]);

		if(argc == 4)
			loop = atoi(argv[3]);
	
		while(count < loop )
		{
			if(count & 1)
				pattern = 0x5a;
			else
				pattern = 0xa5;
			bb_register_write(reg, pattern);
			val = bb_register_read(reg) & 0xff;
			if(val != pattern)
			{
				printf("%dth test fail??????, write %02x to REG%02x, read %02x\n", count, pattern, reg, val);
				break;
			}
			else
				printf("%dth test pass\n", count);
			count++;
		}
	}
	else
	{
	    printf("Usage: bb rxpeon   (enable RXPE)\n"
		   "       bb rxpeoff  (disable RXPE)\n"
		   "       bb txpeon   (enable TXPE)\n"
		   "       bb txpeoff  (disable TXPE)\n"
           "       bb memdump  (dump internal memory)\n"
		   "       bb alldump  (dump all register)\n"
		   "       bb agcdump  (dump AGC table)\n"
		   "       bb spitest [REG] [loop] (test BB SPI read/write)\n");
	}
	return CLI_OK;
    }

    do
    {
	memset(buf, 0, sizeof(buf));
	gets(buf);


	if(buf[0]!=0) 
	{
	    /* XXX:  sscanf(buf, "%x-%x-%x") is buggy, DIY! */

	    param[0] = param[1] = param[2] = 0;
	    get_parameters(buf, param);

	    //printf("-->(%x %x %x)\n", param[0], param[1], param[2]);
	    if(param[0] == 0) 
	    {
		value = bb_register_read(param[1]);
		printf("read BB register 0x%x, value 0x%x\n", param[1], (unsigned long) value);
	    }
	    else if(param[0] == 1) 
	    {
		bb_register_write(param[1], (unsigned char) param[2]);
		value = bb_register_read(param[1]);
		printf("write BB register 0x%x, value 0x%x, result 0x%x\n", param[1], param[2], (unsigned long) value);
	    }
	}

	//printf("---> %s\n", buf);
    } while(buf[0] != 0);

    return CLI_OK;
}

CLI_CMD(bb, bb_cmd, "bb", 0);

static int rf_cmd(int argc, char* argv[])
{
    if(argc >= 2) 
    {
		if(!strcmp("init", argv[1])) 
		{
            printf("init RF to 11g mode\n");
            rf_init();
		}
		else if(!strcmp("channel", argv[1]))
        {
            printf("RF switch to 2.4GHZ channel %d\n", atoi(argv[2]));
            rf_set_channel(0, atoi(argv[2]));
        }
		else if(!strcmp("read", argv[1]))
        {
			int v;
			if(argc != 3)
				goto Usage;
			sscanf(argv[2], "%x", &v);
            printf("read reg %x=0x%x\n", v, rf_read(v));
        }
		else if(!strcmp("write", argv[1]))
        {
			int v, v1;
			if(argc != 4)
				goto Usage;
			sscanf(argv[2], "%x", &v);
			sscanf(argv[3], "%x", &v1);
			printf("write reg %d=0x%x\n", v, v1);
			rf_write(v, v1);
        }
		else if(!strcmp("readall", argv[1]))
        {
			int v;
			for(v=0; v<=0x1f; v++)
            	printf("read reg %x=0x%x\n", v, rf_read(v));
        }
		else if(!strcmp("spi_idx", argv[1])) 
		{
			if(argc != 3)
            	printf("%d\n", my_rf_dev->spi_idx);
			else
            	my_rf_dev->spi_idx = atoi(argv[2]);
		}
#ifdef RFAC_ANALYSIS
		else if(!strcmp("rfac", argv[1]))
		{
			rfac_cmd(argc, argv);
		}
#endif
#if defined(CONFIG_RFC_ANALYST) && defined(RFC_DEBUG)
		else if(!strcmp("rfc_ht_new", argv[1]))
		{
			rfc_ht_cmd_new(argc, argv);
		}
		else if(!strcmp("config_rfc_parm_new", argv[1]))
		{
			if(argc != 3)
				goto Usage;
			config_rfc_parm_new(atoi(argv[2]));
		}
		else if(!strcmp("rxvga_adjust", argv[1]))
		{
			if(argc != 4)
				goto Usage;
			rxvga_adjust(atoi(argv[2]), atoi(argv[3]));
		}
#endif
        else
        {
            goto Usage;
        }
	/* gpio 5 multiplex airoha LE pin and external mdc, 
	so we have to turn off GPIO function after RF configuration completed  */
	//airoha_crtl_stop();

        return CLI_OK;
    }

Usage:    

    printf("Usage: rf init\n"
           "       rf channel <1~14>\n"
		   "       rf read <reg>\n"
		   "       rf write <reg> <value>\n"
		   "       rf readall\n"
		   "       rf spi_idx <idx>\n"	
#ifdef RFAC_ANALYSIS
		   "       rf rfac <loop>\n"
#endif
#if defined(CONFIG_RFC_ANALYST) && defined(RFC_DEBUG)
		   "       rf rfc_ht_new <loop> <tone_mask> <bw> <debug> <rx_scale> <tx_scale> <rx_txvga> <tx_txvga>(bw: bit.0:20MHz, bit.1:40MHz)\n"
		   "       rf txlo_cal <sel> (sel=0: original version, sel=1: light version)\n"
		   "       rf rxvga_adjust <ovth> <okth> (ovth: 110, okth:85)\n"
		   "       rf config_rfc_parm_new <bw> (bw: 0=20mhz, 1=40mhz)\n"
#endif
		   );
    return CLI_OK;
}


CLI_CMD(rf, rf_cmd, "rf", 0);
#if 0
static int set_wifi_cmd(int argc, char* argv[])
{
	if(argc < 2)
		return CLI_SHOW_USAGE;

	switch(atoi(argv[1]))
	{
		case 6:
			/* set param for board 6 */

			/* Set RF param */	
			rf_write(06, 0x36df);
			rf_write(00, 0x3e954);
			rf_write(0x11, 0x15);
			rf_write(07, 0x65f8);

			printf("write RF register 0x06, value 0x036df, result 0x%05x\n", rf_read(0x06));
			printf("write RF register 0x00, value 0x3e954, result 0x%05x\n", rf_read(0x00));
			printf("write RF register 0x11, value 0x00015, result 0x%05x\n", rf_read(0x11));
			printf("write RF register 0x07, value 0x065f8, result 0x%05x\n", rf_read(0x07));

			/* Set BB param */
			bb_register_write(0x02, 0x38);
			bb_register_write(0xF2, 0x98);
			bb_register_write(0x20, 1);
			bb_register_write(0x21, 0x7f);
			bb_register_write(0x22, 0xee);
			bb_register_write(0x23, 0);
			bb_register_write(0x24, 0x70);
			bb_register_write(0x25, 0);
			bb_register_write(0x26, 0x7f);
			bb_register_write(0x27, 0);
			bb_register_write(0x28, 7);
			bb_register_write(0x29, 0x45);
			bb_register_write(0x2A, 0);
			printf("write BB register 0x02, value 0x38, result 0x%x\n", bb_register_read(0x02));
			printf("write BB register 0xF2, value 0x98, result 0x%x\n", bb_register_read(0xF2));
			printf("write BB register 0x20, value 0x01, result 0x%x\n", bb_register_read(0x20));
			printf("write BB register 0x21, value 0x7f, result 0x%x\n", bb_register_read(0x21));
			printf("write BB register 0x22, value 0xee, result 0x%x\n", bb_register_read(0x22));
			printf("write BB register 0x23, value 0x00, result 0x%x\n", bb_register_read(0x23));
			printf("write BB register 0x24, value 0x70, result 0x%x\n", bb_register_read(0x24));
			printf("write BB register 0x25, value 0x00, result 0x%x\n", bb_register_read(0x25));
			printf("write BB register 0x26, value 0x7f, result 0x%x\n", bb_register_read(0x26));
			printf("write BB register 0x27, value 0x00, result 0x%x\n", bb_register_read(0x27));
			printf("write BB register 0x28, value 0x07, result 0x%x\n", bb_register_read(0x28));
			printf("write BB register 0x29, value 0x45, result 0x%x\n", bb_register_read(0x29));
			printf("write BB register 0x2A, value 0x00, result 0x%x\n", bb_register_read(0x2A));
			break;
		case 7:
			/* set param for board 7 */
			rf_write(06, 0x36df);
			rf_write(00, 0x3e954);
			rf_write(0x11, 0x15);
			rf_write(07, 0x65f8);

			printf("write RF register 0x06, value 0x036df, result 0x%05x\n", rf_read(0x06));
			printf("write RF register 0x00, value 0x3e954, result 0x%05x\n", rf_read(0x00));
			printf("write RF register 0x11, value 0x00015, result 0x%05x\n", rf_read(0x11));
			printf("write RF register 0x07, value 0x065f8, result 0x%05x\n", rf_read(0x07));

			/* Set BB param */
			bb_register_write(0x02, 0x38);
			bb_register_write(0xF2, 0x98);
			bb_register_write(0x20, 1);
			bb_register_write(0x21, 0x7f);
			bb_register_write(0x22, 0xee);
			bb_register_write(0x23, 0);
			bb_register_write(0x24, 0x6b);
			bb_register_write(0x25, 0);
			bb_register_write(0x26, 0x7f);
			bb_register_write(0x27, 0);
			bb_register_write(0x28, 8);
			bb_register_write(0x29, 0x44);
			bb_register_write(0x2A, 0);
			printf("write BB register 0x02, value 0x38, result 0x%x\n", bb_register_read(0x02));
			printf("write BB register 0xF2, value 0x98, result 0x%x\n", bb_register_read(0xF2));
			printf("write BB register 0x20, value 0x01, result 0x%x\n", bb_register_read(0x20));
			printf("write BB register 0x21, value 0x7f, result 0x%x\n", bb_register_read(0x21));
			printf("write BB register 0x22, value 0xee, result 0x%x\n", bb_register_read(0x22));
			printf("write BB register 0x23, value 0x00, result 0x%x\n", bb_register_read(0x23));
			printf("write BB register 0x24, value 0x6b, result 0x%x\n", bb_register_read(0x24));
			printf("write BB register 0x25, value 0x00, result 0x%x\n", bb_register_read(0x25));
			printf("write BB register 0x26, value 0x7f, result 0x%x\n", bb_register_read(0x26));
			printf("write BB register 0x27, value 0x00, result 0x%x\n", bb_register_read(0x27));
			printf("write BB register 0x28, value 0x08, result 0x%x\n", bb_register_read(0x28));
			printf("write BB register 0x29, value 0x44, result 0x%x\n", bb_register_read(0x29));
			printf("write BB register 0x2A, value 0x00, result 0x%x\n", bb_register_read(0x2A));
			break;
		default:
			return CLI_SHOW_USAGE;
	}
    return CLI_OK;
}

CLI_CMD(set_wifi, set_wifi_cmd, "set_wifi board_no", 0);
#endif
#ifdef CONFIG_BB_TEST
extern void bb_test_init(void);
CLI_CMD(bb_test, (void *)bb_test_init, "bb_test", 0);
#endif

#ifdef DEBUG_TPUT
void tput_info(MAC_INFO *info)
{
	if(info->tput_enable)
	{
		int i, dp_cnt;
		for(i=0; i<TPUT_DP_NUM; i++)
		{
			dp_cnt = info->tput_dp_cnt[i];
			if(dp_cnt < TPUT_DP_MAX)
			{
				info->tput_dp_arr[i][dp_cnt++] = info->tput_dp[TPUT_DP_NUM] - info->tput_dp[i];
				info->tput_dp_cnt[i] = dp_cnt;
			}
		}
	}
}

static int net_tput_cmd(int argc, char *argv[])
{
	if (2 <= argc)
	{
		MAC_INFO *info = &my_arthur_dev.mac_info;
		if (!strcmp(argv[1], "w2e"))
		{
			info->tput_w2e=1;
			info->tput_enable=1;
			return CLI_OK;
		}
		else if (!strcmp(argv[1], "e2w"))
		{
			info->tput_w2e=0;
			info->tput_enable=1;
			return CLI_OK;
		}
		else if (!strcmp(argv[1], "stop"))
		{
			int i;
			info->tput_enable = 0;
			for(i=0; i<TPUT_DP_NUM; i++)
			{
				info->tput_dp[i] = 0;
				info->tput_dp_cnt[i] = 0;
			}
			printd("\ndiag point data\n");
			for(i=0; i<TPUT_DP_MAX; i++)
			{
				printd("%d %d %d %d %d %d\n", i+1, info->tput_dp_arr[0][i], info->tput_dp_arr[1][i], info->tput_dp_arr[2][i], info->tput_dp_arr[3][i],info->tput_dp_arr[4][i]);
			}
			printd("\n");
			memset(&info->tput_dp_arr[0], 0, sizeof(unsigned int)*TPUT_DP_NUM*TPUT_DP_MAX);
			return CLI_OK;
		}
	}
	return CLI_SHOW_USAGE;
}
CLI_CMD(tput, net_tput_cmd, "tput w2e/e2w/stop", 0);
#endif
