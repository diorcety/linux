/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file
*   \brief
*   \author Montage
*/
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/synclink.h>
#include <net/mac80211.h>
#include <os_compat.h>
#include <rf.h>
#include <ip301.h>
#include <rfc.h>
#include <wla_bb.h>
#include <linux_wla.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/common.h>
#include <wlan_def.h>
#include <wla_mat.h>

//#include <asm/bug.h>

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
#include <asm/mach-cheetah/idb.h>
#endif

#undef TKIP_COUNTERMEASURE

extern struct macaddr_pool maddr_tables;
extern char *curfunc;
extern struct mat_entry *(mat_hash_tbl[MAT_HASH_TABLE_SIZE]);

static char buf[300];
#define MAX_ARGV 8

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
static int wl_print(struct seq_file *m, const char *fmt, ...)
{
	va_list args;
	int len;
	int r;

	if(!m) {
		va_start(args, fmt);
		r = vprintk(fmt, args);
		va_end(args);
	}
	else {
		if (m->count < m->size) {
			va_start(args, fmt);
			len = vsnprintf(m->buf + m->count, m->size - m->count, fmt, args);
			va_end(args);
			if (m->count + len < m->size) {
				m->count += len;
			return 0;
			}
		}
		m->count = m->size;
		return -1;
	}
	return r;
}

void idb_dump_buf_32bit(struct seq_file *m, unsigned int *p, u32 s)
{
	int i;
	while ((int)s > 0) {
	    for (i = 0;  i < 4;  i++) {
	        if (i < (int)s/4) {
				wl_print(m, "%08X ",p[i]);
	        } else {
				wl_print(m, "         ");
	        }
	    }
		wl_print(m, "\n");
	    s -= 16;
	    p += 4;
	}
}

void idb_dump_buf_16bit(struct seq_file *m, unsigned short *p, u32 s)
{
	int i;
	while ((int)s > 0) {
	    for (i = 0;  i < 8;  i++) {
	        if (i < (int)s/2) {
	      wl_print(m, "%04X ", p[i] );
	      if (i == 3) wl_print(m, " ");
	        } else {
	      wl_print(m, "     ");
	        }
	    }
	    wl_print(m, "\n");
	    s -= 16;
	    p += 8;
	}
}

void idb_dump_buf_8bit(struct seq_file *m, unsigned char *p, u32 s)
{
	int i, c;
	while ((int)s > 0) {
	    for (i = 0;  i < 16;  i++) {
	        if (i < (int)s) {
	            wl_print(m, "%02X ", p[i] & 0xFF);
	        } else {
	            wl_print(m, "   ");
	        }
	    if (i == 7) wl_print(m, " ");
	    }
	    wl_print(m, " |");
	    for (i = 0;  i < 16;  i++) {
	        if (i < (int)s) {
	            c = p[i] & 0xFF;
	            if ((c < 0x20) || (c >= 0x7F)) c = '.';
	        } else {
	            c = ' ';
	        }
	        wl_print(m, "%c", c);
	    }
	    wl_print(m, "|\n");
	    s -= 16;
	    p += 16;
	}
}

void reg_dump(struct seq_file *s, unsigned char *addr,int size)
{
    unsigned char *caddr;

    caddr = addr;
    while (caddr < (addr + size))
    {
        if ((((int)caddr % 16)== 0) || caddr == addr)
            wl_print(s, "\n%08x: ", (unsigned int)caddr);

        wl_print(s, "%08x ", *((volatile unsigned int *) caddr));
        caddr += 4;
    }
}

void dump_list(struct seq_file *s, MAC_INFO* info, short head, short tail, short low, short high)
{
	u32 i;
	buf_header *bhdr;
	u8 free[FASTPATH_BUFFER_HEADER_POOL_SIZE+SW_BUFFER_HEADER_POOL_SIZE];
	
	wl_print(s, "******freelist:%d, %d\n", head, tail);

	while(head != tail)
	{
		bhdr = idx_to_bhdr(info, head);
		wl_print(s, "->[%d]\n", head);
		idb_dump_buf_32bit(s, (unsigned int *)bhdr, 8);
		
		if((head < low) || (head >=high))
			wl_print(s, "!!!!!!!!!!!!!!illegal Range!!!!!!!!!!!!\n");
		free[head] = 1;
		head = bhdr->next_index; 
	}
	wl_print(s, "******used:\n");
	
	for(i=low; i<high; i++)
	{
		if(!free[i])
		{
			wl_print(s, "??[%d]\n", i);
			idb_dump_buf_32bit(s, (unsigned int *)idx_to_bhdr(info, i), 8);	
		}
	}
}

void dump_bts(struct seq_file *s, MAC_INFO* info, int sta_idx)
{
	struct wsta_cb *cb;
	struct bts_cb *bcb;
	struct bts_rate *br;
	u32 i;

	cb = info->wcb_tbls[sta_idx];
	if(cb == 0)
		return;
	bcb = &cb->bts;

	for(i=0; i<4; i++)
	{
	    wl_print(s, "[%d|%d] ", cb->cur_tx_rate[i].rate_idx, cb->cur_tx_rate[i].bts_rates);
	}
	wl_print(s, "\n[ridx]      TP Prob        TS   success/attemp\n");
	for(i=0; i<bcb->rates_len; i++)
	{
	    br = &bcb->rates[i];
	    wl_print(s, "[%2d] %9d  %3d %10d  %4d/%-4d\n", br->rate_idx, br->tp, br->avg_prob, br->timestamp, br->success, br->attempts);
	}

}

void dump_bc_qinfo(struct seq_file *s, MAC_INFO* info)
{
	bcast_qinfo *qinfos = info->bcast_qinfos;
	int i;

	for(i=0; i<MAC_MAX_BSSIDS; i++)
	{
		wl_print(s, "BSS:%d\n", i);
		idb_dump_buf_16bit(s, (unsigned short *)&qinfos[i], sizeof(bcast_qinfo));
	}
}

void dump_addr_cache(struct seq_file *s, MAC_INFO* info)
{
	int idx, val;
	for(idx = 0; idx < 32; idx++)
	{
		MACREG_WRITE32(0x86c, (idx << 2)|0x1);
		val = MACREG_READ32(0x86c);
		wl_print(s, "[%d], val=%x\n", idx, val);
	}
}

void dump_sta(struct seq_file *s, MAC_INFO* info, int sta_idx, char rate_tbl)
{
	int i;
	sta_cap_tbl *captbl;
	sta_rx_status_tbl *rx;
	sta_tx_status_tbl *tx;

	if(!rate_tbl)
		captbl = &info->sta_cap_tbls[sta_idx];
	else
		captbl = &info->rate_tbls[sta_idx];
	
	wl_print(s, "sta_idx=%d, 0x%x\n", sta_idx, (unsigned int)captbl);
	mac_stacap_cache_sync(sta_idx, true);
	mac_rx_linkinfo_cache_sync();

	wl_print(s, "\tV:%d NE:%d BG:%d PS:%d AP:%d AMSDU:%d Short-Preamble:%d QOS:%d PSMP:%d gsn:%d, def_key:%d cipher_mode:%d essid:%d bssid:%d parent_ap:%d, force_rts:%d\n", captbl->valid, captbl->NE,  captbl->BG, captbl->power_saving, captbl->AP, captbl->AMSDU, captbl->short_preamble, 	captbl->QOS, captbl->PSMP, captbl->gsn, captbl->wep_defkey, captbl->cipher_mode,  captbl->ess_idx, captbl->bssid, captbl->parent_ap, captbl->force_rts);
	wl_print(s, "\tRX_tbl_addr:%x, TX_tbl_addr:%x\n", captbl->RX_tbl_addr, captbl->TX_tbl_addr);
	wl_print(s, "\tglong_retry:%d, gshort_retry:%d, gack_policy:%d\n", captbl->glong_retry, captbl->gshort_retry, captbl->gack_policy);
	wl_print(s, "\trate=0x%x, 0x%x, 0x%x, 0x%x\n", captbl->rate[0].val, captbl->rate[1].val, captbl->rate[2].val, captbl->rate[3].val);
	wl_print(s, "\ttx_ok_rate=%d, %d, %d, %d\n", captbl->tx_ok_rate[0], captbl->tx_ok_rate[1], captbl->tx_ok_rate[2], captbl->tx_ok_rate[3]);	
	wl_print(s, "\tx_try_cnt0=%d, tx_try_cnt1=%d, tx_fail_cnt=%d, tx_try_cnt2=%d\n",  captbl->tx_try_cnt0,  captbl->tx_try_cnt1, captbl->tx_fail_cnt,  captbl->tx_try_cnt2);
	wl_print(s, "\trx_byte_cnt=%d, rx_rate=0x%x, rssi=0x%x, rx_pkt_cnt=%d\n",  captbl->rx_byte_cnt, captbl->rx_rate, captbl->rssi, captbl->rx_pkt_cnt);

	if(captbl->RX_tbl_addr)
	{
		rx = (sta_rx_status_tbl *)NONCACHED_ADDR(captbl->RX_tbl_addr);
	
		if(!captbl->NE)
		{
			wl_print(s, "###RX_tbl_addr###, size=%d\n", sizeof(sta_rx_status_tbl));
			idb_dump_buf_8bit(s, (unsigned char *)rx, sizeof(sta_rx_status_tbl));
		}
		else
		{
			sta_rx_status_tbl *rx = (sta_rx_status_tbl *)NONCACHED_ADDR(captbl->RX_tbl_addr);
			for(i=0; i<QOS_RX_STATUS_TBLS; i++)
			{
				wl_print(s, "###RX_tbl_addr [%d]###, size=%d\n", i, sizeof(sta_rx_status_tbl));
				idb_dump_buf_8bit(s, (unsigned char *)&rx[i], sizeof(sta_rx_status_tbl));
			}	
		}
	}

	if(captbl->TX_tbl_addr)
	{
		if(!captbl->QOS)
		{
			wl_print(s, "###TX_tbl_addr###, size=%d\n", sizeof(sta_tx_status_tbl));
			idb_dump_buf_8bit(s, (void*)NONCACHED_ADDR(captbl->TX_tbl_addr), sizeof(sta_tx_status_tbl));
		}
		else
		{
			tx = (sta_tx_status_tbl *)NONCACHED_ADDR(captbl->TX_tbl_addr);

			for(i=0; i<QOS_TX_STATUS_TBLS; i++)
			{
				wl_print(s, "###TX_tbl_addr [%d]###, size=%d\n", i, sizeof(sta_tx_status_tbl));
				idb_dump_buf_8bit(s, (unsigned char *)(tx+i), sizeof(sta_tx_status_tbl));
			}	
		}
	}
}

void dump_wcb(struct seq_file *s, MAC_INFO* info, int sta_idx)
{
	struct wsta_cb *cb;

	cb = info->wcb_tbls[sta_idx];
	
	if(cb == 0)
		return;

	wl_print(s, "WCB:0x%x\n", (unsigned int)cb); 
	idb_dump_buf_8bit(s, (unsigned char *)cb, sizeof(struct wsta_cb));

}

void dump_bcap(struct seq_file *s, MAC_INFO* info, int index, char is_ds)
{
	char addr[6];
	int bcap = 0;
	
	memset(addr, 0, 6);
	mac_addr_lookup_engine_find(addr, index, &bcap, BY_ADDR_IDX|(is_ds ? IN_DS_TBL:0));

	wl_print(s, "[%d] addr=%s, bcap=%08x\n", index, wmaddr_ntoa((char *)&addr), bcap);
}


void dump_ba(struct seq_file *s, MAC_INFO* info, int sta_idx)
{
#if defined(WLA_AMPDU_RX_SW_REORDER)
	int j;
	struct wsta_cb *cb;
	struct rx_ba_session *ba;

	cb = info->wcb_tbls[sta_idx];
	if(cb == 0)
		return;
	wl_print(s, "RX:\n");	
	for(j=0; j<8; j++)
	{
		ba = (struct rx_ba_session *)cb->ba_rx[j];
		if(ba)
		{
			wl_print(s, "   [%d]ba=%x, size=%d, start=%d, num=%d, head=%x, tail=%x\n", j, ba, ba->win_size, ba->win_start, ba->stored_num, ba->reorder_q_head, ba->reorder_q_tail);
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
			wl_print(s, "TID:%d\n", i);
			idb_dump_buf_16bit(s, (unsigned short *)info->reorder_buf_mapping_tbl[(sta_idx * MAX_QOS_TIDS) + i], sizeof(ampdu_reorder_buf));
		}
	}

	wl_print(s, "TX:\n");
	MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | sta_idx);	
	while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD)
		;
    regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR); 
	wl_print(s, "state=%08x\n",  regval);
	wl_print(s, "sta_qinfo_tbls:\n");
	idb_dump_buf_16bit(s, (unsigned short *)&info->sta_qinfo_tbls[sta_idx], sizeof(sta_qinfo_tbl));
	if(info->sta_qinfo_tbls[sta_idx].qinfo_addr)
	{
		ucast_qinfo *qinfos = (ucast_qinfo *)NONCACHED_ADDR(info->sta_qinfo_tbls[sta_idx].qinfo_addr);
		for(i=0; i<8; i++)
		{
			wl_print(s, "TID:%d\n", i);
			idb_dump_buf_16bit(s, (unsigned short *)&qinfos[i], sizeof(ucast_qinfo));
		}		
	}
#endif
}
mm_segment_t oldfs;
void initkernelenv(void)
{ 
	oldfs = get_fs(); 
	set_fs(KERNEL_DS); 
} 

void dinitkernelenv(void)
{
	set_fs(oldfs); 
}

struct file *openfile(char *path,int flag,int mode)
{ 
	struct file *fp; 
	
	fp=filp_open(path, flag, 0); 

	if (IS_ERR(fp))
		return NULL;
	else 
		return fp; 
}

int writefile(struct file *fp,char *buf,int writelen) 
{ 
	if (fp->f_op && fp->f_op->write)
		return fp->f_op->write(fp, buf, writelen, &fp->f_pos); 
	else 
		return -1; 
} 

int readfile(struct file *fp, char *buf, int readlen)
{
	if(fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp, buf, readlen, &fp->f_pos);
	else
		return -1;
}

int closefile(struct file *fp) 
{
	filp_close(fp,NULL); 
	return 0; 
}

#ifdef TKIP_COUNTERMEASURE
void dump_ptk(struct seq_file *s, MAC_INFO *info, int cmd)
{
	int i=0, j=0;
	struct wsta_cb *wcb;
	cipher_key *hwkey;
	u8 buf[TKIP_KEY_LEN + TKIP_MICKEY_LEN + TKIP_MICKEY_LEN];
	//int cmd = atoi(argv[1]); /* 0: dump, 1: set rx_mic, 2: set tx_mix */


	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		//bss = &(my_ap_dev->bss[i]);
		wcb = info->wcb_tbls[i];

		if(wcb == NULL)
			continue;

		if(wcb->hw_cap_ready == 0)
			continue;

		hwkey = IDX_TO_PRIVATE_KEY(wcb->captbl_idx);
		
		if(hwkey->tkip_key.cipher_type != CIPHER_TYPE_TKIP)
			continue;

		memcpy(buf, hwkey->tkip_key.key, TKIP_KEY_LEN);
		memcpy(&buf[TKIP_KEY_LEN], hwkey->tkip_key.txmickey,  TKIP_MICKEY_LEN);
		memcpy(&buf[TKIP_KEY_LEN+TKIP_MICKEY_LEN], hwkey->tkip_key.rxmickey,  TKIP_MICKEY_LEN);

		switch(cmd)
		{
			case 1:
				buf[TKIP_KEY_LEN+TKIP_MICKEY_LEN+6] = 0xAA;
				buf[TKIP_KEY_LEN+TKIP_MICKEY_LEN+7] = 0x55;
				wla_cipher_key(wcb, CIPHER_TYPE_TKIP, KEY_TYPE_PAIRWISE_KEY, buf, 0, 0);
			break;
			case 2:
				buf[TKIP_KEY_LEN+6] = 0xAA;
				buf[TKIP_KEY_LEN+7] = 0x55;
				wla_cipher_key(wcb, CIPHER_TYPE_TKIP, KEY_TYPE_PAIRWISE_KEY, buf, 0, 0);
			break;
			case 3:
				buf[0] = 0xAA;
				buf[1] = 0x55;
				wla_cipher_key(wcb, CIPHER_TYPE_TKIP, KEY_TYPE_PAIRWISE_KEY, buf, 0, 0);
			break;
			default:
				wl_print(s, "*** wcb idx =%d ****\n", wcb->captbl_idx);
				wl_print(s, "tk1 = ");
				for(j=0; j < 16; j++)
				{
					wl_print(s, "%02x", buf[j]);
				}
				wl_print(s, "\n");

				wl_print(s, "tx_mic_key = ");
				for(j=TKIP_KEY_LEN; j < TKIP_KEY_LEN+TKIP_MICKEY_LEN; j++)
				{
					wl_print(s, "%02x", buf[j]);
				}
				wl_print(s, "\n");
													
				wl_print(s, "rx_mic_key = ");
				for(j=TKIP_KEY_LEN+TKIP_MICKEY_LEN; j < TKIP_KEY_LEN+TKIP_MICKEY_LEN+TKIP_MICKEY_LEN; j++)
				{
					wl_print(s, "%02x", buf[j]);
				}
				wl_print(s, "\n");
			break;
		}
	}
}
#endif // TKIP_COUNTERMEASURE

static int cipher_suite_to_bit(const u8 *s)
{
	u32 suite = (s[0] << 24) | (s[1] << 16) | (s[2] << 8) | s[3];

	if(suite== RSN_CIPHER_SUITE_TKIP)
		return 1 << AUTH_CIPHER_TKIP;
	if(suite == RSN_CIPHER_SUITE_CCMP)
		return 1 << AUTH_CIPHER_CCMP;
	if(suite == WPA_CIPHER_SUITE_TKIP)
		return 1 << AUTH_CIPHER_TKIP;
	if(suite == WPA_CIPHER_SUITE_CCMP)
		return 1 << AUTH_CIPHER_CCMP;

	return 0;
}


int wlan_parse_wpa_rsn_ie(const u8 *start, u32 len, u32 *pcipher, u32 *gcipher)
{
	u8 *pos;
	u32 i, count, left;

	pos = (u8 *) start + 2;
	left = len - 2;
 
	if(left < SELECTOR_LEN)
		return -1;

	/* get group cipher suite */
	*gcipher |= cipher_suite_to_bit(pos);

	pos += SELECTOR_LEN;
	left -= SELECTOR_LEN;

	/* get pairwise cipher suite list */
	count = (pos[1] << 8) | pos[0];
	pos += 2;
	left -= 2;
	if (count == 0 || left < count * SELECTOR_LEN) 
	{
		return -1;
	}
	for (i = 0; i < count; i++) 
	{
		*pcipher |= cipher_suite_to_bit(pos);
	
		pos += SELECTOR_LEN;
		left -= SELECTOR_LEN;
	}

	return 0;
}
#ifdef CONFIG_CHEETAH_SMART_CONFIG
#ifndef CONFIG_CHEETAH_SMART_TO_USRSPACE
	extern u32 smrtcfg_debug_flag;
#endif
#endif

static int wifidump_func(int argc, char *argv[])
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	struct seq_file *s = (struct seq_file *)sc->seq_file;
	int i;

	if (1 > argc)
	{
		wl_print(s, "%s %d argc:%d %s\n",__func__,__LINE__,argc,argv[0]);
		return 0;
	}

	if (!strcmp(argv[0], "info"))
	{
		wl_print(s, "sta_cap_tbl_count=%d\nrate_tbl_count=%d\n", 
				info->sta_cap_tbl_count, info->rate_tbl_count);
		wl_print(s, "beacon_descriptors_list=%x\ntx_descriptors=%x\nsw_returned_tx_descriptors=%x\n", 
					(unsigned int)info->beacon_descriptors_list, (unsigned int)info->wl_txq.desc_hdr, 
					(unsigned int)info->wl_sw_retq.desc_hdr);
		wl_print(s, "hw_tx_buffer=%x\nfastpath_buf=%x\nfastpath_bufsize=%d\n", (unsigned int)info->hw_tx_buffer, 
					(unsigned int)info->fastpath_buf, (unsigned int)info->fastpath_bufsize);
		wl_print(s, "ac_vo=%d, ac_vi=%d, ac_be=%d, ac_bk=%d\n", info->acq_len[ACQ_VO_IDX], info->acq_len[ACQ_VI_IDX], info->acq_len[ACQ_BE_IDX], info->acq_len[ACQ_BK_IDX]);
		wl_print(s, "w_rx_dump=%d, w_tx_dump=%d, w2e_rx_dump=%d, w2e_tx_dump=%d\n", info->w_rx_dump, info->w_tx_dump, info->w2e_rx_dump, info->w2e_tx_dump);
		wl_print(s, "data_forward_mode=%d, erp_protect=%d, non_gf_exist=%d, bw40mhz_intolerant=%d\n", info->data_forward_mode, info->erp_protect, info->non_gf_exist, info->bw40mhz_intolerant);
		wl_print(s, "no_kick_reorder_buf=%d, txq_limit=%d, fix_acq_limit=%d\n", info->no_kick_reorder_buf, info->txq_limit, info->fix_ac_queue_limit);
#if defined(CONFIG_CHEETAH)
		wl_print(s, "sta_tbl_count=%d, ds_tbl_count=%d\n", info->sta_tbl_count, info->ds_tbl_count);
#endif
		wl_print(s, "wmac_polling=%x\n", info->wmac_polling);
		wl_print(s, "sn_gap=%d\n", info->ampdu_sn_gap);
		wl_print(s, "**********statistic***********\n");
		wl_print(s, "pre_tbtt_int=%d, ts0_int=%d, beacon_tx_miss=%d\n", info->pre_tbtt_int, info->ts0_int, info->beacon_tx_miss);
		wl_print(s, "sw_wrx=%d, sw_wrx_drop=%d\nsw_wtx=%d, sw_wret=%d\n", info->wl_rxq.pkt_count, info->sw_wrx_drop_pkt_cnt, info->wl_txq.pkt_count, info->wl_sw_retq.pkt_count);
		wl_print(s, "sw_erx=%d\nsw_etx=%d\n", info->eth_rxq.pkt_count, info->eth_txq.pkt_count);
		wl_print(s, "sw_w2e_unicast=%d, sw_w2e_bcast=%d, sw_e2w_unicast=%d, sw_e2w_bcast=%d\n", info->sw_w2e_unicast_pkt_cnt, info->sw_w2e_bcast_pkt_cnt, info->sw_e2w_unicast_pkt_cnt, info->sw_e2w_bcast_pkt_cnt);
		wl_print(s, "sw_wtx_no_bhdr=%d, sw_wtx_no_desc=%d, sw_wtx_bcast_pkt_waiting=%d\n", info->sw_wtx_no_bhdr, info->sw_wtx_no_desc, info->sw_wtx_bcast_pkt_waiting);
		wl_print(s, "hw_buf_full=%d, rx_desc_full=%d, rx_fifo_full=%d\n", info->hw_buf_full, info->rx_desc_full, info->rx_fifo_full);
		wl_print(s, "eth_hw_desc_full=%d, eth_sw_desc_full=%d\n", info->eth_hw_desc_full, info->eth_sw_desc_full);
		wl_print(s, "int_mask=0x%x, rxfilter=0x%x\n", info->int_mask, info->filter);
		wl_print(s, "bc_ps_queue_pending=%d\n", info->bc_ps_queue_pending);
//		wl_print(s, "wbuf_mc_used=%d, wbuf_mem_used=%d\n", wbuf_mc_used, wbuf_mem_used);
		wl_print(s, "mac_recover_mode=%d, mac_recover=%d, mac_recover_cnt=%d\n", info->mac_recover_mode, info->mac_recover, info->mac_recover_cnt);
		wl_print(s, "mon_bmgctrl_cnt=%d, mon_bufempty_cnt=%d\n", info->mon_bmgctrl_cnt, info->mon_bufempty_cnt);
		wl_print(s, "mon_4013_cnt=%d, mon_8094_cnt=%d mon_8000_cnt=%d\n", info->mon_4013_cnt, info->mon_8094_cnt, info->mon_8000_cnt);
		wl_print(s, "mon_wronghead=%d, mon_zerodataofs_cnt=%d mon_wrongflag_cnt=%d\n", info->mon_wronghead_cnt, info->mon_zerodataofs_cnt, info->mon_wrongflag_cnt);
		wl_print(s, "mon_rxzombie_cnt=%d\n", info->mon_rxzombie_cnt);
		wl_print(s, "t_closest(%x) t_current(%x) t_delta(%x) t_recovery(%x)\n", info->mon_t_closest, info->mon_t_current, info->mon_t_delta, info->mon_t_recover);
		wl_print(s, "tx_fail_times=%d\n", ((struct wla_softc *)(info->sc))->tx_fail_times);
		return 0;
	}
#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
	else if(!strcmp(argv[0], "mat"))
	{
		int mode=0;
		struct mat_entry *mat_ent;
		if(argc < 2)
			;
		else
			sscanf(argv[1], "%d", &mode);
		
		switch(mode)
		{
			case 0:
			{
				wl_print(s, "now = %d\n", WLA_CURRENT_TIME/HZ);

				for(i = 0; i < MAT_HASH_TABLE_SIZE; i++)
				{
					mat_ent = mat_hash_tbl[i];
					wl_print(s, "***************** MAT TABLE: hash_idx = %d ***************\n", i);
					while(mat_ent)
					{
						wl_print(s, "addr_type = %x\n", mat_ent->addr_type);	
						wl_print(s, "net_addr = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", 
								mat_ent->net_addr[0], mat_ent->net_addr[1], 
								mat_ent->net_addr[2], mat_ent->net_addr[3],
								mat_ent->net_addr[4], mat_ent->net_addr[5], 
								mat_ent->net_addr[6], mat_ent->net_addr[7]);
						wl_print(s, "mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\n", 
								mat_ent->mac_addr[0], mat_ent->mac_addr[1], 
								mat_ent->mac_addr[2], mat_ent->mac_addr[3],
								mat_ent->mac_addr[4], mat_ent->mac_addr[5]);	
						wl_print(s, "ref_time = %d\n", mat_ent->ref_time);
						wl_print(s, "flags = %d\n", mat_ent->flags);
						mat_ent = mat_ent->next;
					}
				}
				break;
			}
			case 1:
				wl_print(s, "***************** FLUSH MAT TABLE  ***************\n");
				mat_flush();
				break;
		}
		return 0;
//mat_help:
		wl_print(s, "mat <sel>\n");
		wl_print(s, "sel = 0: dump mat_table\n");
		wl_print(s, "sel = 1: flush mat_table\n");
		return 0;
	}
#endif	//  CONFIG_UNIVERSAL_REPEATER
	else if (!strcmp(argv[0], "umac"))
	{
		reg_dump(s, (unsigned char *)0xaf003800, 0x100);
		wl_print(s, "\nTX:");
		reg_dump(s, (unsigned char *)0xaf003a00, 0x100);
		wl_print(s, "\nRX:");
		reg_dump(s, (unsigned char *)0xaf003b00, 0x100);
		wl_print(s, "\nETH:");
		reg_dump(s, (unsigned char *)0xaf009800, 0x100);
		wl_print(s, "\nSEC:");
		reg_dump(s, (unsigned char *)0xaf009c00, 0x34);
		wl_print(s, "\n");
	}
	else if (!strcmp(argv[0], "lmac"))
	{
		reg_dump(s, (unsigned char *)0xaf009000, 0x100);
		wl_print(s, "\n");
	}
	else if (!strcmp(argv[0], "txq"))
	{
		wl_print(s, "tx_descr_count=%d\n", info->wl_txq.max);
		idb_dump_buf_32bit(s, (unsigned int *)info->wl_txq.desc_hdr, info->wl_txq.max*16);
	}
	else if (!strcmp(argv[0], "wrxq"))
	{
		wl_print(s, "rx_descr_count=%d\n", info->wl_rxq.max);
		idb_dump_buf_32bit(s, (unsigned int *)info->wl_rxq.desc_hdr, info->wl_rxq.max*16);
	}
	else if (!strcmp(argv[0], "retq"))
	{
		wl_print(s, "rx_descr_count=%d\n", info->wl_sw_retq.max);
		idb_dump_buf_32bit(s, (unsigned int *)info->wl_sw_retq.desc_hdr, info->wl_sw_retq.max*16);
	}
	else if (!strcmp(argv[0], "fretq"))
	{
		wl_print(s, "rx_descr_count=%d\n", info->fp_retq.max);
		idb_dump_buf_32bit(s, (unsigned int *)info->fp_retq.desc_hdr, info->fp_retq.max*16);
	}
	else if (!strcmp(argv[0], "swbuf"))
	{
		int val;
		short head, tail;

		val = MACREG_READ32(SWBUFF_CURR_HT);
		head = val >> 16;
		tail = val & 0xffff;

		wl_print(s, "sw_rx_bhdr_start=%d, sw_tx_bhdr_start=%d, sw_tx_bhdr_head=%d, sw_tx_bhdr_tail=%d\n", info->sw_rx_bhdr_start, info->sw_tx_bhdr_start, info->sw_tx_bhdr_head, info->sw_tx_bhdr_tail);
		dump_list(s, info, head, tail, info->sw_rx_bhdr_start, info->sw_tx_bhdr_start);

		if(info->sw_tx_bhdr_head > 0)
			dump_list(s, info, info->sw_tx_bhdr_head, info->sw_tx_bhdr_tail, info->sw_tx_bhdr_start, info->sw_tx_bhdr_end);
	}
	else if (!strcmp(argv[0], "hwbuf"))
	{
		int val;
		short head, tail;

		val = MACREG_READ32(HWBUFF_CURR_HT);
		head = val >> 16;
		tail = val & 0xffff;

		dump_list(s, info, head, tail, 0, info->sw_rx_bhdr_start);
	}
#if defined(ARTHUR_WIFI2ETH)
	else if (!strcmp(argv[0], "etxq"))
	{
		wl_print(s, "eth_tx_descr_count=%d, free=%d\n", info->eth_txq.max, info->eth_txq.free);
		idb_dump_buf_32bit(s, (unsigned int *)info->eth_txq.desc_hdr, info->eth_txq.max*16);
	}
	else if (!strcmp(argv[0], "erxq"))
	{
		wl_print(s, "eth_rx_descr_count=%d\n", info->eth_rxq.max);
		idb_dump_buf_32bit(s, (unsigned int *)info->eth_rxq.desc_hdr, info->eth_rxq.max*16);
	}
	else if(!strcmp(argv[0], "stat"))
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

		wl_print(s, "====== packet statistics ======\n");
		wl_print(s, "e2w=(%d)\n", e2w);
		wl_print(s, "w2e=(%d)\n", w2e);
		wl_print(s, "jam=(%d)\n", jam);
		wl_print(s, "acbusy=(%d)\n", acbusy);
		wl_print(s, "err=(%d)\n", err);
	}
#endif
	else if(!strcmp(argv[0], "gkey"))
	{	
		int tbl_idx = 0;
		if(argc == 2)
			sscanf(argv[1],"%d",&tbl_idx);

		wl_print(s, "group_keys[%d]\n", tbl_idx);
		idb_dump_buf_32bit(s, (unsigned int *)&info->group_keys[tbl_idx], sizeof(group_cipher_key));
	}
	else if(!strcmp(argv[0], "pkey"))
	{
		int tbl_idx = 0;
		if(argc == 2)
			sscanf(argv[1],"%d",&tbl_idx);

		wl_print(s, "private_keys[%d]\n", tbl_idx);
		idb_dump_buf_32bit(s, (unsigned int *)&info->private_keys[tbl_idx], sizeof(cipher_key));
	}
#if 0
	else if(!strcmp(argv[0], "sta_gkey"))
	{
		//int tbl_idx = 0;
		wl_print(s, "sta_group_keys\n");
		idb_dump_buf_16bit(s, (unsigned short *)&info->sta_group_keys->group_key, sizeof(cipher_key));
	}
#endif
	else if(!strcmp(argv[0], "sta"))
	{
		int tbl_idx = 0;
		if(argc == 2)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			dump_sta(s, info, tbl_idx, 0);
		}
		else
		{
			for(; tbl_idx < info->sta_cap_tbl_count; tbl_idx++)
			{
				if(1 == info->sta_cap_tbls[tbl_idx].valid)
				{
					dump_sta(s, info, tbl_idx, 0);
				}
			}
		}
	}
	else if(!strcmp(argv[0], "rate"))
	{
		int tbl_idx = 0;
		if(argc == 2)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			dump_sta(s, info, tbl_idx, 1);
		}
		else
		{
			for(; tbl_idx < info->rate_tbl_count; tbl_idx++)
			{
				if(1 == info->rate_tbls[tbl_idx].valid)
				{
					dump_sta(s, info, tbl_idx, 1);
				}
			}
		}	
	}
	else if(!strcmp(argv[0], "wcb"))
	{
		int tbl_idx = 0;
		if(argc == 2)
			sscanf(argv[1],"%d",&tbl_idx);
		dump_wcb(s, info, tbl_idx);	
	}
	else if(!strcmp(argv[0], "bcap"))
	{
		int tbl_idx = 0;
		if(argc >= 2)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			if(argc == 3)
			{
				int bcap_val;
				sscanf(argv[2], "%x", &bcap_val);
				mac_addr_lookup_engine_update(info, tbl_idx, bcap_val, BY_ADDR_IDX);
			}
			dump_bcap(s, info, tbl_idx, 0);	
		}
		else
		{	 
			/* dump all */
			for(i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
			{
				dump_bcap(s, info, i, 0);
				//wl_print(s, "#### addr ext_sta_tbls i:%d addr:%p\n",i,&info->ext_sta_tbls[i]);
				idb_dump_buf_16bit(s, (unsigned short *)&((sta_addr_entry *)info->ext_sta_tbls)[i],
					sizeof(sta_addr_entry));
			}
		}	
	}
	else if(!strcmp(argv[0], "ds"))
	{
		int tbl_idx = 0;
		if(argc == 2)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			dump_bcap(s, info, tbl_idx, 1);
		}
		else
		{
			/* dump all */
			for(i=0; i < MAX_DS_TABLE_ENTRIES; i++)
			{
				dump_bcap(s, info, i, 1);
				//wl_print(s, "#### addr ext_ds_tbls i:%d addr:%p\n",i,&info->ext_ds_tbls[i]);
				idb_dump_buf_16bit(s, (unsigned short *)&((sta_addr_entry *)info->ext_ds_tbls)[i],
					sizeof(sta_addr_entry));
			}
		}
	}
	else if(!strcmp(argv[0], "acache"))
	{
		dump_addr_cache(s, info);
	}
	else if(!strcmp(argv[0], "meter"))
	{
		if(!strcmp(argv[1], "erx"))
		{
			unsigned int rate;
			unsigned int now = WLA_CURRENT_TIME;

			rate = (info->sw_e2w_unicast_pkt_bytes * 8)/(now - info->erx_last_sample_time)*100/1000;
			info->sw_e2w_unicast_pkt_bytes = 0;
			info->erx_last_sample_time = now;
			wl_print(s, "%d kbps\n", rate);
		}
	}
	else if(!strcmp(argv[0], "ba"))
	{
		int tbl_idx = 0;
		if(argc == 2)
			sscanf(argv[1],"%d",&tbl_idx);
		dump_ba(s, info, tbl_idx);	
	}
	else if(!strcmp(argv[0], "bcq"))
	{
		dump_bc_qinfo(s, info);	
	}
	else if(!strcmp(argv[0], "mmacd"))
	{
		wl_print(s, "Total number: %d\n",  info->mmac_rx_descr_count);
		idb_dump_buf_32bit(s, (unsigned int *)info->mmac_rx_descriptors, sizeof(mmac_rx_descriptor) * info->mmac_rx_descr_count);
	}
	else if(!strcmp(argv[0], "psbaq"))
	{
		wl_print(s, "Total number: %d\n",  info->psbaq_descr_count);
		for(i=0; i<PSBAQ_DESCRIPTOR_COUNT; i++)
		{
			wl_print(s, "###[%d]###\n", i);
			idb_dump_buf_32bit(s, (unsigned int *)&info->psbaq_descriptors[i], sizeof(psbaq_descriptor));
		}
	}
	else if(!strcmp(argv[0], "dump"))
	{
		int val = 0;

		if(argc != 3)
			return 0;

		sscanf(argv[2],"%d",&val);

		if(!strcmp(argv[1], "wrx"))
			info->w_rx_dump = val;
		else if(!strcmp(argv[1], "wtx"))
			info->w_tx_dump = val;
		else if(!strcmp(argv[1], "erx"))
			info->w2e_rx_dump = val;
		else if(!strcmp(argv[1], "etx"))
			info->w2e_tx_dump = val;

	}
	else if(!strcmp(argv[0], "rxfilter"))
	{
		sscanf(argv[1],"%d",&info->filter);
	}
	else if(!strcmp(argv[0], "no_kick"))
	{
		int val=0;
		sscanf(argv[1],"%d",&val);
		info->no_kick_reorder_buf = val;
	}
	else if(!strcmp(argv[0], "bb_rst"))
	{
		bb_reset();
	}
	else if(!strcmp(argv[0], "acq"))
	{
#if 0			
		if(argc == 3)
		{
			info->acq_len[atoi(argv[1])] = atoi(argv[2]); 
			info->fix_ac_queue_limit = 1;
			MACREG_WRITE32(FC_ACQ_THRESHOLD, ((info->acq_len[ACQ_BK_IDX] << 24) | (info->acq_len[ACQ_BE_IDX] << 16) | (info->acq_len[ACQ_VI_IDX] << 8) | info->acq_len[ACQ_VO_IDX]));
		}
#endif		
	}
	else if(!strcmp(argv[0], "txrate"))
	{
		short j;
		for(i=0; i<MAX_BSSIDS; i++)
		{
			wl_print(s, "MBSS[%d]: role:%d vif_id:%d\n", i,info->mbss[i].role, info->mbss[i].vif_id);
			for(j=0;j<3;j++)
			{
				wl_print(s, "    TX RATE[%d] idx:%d, rate idx=%x\n", j, info->mbss[i].tx_rate[j].rate_captbl, info->mbss[i].tx_rate[j].rate_idx);
			}
		}
	}
	else if(!strcmp(argv[0], "polling"))
	{
		int val=0;
		sscanf(argv[1],"%d",&val);
		info->wmac_polling = val; 
	}
	else if(!strcmp(argv[0], "sn_gap"))
	{
		sscanf(argv[1],"%d",&info->ampdu_sn_gap);
	}
#if defined(WLAN_RC_BTS)
	else if(!strcmp(argv[0], "rc"))
	{
		int tbl_idx = 0;
		unsigned int rc_debug;
		
		if((argc == 3) && !strcmp(argv[1], "dbg"))
		{		
			sscanf(argv[2],"%d",&rc_debug);
		    info->rc_debug = rc_debug;
		}
		else if(argc == 2)
		{
		    sscanf(argv[1],"%d",&tbl_idx);
		}
		dump_bts(s, info, tbl_idx);
	}
#endif
	else if(!strcmp(argv[0], "pm"))
	{
		sta_cap_tbl *captbl;
		int tbl_idx, pm;
		
		if(argc == 3)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			sscanf(argv[2],"%d",&pm);
			
			captbl = &info->sta_cap_tbls[tbl_idx];
			captbl->power_saving = pm;
			mac_stacap_cache_sync(tbl_idx, true);
		}
	
	}
	else if(!strcmp(argv[0], "rts"))
	{
		sta_cap_tbl *captbl;
		int tbl_idx, rts;
		
		if(argc == 3)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			sscanf(argv[2],"%d",&rts);

			captbl = &info->sta_cap_tbls[tbl_idx];
			captbl->force_rts = rts;
			mac_stacap_cache_sync(tbl_idx, true);
		}
	
	}
	else if(!strcmp(argv[0], "t_psq"))
	{
		int tbl_idx, type, cmd;
		
		if(argc == 3)
		{
			sscanf(argv[1],"%d",&tbl_idx);
			sscanf(argv[2],"%d",&type);
			
			if(type == 2)
			{
				wl_print(s, "sta aging..\n");
				MACREG_WRITE32(PSBAQ_QINFO_CSR0, 0);
				MACREG_WRITE32(PSBAQ_QINFO_CSR2, (tbl_idx << 8) | PSBAQ_QINFO_CSR2_CMD_STA_AGING | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
			}
			else
			{
				if(type == 1)
					cmd = (tbl_idx&0xff) | TRIG_TYPE | TRIG_REQUEST;
				else
					cmd = (tbl_idx&0xff) | TRIG_REQUEST;
				wl_print(s, "cmd=%x\n",cmd);
				MACREG_WRITE32(PSQ_TRIGGER, cmd);
			}
		}
	
	}
	else if(!strcmp(argv[0], "mactable"))
	{
		int i;
		unsigned char type;
		char *mac;
		for(i=0; i<MACADDR_TABLE_MAX; i++)
		{
			mac = maddr_tables.m[i].addr;
			type= maddr_tables.m[i].type;
			wl_print(s, "mactable[%d]:\n",i);
			wl_print(s, "	type:%d addr:[%x:%x:%x:%x:%x:%x]\n",type, mac[0]&0xff,mac[1]&0xff,mac[2]&0xff,mac[3]&0xff,mac[4]&0xff,mac[5]&0xff);
		}
	}
	else if(!strcmp(argv[0], "scan_results"))
	{
		struct cfg80211_registered_device *rdev;
		struct cfg80211_internal_bss *scan;
		struct cfg80211_bss *res;
		struct cfg80211_bss_ies *ies=NULL;
		struct file *fp =NULL;

		char write_buf[128];

		int ssid_len;
		char ssid_name[33]={0};

		unsigned char ch = 0;

		char band=0;

		int signal;

		char *sec;
		int sec_num=0;
		
		unsigned char *ie;
		int ie_len;

		u32 pcipher, gcipher;

		rdev = wiphy_to_dev(sc->hw->wiphy);	
		
		initkernelenv();
		fp = openfile("etc/res", O_CREAT | O_WRONLY, 0);

		list_for_each_entry(scan, &rdev->bss_list, list) {
			res = &scan->pub;
			ies = (struct cfg80211_bss_ies *)res->ies;
			if (ies == NULL)
				continue;
			ie_len = ies->len;
			ie = ies->data;

			if (!is_zero_ether_addr(res->bssid))
					;
			
			signal = res->signal/100; //signal
			
			pcipher = gcipher = 0;
			
			while(ie_len >= 2 && ie_len >= ie[1])
			{	
				if (ie[0] == 0) //SSID
				{
					memset(ssid_name,0,33);
					ssid_len = ie[1]>32?32:ie[1];
					memcpy(ssid_name,(char *)ie+2,ssid_len);
					ssid_name[ssid_len]='\0';
				}

				else if(ie[0] == 3)			//channel
					ch = ie[2];

				else if(ie[0] == 45)		//band
					band |= 4;
			
				else if(ie[0] == 48)		//WPA2
				{
					sec_num |= 4;
					wlan_parse_wpa_rsn_ie(ie + 2, ie_len - 2, &pcipher, &gcipher);
				}
				else if(ie[0] == 68)		//WAPI
				{
					sec_num |= 16;
					pcipher |= 1 << AUTH_CIPHER_SMS4;
					gcipher |= 1 << AUTH_CIPHER_SMS4;
				}
				else if(ie[0] == 221)		//Vendor
				{
					/* Microsoft OUI */
					if((ie[2] == 0x00) && (ie[3] == 0x50) && (ie[4] == 0xf2))
					{
						sec=ie+5;
						if(sec[0] == 0x4)		// WPS
							sec_num |= 8;
						else if(sec[0] == 0x1)	// WPA
						{
							sec_num |= 2;
							wlan_parse_wpa_rsn_ie(ie + 6, ie_len - 6, &pcipher, &gcipher);
						}
					}
				}
				ie_len -= ie[1] + 2;
				ie += ie[1] + 2;
			}
			if ((res->capability & 16) && ((sec_num & 6)==0))
				sec_num |= 1;
			
			memset(write_buf,0,sizeof(write_buf));
			sprintf(write_buf,"bssid=%s&ssid=%s&ch=%d&sig=%d&band=%d&sec=%d&pcipher=%d&gcipher=%d\n",wmaddr_ntoa(res->bssid),ssid_name,ch,signal,band,sec_num, pcipher, gcipher);
			
			if (fp != NULL)
			{
				writefile(fp, write_buf, strlen(write_buf));
			}
			sec_num = 0;
		}
		closefile(fp);
		dinitkernelenv();
	}
	else if (!strcmp(argv[0], "vif"))
	{
	    char name[32];
	    int vifid;
	    struct net_device *dev;
	    int bss_desc;
	    struct mbss_t *wif;
		struct ieee80211_sub_if_data *sdata = NULL;
		struct ieee80211_vif *vif;
		struct wla_vif_priv *vp;
		char *role[3]={"NONE","STA","AP"};
	
	    if(argc < 3)
		{	
			wl_print(s, "Name:\trole\tunit\tvifid\tbss_desc\n");
			for(bss_desc=0; bss_desc < MACADDR_TABLE_MAX; bss_desc++)
			{
				dev=sc->bssdev[bss_desc];
				if(dev!=NULL)
				{		
					sdata=IEEE80211_DEV_TO_SUB_IF(dev);
					vif = &sdata->vif;
					vp = (struct wla_vif_priv *)vif->drv_priv;
					wl_print(s, "%s\t%s\t%d\t%d\t%d\n",dev->name,role[info->mbss[bss_desc].role],vp->unit,info->mbss[bss_desc].vif_id,bss_desc);
				}

			}
	        return 0;
		}

	    sscanf(argv[1],"%s",name);
	    sscanf(argv[2],"%d",&vifid);
	
	    for(bss_desc=0; bss_desc < MACADDR_TABLE_MAX; bss_desc++)
	    {
	        dev=sc->bssdev[bss_desc];
			if((dev!=NULL)&&!strcmp(dev->name,name))
		            break;
	    }
	    if (bss_desc >= MAC_MAX_BSSIDS)
	            return 0;
					
		sdata=IEEE80211_DEV_TO_SUB_IF(dev);
		vif = &sdata->vif;
		vp = (struct wla_vif_priv *)vif->drv_priv;
		vp->unit = vifid;	
	
	    wif = &info->mbss[bss_desc];
	    wif->vif_id = vifid;
	
	}
	else if (!strcmp(argv[0], "survey"))
	{
		struct survey_info *survey = NULL;
		int i;
		
		wl_print(s, "Channel:\tTime\tBusy Time\tdBm\n");
		for(i=0; i < 11; i++)
		{
			survey = &sc->survey[i];
			wl_print(s, "%d\t\t%llu\t%llu\t\t%d\n",i+1,survey->channel_time,survey->channel_time_busy,survey->noise);
		}
	}
#ifdef TKIP_COUNTERMEASURE
	else if(!strcmp(argv[0], "ptk"))
	{
		int select;
	    sscanf(argv[1],"%d", &select);
		
		dump_ptk(s, info, select);
	}
#endif //TKIP_COUNTERMEASURE
#ifdef CONFIG_WMAC_RECOVER
	else if(!strcmp(argv[0], "recover"))
	{
		wla_set_recover(1);
	}
#endif
	else if(!strcmp(argv[0], "bb"))
	{
		//unsigned char buf[256];
		//int param[] = { 0, 0, 0, 0, 0, 0};
		unsigned char value74,value75,value76;
		unsigned long value;

		if(argc >= 2)
		{
			if(!strcmp("rxpeon", argv[1]))
				bb_rxpe_on();
			else if(!strcmp("rxpeoff", argv[1]))
				bb_rxpe_off();
			else if(!strcmp("txpeon", argv[1]))
				bb_txpe_on();
			else if(!strcmp("txpeoff", argv[1]))
				bb_txpe_off();
			else if(!strcmp("memdump", argv[1]))
			{
				wl_print(s, "Waiting BR71 trigger finish...");
				while(1)
				{
					value = bb_register_read(0x71);

					if((value & 0x2))
						break;
				}
				wl_print(s, "Done\n");

				wl_print(s, "--------------------------DUMP START--------------------------\n");

				for(i=0;i<65536;i++)
				{
					value74 = bb_register_read(0x74);
					//wl_print(s, " %02x", value);
					value75 = bb_register_read(0x75);
					//wl_print(s, " %02x", value);
					value76 = bb_register_read(0x76);
					//wl_print(s, " %02x\n", value);

					value = ((value74 << 16) | (value75 << 8) | value76);
					wl_print(s, "%d\n", (unsigned int)value & 0x00ffffff);
				}

				wl_print(s, "--------------------------DUMP  DONE--------------------------\n");
				return CLI_OK;
			}
			else if(!strcmp("alldump", argv[1]))
			{
				for(i=0; i<=0xff; i++)
				{
					value = bb_register_read(i);
					wl_print(s, "1-%02x-%02x\n", i, (unsigned int) value);
				}
			}
			else if(!strcmp("agcdump", argv[1]))
			{
				for(i=0; i<17; i++)
				{
					bb_register_write(0x10, i);
					value = bb_register_read(0x11);
					wl_print(s, "1-10-%02x\n", i);
					wl_print(s, "1-11-%02x\n", (unsigned int)value);
				}
			}
			else if(!strcmp("spitest", argv[1]))
			{
				int loop = 10000;
				int reg = 0x2;
				char pattern, val;
				int count = 0;

				if(argc == 3)
					sscanf(argv[2],"%d", &reg);

				if(argc == 4)
					sscanf(argv[3],"%d", &loop);

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
						wl_print(s, "%dth test fail??????, write %02x to REG%02x, read %02x\n", count, pattern, reg, val);
						break;
					}
					else
						wl_print(s, "%dth test pass\n", count);
					count++;
				}
			}
			else if(!strcmp("r", argv[1]))
			{
				int reg;
				int val;
				if(argc == 3)
				{		
					sscanf(argv[2],"%x", &reg);
					val = bb_register_read(reg);
					wl_print(s, "read BB register 0x%x, value 0x%x\n", (unsigned int)reg, (unsigned int) val);
				}

			}
			else if(!strcmp("w", argv[1]))
			{
				int reg;
				int wval, rval;
				if(argc == 4)
				{		
					sscanf(argv[2],"%x", &reg);
					sscanf(argv[3],"%x", &wval);
					bb_register_write(reg, wval);
					rval = bb_register_read(reg);
					wl_print(s, "write BB register 0x%x, value 0x%x, result 0x%x\n", (unsigned int)reg, (unsigned int)wval, 
								(unsigned int) rval);
				}
			}
			else
			{
				wl_print(s, "Usage: bb rxpeon   (enable RXPE)\n"
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
	}
	else if(!strcmp(argv[0], "rf"))
	{
		if(argc >= 2)
		{
			if(!strcmp("init", argv[1]))
			{
				wl_print(s, "init RF to 11g mode\n");
				rf_init();
			}
			else if(!strcmp("channel", argv[1]))
			{
				int channel;
				sscanf(argv[2],"%d",&channel);
				wl_print(s, "RF switch to 2.4GHZ channel %d\n", channel);
				rf_set_channel(0, channel, 0);
			}
			else if(!strcmp("read", argv[1]))
			{
				int v;
				if(argc != 3)
					goto Usage;
				sscanf(argv[2], "%x", &v);
				wl_print(s, "read reg %x=0x%x\n", v, rf_read(v));
			}
			else if(!strcmp("write", argv[1]))
			{
				int v, v1;
				if(argc != 4)
					goto Usage;
				sscanf(argv[2], "%x", &v);
				sscanf(argv[3], "%x", &v1);
				wl_print(s, "write reg %d=0x%x\n", v, v1);
				rf_write(v, v1);
			}
			else if(!strcmp("readall", argv[1]))
			{
				int v;
				for(v=0; v<=0x1f; v++)
					wl_print(s, "read reg %x=0x%x\n", v, rf_read(v));
			}
			else if(!strcmp("spi_idx", argv[1]))
			{
				int spi_idx;
				if(argc != 3)
					wl_print(s, "%d\n", my_rf_dev->spi_idx);
				else
				{
					sscanf(argv[2], "%d", &spi_idx);
					my_rf_dev->spi_idx = spi_idx;
				}
			}
#if defined(CONFIG_RFC_ANALYST) && defined(RFC_DEBUG)			
			else if(!strcmp("rfc_ht_new", argv[1]))
			{
				rfc_ht_cmd_new(argc, argv);
			}
			else if(!strcmp("config_rfc_parm", argv[1]))
			{
				int bw;
				if(argc != 3)
					goto Usage;

				sscanf(argv[2], "%d", &bw);
				config_rfc_parm_new(bw);
			}
#endif			
			else if(!strcmp("config_rfc_parm_new", argv[1]))
			{
				int bw;
				if(argc != 3)
					goto Usage;
				sscanf(argv[2], "%d", &bw);
				config_rfc_parm_new(bw);
			}
			else if(!strcmp("tx_loopback", argv[1]))
			{
				if((argc != 11) && (argc != 13))
					goto Usage;
				//tx_loopback_cmd(argc, argv);
			}
#if 0			
			else if(!strcmp("txlo_cal", argv[1]))
			{
				int sel;
				if(argc != 3)
					goto Usage;
				sscanf(argv[2], "%d", &sel);
				txlo_cal_cmd(sel);
			}
			else if(!strcmp("txlo_2ant", argv[1]))
			{
				int rf;
				if(argc != 3)
					goto Usage;
				sscanf(argv[2], "%d", &rf);
				txlo_cal_cmd_2ant(rf);
			}
#endif			
			else if(!strcmp("rxvga_adjust", argv[1]))
			{
				int ovth, okth;
				if(argc != 4)
					goto Usage;
				sscanf(argv[2], "%d", &ovth);
				sscanf(argv[3], "%d", &okth);
				rxvga_adjust(ovth,okth);
			}
			else
			{
				goto Usage;
			}
			return CLI_OK;
		}
Usage:
		wl_print(s, "Usage: rf init\n"
			"       rf channel <1~14>\n"
			"       rf read <reg>\n"
			"       rf write <reg> <value>\n"
			"       rf readall\n"
			"       rf spi_idx <idx>\n"
			"       rf rfc_ht_new <loop> <tone_mask> <bw> <debug> <rx_scale> <tx_scale> <rx_txvga> <tx_txvga>(bw: bit.0:20MHz, bit.1:40MHz)\n"
			"       rf txlo_cal <sel> (sel=0: original version, sel=1: light version)\n"
			"       rf txlo_2ant <rf> (rf=0: rf0, rf=1: rf1)\n"
			"       rf rxvga_adjust <ovth> <okth> (ovth: 110, okth:85)\n"
			"       rf config_rfc_parm <bw> (bw: 0=20mhz, 1=40mhz)\n"
			"       rf config_rfc_parm_new <bw> (bw: 0=20mhz, 1=40mhz)\n"
			"       rf tx_loopback <bw> <tg_a_i> <tg_a_q> <txvga> <tx_nm> <br22_phase> <br24_gain> <lpf_rst> <lpf_sel> <loop> <sel> (loop & sel is not must, sel=0: test (15,2)&(2,15), sel=1: test(15,2)&(2,2))\n"
			"       rf rf_tx_power_cal \n"
			);
	}
#ifdef CONFIG_CHEETAH_SMART_CONFIG	
	else if(!strcmp(argv[0], "smrtcfg"))
	{
		struct smartcfg *cfg = &sc->smrtcfg;
			
		if(!strcmp("sniffer", argv[1])) {
			if(argc < 3 )
				return 0;
			if(!strcmp("ta", argv[2])) {
				MACREG_UPDATE32(ERR_EN, ERR_EN_DATA_TA_MISS_TOCPU, ERR_EN_DATA_TA_MISS_TOCPU);
				idb_print("TA MISS TO CPU ON\n");
			}
			else if(!strcmp("lmac", argv[2])) {
				MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
				idb_print("LMAC FILTER ALL PASS ON\n");
			}
			else if(!strcmp("all", argv[2])) {
				MACREG_UPDATE32(ERR_EN, ERR_EN_DATA_TA_MISS_TOCPU, ERR_EN_DATA_TA_MISS_TOCPU);
				MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
				idb_print("TA MISS TO CPU ON\n");
				idb_print("LMAC FILTER ALL PASS ON\n");
			}
			else if(!strcmp("dis", argv[2])) {
				MACREG_UPDATE32(ERR_EN, 0, ERR_EN_DATA_TA_MISS_TOCPU);
				MACREG_UPDATE32(RTSCTS_SET, 0, LMAC_FILTER_ALL_PASS);
				idb_print("DISABLE ALL\n");
			}
		}
		else if(!strcmp("start", argv[1])) {
			if(argc < 3) {
				smart_start(cfg, 0);
			}
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
			else if(!strcmp("fixed", argv[2])) {
				smart_start(cfg, 4);
			}
#endif
			else {
				int chansec;
				sscanf(argv[2],"%d",&chansec);
				smart_start(cfg, chansec);
			}
		}
#ifndef CONFIG_CHEETAH_SMART_TO_USRSPACE
		else if(!strcmp("debug", argv[1])) {
    			if(argc < 3)
				return 0;
			if((!strcmp("open", argv[2]))||(!strcmp("1", argv[2])))
			{
			    smrtcfg_debug_flag=0x01;
			}
			else if((!strcmp("close", argv[2]))||(!strcmp("0", argv[2])))
			{
			    smrtcfg_debug_flag=0x00;
			}
		}
#endif
		else if(!strcmp("stop", argv[1])) {
			smart_stop(cfg);
		}
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
		else if(!strcmp("chan", argv[1])) {
			if(argc == 3) {
				int chan;
				sscanf(argv[2],"%d",&chan);
				wla_chgchan_fixed(cfg, chan);
				wl_print(s, "CHAN:%d\n",chan);
			}
		}
#endif
		else {
			wl_print(s, "ENABLE:%d\n",__smartcfg_enable);
			wl_print(s, "BSSDESC:%d\n",cfg->bss_desc);
			//wl_print(s, "BSSID:%s\n",wmaddr_ntoa(cfg->bssid));
			wl_print(s, "TA:%s\n", wmaddr_ntoa(cfg->ta));
			wl_print(s ,"state=%d\ncur_chan=%d\n", cfg->state, cfg->cur_chan);
			
			if(cfg->datalen)
			    wl_print(s, "Data length:%d\n",cfg->datalen);
		
			wl_print(s, "Channel timing:%d secs\n",cfg->chantime/HZ);
			wl_print(s, "Total Timing:%d secs\n",cfg->total/HZ);
		}
	}
#endif
	else if(!strcmp(argv[0], "func")) {
		wl_print(s, "curfunc:%p\n",curfunc);
	}
    return 0;
}

extern int get_args (const char *string, char *argvs[]);
struct idb_command idb_wifidump_cmd =
{
    .cmdline = "wd",
    .help_msg = "wd                         Dump WiFi informations", 
    .func = wifidump_func,
};
#endif

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
static int wl_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(buf, 0, 300);

	if (count > 0 && count < 299) {
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
	}
	
	return count;
}

static int wl_show(struct seq_file *s, void *priv)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	int rc;
	int	argc ;
	char * argv[MAX_ARGV] ;

	sc->seq_file = s;
	argc = get_args( (const char *)buf , argv );
	rc=wifidump_func(argc, argv);
	sc->seq_file = NULL;

	return 0;
}

static int wl_open(struct inode *inode, struct file *file)
{
    int ret;
	
    ret = single_open(file, wl_show, NULL);
		
	return ret;
}
static const struct file_operations wl_fops = {
    .open       = wl_open,
    .read       = seq_read,
	.write		= wl_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#endif

int wla_debug_init(void *wla_sc)
{
#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
	struct proc_dir_entry *res;
    
    register_idb_command(&idb_wifidump_cmd);
		
	res = create_proc_entry("wd", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->proc_fops = &wl_fops;
#endif

    return 0;
}

void wla_debug_exit(void)
{
#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
    unregister_idb_command(&idb_wifidump_cmd);
	remove_proc_entry("wd",NULL);
#endif
}
