/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_rate.c
*   \brief  wla wlan TX rate functions.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
#include <wbuf.h>
//#include <wlan_def.h>
#include <wla_debug.h>

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define RC_LV1		1
#define RC_LV2		2
#define RC_LV3		3

#define RC_DBG(LEV, _str...) \
	do {                           \
		if (info->rc_debug >= LEV) \
			DBG(_str);     \
	} while(0)

#define RC_COUNTER_DELTA(delta, prev_hw_counter, current_hw_counter) \
do {                                                                   \
	u16 current_val = (current_hw_counter);                            \
	int hw_counter_width = sizeof(current_hw_counter) * 8;          	\
	                                                                   \
	if((current_val) < (prev_hw_counter))                              \
	{                                                                  \
		(delta) = ((0x01UL << hw_counter_width) - ((prev_hw_counter) - (current_val))); \
	}                                                                  \
	else                                                               \
	{                                                                  \
		(delta) = ((current_val) - (prev_hw_counter));                 \
	}                                                                  \
																	   \
	(prev_hw_counter) = (current_val);                                 \
} while(0); 

static struct wlan_rate wla_rates[] = {
		{ .hw_value = CCK_1M_CODE, .flags = B_RATE, .bitrate = 10, .perfect_tp = 948 },
		{ .hw_value = CCK_2M_CODE, .flags = B_RATE, .bitrate = 20, .perfect_tp = 1817  },
		{ .hw_value = CCK_5_5M_CODE, .flags = B_RATE, .bitrate = 55, .perfect_tp = 4364  },
		{ .hw_value = CCK_11M_CODE, .flags = B_RATE, .bitrate = 110, .perfect_tp = 7277  },
		/* OFDM */
		{ .hw_value = OFDM_6M_CODE, .flags = G_RATE, .bitrate = 60, .perfect_tp = 5499 },
		{ .hw_value = OFDM_9M_CODE, .flags = G_RATE, .bitrate = 90, .perfect_tp = 7961 },
		{ .hw_value = OFDM_12M_CODE, .flags = G_RATE, .bitrate = 120, .perfect_tp = 10275 },
		{ .hw_value = OFDM_18M_CODE, .flags = G_RATE, .bitrate = 180, .perfect_tp = 14416 },
		{ .hw_value = OFDM_24M_CODE, .flags = G_RATE, .bitrate = 240, .perfect_tp = 18054 },
		{ .hw_value = OFDM_36M_CODE, .flags = G_RATE, .bitrate = 360, .perfect_tp = 24149 },
		{ .hw_value = OFDM_48M_CODE, .flags = G_RATE, .bitrate = 480, .perfect_tp = 28917 },
		{ .hw_value = OFDM_54M_CODE, .flags = G_RATE, .bitrate = 540, .perfect_tp = 30953 },
		/* HT */
		{ .hw_value = MCS_0_CODE, .flags = HT_RATE, .bitrate = 65, .perfect_tp = 6326  },
		{ .hw_value = MCS_1_CODE, .flags = HT_RATE, .bitrate = 130, .perfect_tp = 12430 },
		{ .hw_value = MCS_2_CODE, .flags = HT_RATE, .bitrate = 195, .perfect_tp = 18323  },
		{ .hw_value = MCS_3_CODE, .flags = HT_RATE, .bitrate = 260, .perfect_tp = 24017  },
		{ .hw_value = MCS_4_CODE, .flags = HT_RATE, .bitrate = 390, .perfect_tp = 34767 },
		{ .hw_value = MCS_5_CODE, .flags = HT_RATE, .bitrate = 520, .perfect_tp = 44791 },
		{ .hw_value = MCS_6_CODE, .flags = HT_RATE, .bitrate = 585, .perfect_tp = 49553  },
		{ .hw_value = MCS_7_CODE, .flags = HT_RATE, .bitrate = 650, .perfect_tp = 54084  },
		{ .hw_value = MCS_8_CODE, .flags = HT_RATE, .bitrate = 130, .perfect_tp = 12360  },
		{ .hw_value = MCS_9_CODE, .flags = HT_RATE, .bitrate = 260, .perfect_tp = 23868 },
		{ .hw_value = MCS_10_CODE, .flags = HT_RATE, .bitrate = 380, .perfect_tp = 34610 },
		{ .hw_value = MCS_11_CODE, .flags = HT_RATE, .bitrate = 520, .perfect_tp = 44661 },
		{ .hw_value = MCS_12_CODE, .flags = HT_RATE, .bitrate = 780, .perfect_tp = 62680 },
		{ .hw_value = MCS_13_CODE, .flags = HT_RATE, .bitrate = 1040, .perfect_tp = 78320 },
		{ .hw_value = MCS_14_CODE, .flags = HT_RATE, .bitrate = 1170, .perfect_tp = 85742  },
		{ .hw_value = MCS_15_CODE, .flags = HT_RATE, .bitrate = 1300, .perfect_tp = 92442  },

		{ .hw_value = MCS_32_CODE, .flags = HT_RATE, .bitrate = 60 },

};

u8 tx_retry[4] = {2,2,2,2};
u8 ampdu_tx_retry[4] = {1,1,1,1};

static void rc_setup_multi_txrate(MAC_INFO *info, struct wsta_cb *cb, struct tx_rate_info *txrate_i);

unsigned char get_hw_bitrate_format(int tx_rate_idx)
{
	if(wla_rates[tx_rate_idx - 1].flags & HT_RATE)
		return FMT_HT;
	else if(tx_rate_idx <= CCK_11M)
		return FMT_11B; 
	else
		return FMT_NO_HT; 
}

inline unsigned char get_hw_bitrate_code(int tx_rate_idx)
{
	return wla_rates[tx_rate_idx - 1].hw_value;
}

unsigned int get_hw_bitrate(int tx_rate_idx)
{
	return wla_rates[tx_rate_idx - 1].bitrate;
}

unsigned char primary_ch_offset(u8 bw_type)
{
	if(bw_type == HT40_MINUS)
		return CH_OFFSET_20U;
	else if(bw_type == HT40_PLUS)
		return CH_OFFSET_20L;
	else 
		return CH_OFFSET_20; 
}

short wla_rate_encode(MAC_INFO* info, unsigned char tx_rate_idx, unsigned char retry, 
		unsigned char flags)
{
	rate_code rate;

	rate.val = 0;
	rate.bit.tx_rate = wla_rates[tx_rate_idx - 1].hw_value;	
	rate.bit.txcnt = retry;

	if(wla_rates[tx_rate_idx - 1].flags & HT_RATE)
	{
		rate.bit.ht = 1;
		if(flags & RATE_FLAGS_HT_GF)
			rate.bit.format = FMT_HT_GF; 
		else	
			rate.bit.format = FMT_HT;
 
		if((flags & RATE_FLAGS_HT_40M) && (info->bandwidth_type != BW20_ONLY) && 
			!info->bw40mhz_intolerant)
		{
			rate.bit.ch_offset = CH_OFFSET_40;
			if(flags & RATE_FLAGS_HT_SGI40)
				rate.bit.sgi = 1;
		}
		else
		{
			rate.bit.ch_offset = primary_ch_offset(info->bandwidth_type);
			if(flags & RATE_FLAGS_HT_SGI20)
				rate.bit.sgi = 1;
		}
	}
	else
	{
		if(tx_rate_idx <= CCK_11M)
		{
			rate.bit.format = FMT_11B; 
			if(flags & RATE_FLAGS_SHORT_PREAMBLE)
				rate.bit.tx_rate |= FMT_11B_SHORT_PREAMBLE;
		}
		/* non-HT only run in BW 20Mhz. Let LMAC auto determine 20U or 20L according 
			LMAC_CH_BW_CTRL_CH_OFFSET of BASIC_SET. */
		rate.bit.ch_offset = primary_ch_offset(info->bandwidth_type);
	}

	return rate.val; 
}

int rate_tbl_get_free(MAC_INFO *info)
{
	int tbl_index;
	sta_cap_tbl *captbl;

	for(tbl_index=0; tbl_index<info->rate_tbl_count; tbl_index++)
	{
		captbl = &info->rate_tbls[tbl_index];
		if(!captbl->valid)
			break;
	}
	if(tbl_index >= info->rate_tbl_count)
	{
		return -1;
	}
	return tbl_index;
}

#ifdef CONFIG_SRAM_WLA
#define CLA					aligned(32)
#if (defined(CONFIG_SRAM_SIZE) && (CONFIG_SRAM_SIZE >= 128))
#define IN_SRAM				section(".sram")
#else
#define IN_SRAM	
#endif

sta_tx_status_tbl basic_tx_tbl  __attribute__((CLA, IN_SRAM));
#endif
char setup_rate_table(MAC_INFO *info, sta_cap_tbl *captbl, char bss, u8 *txrate_idx, int rate_flags, char ack_policy)
{
    sta_tx_status_tbl *tx_tbl;
	int i;

	memset(captbl, 0, sizeof(sta_cap_tbl));
#ifdef CONFIG_SRAM_WLA
	tx_tbl = &basic_tx_tbl;
#else
	tx_tbl = WLA_MALLOC(sizeof(sta_tx_status_tbl), DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);
	if(!tx_tbl)
	{
		captbl->valid = 0;	
		return 0;
	}
#endif

	tx_tbl->ack_policy = ack_policy;
	tx_tbl->long_retry = LEGACY_LONG_RETRY_DEFAULT;
	tx_tbl->short_retry = LEGACY_SHORT_RETRY_DEFAULT;
	
	captbl->TX_tbl_addr = (u32) PHYSICAL_ADDR(tx_tbl);
	captbl->RX_tbl_addr = 0;        // XXX: RX table should not be needed in RATE table

	/* these settings will be used when toggling gsn in TX descr */
	captbl->glong_retry = LEGACY_LONG_RETRY_DEFAULT;
	captbl->gshort_retry = LEGACY_SHORT_RETRY_DEFAULT;
	captbl->gack_policy = ack_policy;

	captbl->BG = 1;
	captbl->NE = 1;
	//captbl->gsn = 1;		/* use global sequence number */
	captbl->bssid = bss;
	captbl->valid = 1;

	for(i = 0; i < 4; i++)
	{
		captbl->rate[i].val = wla_rate_encode(info, txrate_idx[i], tx_retry[i], 
					rate_flags);
	}	

	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

	return 1; 
}

void free_rate_table(sta_cap_tbl *captbl)
{
	captbl->valid = 0;
	if(captbl->TX_tbl_addr)
	{
#ifdef CONFIG_SRAM_WLA
		sta_tx_status_tbl *tx_tbls;
		tx_tbls = (sta_tx_status_tbl *)NONCACHED_ADDR(captbl->TX_tbl_addr);
		memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
#else
		WLA_MFREE((void *)CACHED_ADDR(captbl->TX_tbl_addr));
#endif
		captbl->TX_tbl_addr = 0;
	}
}

void wla_setup_tx_rate(MAC_INFO *info, short bss, int rate_index, char type, int flags)
{
	int tbl_idx, i;
	sta_cap_tbl *captbl;
	u32 regval;
	struct tx_rate_t *trate;
	u8 txrate_idx[4];

	if(info->mbss[bss].role == 0)
		return;

	/* MBSSIDs should have same TX rate, but may have different cipher mode */
	trate = &(info->mbss[bss].tx_rate[(short)type]);

	if(trate->rate_idx)
	{
		captbl = &info->rate_tbls[trate->rate_captbl];
		free_rate_table(captbl);
	}

	trate->rate_captbl = 0;
	trate->rate_idx = 0;

	/* negative rate index mean clean old table only */
	if(rate_index < 0)
		return;

	if((tbl_idx = rate_tbl_get_free(info)) < 0)
		panic("Not enough rate table\n");

	captbl = &info->rate_tbls[tbl_idx];

	for(i=0; i<4; i++)
		txrate_idx[i] = rate_index;

	if(!setup_rate_table(info, captbl, bss, txrate_idx, flags, MAC_ACK_POLICY_NORMAL_ACK))
		panic("setup rate table fail\n");

	trate->rate_captbl = tbl_idx;
	trate->rate_idx = rate_index;

	if(type == BASIC_TX_RATE)
	{
		/* FIXME:  If we play client role, need to change parent_ap and AP bits? */
		if(info->mbss[bss].role == WIF_IBSS_ROLE)
		{
			captbl->parent_ap = 1;
			captbl->AP = 0;
		}

		/* set the BC/MC PS TX rates for the BSSID */
		if(bss<4)
		{
			regval = MACREG_READ32(PSBA_PS_BC_BSSID_RATE_SR0);
			regval &= (~(0x0FFUL << (bss * 8)));
			regval |= ((tbl_idx & 0x0FF) << (bss * 8));
			MACREG_WRITE32(PSBA_PS_BC_BSSID_RATE_SR0, regval);
		}
		else
		{
			regval = MACREG_READ32(PSBA_PS_BC_BSSID_RATE_SR1);
			regval &= (~(0x0FFUL << ((bss-4) * 8)));
			regval |= ((tbl_idx & 0x0FF) << ((bss-4) * 8));
			MACREG_WRITE32(PSBA_PS_BC_BSSID_RATE_SR1, regval);
		}
	}
	else if(type == MULTICAST_TX_RATE)
	{
		captbl->gsn = 1;		/* use global sequence number */
	}

	return;
}

static void rc_setup_multi_txrate(MAC_INFO *info, struct wsta_cb *cb, struct tx_rate_info *txrate_i)
{
	int i, idx;
	sta_cap_tbl *captbl = &info->sta_cap_tbls[cb->captbl_idx];
	unsigned char rate_flags = cb->rate_flags;
	char wait;

	for(i = 0; i < 4; i++)
	{
		idx = txrate_i[i].rate_idx;
		/* Chuchen found MCS0 + long AMPDU + SGI may make Intel hard to receive AMPDU */
		if(cb->tx_ampdu_bitmap)
		{
			if(idx == MCS_0)
			{
				rate_flags = cb->rate_flags & ~(RATE_FLAGS_HT_SGI20|RATE_FLAGS_HT_SGI40);
				captbl->rate[i].val = wla_rate_encode(info, idx, ampdu_tx_retry[i], rate_flags);
			}
			else
				captbl->rate[i].val = wla_rate_encode(info, idx, ampdu_tx_retry[i], cb->rate_flags);
		}
		else
		{
			captbl->rate[i].val = wla_rate_encode(info, idx, tx_retry[i], cb->rate_flags);
		}
	}

	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
	if(cb->basic_rc_flag)
		wait = 0;
	else
		wait = 1;
	mac_stacap_cache_sync(cb->captbl_idx, wait);

	RC_DBG(RC_LV2, "sta[%d], rate=%x, %x, %x, %x\n", cb->captbl_idx, captbl->rate[0].val, captbl->rate[1].val, captbl->rate[2].val, captbl->rate[3].val);
}


void wla_rc_init(MAC_INFO *info, struct wsta_cb *cb)
{
	sta_cap_tbl *captbl = &info->sta_cap_tbls[cb->captbl_idx];
	u32 i, j, cur_rate_idx, fix_rate_idx;
	struct bts_cb *bcb = &cb->bts;
	struct bts_rate *br;
	int cur_br_idx = -1, min_br_idx;

	cur_rate_idx = cb->cur_tx_rate[0].rate_idx;

	j = 0;
	min_br_idx = 0;

	fix_rate_idx = info->mbss[(u32)captbl->bssid].tx_rate[FIXED_TX_RATE].rate_idx;
	if(fix_rate_idx)
	{
		for(i = 0; i < 4; i++)
		{
			cb->cur_tx_rate[i].rate_idx = fix_rate_idx;
			cb->cur_tx_rate[i].bts_rates = 0;		
		}
		br = &bcb->rates[0];
		bcb->rates_len = 1;
		memset(br, 0, sizeof(struct bts_rate));
		br->rate_idx = fix_rate_idx;
		if(fix_rate_idx >= MCS_0)
		  cb->rc_allow_ampdu = 20;
	}
#if defined(WLAN_RC_BTS)
	else
	{
		if(cb->tx_ampdu_bitmap)
		{
			cb->rc_allow_ampdu = 20;
			/* AMPDU only supports HT rate */
			for(i=MCS_0; i<=MCS_7; i++)
			{
				if(!(cb->supported_rates & R_BIT(i)))
					continue;
				br = &bcb->rates[j++];
				memset(br, 0, sizeof(struct bts_rate));
				//br->credit = 100;
				br->rate_idx = i;
				if(cur_rate_idx == i)
				{
					cur_br_idx = j-1;
					if(cur_br_idx > 0)
						min_br_idx = cur_br_idx - 1;
				}
			}
		}
		else if(cb->supported_rates & R_BIT(MCS_0))
		{
			/* Uses B and HT rates in 11N mode */
			for(i=CCK_1M; i<=MCS_7; i++)
			{
				if(i == CCK_11M)
					i = MCS_0;
				else if(i == MCS_1)
					i = CCK_11M;
				else if(i == OFDM_6M)
					i = MCS_1;
				if(!(cb->supported_rates & R_BIT(i)))
					continue;
				br = &bcb->rates[j++];
				memset(br, 0, sizeof(struct bts_rate));
				//br->credit = 100;
				br->rate_idx = i;
				if(cur_rate_idx == i)
				{
					cur_br_idx = j-1;
				}
			}
		}
		else
		{
			cb->rc_allow_ampdu = 0;
			/* Uses B and G rate in 11G mode */
			for(i=CCK_1M; i<=OFDM_54M; i++)
			{
				if(i == CCK_11M)
					i = OFDM_6M;
				else if(i == OFDM_12M)
					i = CCK_11M;
				else if(i == OFDM_6M)
					i = OFDM_12M;
				if(!(cb->supported_rates & R_BIT(i)))
					continue;
				br = &bcb->rates[j++];
				memset(br, 0, sizeof(struct bts_rate));
				//br->credit = 100;
				br->rate_idx = i;
				if(cur_rate_idx == i)
				{
					cur_br_idx = j-1;
				}
			}
		}

		bcb->rates_len = j;

		if(cur_br_idx > 0)
			j = cur_br_idx;	
		else
			j = bcb->rates_len-1;
	
		for(i=0; i<3; i++)
		{
			cb->cur_tx_rate[i].bts_rates = j;
			cb->cur_tx_rate[i].rate_idx = bcb->rates[j].rate_idx;
			bcb->rates[j].avg_prob = 100;
			bcb->rates[j].tp = wla_rates[bcb->rates[j].rate_idx-1].perfect_tp * 100;
			if(j > min_br_idx)
				j--;
		}

		bcb->rates[min_br_idx].avg_prob = 100;
		bcb->rates[min_br_idx].tp = wla_rates[bcb->rates[min_br_idx].rate_idx-1].perfect_tp * 100;
		cb->cur_tx_rate[3].bts_rates = min_br_idx;
		cb->cur_tx_rate[3].rate_idx = bcb->rates[min_br_idx].rate_idx;
	}
#endif

	RC_DBG(RC_LV1, ">STA[%d], RATE init(%d)\n", cb->captbl_idx, cb->cur_tx_rate[0].rate_idx );
	rc_setup_multi_txrate(info, cb, &cb->cur_tx_rate[0]);
}


int check_spec_rate(MAC_INFO *info, struct wsta_cb *cb)
{
	struct bts_cb *bcb = &cb->bts;
	int i;

	if(cb->basic_rc_count) 
	{ 
        if(!bcb->aux) 
        {
                cb->basic_rc_flag = 1; 
                bcb->aux = 1; 
                /* Use lowest rate to get highest probability */ 
                for(i=0; i<4; i++)
				{ 
					bcb->aux_tx_rate[i].rate_idx = bcb->rates[0].rate_idx;
					bcb->aux_tx_rate[i].bts_rates = 0; 
				}
                rc_setup_multi_txrate(info, cb, &bcb->aux_tx_rate[0]); 
        }
		return HW_SYNC_TIME;
	} 
	else 
	{ 
        if(bcb->aux && cb->basic_rc_flag) 
        {
                bcb->aux = 0; 
                rc_setup_multi_txrate(info, cb, &cb->cur_tx_rate[0]); 
                cb->basic_rc_flag = 0;
				return HW_SYNC_TIME;
        } 

		return 0;
	}
}

#if defined(WLAN_RC_BTS)
u32 wla_bts_update(MAC_INFO *info, struct wsta_cb *cb, struct rate_stats *tx_rates, int ampdu_retry)
{
	struct bts_cb *bcb = &cb->bts;
	struct bts_rate *br;
	u32 prob, max_tp, max_prob, max_tp_idx, max_prob_idx, delta, total_attempts, tp;
	s32 i, j, compare_range_max, compare_range_min;
	struct tx_rate_info  *txrate_i;	
	u8 weight;
	u32 interval = HW_SYNC_TIME;
	u32 cur_time = WLA_CURRENT_TIME;

	tx_rates[0].attempts += ampdu_retry;	
	if(tx_rates[0].attempts == 0)
	{
		return interval;
	}
	if(bcb->aux)
		txrate_i = bcb->aux_tx_rate;
	else
		txrate_i = cb->cur_tx_rate;

	
	total_attempts = 0;
	compare_range_min = 0;

	if(cb->tx_ampdu_bitmap)
	{
		u16 fallbacks;
		ucast_qinfo *qinfos;
		qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[cb->captbl_idx].qinfo_addr);

		RC_DBG(RC_LV2, "A[%d|%d]:%d/%d,    [%d|%d]:%d/%d,    [%d|%d]:%d/%d,   [%d|%d]:%d/%d, eq/dq:%d/%d\n", txrate_i[0].rate_idx, txrate_i[0].bts_rates,tx_rates[0].success, tx_rates[0].attempts, txrate_i[1].rate_idx, txrate_i[1].bts_rates, tx_rates[1].success, tx_rates[1].attempts , txrate_i[2].rate_idx, txrate_i[2].bts_rates, tx_rates[2].success, tx_rates[2].attempts, txrate_i[3].rate_idx, txrate_i[3].bts_rates, tx_rates[3].success, tx_rates[3].attempts, qinfos->eq_agg_cnt, qinfos->dq_agg_cnt);

		i = txrate_i[0].bts_rates;
		br = &bcb->rates[i];

		/* Under PSBAQ processing, success count means first trial count for that rate. Once no BACK received, it will fallback to next rate and retransmit. We assume these fallback frames is next attemtps minus next success. */
		fallbacks = tx_rates[1].attempts - tx_rates[1].success;
		if(fallbacks > tx_rates[0].success)
			fallbacks = tx_rates[0].success;

		br->success = tx_rates[0].success - fallbacks;
		br->attempts = tx_rates[0].attempts;

		prob = (br->success * 100)/br->attempts;

		/* adjust moving average weight for EWMA */
		if(tx_rates[0].attempts < 2)
			weight = 95;
		else if(tx_rates[0].attempts < 10)
			weight = 80;
		else if(tx_rates[0].attempts < 50)
			weight = 50;
		else if(tx_rates[0].attempts < 100)
			weight = 30;
		else
			weight = 5;

		br->avg_prob = ((prob * (100 - weight)) + (br->avg_prob * weight)) / 100;
		/* throughput = (average probability) * (perfact throughput) */
		br->tp = br->avg_prob * wla_rates[br->rate_idx - 1].perfect_tp;
		br->timestamp = cur_time;

		//br->credit = prob;
		total_attempts = tx_rates[0].attempts;
		RC_DBG(RC_LV3, "fallbacks=%d, success=%d, prob=%d, tp=%d\n", fallbacks, tx_rates[0].success - fallbacks, prob, br->tp);

		i = cb->cur_tx_rate[0].bts_rates;
		
		compare_range_max = i + 1 ;
		//compare_range_min = i - 1;
		compare_range_min = 0;
		if(compare_range_max >= bcb->rates_len) 
			compare_range_max = bcb->rates_len - 1;
		else if(i == 0) 
			compare_range_min = 0;

		/* rise lower rate's TP to be winner */
		if(ampdu_retry)
		{
			if(i == 0 || (ampdu_retry>=5))
			{
				/*Punishment*/
				if(ampdu_retry>=5)
					cb->bad_ampdu_tx = 100;
				else
					cb->bad_ampdu_tx = 30;

				cb->rc_allow_ampdu = 0;
			}
			else
			{
				br = &bcb->rates[i - 1];
				br->avg_prob = 100;
				/* throughput = (average probability) * (perfact throughput) */
				br->tp = br->avg_prob * wla_rates[br->rate_idx - 1].perfect_tp;
			}
			//bcb->aux = 0;
		}
	}
	else
	{
		RC_DBG(RC_LV2, "M[%d]:%d/%d,    [%d]:%d/%d,    [%d]:%d/%d,   [%d]:%d/%d\n", txrate_i[0].rate_idx, tx_rates[0].success, tx_rates[0].attempts, txrate_i[1].rate_idx, tx_rates[1].success, tx_rates[1].attempts , txrate_i[2].rate_idx, tx_rates[2].success, tx_rates[2].attempts, txrate_i[3].rate_idx, tx_rates[3].success, tx_rates[3].attempts);

		/* accumulate statistic of same rate */
		for(i=0; i<4; i++)
		{
			if((i != 3) && (txrate_i[i].rate_idx == txrate_i[i+1].rate_idx))
			{
				tx_rates[i+1].attempts += tx_rates[i].attempts;
				tx_rates[i+1].success += tx_rates[i].success;
				tx_rates[i].attempts = 0;
				tx_rates[i].success = 0;
			}
		}

		for(i=0; i<4; i++)
		{ 
			total_attempts += tx_rates[i].attempts;
			if(tx_rates[i].attempts)
			{
				br = &bcb->rates[txrate_i[i].bts_rates];

				delta = cur_time - br->timestamp;
			
				/* adjust moving average weight for EWMA */
				if(tx_rates[i].attempts < 2)
					weight = 95;
				else if(tx_rates[i].attempts < 10)
					weight = 80;
				else if(tx_rates[i].attempts < 50)
					weight = 50;
				else if(tx_rates[i].attempts < 100)
					weight = 30;
				else
					weight = 5;

				/* probabilities scale from 0 (0%) to 100 (100%) */
				prob = (tx_rates[i].success * 100)/tx_rates[i].attempts;

				br->avg_prob = ((prob * (100 - weight)) + (br->avg_prob * weight)) / 100;
				/* throughput = (average probability) * (perfact throughput) */
				br->tp = br->avg_prob * wla_rates[br->rate_idx - 1].perfect_tp;
				br->timestamp = cur_time;
				br->success = tx_rates[i].success;
				br->attempts = tx_rates[i].attempts;

				//br->credit = prob;
			} 
		} 	

		i = cb->cur_tx_rate[0].bts_rates;

		compare_range_max = i + 2;
		compare_range_min = 0;
		if(compare_range_max >= bcb->rates_len) 
			compare_range_max = bcb->rates_len - 1;
	}

	if((total_attempts < 4) && !bcb->aux)
		return interval;

	/* Only gather statistic under fixed rate mode. */
	if(info->mbss[cb->bss_desc].tx_rate[FIXED_TX_RATE].rate_idx)
		return interval;

	/* Trigger sample frame when current rate is not the best. */
	while(((cur_time - bcb->sample_timestamp) > 100) || bcb->aux)
	{
		j = 0;

		max_tp = bcb->rates[cb->cur_tx_rate[0].bts_rates].tp;
		if(bcb->aux)
		{
			RC_DBG(RC_LV3, "Find next sample rate>>>>>>>>, max_tp=%d\n", max_tp);
			i = bcb->aux_tx_rate[2].bts_rates - 1;
		}
		else
		/* Only trigger sample procedure once within one second. */
		{
			RC_DBG(RC_LV3, "Find sample rate>>>>>>>>, max_tp=%d, compare_range_max=%d\n", max_tp, compare_range_max);
			/* from fastest rate */
			i = compare_range_max;
			bcb->sample_timestamp = cur_time;
		}

		for(; i>=compare_range_min; i--)
		{
			/* skip sampled rate */ 
			br = &bcb->rates[i];
			if(br->timestamp == cur_time)
				continue;
			if(cb->cur_tx_rate[0].bts_rates == i)
				continue;

			prob = (cur_time - br->timestamp)/3 + br->avg_prob;
			if(prob > 100)
				prob = 100;
			tp = wla_rates[br->rate_idx - 1].perfect_tp*prob;
			RC_DBG(RC_LV3, "#############try [%d], prob=%d, tp=%d\n", br->rate_idx, prob, tp);
			if(tp > max_tp)
			{
				RC_DBG(RC_LV3, "######good\n");
				bcb->aux_tx_rate[j].bts_rates = i;
				bcb->aux_tx_rate[j].rate_idx = br->rate_idx;
				j++;
			
				/* AMPDU only try one rate */	
				if((j >= 3)||cb->tx_ampdu_bitmap)
					break;
			}
			else if(prob == 100)
				break;
		}
	
		if(j > 0)
		{
			/* default is rate of highest probability */
			for(i=j; i<3; i++)
			{
				if(bcb->aux_tx_rate[j-1].bts_rates >  cb->cur_tx_rate[0].bts_rates)
					bcb->aux_tx_rate[i] = cb->cur_tx_rate[0];
				else
					bcb->aux_tx_rate[i] = bcb->aux_tx_rate[j-1];
			}
			bcb->aux_tx_rate[3].bts_rates = compare_range_min;
			bcb->aux_tx_rate[3].rate_idx = bcb->rates[compare_range_min].rate_idx;
 	
			RC_DBG(RC_LV2, "@@send sample, AUX rate: %d, %d, %d, %d\n", bcb->aux_tx_rate[0].rate_idx, bcb->aux_tx_rate[1].rate_idx, bcb->aux_tx_rate[2].rate_idx, bcb->aux_tx_rate[3].rate_idx);
			//rc_send_sample_frame(info, cb);
			rc_setup_multi_txrate(info, cb, &bcb->aux_tx_rate[0]);
			bcb->aux = 1;
			interval = 5; /* send sample frame for 50ms */
			return interval;
		}
		else
		{
			if(bcb->aux)
			{
				bcb->aux = 0;
				rc_setup_multi_txrate(info, cb, &cb->cur_tx_rate[0]);
			}
		}

		break;
	}


	max_tp = 0;
	max_prob = 0;
	max_tp_idx = cb->cur_tx_rate[0].bts_rates;
	max_prob_idx = 0;

	for(i=compare_range_max; i>=compare_range_min; i--)
	{
		br = &bcb->rates[i];

		if(max_tp < br->tp)
		{
			max_tp = br->tp;
			max_tp_idx = i;
		}

		if(max_prob < br->avg_prob)
		{
			max_prob = br->avg_prob;
			max_prob_idx = i; 
		}
	}

	RC_DBG(RC_LV3, "max_tp_idx=%d, max_prob_idx=%d\n", max_tp_idx, max_prob_idx);

	/* update rate array if maximum rate changed */
	if(cb->cur_tx_rate[0].bts_rates != max_tp_idx)
	{
		/* When we rise up to higher rate, we give a probation period that second rate keep original rate to avoid false rising. */
		if(max_tp_idx > cb->cur_tx_rate[0].bts_rates)
		{
			bcb->probation = 1;

			cb->cur_tx_rate[2] = cb->cur_tx_rate[1];;
			cb->cur_tx_rate[1] = cb->cur_tx_rate[0];
		}
		else
			bcb->probation = 0;

		RC_DBG(RC_LV1, ">STA[%d], Change rate %d->%d, tp=%d, aux=%d, ampdu_retry=%d\n", cb->captbl_idx, cb->cur_tx_rate[0].rate_idx, bcb->rates[max_tp_idx].rate_idx, max_tp, bcb->aux, ampdu_retry);

		cb->cur_tx_rate[0].rate_idx = bcb->rates[max_tp_idx].rate_idx;
		cb->cur_tx_rate[0].bts_rates = max_tp_idx;	

		i = max_tp_idx;
		if(bcb->probation == 0)
		{
			if(i > compare_range_min)
				i--;
			cb->cur_tx_rate[1].rate_idx = bcb->rates[i].rate_idx;
			cb->cur_tx_rate[1].bts_rates = i;	

			if(i > compare_range_min)
				i--;
			cb->cur_tx_rate[2].rate_idx = bcb->rates[i].rate_idx;
			cb->cur_tx_rate[2].bts_rates = i;	
		}

		cb->cur_tx_rate[3].rate_idx = bcb->rates[compare_range_min].rate_idx;
		cb->cur_tx_rate[3].bts_rates = compare_range_min;	
		bcb->aux = 0;
		rc_setup_multi_txrate(info, cb, &cb->cur_tx_rate[0]);
	}
	else if(bcb->probation)
	{
		i = cb->cur_tx_rate[0].bts_rates;

		if(i > compare_range_min)
			i--;
		cb->cur_tx_rate[1].rate_idx = bcb->rates[i].rate_idx;
		cb->cur_tx_rate[1].bts_rates = i;	

		if(i > compare_range_min)
			i--;
		cb->cur_tx_rate[2].rate_idx = bcb->rates[i].rate_idx;
		cb->cur_tx_rate[2].bts_rates = i;	

		bcb->probation = 0;
		bcb->aux = 0;
		rc_setup_multi_txrate(info, cb, &cb->cur_tx_rate[0]);
	}

	if(!bcb->aux && !bcb->probation)
	{
		br = &bcb->rates[max_tp_idx];

		/* Low HT rate with AMPDU is not good choice, maybe 11Mbps could get higher throughput and long distance. */
		if((br->rate_idx == MCS_0) && cb->rc_allow_ampdu)
			cb->rc_allow_ampdu--;
		else if(cb->bad_ampdu_tx)
		{
			if((br->rate_idx > MCS_4) && (br->avg_prob > 80)) {
				cb->bad_ampdu_tx -= br->rate_idx;
				cb->bad_ampdu_tx=(cb->bad_ampdu_tx>0)?(cb->bad_ampdu_tx):0;
			}
		}
		else if((br->rate_idx  > MCS_2) && (br->avg_prob > 50))
		{
			cb->rc_allow_ampdu = 20;
		}

	}

	return interval;
}

#endif

int wla_rc_update(MAC_INFO *info, struct wsta_cb *cb)
{
	int i;
	u16 tx_fail_cnt;
	struct rate_stats tx_rates[4];
	ucast_qinfo *qinfos;
	sta_cap_tbl *captbl = &info->sta_cap_tbls[cb->captbl_idx];
	u32 next_update = HW_SYNC_TIME;
	int ampdu_retry = 0;

	if(cb->rate_flags & RATE_FLAGS_INIT)
	{
		wla_rc_init(info, cb);
		cb->rate_flags &= ~RATE_FLAGS_INIT;
	}
 
	DCACHE_INVALIDATE((unsigned int)&captbl->tx_ok_rate[0], DCACHE_LINE_SIZE);
		
	RC_COUNTER_DELTA(tx_rates[0].attempts, cb->tx_try_cnt[0], captbl->tx_try_cnt0);
	RC_COUNTER_DELTA(tx_rates[1].attempts, cb->tx_try_cnt[1], captbl->tx_try_cnt1);
	RC_COUNTER_DELTA(tx_rates[2].attempts, cb->tx_try_cnt[2], captbl->tx_try_cnt2);
	RC_COUNTER_DELTA(tx_fail_cnt, cb->tx_fail_cnt, captbl->tx_fail_cnt);
	for(i=0; i < 4; i++)
	{ 
		RC_COUNTER_DELTA(tx_rates[i].success, cb->tx_ok_rate[i], captbl->tx_ok_rate[i]);
	}
	tx_rates[3].attempts = tx_fail_cnt + tx_rates[3].success;
		
#if defined(WLAN_RC_BTS)
	if(cb->tx_ampdu_bitmap)
	{
		qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[cb->captbl_idx].qinfo_addr);

		for(i=0; i<QOS_TX_STATUS_TBLS; i++)
		{
			ampdu_retry = qinfos[i].ba_retry_cnt;
			//if(qinfos[i].ba_retry_cnt >= MAX_CONSECUTIVE_AMPDU_TX_FAILURE)
			if(ampdu_retry >= 9)
			{
				RC_DBG(RC_LV2, "STA[%d], Many AMPDU retry=%d\n", cb->captbl_idx, ampdu_retry);	
				break;
			}
		}
	}

	if(!cb->basic_rc_flag)
		next_update = wla_bts_update(info, cb, tx_rates, ampdu_retry);
	else if((info->wl_txq.pkt_count - info->wl_sw_retq.pkt_count) == 0)
	{
		/* no sw path packets means basic_rc_count MUST return zero */
		cb->basic_rc_count = 0;
		check_spec_rate(info, cb);
	}
#endif
#if 0
	mac_stacap_cache_sync(cb->captbl_idx, false);
#endif

	return next_update;
}

