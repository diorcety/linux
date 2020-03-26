/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_ctrl.c
*   \brief  wla wlan mac ctrl functions.
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
#include <wbuf.h>
#include <wla_mat.h>
#include <linux_wla.h>
#include <asm/mach-cheetah/idb.h>

#ifdef WLA_MULTICAST_PATCH
/* FIXME: for client role rx multicast case */
u8 multicast_addr_list[2][6] = {
	{0x01, 0x00, 0x5E, 0x7F, 0xFF, 0xFA},
	{0x01, 0x00, 0x5E, 0x00, 0x00, 0xFB},
};
#endif	// WLA_MULTICAST_PATCH

short sta_cap_get_free(MAC_INFO *info)
{
	short i, j;
	
	j = 0;
	/* always fill valid bit to 1 on used entry */
	while(j < info->sta_cap_tbl_count)
	{
		j++;
		i = info->sta_cap_tbl_free_idx++;
		info->sta_cap_tbl_free_idx = info->sta_cap_tbl_free_idx % info->sta_cap_tbl_count;

		if(0 == info->sta_cap_tbls[i].valid)
		{
			memset(&info->sta_cap_tbls[i], 0, sizeof(info->sta_cap_tbls[i]));
			info->sta_cap_tbls[i].valid = 1;
			return i;
		}
	}

	WLA_DBG(WLAERROR, "station capability table full\n");
	return -1;
}

struct peer_ap *new_peer_ap(MAC_INFO *info, char type)
{
	short i;

	for(i = 0; i<MAX_PEER_AP; i++)
	{
		if(info->linked_ap[i].type == 0)
		{
			info->linked_ap[i].type = type;
			info->linked_ap_count++;
			return &info->linked_ap[i];
		}
	}
	WLA_DBG(WLAERROR, "Linked AP table full\n");
	return NULL;
}

void wla_set_bandwidth_type(MAC_INFO *info)
{
	char pri_channel;

	if(info->bandwidth_type == BW20_ONLY)
	{
		if(my_bb_dev->set_20mhz_mode)
			my_bb_dev->set_20mhz_mode();
	}
	else
	{
		if(my_bb_dev->set_40mhz_mode)
			my_bb_dev->set_40mhz_mode(info->bandwidth_type);
	}

	pri_channel = primary_ch_offset(info->bandwidth_type);	

	MACREG_UPDATE32(BASIC_SET, (pri_channel << 9)|(pri_channel << 6), 
		LMAC_CH_BW_CTRL_CH_OFFSET|BASIC_CHANNEL_OFFSET);

	/* backward compatible for some FPGA-HW without second CCA circuit */
	if(info->bandwidth_type == BW20_ONLY)
		MACREG_UPDATE32(BASIC_SET, LMAC_DECIDE_CH_OFFSET|LMAC_DECIDE_CTRL_FR, LMAC_CH_BW_CTRL_AUTO_MASK);
	else
		/* David asked LMAC look both CCA0 and CCA1 to transmit, like other vendors. */ 
		//MACREG_UPDATE32(BASIC_SET, LMAC_DECIDE_CH_OFFSET|LMAC_OR_CCA_DIS|LMAC_CCA1_EN|LMAC_DECIDE_CTRL_FR, LMAC_CH_BW_CTRL_AUTO_MASK);
		MACREG_UPDATE32(BASIC_SET, LMAC_DECIDE_CH_OFFSET|LMAC_CCA1_EN|LMAC_DECIDE_CTRL_FR, LMAC_CH_BW_CTRL_AUTO_MASK);

}
void wla_mac_clean(MAC_INFO* info)
{
	int i;

	wla_beacon_flush(info);

	/* free station capability table */
	for(i=0; i<MAC_MAX_BSSIDS; i++)
	{
		wla_setup_tx_rate(info, i, -1, BASIC_TX_RATE, 0); /* clean basic rate table */
		wla_setup_tx_rate(info, i, -1, FIXED_TX_RATE, 0); /* clean TX fixed rate table */
		wla_setup_tx_rate(info, i, -1, MULTICAST_TX_RATE, 0); /* clean TX fixed rate table */
	}

	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		if(info->sta_cap_tbls[i].valid)
		{
			struct wsta_cb *wcb;

			wcb = info->wcb_tbls[i];
			if(!wcb)
				continue;
			WLA_DBG(WLAWARN, "!!!forget to release sta_cap_tbls[%d]\n", i);
			if(info->sta_cap_tbls[i].TX_tbl_addr)
			{
#ifdef CONFIG_SRAM_WLA
				sta_tx_status_tbl *tx_tbls;
				tx_tbls = (sta_tx_status_tbl *)NONCACHED_ADDR(info->sta_cap_tbls[i].TX_tbl_addr);
				memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
#else
				WLA_MFREE((void *)CACHED_ADDR(info->sta_cap_tbls[i].TX_tbl_addr));
#endif
			}
			info->sta_cap_tbls[i].valid = 0;
		}
	}


	memset(info->mbss, 0, sizeof(info->mbss));
	info->bss_num = 0;

}

void wla_mac_release(MAC_INFO* info)
{
	int i;

	info->hw_tx_buffer = 0;

	info->sta_cap_tbls_mem = 0;
	info->sta_cap_tbls = 0;
	info->rate_tbls = 0;
	WLA_MFREE(info->wcb_tbls);
	info->wcb_tbls = 0;

#if defined(WLA_AMPDU_RX_HW_REORDER)	
	/* free reorder buffers */
	if(info->reorder_buf_mapping_tbl)
	{
		for(i=0; i<(info->sta_cap_tbl_count * MAX_QOS_TIDS); i++)
		{
			if(info->reorder_buf_mapping_tbl[i])
			{
#ifdef CONFIG_SRAM_WLA
				ampdu_reorder_buf *reorder_buf;
				reorder_buf = (ampdu_reorder_buf *)NONCACHED_ADDR(info->reorder_buf_mapping_tbl[i]);
				memset(reorder_buf, 0, sizeof(ampdu_reorder_buf));
#else
				WLA_MFREE((void *)VIRTUAL_ADDR(info->reorder_buf_mapping_tbl[i]));	
#endif
			}
		}
		info->reorder_buf_mapping_tbl = 0;
	}
#endif
#if defined(WLA_PSBA_QUEUE)
	if(info->sta_qinfo_tbls)
	{
		info->sta_qinfo_tbls = 0;
	}

	if(info->bcast_qinfos)
	{
		info->bcast_qinfos = 0;
	}
#endif
	/* free descriptors */
	/* wl_rxq Total size = RXs + TXs + fast_TXs + ret_TXs + eth_RXs + eth_TXs */ 
	info->wl_rxq.desc_hdr = 0;
#if defined(WLA_MMAC_RXDESCR)
	info->mmac_rx_descriptors = 0;
#endif
#if defined(WLA_PSBA_QUEUE)
	info->psbaq_descriptors = 0;
#endif

	wla_beacon_flush(info);

	/* free sw/hw buffers and buffer headers */
	if(info->buf_headers)
	{
		buf_header *bhdr;
		/* dig software buffers out */
#ifdef CONFIG_SRAM_WLA
		for(i=info->sw_tx_bhdr_start; i<info->sw_tx_bhdr_end;i++) 
#else
		for(i=info->sw_rx_bhdr_start; i<info->sw_tx_bhdr_end;i++) 
#endif
		{
			bhdr = idx_to_bhdr(info, i);
			if(bhdr->dptr)
			{
				WBUF_FREE(dptr_to_wb((char *)(unsigned int)bhdr->dptr));
				bhdr->dptr = 0;
			}
		}

#if defined(SW_BUF_TRACE)
		/* trace sw buffer usage */
		WLA_MFREE(info->sw_buf_trace);
		info->sw_buf_trace = 0;
#endif
	}

#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
	if(info->fastpath_buf)
	{
#ifdef CONFIG_SRAM_WLA
		memset(info->fastpath_buf, 0, sizeof(info->fastpath_bufsize * info->hw_buf_headers_count));
#else
		WLA_MFREE(info->fastpath_buf);
#endif
		info->fastpath_buf = 0;
	}
#endif
	info->group_keys = 0;
	info->private_keys = 0;

#if defined(CONFIG_CHEETAH)
	info->ext_sta_tbls = 0;
	info->ext_ds_tbls = 0;
#endif
}

/*!-----------------------------------------------------------------------------
 * function: wla_mac_start()
 *
 *      \brief	1. initialize BB
 *				2. chain buffers and descriptors 
 *				3. initialize MAC register
 *				4. enable interrupt
 *		\param 	info: allocated MAC_INFO 
 *      \return	void
 +----------------------------------------------------------------------------*/
int wla_mac_start(MAC_INFO* info)
{
    /*
        enable TX

     */
    MACREG_WRITE32(UMAC_TX_CNTL, 0x43); /* XXX  fix this */


    /* 
        enable RX
     */
	if(!info->mac_recover)
	{
		MACREG_WRITE32(UPRX_EN, 1);
	}	

    /*
        enable LMAC
     */
    MACREG_UPDATE32(LMAC_CNTL, LMAC_CNTL_TSTART, LMAC_CNTL_TSTART);

	if(info->filter & RX_FILTER_BEACON)
	{
		MACREG_UPDATE32(LMAC_CNTL, LMAC_CNTL_BEACON_FILTER, LMAC_CNTL_BEACON_FILTER);
	}

#if defined(WLA_WIFI2ETH)
	w3e_init();
#endif
	info->int_mask = ~(MAC_INT_SW_RTURNQ|MAC_INT_SW_RX
#if defined(WLA_WIFI2ETH)
						|MAC_INT_ETH_RX|MAC_INT_ETH_TXDONE
#endif
#if 0
						|MAC_INT_HW_RTURNQ|MAC_INT_HW_BUF_FULL|MAC_INT_SW_RTURNQ_FULL|MAC_INT_HW_RTURNQ_FULL /* debug */
						|MAC_INT_RX_DESCR_FULL|MAC_INT_RX_FIFO_FULL	/* debug */
#endif
					);
	MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask); 

	info->hwaddr_age_time = WLA_CURRENT_TIME + WLA_HWADDR_AGE;
	if(!info->mac_recover)
	{
		wla_set_timer();
	}

    return 0;
}

/*!-----------------------------------------------------------------------------
 * function: wla_mac_stop()
 *
 *      \brief	1. disable interrupt
 *				2. stop beacon
 *				3. flush all address tables 
 *				4. stop RX, LMAC
 *				5. dig all buffers and free
 *		\param 	info: allocated MAC_INFO 
 *      \return	void
 +----------------------------------------------------------------------------*/
int wla_mac_stop(MAC_INFO* info)
{
	int i;

	if(!info->mac_recover)
	{
		wla_del_timer();
	}

	/* Stop all interrupts */
	MACREG_WRITE32(MAC_INT_MASK_REG, MAC_INT_FULL_MASK);

	/* turn off beacon first */
	wla_beacon_stop(info);

	if(!info->mac_recover)
	{
		mac_addr_lookup_engine_flush(info);
	}

	/* Stop ETH2Wifi, LMAC, Beacon, UMAC RX/TX */
#if defined(WLA_WIFI2ETH)
	w2e_stop();
#endif
	MACREG_UPDATE32(LMAC_CNTL, 0x0, LMAC_CNTL_TSTART);

	/* Don't stop RXEN */
    //MACREG_WRITE32(UPRX_EN, 0);

	MACREG_UPDATE32(UMAC_TX_CNTL, 0, UMAC_TX_CNTL_LU_CACHE_EN | UMAC_TX_CNTL_TX_ENABLE);

	/* Turn off all timers and interrupt */
	MACREG_WRITE32(TS_INT_MASK, -1);
	for(i=0; i<MAC_MAX_BSSIDS; i++)
	{
		MACREG_WRITE32(TS0_CTRL + i*0x10, 0);
	}

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
	mat_flush();
#endif	//  CONFIG_UNIVERSAL_REPEATER
	WLA_DBG(WLAWARN, "wla_mac_stop...done\n");
    return 0;
}

int mac_addr_lookup_engine_find(u8 *addr, int addr_index, int *basic_cap, char flag)
{
    u32 val, cmd;

	wla_lock();
	
	cmd = LU_TRIGGER;
	if(!(flag & IN_DS_TBL))
		cmd |= LU_CMD_SEL_STA_TABLE; 
	if(flag & BY_ADDR_IDX)
	{
		cmd |= (( addr_index << 5 ) & 0x00001FE0UL )|LU_CMD_READ_BY_INDEX;
	}
	else
	{
    	MACREG_WRITE32(LUT_ADDR0, (addr[0] << 8) | (addr[1]));
    	MACREG_WRITE32(LUT_ADDR1, (addr[2] << 24) | (addr[3]  << 16) | (addr[4] << 8) | (addr[5]));
		cmd |= LU_CMD_SEARCH_BY_ADDR;
	}

	MACREG_WRITE32(LUT_CMD, cmd);
    while (0 == (MACREG_READ32(LUT_CMD) & LU_DONE));

    val = MACREG_READ32(LUR_INDX_ADDR);
 	
 	if(!(val & LUR_INDX_ADDR_CMD_SUCCEED))
	{
		wla_unlock();
		return LOOKUP_ERR;
	}

	if(flag & BY_ADDR_IDX)
	{
		if(addr)
    	{
        	addr[0] = (u8) ((val & 0x0000FF00UL) >> 8);
        	addr[1] = (u8) (val & 0x000000FFUL);

        	val = MACREG_READ32(LUR_ADDR);
        	addr[2] = (u8) ((val & 0xFF000000UL) >> 24);
        	addr[3] = (u8) ((val & 0x00FF0000UL) >> 16);
        	addr[4] = (u8) ((val & 0x0000FF00UL) >> 8);
        	addr[5] = (u8) (val & 0x000000FFUL);
    	}
	}
	else
	{
		u32 mask;
		if(flag & IN_DS_TBL)
		{
			if((val & LUR_INDX_ADDR_DS_HIT) == 0)
			{
				wla_unlock();
				return LOOKUP_WRONG_TBL;
			}
			else
				mask = 0x000f0000;
		}
		else
		{ 
			if((val & LUR_INDX_ADDR_STA_HIT) == 0)
			{
				wla_unlock();
				return LOOKUP_WRONG_TBL;
			}
			else
				mask = 0x00ff0000;
		}

		if(addr_index)
		{
			int *addr_p = (int *)addr_index;
			*addr_p = ((val & mask) >> 16);
		}
	}

	val = MACREG_READ32(LUR_BASIC_CAP);
	if(basic_cap)
	{
		*basic_cap = val;
	}

	wla_unlock();

	return (val & 0x000000FFUL);
}

/* return the hash index */
int mac_addr_lookup_engine_update(MAC_INFO* info, u32 addr, u32 basic_cap, u8 flag)
{
	u32 val, cmd;
	unsigned char ci_cmd;	
	u8 *maddr = (u8 *)addr;
	cmd = LU_TRIGGER;

	if(flag & IN_DS_TBL)
	{
		/* check STA index range */
		if(flag & BY_ADDR_IDX)
		{
			if(addr >= MAX_DS_TABLE_ENTRIES)
				return -1;
		}
	}
	else
	{
		/* check DS index range */
		if(flag & BY_ADDR_IDX)
		{
			if(addr >= MAX_ADDR_TABLE_ENTRIES)
				return -1;
		}
		cmd |= LU_CMD_SEL_STA_TABLE;
	}

	wla_lock();

	MACREG_WRITE32(LUBASIC_CAP, basic_cap);

	/* add or delete basic capbility by mac address */
	if(flag & BY_ADDR_IDX)
	/* non-value addr means update or delete basic capbility by address index */
	{
		if(!basic_cap)
		{
			MACREG_WRITE32(LUT_ADDR0, 0);
    		MACREG_WRITE32(LUT_ADDR1, 0);
		}
		else
		{
			u8 raddr[6];
			if(mac_addr_lookup_engine_find(raddr, addr, 0, flag) < 0)
			{
				wla_unlock();
				WLA_DBG(WLAERROR, "%s(), find addr fail!\n", __func__);
				return LOOKUP_ERR;
			}
			MACREG_WRITE32(LUT_ADDR0, (raddr[0] << 8) | (raddr[1]));
    		MACREG_WRITE32(LUT_ADDR1, (raddr[2] << 24) | (raddr[3]  << 16) | (raddr[4] << 8) | (raddr[5]));
		}

		if(flag & IN_DS_TBL)
			cmd |= ((addr << 5)| LU_CMD_UPDATE_DS_BY_INDEX);
		else
			cmd |= ((addr << 5)| LU_CMD_UPDATE_STA_BY_INDEX);
	}
	else
	{
    	MACREG_WRITE32(LUT_ADDR0, (maddr[0] << 8) | (maddr[1]));
    	MACREG_WRITE32(LUT_ADDR1, (maddr[2] << 24) | (maddr[3]  << 16) | (maddr[4] << 8) | (maddr[5]));
		if(flag & IN_DS_TBL)
			cmd |= (basic_cap ? LU_CMD_INSERT_INTO_DS_BY_ADDR : LU_CMD_UPDATE_INTO_DS_BY_ADDR);
		else
			cmd |= (basic_cap ? LU_CMD_INSERT_INTO_STA_BY_ADDR : LU_CMD_UPDATE_INTO_STA_BY_ADDR);
	}

	MACREG_WRITE32(LUT_CMD, cmd);

    while (0 == (MACREG_READ32(LUT_CMD) & LU_DONE));
	
	val = MACREG_READ32(LUR_INDX_ADDR);

	if(val & LUR_INDX_ADDR_CMD_SUCCEED)
	{
		val = ((val & 0x00FF0000) >> 16);
		/* CHEETAH needs to update station's total number */
		ci_cmd = cmd & LU_CMD_MASK;
		if(basic_cap)
		{
			if(ci_cmd == LU_CMD_INSERT_INTO_STA_BY_ADDR)
				info->sta_tbl_count++;
			else if(ci_cmd == LU_CMD_INSERT_INTO_DS_BY_ADDR)
				info->ds_tbl_count++;
		}
		else
		{
			if(flag & IN_DS_TBL)
				info->ds_tbl_count--;
			else
				info->sta_tbl_count--;
		}
	}
	else
		val = -1;
	
	/* Don't adjust  STA_DS_TABLE_CFG dynamically to avoid cache has incorrect value */
	MACREG_UPDATE32(STA_DS_TABLE_CFG, info->sta_tbl_count|((info->ds_tbl_count & 0x1f) << 9), STA_TABLE_MAX_SEARCH|DS_TABLE_MAX_SEACH);

	wla_unlock();
	return val;
}

int mac_addr_lookup_engine_flush(MAC_INFO* info)
{
    WLA_DBG(WLADEBUG, "flush_mac_addr_lookup_engine\n");

	wla_lock();

#if defined(CONFIG_CHEETAH)
	/* flush external sta/ds tables */
	MACREG_UPDATE32(STA_DS_TABLE_CFG, (MAX_STA_DS_CACHE << 24)|DS_TABLE_CLR|
							STA_TABLE_CLR, DS_TABLE_CLR|STA_TABLE_CLR|
							STA_TABLE_MAX_SEARCH|DS_TABLE_MAX_SEACH|
							STA_DS_TABLE_CACHE_CFG);
#endif

    /* flush lookup entries */
    MACREG_WRITE32(LUT_CMD, LU_TRIGGER| LU_FLUSH_STA_TABLE);
    while (0 == (MACREG_READ32(LUT_CMD) & LU_DONE));       /* polling done bit */

    MACREG_WRITE32(LUT_CMD, LU_TRIGGER| LU_FLUSH_DS_TABLE);
    while (0 == (MACREG_READ32(LUT_CMD) & LU_DONE));       /* polling done bit */

#if defined(CONFIG_CHEETAH)
	if(info->ext_sta_tbls)
		memset(info->ext_sta_tbls, 0, sizeof(sta_basic_info)*MAX_ADDR_TABLE_ENTRIES);
	if(info->ext_ds_tbls)
		memset(info->ext_ds_tbls, 0, sizeof(sta_basic_info)*MAX_DS_TABLE_ENTRIES);
	info->sta_tbl_count = info->ds_tbl_count = 0;
#endif
	wla_unlock();
    //WLA_DBG(WLADEBUG, "flush_mac_addr_lookup_engine...Done.\n");

    return 0;
}

void find_all_addrs_and_update(MAC_INFO* info, u8 captbl_idx, u32 basic_cap)
{
	sta_basic_info bcap;
	int i;

	for(i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
	{
		bcap.val = 0;
		mac_addr_lookup_engine_find(0, i, (int *)&bcap, BY_ADDR_IDX);
		if(bcap.bit.vld && (bcap.bit.captbl_idx == captbl_idx))
		{
			mac_addr_lookup_engine_update(info, i, basic_cap, BY_ADDR_IDX);
		}
	}
}

#if defined(CONFIG_WMAC_RECOVER)
extern void cta_switch_ethernet_forward_to_wifi(unsigned int permit);
unsigned int mactbls_restore_reg[11] = {	MAC_BSSID0, MAC_BSSID1,
											MAC_BSSID_FUNC_MAP, PS_FUNC_CTRL,
											MAC_BSSID_LOW_1, MAC_BSSID_LOW_2,
											MAC_BSSID_LOW_3, MAC_BSSID_LOW_4,
											MAC_BSSID_LOW_5, MAC_BSSID_LOW_6,
											MAC_BSSID_LOW_7 };

#define wla_ethernet_forward_to_wifi(a)	cta_switch_ethernet_forward_to_wifi(a)
#else
#define wla_ethernet_forward_to_wifi(a)	NULL
#endif

void wla_mac_reset(MAC_INFO* info)
{
#if defined(CONFIG_WMAC_RECOVER)
	int i;
	for(i=0; i<11; i++)
	{
		info->restore_umac[i] = MACREG_READ32(mactbls_restore_reg[i]);
	}
#endif
	MACREG_UPDATE32(HW_RESET, 0, HW_RESET_WIFI_PAUSE | HW_RESET_BB_PAUSE);
    udelay(1000);
	MACREG_UPDATE32(HW_RESET, 0, HW_RESET_WMAC | HW_RESET_BB|HW_RESET_DCM_BB2MAC);
    udelay(1000);
    MACREG_UPDATE32(HW_RESET, -1, HW_RESET_WMAC | HW_RESET_BB|HW_RESET_DCM_BB2MAC);
#if defined(CONFIG_WMAC_RECOVER)
	for(i=0; i<11; i++)
	{
		MACREG_WRITE32(mactbls_restore_reg[i], info->restore_umac[i]);
	}
#endif
	/* turn off beacon first */
	wla_beacon_stop(info);
	if(!info->mac_recover)
	{
		/* initialize addr/ds table */
		mac_addr_lookup_engine_flush(info);
	}

    MACREG_WRITE32(MAC_INT_MASK_REG, 0x0000ffff);       /* mask all interrupts */
    MACREG_WRITE32(MAC_INT_CLR, 0x0000ffff);       /* clear all interrupt status  */
	WLA_DBG(WLAWARN, "wla_mac_reset...done\n");
    return;
}

#if defined(CONFIG_WMAC_RECOVER)
void wla_mac_ethernet_forward_setup(u32 permit)
{
	if(permit)
    	MACREG_WRITE32(UPRX_EN, 1);
	else
    	MACREG_WRITE32(UPRX_EN, 0);
	wla_ethernet_forward_to_wifi(permit);
}
#endif

void mac_free_buffer_hdr(u32 head_index, u32 tail_index, u8 hw_buf)
{
	wla_lock();
	if(hw_buf)
	WLA_DBG(WLADEBUG, "????????free hardware buffer head=%x, tail=%x\n", head_index, tail_index);
	else
	{
		MAC_INFO *info = WLA_MAC_INFO;

		if((head_index >= info->sw_tx_bhdr_start)||
			(head_index < info->hw_buf_headers_count)||
			(tail_index >= info->sw_tx_bhdr_start)||
			(tail_index < info->hw_buf_headers_count))
		{
			WLA_DBG(WLAERROR, "%s(%d), head_index=%x, tail_index=%x\n", __func__, __LINE__, head_index, tail_index);
			panic("index error\n");
		}
	
	}

    while( MACREG_READ32(MAC_FREE_CMD) & MAC_FREE_CMD_BUSY )
        ;

    MACREG_WRITE32(MAC_FREE_PTR, ((head_index << 16) | tail_index ));

    MACREG_WRITE32(MAC_FREE_CMD,  (hw_buf ? 0 : MAC_FREE_CMD_SW_BUF) | MAC_FREE_CMD_KICK );
	wla_unlock();
}

static char cw_to_hw(int cw)
{
	char i;
		/* HW val	CW range 
		0		0-0
		1		0-1
		2		0-3
		3		0-7
		4		0-15
		5		0-31
		6		0-63
		7		0-127
		8		0-255
		9		0-511
		10		0-1023 */
	for(i = 0; i <= 10; i++)
	{
		if(cw < (1 << i))
			break;
	}
	return i;
}

void wla_ac_paramters(int ac, int cw_max, int cw_min, int aifs, int txop)
{
	if((cw_min >= 0) && (cw_max >= 0))
	{
		MACREG_UPDATE32(CW_SET, (cw_to_hw(cw_max) << (8*ac + 4)) | (cw_to_hw(cw_min) << (8*ac)), (0xff << (8*ac)));
	}

	if(aifs >= 0)
	{
		MACREG_UPDATE32(AIFS_SET, (aifs << (4*ac + 16)), (0xf << (4*ac + 16)));
	}

	if(txop >= 0)
	{
		MACREG_UPDATE32(TXOP_LIMIT, (txop << (8*ac)), (0xff << (8*ac)));
	}
}

inline char mac_get_dtim_counter(void)
{
	return (char) (( MACREG_READ32(DTIM_INTERVAL_REG) & DTIM_INVERVAL_REG_DTIM_COUNTER ) >> 8);
}

void mac_set_key(cipher_key *hwkey, char *key, char cipher_type, char keyidx, char is_txkey)
{
	if((key == 0) || (cipher_type == AUTH_CIPHER_NONE))
	{
		/* delete key */
		memset(hwkey, 0, sizeof(cipher_key));
	}
	else if((cipher_type == CIPHER_TYPE_WEP40) || (cipher_type == CIPHER_TYPE_WEP104))
	{ 
		hwkey->wep_key.cipher_type = (cipher_type << ((3-keyidx)*4) | hwkey->wep_key.cipher_type);
		memcpy(&hwkey->wep_key.key[(int)keyidx][0], key, ((cipher_type == CIPHER_TYPE_WEP104) ? 
				WEP_104_KEY_LEN : WEP_40_KEY_LEN));

		hwkey->wep_key.iv = 0;
		if(is_txkey)
			hwkey->wep_key.txkeyid = keyidx;
	}
	else if(cipher_type == CIPHER_TYPE_TKIP)
	{
		hwkey->tkip_key.cipher_type = cipher_type;
		hwkey->tkip_key.txtsc_lo = 1;
		hwkey->tkip_key.txtsc_hi = 0;
		if(is_txkey)
			hwkey->tkip_key.txkeyid = keyidx;		

		memcpy(hwkey->tkip_key.key, key, TKIP_KEY_LEN);
		memcpy(hwkey->tkip_key.txmickey, &key[TKIP_KEY_LEN], TKIP_MICKEY_LEN);
		memcpy(hwkey->tkip_key.rxmickey, &key[TKIP_KEY_LEN+TKIP_MICKEY_LEN], TKIP_MICKEY_LEN);
	}
	else if(cipher_type == CIPHER_TYPE_CCMP)
	{
		hwkey->ccmp_key.cipher_type = cipher_type;
		hwkey->ccmp_key.txpn_lo = 1;
		hwkey->ccmp_key.txpn_hi = 0;
		if(is_txkey)
			hwkey->ccmp_key.txkeyid = keyidx;		

		memcpy(hwkey->ccmp_key.key, key, CCMP_KEY_LEN);
	}
#if defined (CONFIG_WLA_WAPI) || defined (CONFIG_WAPI)
	else if(cipher_type == CIPHER_TYPE_SMS4)
	{
		hwkey->wapi_key.cipher_type = cipher_type;
		hwkey->wapi_key.txpn[0] = WAPI_PN_DWORD+2;
		hwkey->wapi_key.txpn[1] = WAPI_PN_DWORD;
		hwkey->wapi_key.txpn[2] = WAPI_PN_DWORD;
		hwkey->wapi_key.txpn[3] = WAPI_PN_DWORD;
		if(is_txkey)
			hwkey->wapi_key.txkeyid = keyidx;		

		memcpy(hwkey->wapi_key.key, key, WAPI_CIPHER_KEY_LEN);
		memcpy(hwkey->wapi_key.mickey, &key[WAPI_CIPHER_KEY_LEN], WAPI_MICKEY_LEN);
	}
#endif
}

void mac_stacap_cache_sync(int idx, char wait)
{
    int status;

	if(MACREG_READ32(TXCACHE_CNTL) & TXCACHE_CNTL_STACAP_CACHE_WRITEBACK)
		return;

	status = MACREG_READ32(STACAP_CACHE_STATUS);
	/* only write back TX part of stacap */
	if(((status & STACAP_CACHE_EMPTY) == 0) && (idx == (status & STACAP_CACHE_RA_INDEX)))
	{
    	MACREG_WRITE32(TXCACHE_CNTL, TXCACHE_CNTL_STACAP_CACHE_WRITEBACK);

		if(wait)
		{
			unsigned int cnt = 0;

			while((TXCACHE_CNTL_STACAP_CACHE_WRITEBACK == 
				(MACREG_READ32(TXCACHE_CNTL) & TXCACHE_CNTL_STACAP_CACHE_WRITEBACK)))
			{
				if(++cnt > 5)
				{
#if defined(CONFIG_WMAC_RECOVER)
					if(wla_tx_monitor())
						break;
#endif
					wla_sleep(1);
					if((cnt&0xffff) == 0xffff)
						DBG("%s: waiting cache writeback\n??????waiting(%d)????????\n", __func__, cnt);
				} 
			}
		}
	}

	return;
}

void mac_ratecap_cache_sync(int idx, char wait)
{
	MAC_INFO *info = WLA_MAC_INFO;
    int status;

	if(MACREG_READ32(TXCACHE_CNTL) & TXCACHE_CNTL_STACAP_CACHE_WRITEBACK)
		return;

	status = MACREG_READ32(STACAP_CACHE_STATUS);
	/* only write back TX part of stacap */
	if(((status & STACAP_CACHE_EMPTY) == 0) && ((idx|STACAP_CACHE_NP) == (status & (STACAP_CACHE_RA_INDEX|STACAP_CACHE_NP))))
	{
    	MACREG_WRITE32(TXCACHE_CNTL, TXCACHE_CNTL_STACAP_CACHE_WRITEBACK);

		if(wait)
		{
			WMAC_WAIT_AND_SLEEP(TXCACHE_CNTL_STACAP_CACHE_WRITEBACK ==
               	(MACREG_READ32(TXCACHE_CNTL) & TXCACHE_CNTL_STACAP_CACHE_WRITEBACK), 
				"mac_ratecap_cache_sync(): waiting cache writeback\n");
		}
	}

	return;
}

void mac_rx_linkinfo_cache_sync(void)
{
	/* only write back RX part of stacap */
	MACREG_WRITE32(RX_LINK_INFO_CACHE_CTRL, RX_LINK_INFO_CACHE_OUT);
}

void mac_disassociate(char sta_idx, char np)
{
	MAC_INFO *info = WLA_MAC_INFO;

	WMAC_WAIT_FOREVER((MACREG_READ32(DISASSOC_CTRL) & DISASSOC_CTRL_IDLE) != DISASSOC_CTRL_IDLE, 
			"mac_disassociate(1): waiting idle\n");

    if(np)
        MACREG_WRITE32(DISASSOC_CTRL, (DISASSOC_STA_CAP_INDEX & sta_idx) | DISASSOC_CTRL_TX_KICK);
    else
        MACREG_WRITE32(DISASSOC_CTRL, (DISASSOC_STA_CAP_INDEX & sta_idx) | DISASSOC_CTRL_SECURITY_AND_RX_KICK | DISASSOC_CTRL_TX_KICK);

	WMAC_WAIT_FOREVER((MACREG_READ32(DISASSOC_CTRL) & DISASSOC_CTRL_IDLE) != DISASSOC_CTRL_IDLE, 
			"mac_disassociate(2): waiting idle\n");

	return;
}

/* CAUTION: use these two function as pair, as it will use lock to avoid context switch */
void mac_rekey_start(int index, int key_type, char cipher_type, int rsc_init_value)
{
	MAC_INFO *info = WLA_MAC_INFO;
    int key_ctrl = 0;

	WMAC_WAIT_FOREVER(MACREG_READ32(SEC_KEY_CTRL) & SEC_KEY_CTRL_HW_STATUS, 
			"mac_rekey_start(1): waiting idle\n");
	
	key_ctrl = ((index << 16) & SEC_KEY_CTRL_STA_IDX) | SEC_KEY_CTRL_REKEY_REQ | 
				((0xff & rsc_init_value) << 8) | (cipher_type & 0xf); 
	if(key_type >= KEY_TYPE_GLOBAL_KEY)
		key_ctrl |= SEC_KEY_CTRL_STA_IDX_GLOBAL;
#if 0
	else if(key_type == KEY_TYPE_STA_GLOBAL_KEY)
		key_ctrl |= SEC_KEY_CTRL_REKEY_STA_GKEY;
#endif
//	wla_lock();
	MACREG_WRITE32(SEC_KEY_CTRL, key_ctrl);


	WMAC_WAIT_FOREVER(MACREG_READ32(SEC_KEY_CTRL) & SEC_KEY_CTRL_HW_STATUS, 
			"mac_rekey_start(2): waiting idle\n");
	
}

void mac_rekey_done(void)
{
	/* info the HW that the modification is done */
	MACREG_WRITE32(SEC_KEY_CTRL, 0);
//    wla_unlock();
}

/* proper lock mechanism from caller is presumed */
void mac_invalidate_ampdu_scoreboard_cache(int sta_index, int tid)
{
	MAC_INFO *info = WLA_MAC_INFO;

    /* XXX TODO: check status bit before or after this operation */
	wla_lock();

	if((MACREG_READ32(AMPDU_BMG_CTRL) & (REORDER_FLUSH|BMG_CLEAR|VARIABLE_TABLE_1_CACHE_FLUSH | VARIABLE_TABLE_0_CACHE_FLUSH)) || (info->stop_ampdu_rx))
	{
		wla_unlock();
		return;
	}

    MACREG_WRITE32(AMPDU_BMG_CTRL, (BMG_CLEAR | (tid << 8) | (sta_index)));
	wla_unlock();
}

char mac_flush_all_ampdu_reorder_buf(MAC_INFO *info, int sta_index, int tid)
{
#if defined(WLA_AMPDU_RX_SW_REORDER)
	struct wsta_cb *cb;
	struct rx_ba_session *ba;

	cb = info->wcb_tbls[sta_index];
	ba = cb->ba_rx[tid];	
	if(ba)
	{
		reorder_buf_release(ba, WLAN_SEQ_ADD(ba->win_start, WLAN_SEQ_BA_RANGE), 0);

		/* for debug */
		if(ba->reorder_q_head || ba->reorder_q_tail || ba->stored_num)
		{
			WLA_DBG(WLADEBUG, "???????????###########ReorderB still has buffer after flush_reorder_queue, ba=%x\n", ba);
			dump_ba(info, sta_index);	
		}
	}
		return 0;
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	if(info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid])
	{
		/* relay on stop_ampdu_rx flag to protect teardown procedure */ 
    	MACREG_WRITE32(AMPDU_BMG_CTRL, (REORDER_FLUSH | (tid << 8) | (sta_index)));
		return 0;
	}
	else
		return -1;
#endif
}

void mac_flush_one_ampdu_reorder_buf(int sta_index, int tid)
{
#if defined(WLA_AMPDU_RX_HW_REORDER)
	MAC_INFO *info = WLA_MAC_INFO;
	struct wsta_cb *wcb = info->wcb_tbls[sta_index];

	wla_lock();

	if((wla_ampdu(GET_AMPDU_RX_STATUS, wcb, tid, 0, 0) == 0)||(MACREG_READ32(AMPDU_BMG_CTRL) & (REORDER_FLUSH|BMG_CLEAR|VARIABLE_TABLE_1_CACHE_FLUSH|VARIABLE_TABLE_0_CACHE_FLUSH)) || (info->stop_ampdu_rx))
	{
		wla_unlock();
		return;
	}

    MACREG_WRITE32(AMPDU_BMG_CTRL, (REORDER_FLUSH | REORDER_FLUSH_TYPE | (tid << 8) | (sta_index)));

	//WMAC_WAIT_FOREVER(MACREG_READ32(AMPDU_BMG_CTRL) & REORDER_FLUSH, 
	//		"waiting on ampdu reorder buffer flush one\n");

	wla_unlock();
#endif
}

int mac_attach_ampdu_reorder_buf(MAC_INFO *info, int sta_index, int tid, u16 ssn, u16 bufsize)
{
#if defined(WLA_AMPDU_RX_SW_REORDER)
	struct wsta_cb *cb;
	struct rx_ba_session *ba;

	cb = info->wcb_tbls[sta_index];

	ba = WLA_MALLOC(sizeof(struct rx_ba_session), DMA_MALLOC_BZERO);
	if(NULL == ba)
		goto fail;

	cb->ba_rx[tid] = ba;	

	//ba->win_start = ssn; 
	ba->win_start = 0xffff; 			
	ba->win_size = 1<<bufsize;
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	ampdu_reorder_buf *reorder_buf;

    if(NULL==info->reorder_buf_mapping_tbl)
        goto fail;
#if 0 /* Reuse exist reorder_buf */
    if(info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid])
    {
        WLA_DBG(WLAERROR, "double attach ampdu reorder buffer, sta_index %d, tid %d\n", sta_index, tid);
        goto fail;
    }

    reorder_buf = WLA_MALLOC(sizeof(ampdu_reorder_buf), 
						DMA_MALLOC_UNCACHED | DMA_MALLOC_BZERO |DMA_MALLOC_ALIGN);
    if(NULL==reorder_buf)
        goto fail;
#endif
	if(info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid])
    {
		reorder_buf = (ampdu_reorder_buf *)NONCACHED_ADDR(info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid]);
		memset(reorder_buf, 0, sizeof(ampdu_reorder_buf));
	}
	else
	{
#ifdef CONFIG_SRAM_WLA
		reorder_buf = &info->reorder_buf[(sta_index * MAX_QOS_TIDS) + tid];
#else
		reorder_buf = WLA_MALLOC(sizeof(ampdu_reorder_buf), 
						DMA_MALLOC_UNCACHED | DMA_MALLOC_BZERO |DMA_MALLOC_ALIGN);
    	if(NULL==reorder_buf)
        	goto fail;
#endif

	}


    reorder_buf->buf_size = bufsize;
    reorder_buf->ssn = ssn;
    reorder_buf->head_seq_num = ssn;
	
	{
		int i;
		i = 5;
		while(i<72)
		{
			if(((int *)reorder_buf)[i++] != 0)
			{
				WLA_DBG(WLADEBUG, " reorder bufer dump:, sta_index=%d\n", sta_index);
				WLA_DUMP_BUF_16BIT((char *)reorder_buf, (64+8)*4);
			}
		}
	}
    info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid] = (int)reorder_buf;
#endif
#if defined(WLA_AMPDU_RX_SW_REORDER)||defined(WLA_AMPDU_RX_HW_REORDER)
	return 0;
fail:
#endif
    return -1;
}

int mac_detach_ampdu_reorder_buf(MAC_INFO* info, int sta_index, int tid)
{
#if defined(WLA_AMPDU_RX_SW_REORDER)
	struct wsta_cb *cb;

	cb = info->wcb_tbls[sta_index];
	if(cb->ba_rx[tid] != 0)
	{
		WLA_MFREE(cb->ba_rx[tid]);
		cb->ba_rx[tid] = 0;
	}
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	
    if(info->reorder_buf_mapping_tbl && info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid])
    {
#if 0 /* keep reorder buffer to reuse */
		ampdu_reorder_buf *reorder_buf;
        reorder_buf = (ampdu_reorder_buf *) CACHED_ADDR(info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid]);

		info->reorder_buf_mapping_tbl[(sta_index * MAX_QOS_TIDS) + tid] = 0;

        WLA_MFREE(reorder_buf);
		WLA_DBG(WLADEBUG, "%s(), free reorder_buf=%p, size=%x\n", __func__, reorder_buf, sizeof(ampdu_reorder_buf));
#endif
    }
	else
	{
		WLA_DBG(WLAERROR, "?????%s(), no reorder buffer pointer, sta_index=%d, tid=%d\n", __func__, sta_index, tid);
	}
#endif
    return 0;
}

#if defined(WLA_PSBA_QUEUE)
void mac_detach_psbaq_ucast_qinfo(MAC_INFO* info, int sta_index)
{
	int i, tbl_num;
	ucast_qinfo *qinfos;

	if(info->sta_qinfo_tbls[sta_index].qinfo_addr)
	{
		qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[sta_index].qinfo_addr);
		if(info->sta_qinfo_tbls[sta_index].qos)
			tbl_num = QOS_TX_STATUS_TBLS;
		else
			tbl_num = 1;
		for(i=0; i<tbl_num; i++)
		{
			if(qinfos[i].pkt_cnt)
				WLA_DBG(WLADEBUG, "invalid pkt_cnt to detach PSBAQ: tid:%d, qmode=%d, pkt_cnt=%d\n", i, qinfos[i].qmode, qinfos[i].pkt_cnt);
		}

		memset(&info->sta_qinfo_tbls[sta_index], 0, sizeof(sta_qinfo_tbl));
	}
}

void psbaq_invalidate(u8 bssid, u8 captbl_idx)
{
	MAC_INFO *info = WLA_MAC_INFO;
	
	WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before trigger invalidate\n", info->psbaq_cmd_waiting_max_cnt);

	/* invalidate PSBAQ, given the sta index and bssid index */
	MACREG_WRITE32(PSBAQ_QINFO_CSR0, (bssid));
	MACREG_WRITE32(PSBAQ_QINFO_CSR2, (captbl_idx << 8) | PSBAQ_QINFO_CSR2_CMD_STA_INV | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
	/* 
	   Eric asked this waiting is not need. If invalidation is busy, we need furthur action to break the waiting.
	   Roger enables this waiting again. Under PS stress test environment, sw_retq may get repeated frames and trap. 
		We guess PSBAQ's state has beed corrupted by qinfo released 
	*/
	WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"trigger invalidate and waiting on PSBAQ command idle\n", info->psbaq_cmd_waiting_max_cnt);
}

int psbaq_set_tid_delivery_status(MAC_INFO* info, int index, int tid, char delivery_enable)
{
	u32 cmd = 0;
	ucast_qinfo *qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[index].qinfo_addr);

	{
		unsigned int cnt = 0;
		while((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER))
		{
			++cnt;
#if defined(CONFIG_WMAC_RECOVER)
			if(wla_tx_monitor())
				break;
#endif
			wla_sleep(1);
			if((cnt&0xffff) == 0xffff)
				DBG("%s:waiting on PSBAQ command idle before APSD setup(%d)\n", __func__, cnt);
		}
	}

	if(qinfos[tid].force_noagg)
		cmd |= PSBAQ_QINFO_CSR0_FORCE_NOAGG;
	cmd |= qinfos[tid].max_ampdu_len << 18;

    if(delivery_enable)
	{
		WLA_DBG(WLAWARN, "APSD DEIVERY: STA=%d, TID=%d\n", index, tid);
        cmd |= 0x1;
	}
    
	MACREG_WRITE32(PSBAQ_QINFO_CSR0, cmd);

    MACREG_WRITE32(PSBAQ_QINFO_CSR2, (index << 8) | (tid << 4) | PSBAQ_QINFO_CSR2_CMD_PS_CONF_W | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
#if 0 
    while(MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER)
    {
        WLA_DBG(WLADEBUG, "hardware busy on setting delivery status, STA index %d, TID %d\n", index, tid);
    }
#endif
    return 0;
}

unsigned int psbaq_all_status(unsigned char captbl_idx)
{
	u32 regval;
	MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | captbl_idx);
	while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD)
			;
    regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR);
	return regval;
}

unsigned char psbaq_qmode(unsigned char captbl_idx, unsigned char tid)
{
	u32 regval;
	MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | captbl_idx);
	while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD)
			;
    regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR);
	return (u8)((regval >> (tid*4)) & (QMODE_DRAIN_BIT|QMODE_BA_BIT|QMODE_PS_BIT));
}

char psbaq_qmode_is_busy(MAC_INFO *info, unsigned char captbl_idx, unsigned char tid)
{

	u16 val = psbaq_qmode(captbl_idx, tid);

	/* psbaq only at idle/ps-idle/ps-drain/drain mode can accept command */
	if(val & QMODE_DRAIN_BIT)
	{
		ucast_qinfo *qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[captbl_idx].qinfo_addr);
		if(qinfos[tid].tx_bm_lo || qinfos[tid].tx_bm_hi)
			return 1;
		WLA_MSG("!!!!!!!!DRAIN mode but no bitmap!!!!!!!!!!\n");
	}
	else if(val & QMODE_BA_BIT)
		return 1;
	return 0;
}

#endif

struct wsta_cb *update_ps_state(struct wbuf *wb, char ps_poll)
{
	MAC_INFO *info = WLA_MAC_INFO;
	short sa_idx = wb->sa_idx;
	struct wsta_cb *wcb = info->wcb_tbls[sa_idx];
	/* avoid one case: sta send frame before the hw release the addr table,
		that will cause the wb->sa_idx has value and wcb get NULL */
	if(wcb == 0)
		return 0;

	if(wb->ps)
	{
		if(wcb->ps_mode == 0)
			WLA_DBG(WLADEBUG, "STA[%d] sleep!\n", sa_idx);	
	}
	else if(wcb->ps_mode)
	{
		WLA_DBG(WLADEBUG, "STA[%d] awake!\n", sa_idx);	
	}

	wcb->ps_mode = wb->ps;
	wcb->activity = 1;

	return wcb;
}

void wla_beacon_interrupt_setup(MAC_INFO *info, u32 bss_idx, u32 role, u32 en)
{
	u32 val=0;
	u32 offset = 0x1 << bss_idx;
	u32 bss_bitmap=0;

	/* enable interrupt => set mask to "0" */
	if(!en)
		val = offset;
	
	MACREG_UPDATE32(TS_INT_MASK, val, offset);
	
#ifdef TBTT_EXCEPTION_HANDLE
	if(role != WIF_AP_ROLE)		// STA & IBSS
	{
		/* Next TBTT Exception */
		bss_bitmap = (offset << 16) | (offset << 24);
		
		if(!en)
			val = bss_bitmap;
		else
			val = 0;
	
		MACREG_UPDATE32(TS_AP_INT_MASK, val, bss_bitmap);
	}	
#endif  // TBTT_EXCEPTION_HANDLE
}


void wla_set_protection(u8 type)
{
    MAC_INFO *info = WLA_MAC_INFO;
    u32 protection_word = 0;

    info->erp_protect = 0;
    info->non_gf_exist = 0;

    if(type == GF_PROTECTION)
    {
        protection_word = (PROTECT_MODE_HT_NON_MEMBER<<PROTECT_SHIFT);
        info->non_gf_exist = 1;
    }
    else if(type == OFDM_PROTECTION)
    {
        protection_word = (PROTECT_MODE_HT_NON_MEMBER<<PROTECT_SHIFT)|BG_PROTECT|ERP_PROTECT;
        info->erp_protect = 1;
        info->non_gf_exist = 1;
    }

    MACREG_UPDATE32(RTSCTS_SET, protection_word, PROTECT_MODE|BG_PROTECT|ERP_PROTECT);
}

void wla_set_channel(MAC_INFO *info)
{
	u8 chan  = info->curr_channel;
	int level = info->txpowlevel;
	
	rf_set_channel(0, chan, level);

	if(info->bandwidth_type != BW20_ONLY)
	{ 
		rf_set_40mhz_channel(chan, info->bandwidth_type, level);
	}
}

#if 0
void wla_register_mgt_handler(MGT_FUNCPTR func)
{
	MAC_INFO *info = WLA_MAC_INFO;

	info->wla_mgt_handler = func;
	info->wla_eapol_handler = func; 
}
#endif
char wla_put_beacon(struct wbuf *wb1, struct wbuf *wb2, struct wbuf *wb3)
{
	return wla_add_beacon(WLA_MAC_INFO, wb1, wb2, wb3);
}

void wla_flow_control(MAC_INFO *info)
{
	u32 regval;
	u16 ac_be, ac_bk, ac_vo, ac_vi; 

	regval = MACREG_READ32(AC_QUEUE_COUNTER1);
	ac_be = regval >> 16;
	ac_bk = regval & AC_BK_QUEUE_COUNT;

	regval = MACREG_READ32(AC_QUEUE_COUNTER2);
	ac_vo = regval >> 16;
	ac_vi = regval & AC_VI_QUEUE_COUNT;

	/* JH redefine these register that it will latch maximum counter per ACQ 
	   rather than show dynamic counter. */
	MACREG_WRITE32(AC_QUEUE_COUNTER1, 0);
	MACREG_WRITE32(AC_QUEUE_COUNTER2, 0);
#if 0
	if(ac_vi)
		WLA_DBG(WLADEBUG, "ac_vi=%d\n", ac_vi);
	if(ac_vo)
		WLA_DBG(WLADEBUG, "ac_vo=%d\n", ac_vo);
	if(ac_be)
		WLA_DBG(WLADEBUG, "ac_be=%d\n", ac_be);
	if(ac_bk)
		WLA_DBG(WLADEBUG, "ac_bk=%d\n", ac_bk);
#endif	
	if(ac_vi|| ac_vo)
	{
		info->acq_len[ACQ_VO_IDX] = ACQ_TOTAL_SIZE/4;
		info->acq_len[ACQ_VI_IDX] = ACQ_TOTAL_SIZE/4;
	}
	else
	{
		if(--info->acq_len[ACQ_VO_IDX] < ACQ_MIN_SIZE)
			info->acq_len[ACQ_VO_IDX] = ACQ_MIN_SIZE;
		if(--info->acq_len[ACQ_VI_IDX] < ACQ_MIN_SIZE)
			info->acq_len[ACQ_VI_IDX] = ACQ_MIN_SIZE;
	}
	

	if(ac_bk)
	{
		info->acq_len[ACQ_BK_IDX] = (ACQ_TOTAL_SIZE - info->acq_len[ACQ_VO_IDX] - 
				info->acq_len[ACQ_VI_IDX])/2;	
	}
	else
	{
		if(--info->acq_len[ACQ_BK_IDX] < ACQ_MIN_SIZE)
			info->acq_len[ACQ_BK_IDX] = ACQ_MIN_SIZE;
	}

	/* BE takes remaining */
	info->acq_len[ACQ_BE_IDX] = ACQ_TOTAL_SIZE - info->acq_len[ACQ_VO_IDX] - 
				info->acq_len[ACQ_VI_IDX] - info->acq_len[ACQ_BK_IDX];

	MACREG_WRITE32(FC_ACQ_THRESHOLD, ((info->acq_len[ACQ_BK_IDX] << 24) | (info->acq_len[ACQ_BE_IDX] << 16) | (info->acq_len[ACQ_VI_IDX] << 8) | info->acq_len[ACQ_VO_IDX]));
}


void wla_start_tsync(u32 bss_idx, u32 beacon_intval, u32 dtimi_intval, u32 timeout)
{
	u16 offset = bss_idx * 0x10;
	u32 mask = (0x1000000 << bss_idx )|(0x10000 << bss_idx)|(0x100 << bss_idx)|(1 << bss_idx);

	if(timeout == 0)	/* add for IBSS mode to fill atim window (TU) */
		timeout = ((beacon_intval/3) & 0xffff) - 1;

	MACREG_UPDATE32(TS_INT_MASK, mask, mask);

	/* JH modify beacon sync window to entire TBTT. So we resume original duration of timeout(1/3 TBTT). */
	//MACREG_WRITE32(TS0_BE + offset, (beacon_intval << 16)|((((beacon_intval*1000)/2)&0xffff) - 1));
	MACREG_WRITE32(TS0_BE + offset, (beacon_intval << 16)|(timeout));

	MACREG_WRITE32(TS0_DTIM + offset, dtimi_intval);

	/* disable TS_ENABLE to reset timestamp & next_tbtt */
	MACREG_WRITE32(TS0_CTRL + offset, 0);
	/* FIXME: only enable TS_ENABLE & disable all other bits, is that right? */
	MACREG_WRITE32(TS0_CTRL + offset, TS_ENABLE);
	/* clear loss counter for client mode check ap alive */
	MACREG_WRITE32(TS0_BEACON_LOSS+offset, 0);
}

void wla_set_beacon_interrupt(u32 bss_idx, u32 role, u32 en)
{
	MAC_INFO *info = WLA_MAC_INFO;
	u32 val = 0;

	/* setup SW PRETBTT & TBTT ERROR interrupt */
	wla_beacon_interrupt_setup(info, bss_idx, role, en);

	/* if all sw interrupts disabled, disable the MAC INT MASK */
	if(!en && ((MACREG_READ32(TS_INT_MASK) & 0xFF) == 0xFF))
		val = MAC_INT_PRETBTT;
	else
		val = 0;

	MACREG_UPDATE32(MAC_INT_MASK_REG, val, MAC_INT_PRETBTT);
	info->int_mask = (info->int_mask & ~MAC_INT_PRETBTT) | val;
}

void wla_set_ap_isolated(u32 isolated)
{
	MAC_INFO *info = WLA_MAC_INFO;
	u32 val;
	if(isolated)
	{
		val = WIRELESS_SEPARATION;
		info->ap_isolated = 1;
	}
	else
	{
		val = 0;
		info->ap_isolated = 0;
	}
	MACREG_UPDATE32(RX_ACK_POLICY, val, WIRELESS_SEPARATION);
}

void wla_set_bss_isolated(u32 isolated)
{
	MAC_INFO *info = WLA_MAC_INFO;
	u32 val;
	if(isolated)
	{
		val = 1;
		info->bss_isolated = 1;
	}
	else
	{
		val = 0;
		info->bss_isolated = 0;
	}
	MACREG_WRITE32(BSSID_CONS_CHK_EN, val);
}

#ifdef CONFIG_WDS_PATCH
void wla_wds_bss_isolated(void)
{
	/*
		FIXED ME: Unicast frame from peer AP to STA with
		incorrect DA while enable hw bss isolation in WDS.
	*/
	MACREG_WRITE32(BSSID_CONS_CHK_EN, 0);
}
#endif

u32 wla_get_ssn(struct wsta_cb *wcb, u8 tid)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	sta_tx_status_tbl *tx_tbls;

	mac_stacap_cache_sync(wcb->captbl_idx, true);

	captbl = &info->sta_cap_tbls[wcb->captbl_idx];
	tx_tbls = (sta_tx_status_tbl *)CACHED_ADDR(captbl->TX_tbl_addr);

	return tx_tbls[tid].tx_sequence;
}

char ds_table(char mode)
{
	
	if((mode == LINKED_AP) || (mode == MAT_LINKED_AP)||(mode == WDS_LINKED_AP))
		return IN_DS_TBL;
	else
		return 0;
}

void send_dummy_frame(MAC_INFO *info, struct wsta_cb *wcb, u8 tid)
{
	struct wbuf *wb;
	char *datap;

	if (!(wb=WBUF_ALLOC(0)))
	{
		WLA_MSG("%s: no buf for send\n",__func__);
		return;
	}

	/* send a dummy frame (ETHERNET header+LLC header) */
	wb->sta_idx = wcb->captbl_idx;
	wb->flags |= WBUF_FIXED_SIZE|WBUF_TX_NO_SECU;
	wb->wh = 0;

	wb->data_off = sizeof(struct wbuf);
	wb->pkt_len = ETHER_HDR_LEN + 16; /* garbage payload */
	wb->tid = tid;

	datap = (char *)wb + wb->data_off;
	
	if(mac_addr_lookup_engine_find(datap, wcb->addr_idx, 0, (ds_table(wcb->mode)|BY_ADDR_IDX)) < 0)
		goto fail;
	memcpy(datap+6, maddr_tables.m[(int)wcb->bss_desc].addr, 6);

	DCACHE_STORE((unsigned int)wb, WLAN_TX_RESERVED_LEADIND_SPACE + wb->pkt_len);
	
	wla_tx_frame_handler(info, wb, wb);

	return;
fail:
	WBUF_FREE(wb);
}

int wla_ampdu(u32 cmd, struct wsta_cb *wcb, u8 tid, u16 ssn, u16 bufsize)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	sta_basic_info bcap;
	ucast_qinfo *qinfos;
	int ampdu_bitmap;
	u8 flag;

	if(wcb->hw_cap_ready == 0)
		return -1;

	captbl = &info->sta_cap_tbls[wcb->captbl_idx];
	flag = ds_table(wcb->mode)| BY_ADDR_IDX;
	if((mac_addr_lookup_engine_find(0, wcb->addr_idx, (int *)&bcap, flag) < 0) || 
		(bcap.bit.vld == 0))
	{
		WLA_MSG("%s(%d), addr_entry[%d] is lost! captbl=%d cmd:%d\n", __func__, __LINE__, wcb->addr_idx, wcb->captbl_idx,cmd);
		return -1;
	}
	ampdu_bitmap = 1 << tid;

	switch(cmd)
	{
		case GET_AMPDU_RX_STATUS:
			if(bcap.bit.rx_ampdu & ampdu_bitmap)
			{
				return 1;
			}
			break;
		case GET_AMPDU_TX_STATUS:
			if(wcb->tx_ampdu_bitmap & ampdu_bitmap)
			{
				return 1;
			}
			break;
		case START_AMPDU_RX:
		{
			if(mac_attach_ampdu_reorder_buf(info, wcb->captbl_idx, tid, ssn, bufsize) != 0)
				return -1;

			bcap.bit.rx_ampdu |= ampdu_bitmap;
			mac_invalidate_ampdu_scoreboard_cache(wcb->captbl_idx, tid);
			mac_addr_lookup_engine_update(info, wcb->addr_idx, bcap.val, flag);
			if(flag & IN_DS_TBL) 
				find_all_addrs_and_update(info, wcb->captbl_idx, bcap.val);
			WLA_DBG(WLADEBUG, "START_AMPDU_RX Done!! (%ld)\n", jiffies);
			break;
		}
		case START_AMPDU_TX:
		{
			u16 new_ssn;
#if defined(WLA_PSBA_QUEUE)
			qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[wcb->captbl_idx].qinfo_addr);

#ifdef TX_NOAGG_BY_PSBAQ
			if(info->txq_limit)
			{
				WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before AMPDU teardown\n", info->psbaq_cmd_waiting_max_cnt);

				MACREG_WRITE32(PSBAQ_QINFO_CSR2, (wcb->captbl_idx << 8) | (tid << 4) | PSBAQ_QINFO_CSR2_CMD_TEARDOWN | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
			}

#endif
			WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before AMPDU startup\n",info->psbaq_cmd_waiting_max_cnt);

			WMAC_WAIT_FOREVER(psbaq_qmode_is_busy(info, wcb->captbl_idx, tid), 
						"waiting on qmode idle to setup ampdu\n");
			qinfos[tid].tx_queue_len = MAX_AMPDU_QUEUE_LENGTH;

			wcb->tx_ampdu_bitmap |= ampdu_bitmap;

			/* AMPDU should only run in MCS rates, re-assign multirate */
			wcb->rate_flags |= RATE_FLAGS_INIT;
			wla_rc_update(info, wcb);


#if defined(INTEL_SETUP_BA_PATCH)
			if(wcb->chip_vendor == VENDOR_INTEL)
				new_ssn = ssn;
			else
#endif
			{
				u16 sn_gap=1000;
				/* Big sequence number jump to avoid AMPDU duplicates with MPDU */
				if(info->ampdu_sn_gap)
					sn_gap = info->ampdu_sn_gap;
				new_ssn = WLAN_SEQ_ADD(wla_get_ssn(wcb, tid), sn_gap);
			}

			MACREG_WRITE32(PSBAQ_QINFO_CSR0, (wcb->max_ampdu_factor << 18)|(new_ssn << 6) | (bufsize - 1));
			MACREG_WRITE32(PSBAQ_QINFO_CSR2, (wcb->captbl_idx << 8) | (tid << 4) | PSBAQ_QINFO_CSR2_CMD_SETUP | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
			send_dummy_frame(info, wcb, tid); /* send a dummy frame to trigger HW to send BAR */
#if 0
			do {
				MACREG_WRITE32(PSBA_PS_UCAST_TIM_CR, PS_UCAST_TIM_CMD | wcb->captbl_idx);
				while(MACREG_READ32(PSBA_PS_UCAST_TIM_CR) & PS_UCAST_TIM_CMD)
					;
    			regval = MACREG_READ32(PSBA_PS_UCAST_TIM_SR); 
			}while(!(regval & QMODE_BA_BIT));
			WLA_MSG("@@@@@@@@@@@new_ssn=%d, current sn=%d\n", new_ssn, wla_get_ssn(wcb, tid));
#endif
#if 0
			while(MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER)
			{
				WLA_DBG(WLADEBUG, "waiting on BA session startup, STA index %d, TID %d\n", wcb->captbl_idx, tid);
			}
#endif
			/* SEND BAR here to sync. the sequence number */
#endif
			WLA_DBG(WLADEBUG, "START_AMPDU_TX Done!! (%ld)\n", jiffies);
			break;
		}
		case STOP_AMPDU_RX:
		{
			int val;
			if((bcap.bit.rx_ampdu & ampdu_bitmap) == 0)
				return -1;

			wla_lock();
			info->stop_ampdu_rx= 1;
			wla_unlock();
			while((val = (MACREG_READ32(AMPDU_BMG_CTRL)) & (REORDER_FLUSH|BMG_CLEAR|VARIABLE_TABLE_1_CACHE_FLUSH|VARIABLE_TABLE_0_CACHE_FLUSH)))
			{
#if defined(CONFIG_WMAC_RECOVER)
				if(info->mac_recover_mode)
				{
					if(wla_monitor_check(1,
								&info->mon_bmgctrl_hit,
								&info->mon_bmgctrl_cnt,
								"Waiting for AMPDU_BMG_CTRL is idle"))
					{
						break;
					}
				}
#endif
				//wla_unlock();
				WLA_DBG(WLADEBUG, "Waiting for AMPDU_BMG_CTRL is idle..%x\n", MACREG_READ32(AMPDU_BMG_CTRL));
				wla_sleep(1);
				//wla_lock();
			}
			//wla_unlock();

			if(mac_flush_all_ampdu_reorder_buf(info, wcb->captbl_idx, tid) < 0)
			{
				WLA_DBG(WLADEBUG, "NO reorder buffer to STOP?\n");
				info->stop_ampdu_rx = 0;
				return -1;
			}
			bcap.bit.rx_ampdu &= ~ampdu_bitmap;
			mac_addr_lookup_engine_update(info, wcb->addr_idx, bcap.val, flag);
			if(flag & IN_DS_TBL) 
				find_all_addrs_and_update(info, wcb->captbl_idx, bcap.val);

#if defined(WLA_AMPDU_RX_HW_REORDER)
			MACREG_WRITE32(MMAC_RX_CTRL, MMAC_RX_CTRL_STA_LOOKUP_TBL_CLEARED);
			{
				unsigned int cnt = 0;

				while((MACREG_READ32(MMAC_RX_CTRL) & MMAC_RX_CTRL_STA_LOOKUP_TBL_CLEARED))
				{
					if(++cnt > 5)
					{
#if defined(CONFIG_WMAC_RECOVER)
						if(info->mac_recover_mode)
						{
							if(wla_monitor_check(1,
										&info->mon_bmgctrl_hit,
										&info->mon_bmgctrl_cnt,
										"waiting MMAC_RX_CTRL equal zero"))
							{
								break;
							}
						}
#endif
						wla_sleep(1);
						if((cnt&0xffff) == 0xffff) 
							DBG("waiting MMAC_RX_CTRL equal zero: waiting cache writeback\n??????waiting(%d)????????\n", cnt); 
					}
				}
			}
#endif
			/* flush cache before detach reorder buffer */
			MACREG_WRITE32(AMPDU_BMG_CTRL, VARIABLE_TABLE_1_CACHE_FLUSH | VARIABLE_TABLE_0_CACHE_FLUSH);

			info->stop_ampdu_rx = 0;
	
			mac_invalidate_ampdu_scoreboard_cache(wcb->captbl_idx, tid);
			WMAC_WAIT_FOREVER(MACREG_READ32(AMPDU_BMG_CTRL) & BMG_CLEAR, 
							"waiting BMG_CLEAR equal zero\n");

			mac_detach_ampdu_reorder_buf(info, wcb->captbl_idx, tid);
			WLA_DBG(WLADEBUG, "STOP_AMPDU_RX Done!! (%ld)\n", jiffies);

			break;
		}
		case STOP_AMPDU_TX:
		{
			if((wcb->tx_ampdu_bitmap & ampdu_bitmap) == 0)
				return -1;

#if defined(WLA_PSBA_QUEUE)
			qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[wcb->captbl_idx].qinfo_addr);

			WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before AMPDU teardown\n", info->psbaq_cmd_waiting_max_cnt);

			if((psbaq_qmode(wcb->captbl_idx, tid) & (QMODE_BA_BIT|QMODE_DRAIN_BIT)) != QMODE_BA_BIT)
			{
				WLA_DBG(WLADEBUG, "psbaq_qmode(%d, %d)=%x, can not teardown BA?\n", wcb->captbl_idx, tid, psbaq_qmode(wcb->captbl_idx, tid));
				break;
			}
			wcb->tx_ampdu_bitmap &= ~ampdu_bitmap;

			if(0) /* for debug */
			{
				int i;
				sta_cap_tbl *captbl = &info->sta_cap_tbls[wcb->captbl_idx];
				sta_tx_status_tbl *tx_tbls = (sta_tx_status_tbl *)CACHED_ADDR(captbl->TX_tbl_addr);
				mac_stacap_cache_sync(wcb->captbl_idx, true);
				for(i=0; i<8; i++)
				{
					WLA_MSG("[%d], tx_tbls[i].tx_sequence=%d, tx_ok_cnt=%d\n", i, tx_tbls[i].tx_sequence, tx_tbls[i].tx_ok_cnt);
				}
			}

			MACREG_WRITE32(PSBAQ_QINFO_CSR2, (wcb->captbl_idx << 8) | (tid << 4) | PSBAQ_QINFO_CSR2_CMD_TEARDOWN | PSBAQ_QINFO_CSR2_CMD_TRIGGER);

			WMAC_WAIT_FOREVER((psbaq_qmode(wcb->captbl_idx, tid) == QMODE_BA_BIT), 
						"waiting on qmode exit BA mode\n");
	
			qinfos[tid].tx_queue_len = MAX_PS_QUEUE_LENGTH;
			wcb->rate_flags |= RATE_FLAGS_INIT;
			wla_rc_update(info, wcb);

#ifdef TX_NOAGG_BY_PSBAQ
			if(info->txq_limit)
			{
				WMAC_WAIT_FOREVER_AND_CNT((MACREG_READ32(PSBAQ_QINFO_CSR2) & PSBAQ_QINFO_CSR2_CMD_TRIGGER), 
						"waiting on PSBAQ command idle before nogg-ba setup\n", info->psbaq_cmd_waiting_max_cnt);

				WMAC_WAIT_FOREVER(psbaq_qmode_is_busy(info, wcb->captbl_idx, tid), 
						"waiting on qmode idle to setup ampdu\n");

				/* psbaq only at idle/ps-idle/ps-drain mode can accept command */
				//WMAC_WAIT_FOREVER((qinfos[tid].qmode & (QMODE_BA_BIT|QMODE_DRAIN_BIT)), 
				//		"waiting on qmode idle to setup ampdu\n");


				MACREG_WRITE32(PSBAQ_QINFO_CSR0, PSBAQ_QINFO_CSR0_FORCE_NOAGG);
				MACREG_WRITE32(PSBAQ_QINFO_CSR2, (wcb->captbl_idx << 8) | (tid << 4) | PSBAQ_QINFO_CSR2_CMD_SETUP | PSBAQ_QINFO_CSR2_CMD_TRIGGER);
			}
#endif
#endif

			WLA_DBG(WLADEBUG, "STOP_AMPDU_TX Done!! (%ld)\n", jiffies);
			break;
		}
	}
	
	return 0; 
}

void wla_forward(u32 cmd, struct wsta_cb *wcb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	sta_basic_info bcap;
	char flag;

	if(wcb->hw_cap_ready == 0)
		return;

	flag = ds_table(wcb->mode)| BY_ADDR_IDX;

	captbl = &info->sta_cap_tbls[wcb->captbl_idx];
	if(mac_addr_lookup_engine_find(0, wcb->addr_idx, (int *)&bcap, flag) < 0) 
		return;

	switch(cmd)
	{
		case DATA_NO_FORWARD:
			bcap.bit.tosw = 1;
			wcb->controlled_port = 0;
			break;
		case DATA_CAN_FORWARD:
			if((info->data_forward_mode == SW_FORWARD_DATA) || 
				(wcb->mode == MAT_LINKED_AP))
				bcap.bit.tosw = 1;
			else
				bcap.bit.tosw = 0;

			wcb->controlled_port = 1;
			break;
	}
	mac_addr_lookup_engine_update(info, wcb->addr_idx, bcap.val, flag);
	if(flag & IN_DS_TBL) 
		find_all_addrs_and_update(info, wcb->captbl_idx, bcap.val);
	return; 
}

void wla_cipher_key(struct wsta_cb *wcb, unsigned char cipher_type, unsigned char key_type,
						char *key, char keyidx, char is_txkey)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl, *captbl1;
	sta_basic_info bcap;
	sta_rx_status_tbl *rx_tbls;
	int addr_idx, idx, i;
	cipher_key *hwkey;
	char flag = 0;
	u8 wif_role;
	u8 rate_tbl_sync=0;

	if(info->mac_is_ready == false) 
		return;

	if(key_type == KEY_TYPE_GLOBAL_KEY)
	{
		idx = (int) wcb;
		addr_idx = 0;
	}
	else
	{
		if(wcb->hw_cap_ready == 0)
			return;
		
		addr_idx = wcb->addr_idx;
		idx = wcb->captbl_idx;
		captbl = &info->sta_cap_tbls[idx];
		flag = ds_table(wcb->mode); 
		mac_addr_lookup_engine_find(0, addr_idx, (int *)&bcap, flag|BY_ADDR_IDX);
	}

WLA_DBG(WLADEBUG, "cipher_type=%d, key_type=%d, tbl_idx=%d, keyidx=%d, is_txkey=%d, key=\n", cipher_type, key_type, idx, keyidx, is_txkey);

	if(key)
		WLA_DUMP_BUF(key, 32);
#if defined (CONFIG_WLA_WAPI) || defined (CONFIG_WAPI)
	if(cipher_type == CIPHER_TYPE_SMS4)
	{
		if(key_type == KEY_TYPE_STA_PAIRWISE_KEY)
			mac_rekey_start(idx, key_type, cipher_type, ((WAPI_PN_DWORD + 1) & 0xff));
		else if(key_type == KEY_TYPE_PAIRWISE_KEY)
			mac_rekey_start(idx, key_type, cipher_type, WAPI_PN_DWORD & 0xff);
		else
			mac_rekey_start(idx, key_type, cipher_type, 0);
	}
	else
#endif
		mac_rekey_start(idx, key_type, cipher_type, 0);

	captbl = 0;
	if(key_type <= KEY_TYPE_STA_PAIRWISE_KEY)
	{
		hwkey = IDX_TO_PRIVATE_KEY(idx);
		mac_set_key(hwkey, key, cipher_type, keyidx, is_txkey);
 
		captbl = IDX_TO_STACAP(idx);
		captbl->cipher_mode = cipher_type;

		bcap.bit.wep_defkey = 0;
		captbl->wep_defkey = 0;
		mac_addr_lookup_engine_update(info, addr_idx, bcap.val, flag|BY_ADDR_IDX);
		if(flag & IN_DS_TBL) 
			find_all_addrs_and_update(info, idx, bcap.val);
	
		if(cipher_type == CIPHER_TYPE_SMS4)
		{
			captbl->tx_inc = WAPI_TX_INC_TWO;
			if(key_type == KEY_TYPE_STA_PAIRWISE_KEY)
			{
				captbl->rx_check_inc = WAPI_RX_CHECK_INC_ODD;
			}
			else
			{
				captbl->rx_check_inc = WAPI_RX_CHECK_INC_EVEN;
				hwkey->wapi_key.txpn[0] += 1;
			}
		}
		
		rx_tbls = (sta_rx_status_tbl*) NONCACHED_ADDR(captbl->RX_tbl_addr);
		for(i=0;i<QOS_RX_STATUS_TBLS;i++)
		{
			if(cipher_type == CIPHER_TYPE_SMS4)
			{
				rx_tbls[i].rsc[0] = WAPI_PN_DWORD;
                rx_tbls[i].rsc[1] = WAPI_PN_DWORD;
				rx_tbls[i].rsc[2] = WAPI_PN_DWORD;
                rx_tbls[i].rsc[3] = WAPI_PN_DWORD;
			}
			else
			{
                rx_tbls[i].rsc[0] = 0;
                rx_tbls[i].rsc[1] = 0;
			}
		}

	}
	else if(key_type == KEY_TYPE_GLOBAL_KEY)
	{
		hwkey = &info->group_keys[idx].group_key;
		mac_set_key(hwkey, key, cipher_type, keyidx, is_txkey);
		wif_role = info->mbss[idx].role;

		/* FIXME: IBSS shoule do which policy ? */
		if(wif_role & WIF_AP_ROLE)
		{
			if(info->mbss[idx].tx_rate[BASIC_TX_RATE].rate_idx)
			{
				captbl = IDX_TO_RATETBL(info->mbss[idx].tx_rate[BASIC_TX_RATE].rate_captbl);
				captbl->cipher_mode = cipher_type;
				captbl->wep_defkey = 0;

				if((cipher_type == AUTH_CIPHER_WEP40) || (cipher_type == AUTH_CIPHER_WEP104))
					captbl->wep_defkey = 1;
				else if(cipher_type == CIPHER_TYPE_SMS4)
					captbl->tx_inc = WAPI_TX_INC_ONE;
	
				rate_tbl_sync = info->mbss[idx].tx_rate[BASIC_TX_RATE].rate_captbl;
			}
			if(info->mbss[idx].tx_rate[FIXED_TX_RATE].rate_idx)
			{
				captbl1 = IDX_TO_RATETBL(info->mbss[idx].tx_rate[FIXED_TX_RATE].rate_captbl);
				captbl1->cipher_mode = captbl->cipher_mode;
				captbl1->wep_defkey = captbl->wep_defkey;
				captbl1->tx_inc = captbl->tx_inc;

				DCACHE_STORE((unsigned int)captbl1, 16);
			}
		}
		else //if(wif_role & WIF_STA_ROLE)
		{
			captbl = IDX_TO_STACAP(idx);
	
			for(i=0;i<QOS_RX_STATUS_TBLS;i++)
			{
				if(cipher_type == CIPHER_TYPE_SMS4)
					info->group_keys[idx].rsc[i] = WAPI_PN_64BITS;
				else
					info->group_keys[idx].rsc[i] = 0;
			}
		}
	}

	mac_rekey_done();
	if(captbl)
	{
		DCACHE_STORE((unsigned int)captbl, 16);
	}
	mac_stacap_cache_sync(idx, true);
	
	/* do the rate table cache sync for the cipher change */
	if(rate_tbl_sync)
		mac_ratecap_cache_sync(rate_tbl_sync, true);
	
	/* Add for wapi unicast drop issue (the rsc/pn is garbage in the hw cache) */
	/* Also for the general case to safety setup key 						  */
	mac_rx_linkinfo_cache_sync();
}

#if 1
int wla_cfg(unsigned int cmd, ...)
{
	MAC_INFO *info = WLA_MAC_INFO;
	va_list args;

	va_start(args, cmd);

	switch((int)cmd)
	{
		case SET_BEACON_PARAMS:
		{
			int beacon_interval, dtim_period; 
			beacon_interval = va_arg (args, int);
			dtim_period = va_arg (args, int);

			info->beacon_interval = beacon_interval/10;
			wla_beacon_setup(beacon_interval, dtim_period);
			break;	
		}
		case SET_CHANNEL:
		{
			int chan, mode;
			chan = va_arg (args, int);
			mode = va_arg (args, int);

			if(mode == BW40MHZ_SCA)
				info->bandwidth_type = HT40_PLUS;
			else if(mode == BW40MHZ_SCB)
				info->bandwidth_type = HT40_MINUS;
			else
				info->bandwidth_type = BW20_ONLY;
			
			info->curr_channel = chan;
	
			wla_set_channel(info);
			wla_set_bandwidth_type(info);
			break;
		}
		case SET_TX_RATE:
		{
			int type, flags, rate, bss_desc;

			type = va_arg (args, int);
			bss_desc = va_arg (args, int);
			flags = va_arg (args, int);
			rate = va_arg (args, int);
			
			wla_setup_tx_rate(info, bss_desc, rate, type, flags);	
			break;
		}
		case DEL_TX_RATE:
		{
			int is_basic_rate, bss_desc;

			is_basic_rate = va_arg (args, int);
			bss_desc = va_arg (args, int);
			va_end(args);

			wla_setup_tx_rate(info, bss_desc, -1, is_basic_rate, 0);
			break;
		}
		case SET_FRAG_THRESHOLD:
		{
			int val;
			val = va_arg (args, int);
			MACREG_UPDATE32(TXFRAG_CNTL, val, 0x0000FFFFUL);
			
			break;
		}
		case SET_RTS_THRESHOLD:
		{
			int val;
			val = va_arg (args, int);
			MACREG_UPDATE32(RTSCTS_SET, (val<<16), LMAC_RTS_CTS_THRESHOLD);
			
			break;
		}
		case SET_WDS_MODE:
		{
			info->wds_mode = va_arg (args, int);
			if(info->wds_mode == AP_WDS_BRIDGE)
			{
				wla_beacon_stop(info);
				info->wla_mgt_handler = 0;
				info->wla_eapol_handler = 0;
			}
			break;
		}
		case SET_RX_FILTER:
		{
			info->filter |= va_arg (args, int);
			if(info->filter & RX_FILTER_BEACON)
			{
				MACREG_UPDATE32(LMAC_CNTL, LMAC_CNTL_BEACON_FILTER, LMAC_CNTL_BEACON_FILTER);
			}
			break;
		}
		case DEL_RX_FILTER:
		{
			info->filter &= ~(va_arg (args, int));
			if((info->filter & RX_FILTER_BEACON) == 0)
				MACREG_UPDATE32(LMAC_CNTL, 0, LMAC_CNTL_BEACON_FILTER);

			break;
		}
#ifdef CONFIG_TX_MCAST2UNI
		case SET_TX_MCAST2UNI:
		{
			info->mcast2ucast_mode = va_arg (args, int);
			break;
		}
#endif
		case SET_POWER_SAVING:
		{
			info->power_saving_mode = va_arg (args, int);
			if(info->power_saving_mode > 0)
			{
				/* enable hardware PS function */
				/* program hardware to postpone UAPSD tigger frame event & PSPoll betwwen pre-tbtt and beacon TX */
   				MACREG_WRITE32(PS_FUNC_CTRL, PS_FUNC_POSTPONE_UAPSD_TRIGGER|PS_FUNC_POSTPONE_PSPOLL);
			}
			else
				MACREG_WRITE32(PS_FUNC_CTRL, PS_FUNC_DISABLE);

			break;
		}
		case SET_SLOTTIME:
		{
			info->slottime = va_arg (args, int);
			MACREG_UPDATE32(OFDM_DEFER_SET, (info->slottime << 24), 0xff000000);
			break;
		}
		case SET_40MHZ_INTOLERANT:
		{
			int i;
			struct wsta_cb *wcb;

			info->bw40mhz_intolerant = va_arg (args, int);
			for(i=0; i<info->sta_cap_tbl_count; i++)
			{
				if(info->sta_cap_tbls[i].valid)
				{
					wcb = info->wcb_tbls[i];
					if(wcb)
					{
						wcb->rate_flags |= RATE_FLAGS_INIT;
						wla_rc_update(info, wcb);
					}
				}
			}
			break;
		}
		case SET_FORWARD_MODE:
		{
			info->data_forward_mode = va_arg (args, int); 
			break;
		}
		case SET_TXQ_LIMIT:
		{
			info->txq_limit = va_arg (args, int); 
			break;
		}
		case SET_RATE_ADAPTION:
		{
			info->rate_adaption_policy = va_arg (args, int); 
			break;
		}
		case SET_ACQ_LEN:
		{
			int i = va_arg (args, int);
			if(i != 0xffffffff)
			{
				info->fix_ac_queue_limit = 1;
				memcpy(&info->acq_len[0], &i, 4);
				MACREG_WRITE32(FC_ACQ_THRESHOLD, i);
			}
			break;
		}
		case SET_WME_ACM:
		{
			int bss_desc, wme_acm;

			bss_desc = va_arg (args, int);
			wme_acm = va_arg (args, int);
			WLA_DBG(WLADEBUG, "wme acm update: bss_desc(%d) acm(%x)\n", bss_desc, wme_acm);
			info->mbss[bss_desc].wme_acm = wme_acm;
			break;
		}
		default:
			break;	
	}

	va_end(args);
	return RETURN_OK;
}
#endif

u32 wla_get_cca(void)
{
	u32 curr_count;

	curr_count = MACREG_READ32(LMAC_MEDBUSY_CNT);

	MACREG_WRITE32(LMAC_MEDBUSY_CNT, LMAC_MEDBUSY_CNT_CLEAR);
	MACREG_WRITE32(LMAC_MEDBUSY_CNT, 0);
	return curr_count;
}

void wla_tx_power(u32 level)
{
	MAC_INFO *info = WLA_MAC_INFO;
	info->txpowlevel = rf_set_tx_gain(level);
}


void wla_rf_reg(u8 num, u32 val)
{
	u32 i;
	struct rf_regs *regs = &rf_tbl[0];

	for(i=0; i<MAX_RF_REG_NUM; i++)
		if(regs[i].num == 0)
			break;
	if(i == MAX_RF_REG_NUM)
	{
		WLA_DBG(WLAERROR, "no enough space to store RF setting!!\n");
		return;
	}
	regs[i].num = num;
	regs[i].val = val;
}

void wla_rf_txvga_reg(u8 ch, u32 val)
{
	if(ch >= MAX_RF_K_CH_NUM)
	{
		WLA_DBG(WLAERROR, "out of maximum channel number\n");
		return;
	}

	rf_reg7_txvag[ch] = val;
}

u32 wla_get_rf_txvga_reg(u8 ch)
{
	if(ch >= MAX_RF_K_CH_NUM)
	{
		WLA_DBG(WLAERROR, "out of maximum channel number\n");
		return 0;
	}

	return rf_reg7_txvag[ch];
}

void wla_rf_freq_ofs_reg(u32 val)
{
	rf_reg11_freq_ofs = val;
}

u32 wla_get_rf_freq_ofs_reg()
{
	return rf_reg11_freq_ofs;
}

void wla_bb_reg(char *rfc_data, u32 len)
{
	struct bb_regs *reg = &rfc_tbl[0];

	memset((char *)reg, 0, sizeof(rfc_tbl));

	/* read fixed parameter */
	if(rfc_data)
	{
		if(len > sizeof(rfc_tbl))
		{
			WLA_DBG(WLADEBUG, "???init rfc too long, len=%d\n", len);
		}
		else
			memcpy(rfc_tbl, rfc_data, len);
	}	
}

u16 wla_sta_forward_path(char *addr)
{
	sta_basic_info bcap;
	MAC_INFO *info = WLA_MAC_INFO;

	if(!info->mac_is_ready)
		return 0;

	if(mac_addr_lookup_engine_find(addr, 0, (int *)&bcap, 0) >= 0)
	{
		u16 val = bcap.bit.bssid << STA_BSSDESC_SHIFT;
		if(bcap.bit.tosw)
			val |= SOFTWARE_FORWARD;
		else
			val |= HARDWARE_FORWARD;
		return val;
	}

	return 0;
}

u8 wla_mac_status(void)
{
	MAC_INFO *info = WLA_MAC_INFO;
	return info->mac_is_ready;
}

void wla_set_recover(u8 val)
{
	MAC_INFO *info = WLA_MAC_INFO;
	info->mac_recover = val;
}

u32 wla_read_beacon_loss_count(u32 bss_idx)
{
	u8 offset = 0x10*bss_idx;
	u32 lose = MACREG_READ32(TS0_BEACON_LOSS+offset);

	/* clear loss counter */
	MACREG_WRITE32(TS0_BEACON_LOSS+offset, 0);

	return lose;
}

#ifdef CONFIG_WLAN_CLIENT_PS
u32 wla_busy_timestamp(u32 bss_desc, u32 timestamp, u32 act)
{
	MAC_INFO *info = WLA_MAC_INFO;
	
	if(act)	// set timestamp to the mac info
	{
		info->mbss[bss_desc].busy_timestamp = timestamp;
	}

	return info->mbss[bss_desc].busy_timestamp;
}

int wla_ps(u8 bss_desc, u8 state, u8 is_tx, u8 op)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct mbss_t *mbss = &info->mbss[bss_desc];
	u8 st=0;
	u8 idx=0;

	if(is_tx)
		st = mbss->ps_tx_state;
	else
		st = mbss->ps_rx_state;

	if(op)		/* op == 1: set ps state, op == 0: get ps state */
	{
		if(st != state)
		{
			//ARTHUR_DBG(WARN, "%s(): bss_desc=%d, ori state=%d, new state=%d, is_tx=%d\n", __FUNCTION__, bss_desc, st, state, is_tx);
			
			sta_cap_tbl *captbl=NULL;
			if(mbss->ln_ap)
			{
				idx = mbss->ln_ap->sta_captbl;
				captbl = &info->sta_cap_tbls[idx];
			}
			else
				return -1;

			wla_lock();

			switch(state)
			{
				case PS_STATE_NONE:
				case PS_STATE_AWAKE:
					/* set the data path mode to the cdb setting */
					/*FIXME:*/
					wla_cfg(SET_FORWARD_MODE, WLA_CDB_GET($wl_forward_mode, 0));
					captbl->power_saving = 0;
					break;
				case PS_STATE_DOZE:
					/* FIXME: disable tx or rx power saving */
					/* set the data path mode to the software path */
					wla_cfg(SET_FORWARD_MODE, 1);
					captbl->power_saving = 1;
					break;
				case PS_STATE_SLEEP:
					/* FIXME: (1) enable tx or rx power saving only all bss are in sleep state
					          (2) set sta_cap_tbl->power_saving  */
					break;
			}

			st = state;
			
			if(is_tx)
				mbss->ps_tx_state = st;
			else
				mbss->ps_rx_state = st;
	
			mac_stacap_cache_sync(idx, true); 
	
			wla_unlock();
		}
	}

	return st;
}

void wla_cfg_ps_policy(u8 bss_desc, u8 ps_policy)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct mbss_t *mbss = &info->mbss[bss_desc];

	/* only called once in wm_reload_config(), then don't need lock protect */

	mbss->ps_policy = ps_policy;
}

u8 wla_get_ps_policy(u8 bss_desc)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct mbss_t *mbss = &info->mbss[bss_desc];

	return mbss->ps_policy;
}

#endif // CONFIG_WLAN_CLIENT_PS

void wla_software_intr(int *parm)
{
	struct mbss_t *mbss=NULL;
	int ap_trigger=0;
	int i,j;
	unsigned int ts_status = (unsigned int) parm;
	MAC_INFO *info = WLA_MAC_INFO;

	for(i=0; i<MAC_MAX_BSSIDS; i++)
	{
		j=1<<i;
		if(!(ts_status & j))
			continue;
		ts_status &= ~j;

		mbss = &info->mbss[i];

		if((mbss->role == WIF_AP_ROLE) || (mbss->role == WIF_IBSS_ROLE))
		{
			if(i == 0)
				info->ts0_int++;

			if(ap_trigger == 0)
			{
				wla_beacon(info);
				ap_trigger = 1;
			}
		}
#ifdef CONFIG_WLAN_CLIENT_PS
		else if(mbss->role == WIF_STA_ROLE)
		{
			client_tbtt_handle(info, i);
		}
#endif // CONFIG_WLAN_CLIENT_PS
	}
}


void wla_mgt_send(struct wbuf *wb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wlan_hdr *hdr = (struct wlan_hdr *)((unsigned int)wb + wb->data_off);
	int add2cache=0;

	/* 	1. 	record the RA to the cache : (re)assoc_req / (re)assoc_resp 
		2.	clear the cache when deauth 									*/
	if(hdr->type == WLAN_FC_TYPE_MGT)
	{
		if((hdr->subtype == WLAN_FC_SUBTYPE_ASSOC_RESP) || (hdr->subtype == WLAN_FC_SUBTYPE_REASSOC_RESP))
		{
			if(((struct wlan_assoc_resp_frame *)hdr)->statuscode == 0)
				add2cache = 1;
		}
		else if((hdr->subtype == WLAN_FC_SUBTYPE_ASSOC_REQ) || (hdr->subtype == WLAN_FC_SUBTYPE_REASSOC_REQ))
		{
			add2cache = 1;
		}
		else if((hdr->subtype == WLAN_FC_SUBTYPE_DEAUTH))
		{
			if(memcmp(info->peer_cache, hdr->addr1, 6) == 0)
				memset(info->peer_cache, 0, 6);
		}

		if(add2cache)
			memcpy(info->peer_cache, hdr->addr1, 6);
	}

	wla_tx_frame_handler(info, wb, wb);
}

void wla_set_timer(void)
{
	unsigned int closest_time, delta;
	MAC_INFO *info = WLA_MAC_INFO;

	closest_time = info->hwaddr_age_time;
	if(info->sta_timeout_list)
		if(info->sta_timeout_list->sync_time < info->hwaddr_age_time)
			closest_time = info->sta_timeout_list->sync_time;

	if(closest_time < WLA_CURRENT_TIME)
		delta = closest_time+(0xffffffff - WLA_CURRENT_TIME);
	else
		delta = closest_time - WLA_CURRENT_TIME;

	/* minimum delta is 1 sec */ 
	if(delta > 100)
		delta = 100;
	
	info->mon_t_closest = closest_time;
	info->mon_t_current = WLA_CURRENT_TIME;
	info->mon_t_delta = delta;

	wla_add_timer(delta);
}

void wla_hw_sync(void)
{
	struct wsta_cb *wcb;
	MAC_INFO *info = WLA_MAC_INFO;
	unsigned int now, next_timeout;
#define ONE_WEEK (7*24*60*60*100)

	now = WLA_CURRENT_TIME;

#if defined(CONFIG_WMAC_RECOVER)
	{
		int val;
#ifndef CONFIG_MPEGTS
		short head, tail;

		val = MACREG_READ32(HWBUFF_CURR_HT);
		head = val >> 16;
		tail = val & 0xffff;
		wla_monitor_check((head == tail),
					&info->mon_bufempty_hit,
					&info->mon_bufempty_cnt,
					"Run out hwbuf");
#endif
		MACREG_WRITE32(DEBUG_GROUP_SEL, 0x129);
		val = MACREG_READ32(MMAC_DEBUG_SIGNAL);
		wla_monitor_check((val == 0x003d609c),
					&info->mon_rxzombie_hit,
					&info->mon_rxzombie_cnt,
					"RX zombie");
	}
	if(info->mac_recover)
	{
		wla_recover();
		info->mac_recover = 0;
	}
#endif
	wla_lock();
	/* per station */	
	while(info->sta_timeout_list)
	{
		wcb = info->sta_timeout_list;
		if(TIME_BEFORE(now, wcb->sync_time))
			break;

		info->sta_timeout_list = wcb->next;

		if(!wcb->hw_cap_ready && !wcb->sta)
		{
			printk("!!!! Ready to post release addr_entry:%d captbl:%d\n",wcb->addr_idx, wcb->captbl_idx);
			if(wla_post_release_sta(wcb) != 0)
			{
				/* polling HW disassociate is completed */
				wla_add_sta_timer(wcb, 100);
			}
			continue;
		}

		wcb->activity = 0;
		/* Do not lock wla_update_sta_status which has waiting loops */
		wla_unlock();
		next_timeout = wla_update_sta_status(wcb, now);
		wla_lock();

		if(wcb->activity) {
			struct wla_softc *sc = info->sc;
			struct ieee80211_sub_if_data *sdata = NULL;
			struct ieee80211_vif *vif=NULL;
			struct net_device *dev;
			
			if(wcb->bss_desc < 4) {
				dev=sc->bssdev[wcb->bss_desc];
				if(dev!=NULL)
				{		
					sdata = IEEE80211_DEV_TO_SUB_IF(dev);
					vif = &sdata->vif;
					if((vif->type == NL80211_IFTYPE_STATION) && (vif->bss_conf.assoc))
						ieee80211_sta_reset_conn_monitor(sdata);
				}
			}
		}

		if(next_timeout)
			wla_add_sta_timer(wcb, next_timeout);
	}	
	wla_unlock();


	/* addr_tbl aging check */
	if(TIME_AFTER_EQ(now, info->hwaddr_age_time))
	{
		sta_basic_info bcap;
		unsigned int bitmap;
		unsigned int *hwaddr_lookup_hit_bitmap = MACREG_ADDR(SA_AGE_BITMAP);
		int i, j, addr_idx;
	
		info->hwaddr_age_time = now + WLA_HWADDR_AGE;

		for(i=0; i < MAX_ADDR_TABLE_ENTRIES/BITS_PER_LONG; i++)
		{
			bitmap = hwaddr_lookup_hit_bitmap[i];
			hwaddr_lookup_hit_bitmap[i] = 0;
			for(j=0; j<BITS_PER_LONG; j++)
			{
				addr_idx = i*BITS_PER_LONG+j;
				bcap.val = 0;
				mac_addr_lookup_engine_find(0, addr_idx, (int *)&bcap, BY_ADDR_IDX);
				if(bcap.bit.vld == 0)
					continue;

#ifdef WLA_MULTICAST_PATCH
				if(bcap.bit.captbl_idx == 255)
					continue;
#endif

				wcb = info->wcb_tbls[bcap.bit.captbl_idx];
				if(!wcb) {
					idb_print("[%s]:Check wcb: idx:%d max:%d\n",__func__,bcap.bit.captbl_idx,info->sta_cap_tbl_count);
					continue;
				}

				if((wcb->mode != LINKED_AP) && (wcb->mode != WDS_LINKED_AP)) 
					continue;

				if((bitmap & (0x1UL << j)) == 0)
				{
					/* If no packet from this STA, release it. */
					mac_addr_lookup_engine_update(info, addr_idx, 0, BY_ADDR_IDX);
				}
			}
		}
	}

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
	if(TIME_AFTER_EQ(now, info->mat_timestamp))
	{
		mat_timeout_handle();
		info->mat_timestamp = now + MAT_CHECK_TIME;
	}
#endif
	if(TIME_AFTER_EQ(now, info->acq_timestamp))
	{
		if(!info->fix_ac_queue_limit)
			wla_flow_control(info);
		info->acq_timestamp = now + WLA_ACQ_CHECK_TIME;
	}

	if((TIME_AFTER_EQ(now, info->bcn_mon_timestamp)))
	{
		u32 loss=0;
		struct wla_softc *sc = info->sc;
		struct ieee80211_sub_if_data *sdata = NULL;
		struct ieee80211_vif *vif;
	    int bss_desc;
	    struct net_device *dev;
		
		for(bss_desc=0; bss_desc < 4; bss_desc++)
		{
			dev=sc->bssdev[bss_desc];
			if(dev!=NULL)
			{		
				sdata=IEEE80211_DEV_TO_SUB_IF(dev);
				vif = &sdata->vif;
				if((vif->type == NL80211_IFTYPE_STATION) && (vif->bss_conf.assoc)) {
					struct wla_vif_priv *vp = (void *)vif->drv_priv;
					loss=wla_read_beacon_loss_count(bss_desc);
					if(loss < vp->bcn_threshold) {
						ieee80211_sta_reset_beacon_monitor(sdata);
					}
				}
			}
		}
		info->bcn_mon_timestamp = now + WLA_BCN_MON_TIME;
	}
		
	wla_set_timer();
}

void wla_add_sta_timer(struct wsta_cb *new, unsigned int timeout)
{
	struct wsta_cb *wcb, *prev;
	unsigned int expire;
	MAC_INFO *info = WLA_MAC_INFO;
	
	wla_lock();
	expire = WLA_CURRENT_TIME + timeout;
	new->sync_time = expire;

	/* insert new one by expire time */
	for(wcb=info->sta_timeout_list, prev=0; wcb; prev=wcb, wcb = wcb->next)
	{
		if(wcb->sync_time > expire)
		{
			/* If next sync time large than one week, it means 
				current time is overflow. */
			if((wcb->sync_time - expire) < (7*24*60*60*100))
				break;
		}
	}

	if(prev)
	{
		new->next = prev->next;
		prev->next = new;
	}
	else
	{
		new->next = info->sta_timeout_list;
		info->sta_timeout_list = new;

		wla_set_timer();
	}

	wla_unlock();
}
#if 0 /* No need, sta timer should not be forced to unplug */
void wla_del_sta_timer(struct wsta_cb *wcb)
{
	struct wsta_cb *one, *prev;
	MAC_INFO *info = WLA_MAC_INFO;

	wla_lock();
	for(one=info->sta_timeout_list, prev=0; one; prev=one, one=one->next)
	{
		if(one == wcb)
		{
			if(prev)
				prev->next = one->next;
			else
			{
				info->sta_timeout_list = one->next;
				wla_set_timer();
			}
			break;
		}
	}
	wla_unlock();

	wcb->next = 0;
}
#endif

#if defined(CONFIG_WMAC_RECOVER)
int wla_monitor_check(unsigned int val, unsigned int *cnt, unsigned int *cnt1, char *msg)
{
	MAC_INFO *info = WLA_MAC_INFO;

	if(val)
		*cnt += 1;
	else
		*cnt = 0;

	if(*cnt >= 10)
	{
		info->mon_t_recover = WLA_CURRENT_TIME;
		DBG("%s: doing mac recovery(%d) t_closest(%x) t_current(%x) t_delta(%x) t_recovery(%x)\n", msg, *cnt,
				info->mon_t_closest, info->mon_t_current, info->mon_t_delta, info->mon_t_recover);
		*cnt = 0;
		*cnt1 += 1;
		/* Reset counter */
		memset(&info->mon_4013_hit, 0, sizeof(unsigned int)*6);
		info->mac_recover = 1;
		return 1;
	}
	else
		return 0;
}

void wla_rx_monitor(unsigned int *cnt, char *msg)
{
	MAC_INFO *info = WLA_MAC_INFO;
	info->mon_t_recover = WLA_CURRENT_TIME;
	DBG("%s: doing mac recovery t_closest(%x) t_current(%x) t_delta(%x) t_recovery(%x)\n", msg,
			info->mon_t_closest, info->mon_t_current, info->mon_t_delta, info->mon_t_recover);
	*cnt += 1;
	/* Reset counter */
	memset(&info->mon_4013_hit, 0, sizeof(unsigned int)*6);
	info->mac_recover = 1;
}

int wla_tx_monitor(void)
{
	MAC_INFO *info = WLA_MAC_INFO;
	if(info->mac_recover_mode)
	{
		unsigned short val = MACREG_READ32(TX_DEBUG_SIGNAL) & 0xffff;
		if(wla_monitor_check((0x4013 == val),
					&info->mon_4013_hit,
					&info->mon_4013_cnt,
					"mac_stacap_cache_sync_4013()"))
		{
			return 1;
		}
		if(wla_monitor_check((0x8094 == val),
					&info->mon_8094_hit,
					&info->mon_8094_cnt,
					"mac_stacap_cache_sync_8094()"))
		{
			return 1;
		}
		if(wla_monitor_check((0x8000 == val),
					&info->mon_8000_hit,
					&info->mon_8000_cnt,
					"mac_stacap_cache_sync_8000()"))
		{
			return 1;
		}
	}
	return 0;
}
#endif

void wla_set_recovery(u32 recovery)
{
	MAC_INFO *info = WLA_MAC_INFO;
	info->mac_recover_mode = recovery;
}

#ifdef WLA_MULTICAST_PATCH
void wla_add_multicast_addr_tbl(int bss_idx)
{
	MAC_INFO *info = WLA_MAC_INFO;
	int i;
	sta_cap_tbl* captbl;

	for(i=0; i<sizeof(multicast_addr_list)/6; i++)
	{
		if(mac_addr_lookup_engine_find((multicast_addr_list[i]), 0, 0, 0) < 0)
		{
			mac_addr_lookup_engine_update(info, (int)(multicast_addr_list[i]), 0xff | (bss_idx << 8)| 0x40000000, 0);
			captbl = &info->sta_cap_tbls[WLA_MULTICAST_IDX];
			captbl->valid=1;
		}
	}
}
#endif // WLA_MULTICAST_PATCH
