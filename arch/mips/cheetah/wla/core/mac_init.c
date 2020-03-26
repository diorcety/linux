/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_init.c
*   \brief  wla wlan mac init functions.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
#include <wla_util.h>
#include <wla_bb.h>
#include <wla_debug.h>
#include <wla_mat.h>

extern struct MAC_INFO *mac_info;

#define MAX_BUFFER_SIZE_ON_HT_MODE     (1024 * 1024)       /* limit the memory usage , as HT mode comsumes more memory */

static int program_mac_registers(MAC_INFO* info);

#define CLA					aligned(32)

#if !defined(CONFIG_SRAM_ENABLE) && !defined(CONFIG_SRAM_SIZE)
#define CONFIG_SRAM_SIZE 0
#endif

#if (CONFIG_SRAM_SIZE > 0)
#define IN_SRAM				section(".sram")
#else
#define IN_SRAM	
#endif

char ext_sta_tbls[sizeof(sta_addr_entry)*MAX_ADDR_TABLE_ENTRIES] __attribute__((CLA, IN_SRAM));
char ext_ds_tbls[sizeof(sta_addr_entry)*MAX_DS_TABLE_ENTRIES] __attribute__((CLA, IN_SRAM));

sta_tx_status_tbl fix_tx_tbls __attribute__((CLA, IN_SRAM));
sta_rx_status_tbl fix_rx_tbls __attribute__((CLA, IN_SRAM));

#if MMAC_RX_DESCRIPTOR_COUNT > 0
char mmac_rx_descriptors[sizeof(mmac_rx_descriptor)*MMAC_RX_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
#endif
#if defined(WLA_PSBA_QUEUE)
char psbaq_descriptors[sizeof(psbaq_descriptor)*PSBAQ_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
#endif
char buf_headers[sizeof(buf_header)*FASTPATH_BUFFER_HEADER_POOL_SIZE+sizeof(buf_header)*SW_BUFFER_HEADER_POOL_SIZE] __attribute__((CLA, IN_SRAM));
char hw_tx_buffer[MAC_TX_TEMP_DRAM_BUFFER_SIZE] __attribute__((CLA, IN_SRAM));
char reorder_buf_mapping_tbl[sizeof(int)*MAX_STA_CAP_TBL_COUNT*MAX_QOS_TIDS] __attribute__((CLA, IN_SRAM));
#ifdef CONFIG_SRAM_WLA
ampdu_reorder_buf reorder_buf[MAX_STA_CAP_TBL_COUNT*MAX_QOS_TIDS] __attribute__((CLA, IN_SRAM));
char bufs[DEF_SW_PATH_BUF_SIZE*RX_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));

#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
char fastpath_buf[DEF_FASTPATH_BUF_SIZE*FASTPATH_BUFFER_HEADER_POOL_SIZE] __attribute__((CLA, IN_SRAM));
#endif
#endif

char sta_cap_tbls_mem[sizeof(sta_cap_tbl)*MAX_STA_CAP_TBL_COUNT+sizeof(sta_cap_tbl)*MAX_RATE_TBL_COUNT+DCACHE_LINE_SIZE] __attribute__((CLA, IN_SRAM));

char wl_rxq[sizeof(rx_descriptor)*RX_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
char wl_txq[sizeof(tx_descriptor)*TX_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
char wl_sw_retq[sizeof(tx_descriptor)*SW_RET_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
char fp_retq[sizeof(tx_descriptor)*HW_RET_DESCRIPTOR_COUNT] __attribute__((CLA, IN_SRAM));
#if (FASTPATH_DESCR_COUNT > 0)
char fastpath_descriptors[sizeof(rx_descriptor)*FASTPATH_DESCR_COUNT] __attribute__((CLA, IN_SRAM));
#endif

char sta_qinfo_tbls[sizeof(sta_qinfo_tbl)*MAX_STA_CAP_TBL_COUNT] __attribute__((CLA, IN_SRAM));
char bcast_qinfos[sizeof(bcast_qinfo)*MAC_MAX_BSSIDS] __attribute__((CLA, IN_SRAM));

char group_keys[sizeof(group_cipher_key)*MAC_MAX_BSSIDS] __attribute__((CLA, IN_SRAM));
char private_keys[sizeof(cipher_key)*MAX_STA_CAP_TBL_COUNT] __attribute__((CLA, IN_SRAM));

/*=============================================================================+
| Functions
+=============================================================================*/
static int program_mac_registers(MAC_INFO *info)
{
#ifdef WLA_MULTICAST_PATCH
	sta_cap_tbl* captbl;
#endif	
	info->umac_revsion = MACREG_READ32(REVISION_REG);

#if 0 /* debug port */
	REG_WRITE32(0xaf005090, 0x90000001);	
	REG_WRITE32(0xaf009200, 0x333311);	
	REG_WRITE32(0xaf003b44, 0x21);	
#endif
	MACREG_UPDATE32(RESERVED_CMD, DOUBLE_FREE_PATCH, DOUBLE_FREE_PATCH); 
    MACREG_WRITE32(TXQ_CACHE_BADDR, PHYSICAL_ADDR(info->hw_tx_buffer));

#if defined(CONFIG_CHEETAH)
	/* setup external sta/ds tables */
	MACREG_WRITE32(EXT_STA_TABLE_ADDR, PHYSICAL_ADDR(info->ext_sta_tbls));
	MACREG_WRITE32(EXT_DS_TABLE_ADDR, PHYSICAL_ADDR(info->ext_ds_tbls));

	MACREG_WRITE32(STA_DS_TABLE_CFG, (MAX_STA_DS_CACHE << 24) |STA_DS_TABLE_CFG_DONE | ((MAX_ADDR_TABLE_ENTRIES - 1) << 16));
	MACREG_WRITE32(EXT_DS_TABLE_MAX, MAX_DS_TABLE_ENTRIES);
#endif

    MACREG_WRITE32(STACAP_BADDR, PHYSICAL_ADDR(info->sta_cap_tbls));
#if (MAX_STA_CAP_TBL_COUNT > 64)
    MACREG_WRITE32(STACAP_BADDR2, PHYSICAL_ADDR(&info->sta_cap_tbls[MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#endif
#if (MAX_STA_CAP_TBL_COUNT > 128)
    MACREG_WRITE32(STACAP_BADDR3, PHYSICAL_ADDR(&info->sta_cap_tbls[2*MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#endif
#if (MAX_STA_CAP_TBL_COUNT > 192)
    MACREG_WRITE32(STACAP_BADDR4, PHYSICAL_ADDR(&info->sta_cap_tbls[3*MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#else 
    MACREG_WRITE32(STACAP_BADDR4, PHYSICAL_ADDR(info->sta_cap_tbls));
#endif
    MACREG_WRITE32(SEC_GPKEY_BA, PHYSICAL_ADDR(info->group_keys));
	//MACREG_WRITE32(SEC_STA_GPKEY_BA, PHYSICAL_ADDR(info->sta_group_keys));

    MACREG_WRITE32(SEC_PVKEY_BA, PHYSICAL_ADDR(info->private_keys));
#if (MAX_STA_CAP_TBL_COUNT > 64)
    MACREG_WRITE32(SEC_PVKEY_BA2, PHYSICAL_ADDR(&info->private_keys[MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#endif
#if (MAX_STA_CAP_TBL_COUNT > 128)
    MACREG_WRITE32(SEC_PVKEY_BA3, PHYSICAL_ADDR(&info->private_keys[2*MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#endif
#if (MAX_STA_CAP_TBL_COUNT > 192)
    MACREG_WRITE32(SEC_PVKEY_BA4, PHYSICAL_ADDR(&info->private_keys[3*MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR]));
#endif
    MACREG_WRITE32(STACAP_NP_BADDR, PHYSICAL_ADDR(info->rate_tbls));

    MACREG_WRITE32(CAPT_SIZE, CAPT_SIZE_64_BYTES);

	MACREG_WRITE32(INR_TRG_TICK_NUM, 0x0);

    MACREG_WRITE32(TXDSC_BADDR, PHYSICAL_ADDR(info->wl_txq.desc_hdr));
    MACREG_WRITE32(RXDSC_BADDR, PHYSICAL_ADDR(info->wl_rxq.desc_hdr));

    MACREG_WRITE32(FRQ_BADDR, PHYSICAL_ADDR(info->fp_retq.desc_hdr));
    MACREG_WRITE32(SRQ_BADDR, PHYSICAL_ADDR(info->wl_sw_retq.desc_hdr));

    MACREG_WRITE32(SWBL_BADDR, PHYSICAL_ADDR(info->buf_headers));
    MACREG_WRITE32(BEACON_SWBL_BADDR, PHYSICAL_ADDR(info->buf_headers));
#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
    MACREG_WRITE32(HWBL_BADDR, PHYSICAL_ADDR(info->buf_headers));

    /* the link list head & tail index are simply ( 0  , total element of macframe headers - 1 ) */
    MACREG_WRITE32(HWB_HT, (((u32) 0x0000 << 16)|((u16)info->hw_buf_headers_count - 1)));
#endif
#if (FASTPATH_DESCR_COUNT > 0)
	MACREG_WRITE32(FASTPATH_DESCR_BADDR, PHYSICAL_ADDR(info->fastpath_descriptors));
#endif
    /* 
        the link list head & tail index for SW path are ( 0  , total element for RX macframe headers - 1 ) 
     */
    MACREG_WRITE32(SWB_HT, ((info->sw_rx_bhdr_start << 16)|((u16)info->sw_tx_bhdr_start-1)));

#if defined(WLA_MMAC_RXDESCR)
	MACREG_WRITE32(MMAC_RXDESCR_BADDR, PHYSICAL_ADDR(info->mmac_rx_descriptors));
	MACREG_WRITE32(MMAC_RXDESCR_BADDR_VALID, 0x1);
    MACREG_WRITE32(MMAC_RXDESCR_HEAD_PTR, 1); /* XXX: skip zero for debug */
    MACREG_WRITE32(MMAC_RXDESCR_TAIL_PTR, info->mmac_rx_descr_count - 1);
    MACREG_WRITE32(MMAC_RXDESCR_COUNT, info->mmac_rx_descr_count -1 ); /* XXX: skip zero for debug */
    MACREG_WRITE32(MMAC_RXDESCR_LINK, 0x1);
#endif
    /*
        Set the buffer sizes of hwpool & swpool
     */
    MACREG_WRITE32(RX_BLOCK_SIZE, (info->sw_path_bufsize << 16 ) | info->fastpath_bufsize);
    MACREG_WRITE32(BUFF_POOL_CNTL, 1);

	/* make sure LLC header alignment in RX */
	MACREG_WRITE32(MAC_LLC_OFFSET, WLAN_RX_LLC_OFFSET);

	/*
        Enable packets forwarding between different BSSIDs
     */
    MACREG_WRITE32(BSSID_CONS_CHK_EN, 0);

#if 0
    /* configure UMAC RX to always send ACK even RX fifo full */
    MACREG_UPDATE32(RX_ACK_POLICY, RX_ACK_POLICY_ALWAYS_ACK, RX_ACK_POLICY_ALWAYS_ACK);
#endif

	/* FIXED ME: disable temp queue to avoid wifi rx suspend */
#if 1
	MACREG_UPDATE32(RX_ACK_POLICY, RX_ACK_POLICY_AMPDU_BA|RX_TEMP_QUEUE_DISABLE, RX_ACK_POLICY_AMPDU_BA|RX_TEMP_QUEUE_DISABLE);
#else
	MACREG_UPDATE32(RX_ACK_POLICY, RX_ACK_POLICY_AMPDU_BA, RX_ACK_POLICY_AMPDU_BA);
#endif
	MACREG_UPDATE32(RXDMA_CTRL, RXDMA_CTRL_ACCEPT_CACHE_OUT_REQ, RXDMA_CTRL_ACCEPT_CACHE_OUT_REQ);

#if defined(WLA_AMPDU_RX_HW_REORDER)
	/* RX AMPDU */
	MACREG_WRITE32(AMPDU_RX_BITMAP_BADDR, PHYSICAL_ADDR(info->reorder_buf_mapping_tbl));
    MACREG_UPDATE32(RX_ACK_POLICY, RX_AMPDU_REORDER_ENABLE, RX_AMPDU_REORDER_ENABLE);
#endif
#if defined(WLA_PSBA_QUEUE)
	/* TX AMPDU */
#if 0	/* Disable automatic AMPDU degrade. Because we need retry count to to do rate adapation */  
	MACREG_WRITE32(PSBA_BA_CR, TX_WINMV_FORCE_EN|((TX_WINMV_FORCE_THRESHOLD & AMPDU_MAX_TRY) << 8));
#endif
	MACREG_WRITE32(PSBAQ_DESC_BADDR, PHYSICAL_ADDR(info->psbaq_descriptors));
#if defined(CONFIG_CHEETAH)
	MACREG_WRITE32(PSBAQ_DESC_CR0, ((0 << 16) | (info->psbaq_descr_count - 1)));
#else
	MACREG_WRITE32(PSBAQ_DESC_CR0, ((0 << 16) | (sizeof(psbaq_descriptor) * (info->psbaq_descr_count - 1))));
#endif
	MACREG_WRITE32(PSBAQ_DESC_CR1, (info->psbaq_descr_count << 16) | PSBAQ_DESC_CR1_DESC_CFG);

	MACREG_WRITE32(PSBAQ_QINFO_BC_BADDR, PHYSICAL_ADDR(info->bcast_qinfos));
	MACREG_WRITE32(PSBAQ_QINFO_UNICAST_BADDR, PHYSICAL_ADDR(info->sta_qinfo_tbls));
#endif
    /* turn on RSC/RXPN check, the default setting in hardware is off */
	/* turn off RSC/RXPN check to work around */
    MACREG_UPDATE32(SECENG_CTRL, SECENG_CTRL_ICV_ERR_DROP|SECENG_CTRL_REKEY_ERR_DROP|SECENG_CTRL_RSC_CHECK_OFF|SECENG_CTRL_WAPI_RSC_CHECK_OFF|SECENG_CTRL_STA_RSC_CHECK_OFF|SECENG_CTRL_AMPDU_RSC_CHECK_OFF, SECENG_CTRL_RSC_CHECK_OFF | SECENG_CTRL_WAPI_RSC_CHECK_OFF|SECENG_CTRL_STA_RSC_CHECK_OFF|SECENG_CTRL_AMPDU_RSC_CHECK_OFF|SECENG_CTRL_ICV_ERR_DROP|SECENG_CTRL_MIC_ERR_DROP|SECENG_CTRL_GKEY_RSC_ERR_DROP|SECENG_CTRL_REKEY_ERR_DROP|SECENG_CTRL_KEY_INVALID_DROP|SECENG_CTRL_KEY_NONE_DROP|SECENG_CTRL_BYPASS_DROP);

	MACREG_UPDATE32(RTSCTS_SET, 0x0, LMAC_FILTER_ALL_PASS);
   /* default to disable RTS/CTS protection */
    MACREG_UPDATE32(RTSCTS_SET, 0x0fffUL << 16, LMAC_RTS_CTS_THRESHOLD);
	wla_set_protection(NO_PROTECTION);
#if 0
	/* remaining TXOP threshold and protection mode */  
	MACREG_UPDATE32(TXOP_THRESHOLD, TXOP_TO_PKT|PROTECT_LEGACY|1500, TXOP_TO_PKT|PROTECT_LEGACY|REMAINING_TXOP_THRESHOLD);
#endif
	/* XXX: shall be coded as HW default value, check and remove later */
    MACREG_WRITE32(LSIG_RATE, 0xb);
	
	/* LMAC tuning for RF/BB timing */
	MACREG_WRITE32(OFDM_DEFER_SET, 0x9004064UL);
	//MACREG_WRITE32(PHYDLY_SET, 0x20501304UL);
	MACREG_WRITE32(PHYDLY_SET, 0x20500004UL); /* for broadcom */
	MACREG_WRITE32(RF_TIME_CTRL, 0x142a0a00UL); /* increase time of PA-on -> PA-off */ /* for Intel, increase time delay of PA on */ 

#if defined(ENABLE_NAV_HONOURING)
	/* Enable honor NAV */
	MACREG_WRITE32(LMAC_CNTL, LMAC_CNTL_NAV_CHK);
#endif
	/* sky: protect bit mismatch forward to cpu fail */
	/* FIXME: let ICV err to CPU for lynx fpga bug (group key error) */
	MACREG_WRITE32(ERR_EN, 	ERR_EN_SEC_MISMATCH_TOCPU
							| ERR_EN_MANGMENT_TA_MISS_TOCPU
							| ERR_EN_TID_ERROR_TOCPU
							| ERR_EN_ICV_ERROR_TOCPU
#if (FASTPATH_DESCR_COUNT > 0)
							| FASTPATH_WIFI_TO_WIFI);
#else
							);
#endif

#if 1
	/* flow control */
	/* If AC[i] number > (thd_ac[i]+thd_range), then enable public couonter */ 
	/* Every AC queue has 32(27+5) reserved descriptors */
	MACREG_WRITE32(FC_ACQ_THRESHOLD, ((info->acq_len[ACQ_BK_IDX] << 24) | (info->acq_len[ACQ_BE_IDX] << 16) | (info->acq_len[ACQ_VI_IDX] << 8) | info->acq_len[ACQ_VO_IDX])); 
	/* If descriptors number > (thd_ac[i]+thd_range), and public counter > (thd_public _thd_range),
		then stop ac[i] */	
	//MACREG_WRITE32(FC_PUBLIC_THRESHOLD, FC_ENABLE|(0x5 << 8)|(0x00 & FC_THD_PUBLIC)); /* Non-zero th_range may casue P3 does not drop overflow packets, JH ask to remove */
	MACREG_UPDATE32(FC_PUBLIC_THRESHOLD, FC_ENABLE, FC_ENABLE);
#endif
	/* 
        Ethernet module settings
	*/
    /* Enable Wifi-Eth HW forwarding when bridge is enable by upper-layer */
#if defined(CONFIG_WLAN_MISMATCH_DATA_TO_LOCAL) || defined(CONFIG_WLAN_MISMATCH_DATA_TO_CPU)
	MACREG_WRITE32(DA_MISS_TOCPU, 1);
#else
    MACREG_WRITE32(DA_MISS_TOCPU, 0);       // Wifi forwards the frame to Ethernet if DA is missed
#endif
	/* set to 4K for A-MSDU reception */
    MACREG_WRITE32(RX_MPDU_MAXSIZE, 4096);

#ifdef WLA_MULTICAST_PATCH
	captbl = &info->sta_cap_tbls[WLA_MULTICAST_IDX];
	captbl->TX_tbl_addr = (u32) PHYSICAL_ADDR(&fix_tx_tbls);
	captbl->RX_tbl_addr = (u32) PHYSICAL_ADDR(&fix_rx_tbls);
#endif // WLA_MULTICAST_PATCH


    return 0;
}

void rxq_init(struct rx_q *q)
{
	int i;
	/* assign ownership of all RX descriptors to hardware */
	for (i=0;i<q->max;i++) 
	{
        q->desc_hdr[i].own = 1;
        q->desc_hdr[i].eor = 0;
		q->desc_hdr[i].bhdr_tail_index = 0;
		q->desc_hdr[i].bhdr_head_index = 0;
    }
    q->desc_hdr[i-1].eor = 1;      /* indicate the HW it is the last descriptors */
	q->index = 0;
	q->pkt_count = 0;
}

void txq_init(struct tx_q *q)
{
	int i;
	/* assign ownership of all TX descriptors to SW */
	for (i=0;i<q->max;i++) 
	{
        q->desc_hdr[i].own = 0;
        q->desc_hdr[i].eor = 0;
		q->desc_hdr[i].bhdr_tail_index = 0;
		q->desc_hdr[i].bhdr_head_index = 0;
    }
    q->desc_hdr[i-1].eor = 1;      /* indicate the HW it is the last descriptors */
	q->windex = 0;
	q->rindex = 0;
	q->free = q->max;
	q->pkt_count = 0;
}

void retq_init(struct ret_q *q)
{
	int i;
	/* assign ownership of all RX descriptors to hardware */
	for (i=0;i<q->max;i++) 
	{
        q->desc_hdr[i].own = 1;
        q->desc_hdr[i].eor = 0;
		q->desc_hdr[i].bhdr_tail_index = 0;
		q->desc_hdr[i].bhdr_head_index = 0;
    }
    q->desc_hdr[i-1].eor = 1;      /* indicate the HW it is the last descriptors */
	q->index = 0;
	q->pkt_count = 0;
}

char wla_mac_init(MAC_INFO *info)
{
	int i;
	buf_header *bhdr, *prev;
	char *wb;

	/* configure parameters of MAC sta/descriptor/buffer */
	info->sta_cap_tbl_count = MAX_STA_CAP_TBL_COUNT;
    info->rate_tbl_count = MAX_RATE_TBL_COUNT;
    info->sw_buf_headers_count = SW_BUFFER_HEADER_POOL_SIZE;
    info->hw_buf_headers_count = FASTPATH_BUFFER_HEADER_POOL_SIZE;

	info->wl_txq.max = TX_DESCRIPTOR_COUNT;
    info->wl_rxq.max = RX_DESCRIPTOR_COUNT;
#if defined(WLA_WIFI2ETH)
	info->eth_txq.max = TX_DESCRIPTOR_COUNT;
    info->eth_rxq.max = RX_DESCRIPTOR_COUNT;
#endif
	info->fp_retq.max = HW_RET_DESCRIPTOR_COUNT;
	info->wl_sw_retq.max = SW_RET_DESCRIPTOR_COUNT;

#if defined(WLA_MMAC_RXDESCR)
	info->mmac_rx_descr_count = MMAC_RX_DESCRIPTOR_COUNT;
#endif
#if defined(WLA_PSBA_QUEUE)
	info->psbaq_descr_count = PSBAQ_DESCRIPTOR_COUNT;
#endif

	info->ext_sta_tbls = (sta_addr_entry *)NONCACHED_ADDR(ext_sta_tbls);
	info->ext_ds_tbls = (sta_addr_entry *)NONCACHED_ADDR(ext_ds_tbls);
	memset(info->ext_sta_tbls, 0, sizeof(ext_sta_tbls));
	memset(info->ext_ds_tbls, 0, sizeof(ext_ds_tbls));

	/* the sw_allocated_buf_headers_count should be less then sw_buf_headers_count 
       it is the value to chain linklist in SW buffer header pool for RX path
     */
	info->sw_allocated_buf_headers_count =  SW_ALLOCATED_BUF_COUNT;
	info->sw_rx_bhdr_start = info->hw_buf_headers_count;
	info->sw_tx_bhdr_start = info->hw_buf_headers_count + info->sw_allocated_buf_headers_count;
	info->sw_tx_bhdr_end = info->hw_buf_headers_count + info->sw_buf_headers_count;

    info->fastpath_bufsize = DEF_FASTPATH_BUF_SIZE;
    info->sw_path_bufsize = DEF_SW_PATH_BUF_SIZE;

	/* allocate sta capability table */
	info->sta_cap_tbls_mem = (char *)NONCACHED_ADDR(sta_cap_tbls_mem);
	memset(info->sta_cap_tbls_mem, 0, sizeof(sta_cap_tbls_mem));

	/* To ensure statistic word of sta cap in one cache line */
	info->sta_cap_tbls = (sta_cap_tbl *)(((u32)(info->sta_cap_tbls_mem + STA_CAP_STATISTIC_ALIGN_SIZE + DCACHE_LINE_SIZE - 1) & ~(DCACHE_LINE_SIZE-1)) - STA_CAP_STATISTIC_ALIGN_SIZE);
	/* allocate basic rate table */
	info->rate_tbls = (sta_cap_tbl *)((int)(info->sta_cap_tbls)+sizeof(sta_cap_tbl)*MAX_STA_CAP_TBL_COUNT);

	info->wcb_tbls = (struct wsta_cb **)WLA_MALLOC(sizeof(int) * info->sta_cap_tbl_count, DMA_MALLOC_CACHED|DMA_MALLOC_BZERO);
	if(info->wcb_tbls == 0)
		goto error;
	/* allocate buffer header of sw/hw */
	info->buf_headers = (buf_header *)NONCACHED_ADDR(buf_headers);
#if defined(SW_BUF_TRACE)
	/* trace sw buffer usage */
	info->sw_buf_trace = (char *)WLA_MALLOC(info->sw_buf_headers_count, DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO);
#endif
#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
	/* XXX: assume info->fastpath_bufsize to be multiples of 32bytes */
#ifdef CONFIG_SRAM_WLA
	info->fastpath_buf = (char *)NONCACHED_ADDR(fastpath_buf);
	memset(info->fastpath_buf, 0, sizeof(fastpath_buf));
#else
    info->fastpath_buf = (u8 *)WLA_MALLOC(info->fastpath_bufsize * info->hw_buf_headers_count, DMA_MALLOC_UNCACHED|DMA_MALLOC_ALIGN);
#endif
#endif
	/* Total size = RXs + TXs + sw_ret_TXs + fast_ret_TXs + fast_TXs */ 
	/* init rx descriptors pool */
    info->wl_rxq.desc_hdr = (rx_descriptor *)NONCACHED_ADDR(wl_rxq);
	info->wl_txq.desc_hdr = (tx_descriptor *)NONCACHED_ADDR(wl_txq);
	info->wl_sw_retq.desc_hdr = (tx_descriptor *)NONCACHED_ADDR(wl_sw_retq);
	info->fp_retq.desc_hdr = (tx_descriptor *)NONCACHED_ADDR(fp_retq);
#if (FASTPATH_DESCR_COUNT > 0)
	info->fastpath_descriptors = (rx_descriptor *)NONCACHED_ADDR(fastpath_descriptors);
#endif

#if defined(WLA_MMAC_RXDESCR)
	if(info->mmac_rx_descr_count)
	{
		info->mmac_rx_descriptors = (mmac_rx_descriptor *)NONCACHED_ADDR(mmac_rx_descriptors);
		memset(info->mmac_rx_descriptors, 0, sizeof(mmac_rx_descriptors)); 
	}
#endif
#if defined(WLA_AMPDU_RX_HW_REORDER)
	/* pre-allocated reorder buffer address table */	
	info->reorder_buf_mapping_tbl = (int *)NONCACHED_ADDR(reorder_buf_mapping_tbl);
	memset(info->reorder_buf_mapping_tbl, 0, sizeof(reorder_buf_mapping_tbl));
#ifdef CONFIG_SRAM_WLA
	info->reorder_buf = (ampdu_reorder_buf *)NONCACHED_ADDR(reorder_buf);
	memset(info->reorder_buf, 0, sizeof(reorder_buf));
#endif
#endif
#if defined(WLA_PSBA_QUEUE)
	if(info->psbaq_descr_count)
	{
		info->psbaq_descriptors = (psbaq_descriptor *)NONCACHED_ADDR(psbaq_descriptors);
		memset(info->psbaq_descriptors, 0, sizeof(psbaq_descriptors));	
	}

	/* pre-allocated tx ps/ba qinfo address table */
	info->sta_qinfo_tbls = (sta_qinfo_tbl *)NONCACHED_ADDR(sta_qinfo_tbls);
	memset(info->sta_qinfo_tbls, 0, sizeof(sta_qinfo_tbls));

	/* pre-allocated tx broadcast/multicast qinfo address table */
	info->bcast_qinfos = (bcast_qinfo *)NONCACHED_ADDR(bcast_qinfos);
	memset(info->bcast_qinfos, 0, sizeof(bcast_qinfos));

#endif 
	/* init all security related table */
	info->group_keys = (group_cipher_key *)NONCACHED_ADDR(group_keys);
    info->private_keys = (cipher_key *)NONCACHED_ADDR(private_keys);

	info->hw_tx_buffer = (u8 *)NONCACHED_ADDR(hw_tx_buffer); /* ??? */
	memset(info->hw_tx_buffer, 0, sizeof(hw_tx_buffer));

#if 1
	info->acq_len[ACQ_BK_IDX] = info->acq_len[ACQ_VI_IDX] = info->acq_len[ACQ_VO_IDX] = ACQ_MIN_SIZE;
	info->acq_len[ACQ_BE_IDX] = ACQ_TOTAL_SIZE - ACQ_MIN_SIZE*3;
#else
	info->acq_len[ACQ_BK_IDX] = info->acq_len[ACQ_BE_IDX] = info->acq_len[ACQ_VI_IDX] = info->acq_len[ACQ_VO_IDX] = ACQ_TOTAL_SIZE/4;
#endif

#if (FASTPATH_DESCR_COUNT > 0)
	for (i=0;i<FASTPATH_DESCR_COUNT;i++) 
	{
		info->fastpath_descriptors[i].own = 1;
	}
	info->fastpath_descriptors[i - 1].eor = 1; 
#endif

	/* Buffer headers allocation:
		|---------------------------------------|
		|                                       |
		|      Hardware path used               |
		|      (freelist maintenance by UMAC)   |
		|                                       |
		|---------------------------------------|
		|      software RX path used            |
		|      (freelist maintenance by UMAC)   |
		|---------------------------------------|
		|      software TX path used            |
		|   (freelist maintenance by software)  |
		|---------------------------------------|
	*/
	
	memset(&info->buf_headers[0], 0, sizeof(buf_headers));

#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
	/* 
        chain buffer headers from element[0] to element[info->hw_buf_headers_count-1]
        this linklist is used for HW as buffer to store & forward frame in fastpath         
     */
    for (i=0;i<info->hw_buf_headers_count;i++) 
	{
        info->buf_headers[i].next_index = i + 1;
        info->buf_headers[i].offset = 0;
        info->buf_headers[i].ep = 0;

        info->buf_headers[i].dptr = (unsigned int) &info->fastpath_buf[i * info->fastpath_bufsize];
    }
    info->buf_headers[i-1].next_index = 0;
    info->buf_headers[i-1].ep = 1;
#endif

    /* 
        chain buffer headers from element[sw_rx_bhdr_start] to element[sw_tx_bhdr_start-1]
        this linklist is used for HW to store receive frame (RX to software path)           
     */
#ifdef CONFIG_SRAM_WLA
	info->bufs = (char *)NONCACHED_ADDR(bufs);
	memset(info->bufs, 0, sizeof(bufs));
	wb = (char *)info->bufs;
#endif
	bhdr = prev = 0; /* avoid compiler warning */
    for(i=info->sw_rx_bhdr_start;i<info->sw_tx_bhdr_start;i++) 
	{
		bhdr = idx_to_bhdr(info, i); 
#ifndef CONFIG_SRAM_WLA
		if((wb = (char *)WBUF_ALLOC(0)) == 0)
		{
			WLA_DBG(WLAERROR, "NO enough cluster for wla\n");
			return -1;
		}
#endif
#if defined(WBUF_IS_CACHED)
		DCACHE_INVALIDATE((unsigned int)wb, DEF_SW_PATH_BUF_SIZE);
#endif
        bhdr->dptr = (u32)wb_to_dptr((struct wbuf *)wb);
#ifdef CONFIG_SRAM_WLA
		wb += DEF_SW_PATH_BUF_SIZE;
#endif

		if(i != info->sw_rx_bhdr_start)
			prev->next_index = bhdr_to_idx(info, bhdr);
		prev = bhdr;
    }
    bhdr->ep = 1;

#if defined(SW_BUF_TRACE)
	memset(&info->sw_buf_trace[0], 1, info->sw_buf_headers_count);
#endif
	/* 
        chain buffer headers from element[sw_tx_bhdr_start] to  element[info->sw_tx_bhdr_end]
        this linklist is used as freelist for SW to allocate in TX path
     */
	info->sw_tx_bhdr_head = info->sw_tx_bhdr_tail = -1;
	for (i=info->sw_tx_bhdr_start; i<info->sw_tx_bhdr_end;i++) 
	{
		bhdr_insert_tail(info, i, i);
    }

	rxq_init(&info->wl_rxq);
	txq_init(&info->wl_txq);

	retq_init(&info->fp_retq);
	retq_init(&info->wl_sw_retq);
#if defined(WLA_WIFI2ETH)
	rxq_init(&info->eth_rxq);
	txq_init(&info->eth_txq);
#endif
#if defined(WLA_MMAC_RXDESCR)
	if(info->mmac_rx_descriptors)
	{
		/* init mmac-internal rx descriptors pool */
    	for(i=0; i<info->mmac_rx_descr_count; i++) 
    	{
        	info->mmac_rx_descriptors[i].next_index = i+1;
    	}
    	info->mmac_rx_descriptors[i-1].next_index = 0;
		/* XXX: fill 0xff in 0th descriptor for debug */
		memset(&info->mmac_rx_descriptors[0], 0xff, sizeof(mmac_rx_descriptor));
		info->mmac_rx_descriptors[0].next_index = 0;
	}
#endif
#if defined(WLA_PSBA_QUEUE)
	if(info->psbaq_descriptors)
	{
		/* init tx ps/ba descriptors pool */
    	for(i=0; i<info->psbaq_descr_count; i++) 
    	{
			info->psbaq_descriptors[i].next_descr = i+1;
    	}
    	info->psbaq_descriptors[i-1].next_descr = 0;
	}

	if(info->bcast_qinfos)
	{
		for(i=0; i<MAC_MAX_BSSIDS; i++)
		{
			info->bcast_qinfos[i].tx_queue_len = PSBAQ_BCAST_TX_QUEUE_LEN;
			info->bcast_qinfos[i].tx_head_ptr = 0xffff;
			info->bcast_qinfos[i].tx_tail_ptr = 0xffff; 
		}
	}
#endif

	info->sta_cap_tbl_free_idx = 0;

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
	mat_init();
	info->mat_timestamp = WLA_CURRENT_TIME + MAT_CHECK_TIME;
#endif
	program_mac_registers(info);

	return 0;
error:
	wla_mac_release(info);
	return -1;
}

#if defined(CONFIG_WMAC_RECOVER)
void wla_pre_clean(MAC_INFO *info)
{
	int i, j;
	struct wsta_cb *wcb;
	sta_addr_entry *sta;
	sta_cap_tbl *captbl;

#if defined(WLA_AMPDU_RX_SW_REORDER)
	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		wcb = info->wcb_tbls[i];
		if(!wcb)
			continue;
		for(j=0; j<8; j++)
		{
			if(wcb->ba_rx[j] == 0)
				continue;
			WLA_MFREE(wcb->ba_rx[j]);
			wcb->ba_rx[j] = 0;
		}
	}
#endif
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
	/* pre-allocated reorder buffer address table */
	info->reorder_buf_mapping_tbl = (int *)NONCACHED_ADDR(reorder_buf_mapping_tbl);
	memset(info->reorder_buf_mapping_tbl, 0, sizeof(reorder_buf_mapping_tbl));
#endif
#if defined(WLA_PSBA_QUEUE)
	{
		ucast_qinfo *qinfos;
		int tbl_num;
	
		for(i=0; i<info->sta_cap_tbl_count; i++)
		{
			if(!info->sta_qinfo_tbls[i].qinfo_addr)
				continue;
			qinfos = (ucast_qinfo *)VIRTUAL_ADDR(info->sta_qinfo_tbls[i].qinfo_addr);
			if(info->sta_qinfo_tbls[i].qos)
				tbl_num = QOS_TX_STATUS_TBLS;
			else
				tbl_num = 1;
			for(j=0; j<tbl_num; j++)
			{
				memset(&qinfos[j], 0, sizeof(ucast_qinfo));
				qinfos[j].tx_head_ptr = 0xffff;
				qinfos[j].tx_tail_ptr = 0xffff;
				qinfos[j].tx_queue_len = MAX_PS_QUEUE_LENGTH;
			}
		}
	}
#endif
	{
		sta_rx_status_tbl *rx_tbls;
		for(i=0; i<info->sta_cap_tbl_count; i++)
		{
			struct wsta_cb *wcb;
			
			captbl = &info->sta_cap_tbls[i];
			if(!captbl->valid)
				continue;

			wcb = info->wcb_tbls[i];
			if(!wcb)
				continue;
		
			rx_tbls = (sta_rx_status_tbl *)NONCACHED_ADDR(captbl->RX_tbl_addr);
			if(rx_tbls)
			{
				int tbl_num;
		    	u32 ps_trig_en;
		
				if(captbl->NE)
					tbl_num = QOS_RX_STATUS_TBLS;
				else
					tbl_num = 1;
		 
				for(j=0; j< tbl_num; j++)
				{
					ps_trig_en = rx_tbls[j].ps_trig_en;
					memset(&rx_tbls[j], 0, sizeof(sta_rx_status_tbl));
					rx_tbls[j].ps_trig_en = ps_trig_en;
				}
			}
		}
	}
	/* Prohibit rx AMPDU */
	for(i=0; i<MAX_ADDR_TABLE_ENTRIES; i++)
	{
		sta = IDX_TO_ADDR_TBL(i);
		if(sta->basic_cap.bit.rx_ampdu == 0)
			continue;
		sta->basic_cap.bit.rx_ampdu = 0;
	}
	/* Prohibit tx AMPDU */
	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		captbl = &info->sta_cap_tbls[i];
		if(!captbl->valid)
			continue;
		wcb = info->wcb_tbls[i];
		if (!wcb)
			continue;
		wcb->tx_ampdu_bitmap = 0;
	}
}

char wla_mac_recover(MAC_INFO *info)
{
	int i;
	buf_header *bhdr, *prev;
	char *wb;

	if(info->hw_tx_buffer)
	{
		memset(info->hw_tx_buffer, 0, sizeof(hw_tx_buffer));
	}
#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
	if(info->fastpath_buf)
	{
		memset(info->fastpath_buf, 0, sizeof(info->fastpath_bufsize * info->hw_buf_headers_count));
	}
#endif
#if (FASTPATH_DESCR_COUNT > 0)
	for (i=0;i<FASTPATH_DESCR_COUNT;i++) 
	{
		info->fastpath_descriptors[i].own = 1;
	}
	info->fastpath_descriptors[i - 1].eor = 1; 
#endif

	if(info->buf_headers)
	{
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
				wbuf_free(dptr_to_wb((char *)(u32)bhdr->dptr));
				bhdr->dptr = 0;
			}
		}

		memset(&info->buf_headers[0], 0, sizeof(buf_headers));
#if (FASTPATH_BUFFER_HEADER_POOL_SIZE > 0)
		/* 
	        chain buffer headers from element[0] to element[info->hw_buf_headers_count-1]
	        this linklist is used for HW as buffer to store & forward frame in fastpath         
	     */
	    for (i=0;i<info->hw_buf_headers_count;i++) 
		{
	        info->buf_headers[i].next_index = i + 1;
	        info->buf_headers[i].offset = 0;
	        info->buf_headers[i].ep = 0;
	
	        info->buf_headers[i].dptr = (unsigned int) &info->fastpath_buf[i * info->fastpath_bufsize];
	    }
	    info->buf_headers[i-1].next_index = 0;
	    info->buf_headers[i-1].ep = 1;
#endif
	    /* 
	        chain buffer headers from element[sw_rx_bhdr_start] to element[sw_tx_bhdr_start-1]
	        this linklist is used for HW to store receive frame (RX to software path)           
	     */
#ifdef CONFIG_SRAM_WLA
		memset(info->bufs, 0, sizeof(bufs));
		wb = (char *)info->bufs;
#endif
		bhdr = prev = 0; /* avoid compiler warning */
	    for(i=info->sw_rx_bhdr_start;i<info->sw_tx_bhdr_start;i++) 
		{
			bhdr = idx_to_bhdr(info, i); 
#ifndef CONFIG_SRAM_WLA
			if((wb = (char *)WBUF_ALLOC(0)) == 0)
			{
				WLA_DBG(WLAERROR, "NO enough cluster for wla\n");
				return -1;
			}
#endif
#if defined(WBUF_IS_CACHED)
			DCACHE_INVALIDATE((unsigned int)wb, DEF_SW_PATH_BUF_SIZE);
#endif
			bhdr->dptr = (u32)wb_to_dptr((struct wbuf *)wb);
#ifdef CONFIG_SRAM_WLA
			wb += DEF_SW_PATH_BUF_SIZE;
#endif
	
			if(i != info->sw_rx_bhdr_start)
				prev->next_index = bhdr_to_idx(info, bhdr);
			prev = bhdr;
	    }
	    bhdr->ep = 1;
	
#if defined(SW_BUF_TRACE)
		memset(&info->sw_buf_trace[0], 1, info->sw_buf_headers_count);
#endif
		/* 
	        chain buffer headers from element[sw_tx_bhdr_start] to  element[info->sw_tx_bhdr_end]
	        this linklist is used as freelist for SW to allocate in TX path
	     */
		info->sw_tx_bhdr_head = info->sw_tx_bhdr_tail = -1;
		for (i=info->sw_tx_bhdr_start; i<info->sw_tx_bhdr_end;i++) 
		{
			bhdr_insert_tail(info, i, i);
	    }
	}
#if defined(WLA_PSBA_QUEUE)
	if(info->psbaq_descriptors)
	{
		memset(info->psbaq_descriptors, 0, sizeof(psbaq_descriptors));
		/* init tx ps/ba descriptors pool */
    	for(i=0; i<info->psbaq_descr_count; i++) 
    	{
#if !defined(CONFIG_CHEETAH)
        	info->psbaq_descriptors[i].next_descr = (i+1) * sizeof(psbaq_descriptor);
#else
			info->psbaq_descriptors[i].next_descr = i+1;
#endif
    	}
    	info->psbaq_descriptors[i-1].next_descr = 0;
	}
#endif

	rxq_init(&info->wl_rxq);
	txq_init(&info->wl_txq);

	retq_init(&info->fp_retq);
	retq_init(&info->wl_sw_retq);
#if defined(WLA_MMAC_RXDESCR)
	if(info->mmac_rx_descriptors)
	{
		memset(info->mmac_rx_descriptors, 0, sizeof(mmac_rx_descriptors)); 
		/* init mmac-internal rx descriptors pool */
    	for(i=0; i<info->mmac_rx_descr_count; i++) 
    	{
        	info->mmac_rx_descriptors[i].next_index = i+1;
    	}
    	info->mmac_rx_descriptors[i-1].next_index = 0;
		/* XXX: fill 0xff in 0th descriptor for debug */
		memset(&info->mmac_rx_descriptors[0], 0xff, sizeof(mmac_rx_descriptor));
		info->mmac_rx_descriptors[0].next_index = 0;
	}
#endif

	program_mac_registers(info);

	/* Don't adjust  STA_DS_TABLE_CFG dynamically to avoid cache has incorrect value */
	MACREG_UPDATE32(STA_DS_TABLE_CFG, info->sta_tbl_count|((info->ds_tbl_count & 0x1f) << 9), STA_TABLE_MAX_SEARCH|DS_TABLE_MAX_SEACH);

	return 0;
}
#endif

