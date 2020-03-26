/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_reg.c
*   \brief  wla MAC's register.
*   \author Montage
*/

#ifndef __MAC_REGS_H__
#define __MAC_REGS_H__

#define LMAC_REG_OFFSET 0x00006000UL

#define ETH_REG_OFFSET	0x00005000UL

#define SEC_REG_OFFSET  0x00006C00UL

#define UNCACHE_BASE		(0xa0000000)
#define CACHE_BASE			(0x80000000)
#define	PHYSICAL_ADDR(va) 	(((unsigned int)va)&0x0fffffff)
#define	VIRTUAL_ADDR(pa)	(((unsigned int)pa)|UNCACHE_BASE)
#define	CACHED_ADDR(va)		((PHYSICAL_ADDR(va))|CACHE_BASE)
#define	NONCACHED_ADDR(va)	(((unsigned int)va)|UNCACHE_BASE)

/* HW characterics; DO NOT CHANGE IT! */
#define MAC_TX_STATUS_TBL_COUNT     17          /* total elements of HT mode TX status tables */
#define MAC_RX_STATUS_TBL_COUNT     17          /* total elements of HT mode RX status tables */

#define MAC_TX_TEMP_DRAM_BUFFER_SIZE            ( 64 * 8 )

#define MAC_HT_TX_RETRY_BUFFER_STRUCT_SIZE      ( 64 * 16 )
#define MAC_HT_RX_BUFFER_STURCT_SIZE            ( 8 )   /* times the MPDU_window ( e.g. 8 x 64 ) */
#define MAC_LAGECY_RX_BUFFER_STURCT_SIZE        ( 8 )   /* times the MPDU_window ( e.g. 8 x 64 ) */

#define MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR     64

#define REG_READ32(x)  (*(volatile u32*)((x)))
#define REG_WRITE32(x,val) (*(volatile u32*)((x)) = (u32)(val))

#define MACREG_ADDR(x)			((u32 *)(UMAC_REG_BASE+(x)))
#define MACREG_READ32(x)  (*(volatile u32*)(UMAC_REG_BASE+(x)))
#ifdef DEBUG_DUMP
#define MACREG_WRITE32(x,val)  \
do {        \
MACDBG("// write UMAC %08x %08x\n",         UMAC_REG_BASE+(x), (val));    \
MACDBG("*(volatile u32*)( 0x%08x ) = (u32) 0x%08x;\n", UMAC_REG_BASE+(x), (val)); \
*(volatile u32*)(UMAC_REG_BASE+(x)) = (u32)(val); \
} while (0)
#else
#define MACREG_WRITE32(x,val) (*(volatile u32*)(UMAC_REG_BASE+(x)) = (u32)(val))
#endif

#ifdef DEBUG_DUMP
#define MACREG_UPDATE32(x,val,mask) do {           \
    u32 newval;                                        \
    newval = *(volatile u32*) (UMAC_REG_BASE+(x));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));\
    MACDBG("// write MAC %08x %08x\n",         UMAC_REG_BASE+(x), (val));    \
    MACDBG("*(volatile u32*)( 0x%08x ) = (u32) 0x%08x;\n", UMAC_REG_BASE+(x), (newval)); \
    *(volatile u32*)(UMAC_REG_BASE+(x)) = newval;      \
} while(0)
#else
#define MACREG_UPDATE32(x,val,mask) do {           \
    u32 newval;                                        \
    newval = *(volatile u32*) (UMAC_REG_BASE+(x));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));\
    *(volatile u32*)(UMAC_REG_BASE+(x)) = newval;      \
} while(0)
#endif

#define STACAP_BADDR    0x800       /* based address of STA station capability table (sta_cap_tbl) ; 256 max */
#define TXDSC_BADDR     0x804       /* based address of TX descriptor arrays (tx_descr) */
#define RXDSC_BADDR     0x808       /* based address of TX descriptor arrays (rx_descr) */

#define SWBL_BADDR      0x80C       /* based address of buffer header (data_frame_header) arrays (SW path pool) */
#define HWBL_BADDR      0x810       /* based address of buffer header (data_frame_header) arrays (fastpath pool) */
#define SRQ_BADDR       0x814       /* BA of TX done return queue (SW path) */
#define FRQ_BADDR       0x818       /* BA of TX done return queue (fastpath) */
#define HWB_HT          0x81C       /* HW link list tail (data_frame_header link list for HW fastpath Tx/Rx) */
#define SWB_HT          0x820       /* SW RX link list tail (data_frame_header link list for software Rx) */
#define BEACON_BADDR    0x824       /* beacon queue base address */

#define FASTPATH_DESCR_BADDR    0x82C	/* base address of wlan rx to wlan tx queue */

#define BUFF_POOL_CNTL  0x840       /* buffer pool control */
#define HWBUFF_CURR_HT  0x844       /* HW buffer pool current head and tail */
#define SWBUFF_CURR_HT  0x848       /* SW buffer pool current head and tail */

#define CAPT_SIZE       0x84C       /* size of station capability table */
    #define CAPT_SIZE_64_BYTES  0
    #define CAPT_SIZE_128_BYTES  1

#define MAC_MEDIA_STATUS    0x850   /* media status */
#define MAC_FREE_PTR        0x854   /* free buffer head/tail index */
#define MAC_FREE_CMD        0x858   /* free buffer command */
    #define MAC_GET_CMD_BUSY	0x10
    #define MAC_GET_CMD_KICK	0x8
    #define MAC_FREE_CMD_BUSY   0x4
    #define MAC_FREE_CMD_SW_BUF 0x2 /* 1 for free to SW buffer, 0 for free to fastpath */
    #define MAC_FREE_CMD_KICK   0x1
#define MAC_GET_BUSY		MAC_GET_CMD_BUSY
#define MAC_BUFF_SW_DPTR    0x84C   /* got SW DPTR of buffer header */
#define MAC_BUFF_HW_DPTR    0x850   /* got HW DPTR of buffer header */
#define MAC_BUFF_INDEX		0x910   /* got buffer index */
    #define MAC_BUFF_INDEX_SW       	0x0000FFFFUL
    #define MAC_BUFF_INDEX_HW       	0xFFFF0000UL

#define STACAP_BADDR2     0x85C        /* 64 entry per-stacap base addr */
#define STACAP_BADDR3     0x860        /* 64 entry per-stacap base addr */
#define STACAP_BADDR4     0x864        /* 64 entry per-stacap base addr */
#define STACAP_NP_BADDR   0x868        /* first NP stacap baddr */

#define EXT_DS_TABLE_MAX	0x870

#define EXT_STA_TABLE_ADDR  0x874
#define EXT_DS_TABLE_ADDR   0x878

#define STA_DS_TABLE_CFG    0x87C
    #define STA_DS_TABLE_CFG_DONE       0x80000000UL
	#define STA_TABLE_MAX_SEARCH		0x000001FFUL
	#define DS_TABLE_MAX_SEACH			0x00003E00UL
	#define STA_TABLE_HASH_MODE			0x00008000UL
    #define STA_TABLE_HASH_MASK         0x00FF0000UL
	#define STA_DS_TABLE_CACHE_CFG		0x1F000000UL
	#define STA_TABLE_CLR				0x20000000UL
	#define DS_TABLE_CLR				0x40000000UL

#define LUBASIC_CAP     0x880
    #define LUBASIC_CAP_VALID           0x80000000UL

#define LUT_ADDR0       0x884
#define LUT_ADDR1       0x888
#define LUT_CMD         0x88C
    // ADDR Lookup Engine Control
    #define LU_TRIGGER  0x80000000
    #define LU_DONE     0x40000000

    #define LU_CMD_SEL_STA_TABLE                0x00000010

	#define LU_CMD_MASK							0x0000000F
    #define LU_CMD_SEARCH_BY_ADDR               0
    #define LU_CMD_READ_BY_INDEX                2
    #define LU_CMD_INSERT_INTO_STA_BY_ADDR      3
    #define LU_CMD_INSERT_INTO_DS_BY_ADDR       4
    #define LU_CMD_UPDATE_INTO_STA_BY_ADDR      5
    #define LU_CMD_UPDATE_INTO_DS_BY_ADDR       6
    #define LU_FLUSH_STA_TABLE                  7
    #define LU_FLUSH_DS_TABLE                   8
    #define LU_CMD_UPDATE_STA_BY_INDEX          9
    #define LU_CMD_UPDATE_DS_BY_INDEX           10

#define LUR_BASIC_CAP   0x890

#define LUR_INDX_ADDR   0x894
	#define LUR_INDX_ADDR_CACHE_NO_HIT  0x20000000UL
    #define LUR_INDX_ADDR_CMD_SUCCEED   0x10000000UL
    #define LUR_INDX_ADDR_DS_HIT        0x08000000UL
    #define LUR_INDX_ADDR_STA_HIT       0x04000000UL
	#define LUR_INDX_ADDR_DS_TBL_FULL	0x02000000UL
	#define LUR_INDX_ADDR_STA_TBL_FULL	0x01000000UL

#define LUR_ADDR        0x898

/* Interrupt register */
#define MAC_INT_MASK_REG    0x89c
#define MAC_INT_CLR         0x8a0
#define MAC_INT_STATUS      0x8a4

    #define MAC_INT_SW_TX_ACCEPT    0x00000001      /* TX descr. handled */
    #define MAC_INT_ETH_DESCR_FULL  0x00000002      /* ether2wifi descriptor full */
    #define MAC_INT_HW_RTURNQ       0x00000004      /* indicate there are some packets into HW return Q */
    #define MAC_INT_SW_RTURNQ       0x00000008      /* indicate there are some packets into SW return Q */
    #define MAC_INT_HW_RTURNQ_FULL  0x00000010
    #define MAC_INT_SW_RTURNQ_FULL  0x00000020      /* return queue full */
    #define MAC_INT_SW_RX           0x00000040      /* SW buffer gets data */
    #define MAC_INT_RX_DESCR_FULL   0x00000080      /* RX descriptor full */
    #define MAC_INT_RX_FIFO_FULL    0x00000100      /* RX STA fifo full */
    #define MAC_INT_BEACON_DONE     0x00000200      /* Beacon TX done */
    #define MAC_INT_ETH_RX          0x00000400
    #define MAC_INT_ETH_TXDONE      0x00000800
    #define MAC_INT_PRETBTT         0x00001000      /* pre-tbtt timer */
    #define MAC_INT_SW_BUF_FULL     0x00004000      /* software buffer full */
    #define MAC_INT_HW_BUF_FULL     0x00008000      /* fastpath buffer full */

	#undef MAC_INT_PRETBTT
    #define MAC_INT_PRETBTT         MAC_INT_BEACON_DONE
    
	#define MAC_INT_FULL_MASK       0xFFFFFFFFUL
    //#define MAC_INT_DEF_MASK      (MAC_INT_HW_BUF_FULL | MAC_INT_SW_BUF_FULL | MAC_INT_ETH_DESCR_FULL | MAC_INT_HW_RTURNQ_FULL | MAC_INT_HW_RTURNQ | MAC_INT_PRETBTT | MAC_INT_SW_TX_ACCEPT)
    #define MAC_INT_DEF_MASK        (MAC_INT_HW_BUF_FULL | MAC_INT_SW_BUF_FULL | MAC_INT_ETH_DESCR_FULL | MAC_INT_HW_RTURNQ_FULL | MAC_INT_HW_RTURNQ | MAC_INT_SW_TX_ACCEPT)

	#define MAC_INT_TXRX_MASK		(MAC_INT_SW_RTURNQ|MAC_INT_SW_RTURNQ_FULL|MAC_INT_ETH_TXDONE|MAC_INT_SW_RX|MAC_INT_RX_DESCR_FULL|MAC_INT_ETH_RX)
	
#define MAC_BSSID0      0x8a8       /* BSSID bitmap & BSSID[47:32] */
    #define MAC_MBSSID_BITMAP       0x00FF0000UL
#define MAC_BSSID1      0x8ac       /* BSSID[31:0] */
	#define MSSID_MODE				(1 << 24) /* force TX only use first BSSID, 
											It is used for single mac address but multiple SSID  */ 		

#define MAC_BSSID_FUNC_MAP  0x8b0

/* check VLAN registers are available? */
#define MAC_VLAN1_0     0x8b0
#define MAC_VLAN3_2     0x8b4
#define MAC_VLAN5_4     0x8b8
#define MAC_VLAN7_6     0x8bc

#define BB_SPI_BASE     0x8c0      
#define     BB_SPI_DONE     0x80000000UL

#define BB_SPI_CLK_DIV  0x8c4
#define MAC_LLC_OFFSET  0x8cc      
 
#define BA_BITMAP_BADDR 0x8d0
    #define BM_LU_IDX_INVALID   0xFF    /* the magic value is for indicating it is a invalid index */

#define DISASSOC_CTRL   0x8d4
    #define DISASSOC_STA_CAP_INDEX              0x000000FFUL
    #define DISASSOC_CTRL_IDLE                  0x0000FF00UL
    #define DISASSOC_CTRL_TX_KICK               0x40000000UL
    #define DISASSOC_CTRL_SECURITY_AND_RX_KICK  0x80000000UL

#define STA_MODE_CTRL       0x8e8
    #define STA_MODE_ENABLE     0x01000000UL
    #define STA_MODE_BITMAP     0x00FF0000UL
    #define STA_MODE_BSSID_HI   0x0000FFFFUL    /* BSSID [47:32] */
#define STA_MODE_BSSID_LO   0x8ec               /* BSSID [31:0] */

#define PS_FUNC_CTRL    0x8f0
    #define PS_FUNC_DISABLE                     0x00000001UL
    #define PS_FUNC_POSTPONE_UAPSD_TRIGGER      0x00000002UL
    #define PS_FUNC_POSTPONE_PSPOLL             0x00000004UL

#define MAC_BSSID_LOW_1 0x8f4
#define MAC_BSSID_LOW_2 0x8f8
#define MAC_BSSID_LOW_3 0x8fc
#define MAC_BSSID_LOW_4 0x900
#define MAC_BSSID_LOW_5 0x904
#define MAC_BSSID_LOW_6 0x908
#define MAC_BSSID_LOW_7 0x90c

#if defined(CONFIG_CHEETAH)
#define REVISION_REG    0x20dc
#else
#define REVISION_REG    0x9fc
#endif

/* TX Register set */
#define UMAC_TX_CNTL    0xA00       /* upper mac transimmion control */
    #define UMAC_TX_CNTL_TXINFO_LEN     0x00000070UL
    #define UMAC_TX_CNTL_LU_CACHE_EN    0x00000002UL
    #define UMAC_TX_CNTL_TX_ENABLE      0x00000001UL
#define TXCACHE_CNTL    0xA04
    #define TXCACHE_CNTL_STACAP_CACHE_WRITEBACK     0x00000100UL
    #define TXCACHE_CNTL_STACAP_CACHE_CLEAR         0x00000200UL
#define TXFRAG_CNTL     0xA08
    #define TXFRAG_CNTL_THRESHOLD   0x0000FFFFUL
#define TXRTURNQ_CNTL   0xA0C
#define GETRECYCQ_TRIG  0xA10
#define TX_SERVICE_RATE 0xA14
#define TXQ_TRIGGER     0xA18       /* trigger transmit */
#define LSIG_RATE       0xA1C

#define TX_DEBUG_SIGNAL	0xA28
#define PSQ_TRIGGER		0xA34
	#define RA_INDEX			0x000000FFUL
	#define TRIG_TYPE			0x00000100UL
	#define TRIG_REQUEST		0x00000200UL

#define SLOTTIME_SET    0xA40       /* slot time */
#define CW_SET          0xA44
    #define CW_SET_VO_CWMAX     0xF0000000UL
    #define CW_SET_VO_CWMIN     0x0F000000UL
    #define CW_SET_VI_CWMAX     0x00F00000UL
    #define CW_SET_VI_CWMIN     0x000F0000UL
    #define CW_SET_BE_CWMAX     0x0000F000UL
    #define CW_SET_BE_CWMIN     0x00000F00UL
    #define CW_SET_BK_CWMAX     0x000000F0UL
    #define CW_SET_BK_CWMIN     0x0000000FUL

#define CW_SET_BG       0xA48

#define TXQ_CACHE_BADDR   0xA4C   /* external 64 x 8 bytes buffer to TX logic to cache data */

#define RW_TXFIFO_DATA      0xA68
#define RW_TXFIFO_CMD       0xA6C

#define GLOBAL_SN0_1        0xA70
#define GLOBAL_SN2_3        0xA74
#define GLOBAL_SN4_5        0xA78
#define GLOBAL_SN6_7        0xA7C
#define LOAD_SN_HB_INFO         0xA80
#define HEADER_BUFFER_RW_CMD    0xA84
#define HEADER_BUFFER_RW_DATA   0xA88

#define RESERVED_CMD				0xA90
	#define DOUBLE_FREE_PATCH		0x00000004UL

#define PSBA_PS_AID0_TIM_CR         0xA94
    #define PS_AID0_TIM_DQ_STATUS   0x00010000UL
    #define PS_AID0_TIM_DQ_MASK     0x0000FF00UL
    #define PS_AID0_TIM_DQ          0x000000FFUL
#define PSBA_PS_UCAST_TIM_CR        0xA98
    #define PS_UCAST_TIM_CMD       	0x00008000UL
    #define CR_DEBUG_GRP_SEL        0x00000300UL
        #define CR_DEBUG_GRP_SEL_PS_UCAST_TIM 0x0UL
    #define PS_UCAST_TIM_IDX        0x000000FFUL
#define PSBA_PS_UCAST_TIM_SR        0xA9C
#define PSBA_PS_BC_BSSID_RATE_SR0   0xAA0
#define PSBA_PS_BC_BSSID_RATE_SR1   0xAA4

#define PSBAQ_DESC_BADDR    0xAD8
#define PSBAQ_DESC_CR0      0xADC
    #define PSBAQ_DESC_CR0_DESC_HEAD    0xFFFF0000UL
    #define PSBAQ_DESC_CR0_DESC_TAIL    0x0000FFFFUL
#define PSBAQ_DESC_CR1      0xAE0
    #define PSBAQ_DESC_CR1_DESC_COUNT   0xFFFF0000UL
    #define PSBAQ_DESC_CR1_DESC_CFG     0x00000001UL
#define PSBAQ_QINFO_BC_BADDR        0xAE4
#define PSBAQ_QINFO_UNICAST_BADDR   0xAE8
#define PSBAQ_QINFO_CSR0            0xAEC
	#define PSBAQ_QINFO_CSR0_FORCE_NOAGG	0x00100000UL
	#define PSBAQ_QINFO_CSR0_MAX_AMPDU_SIZE	0x000C0000UL
	#define PSBAQ_QINFO_CSR0_WIN_START		0x00030000UL
	#define PSBAQ_QINFO_CSR0_WIN_SIZE		0x0000001FUL
	#define PSBAQ_QINFO_CSR0_PS_MAX_SP_LEN	0x00000006UL
	#define PSBAQ_QINFO_CSR0_PS_AC_ENA		0x00000001UL
#define PSBAQ_QINFO_CSR1            0xAF0
#define PSBAQ_QINFO_CSR2            0xAF4
    #define PSBAQ_QINFO_CSR2_RA_IDX         0x0000FF00UL
    #define PSBAQ_QINFO_CSR2_TID            0x000000F0UL
    #define PSBAQ_QINFO_CSR2_CMD_SETUP      0x00000000UL
    #define PSBAQ_QINFO_CSR2_CMD_TEARDOWN   0x00000004UL
    #define PSBAQ_QINFO_CSR2_CMD_PS_CONF_W  0x00000008UL
    #define PSBAQ_QINFO_CSR2_CMD_STA_INV    0x0000000CUL
	#define PSBAQ_QINFO_CSR2_CMD_STA_AGING	0x0000000EUL
    #define PSBAQ_QINFO_CSR2_CMD_TRIGGER    0x00000001UL
#define PSBA_BA_CR          0xAF8
    #define TX_WINMV_FORCE_THRESHOLD        0x0000FF00UL
    #define TX_WINMV_FORCE_EN               0x00000080UL
    #define TX_RATE_ADAPTION_WINMV_EN       0x00000001UL

#define UMAC_DEBUG_MUX				0xAFC
#define AC_QUEUE_COUNTER1   0xAB8
    #define AC_BE_QUEUE_COUNT   0xFFFF0000UL
    #define AC_BK_QUEUE_COUNT   0x0000FFFFUL
#define AC_QUEUE_COUNTER2   0xABC
    #define AC_VO_QUEUE_COUNT   0xFFFF0000UL
    #define AC_VI_QUEUE_COUNT   0x0000FFFFUL

#define STACAP_CACHE_STATUS     0xAC0
    #define STACAP_CACHE_EMPTY      0x00002000UL
    #define STACAP_CACHE_NP         0x00001000UL
    #define STACAP_CACHE_TID        0x00000F00UL
    #define STACAP_CACHE_RA_INDEX   0x000000FFUL

#define FC_PUBLIC_THRESHOLD	0xACC
	#define FC_ENABLE			0x80000000UL
	#define FC_THD_RANGE		0x0000FF00UL
	#define EDCA_TXOP_LIMIT		0x000000FFUL

#define	FC_ACQ_THRESHOLD	0xAD0
	#define FC_THD_AC_BK		0xFF000000UL
	#define FC_THD_AC_BE		0x00FF0000UL
	#define FC_THD_AC_VI		0x0000FF00UL
	#define FC_THD_AC_VO		0x000000FFUL

#define SEARCH_ENGINE_THD	0xAD4
	#define THD_SH0				0xFF000000UL
	#define THD_SH1				0x00FF0000UL
	#define THD_SH2				0x0000FF00UL
	#define THD_SH3				0x000000FFUL

/* RX Register set */
#define BA_LIFE_TIME_REG 0xB00
#define RX_BLOCK_SIZE   0xB04
#define SUB_BLK_SIZE    0xB08
#define LNK_BLK_SIZE    0xB0C       /* max dequeue MSDU size */

#define RXQ_MSDU_MSDU_SIZE  0xB14       /* software description interrupt register set */
#define UPRX_EN         0xB18       /* upper mac rx enable */

#define ERR_EN          0xB1C       /* error drop to cpu */
    #define ERR_EN_DATA_TA_MISS_TOCPU       0x00000001UL
    #define ERR_EN_MANGMENT_TA_MISS_TOCPU   0x00000002UL
    #define ERR_EN_ICV_ERROR_TOCPU          0x00000004UL
    #define ERR_EN_FCS_ERROR_TOCPU          0x00000008UL
    #define ERR_EN_BSSID_CONS_ERROR_TOCPU   0x00000010UL
    #define ERR_EN_TID_ERROR_TOCPU          0x00000020UL
	#define ERR_EN_SEC_MISMATCH_TOCPU		0x00000040UL
	#define FASTPATH_WIFI_TO_WIFI			0x00000080UL


#define RXDMA_CTRL      0xB20
    #define RXDMA_CTRL_ACCEPT_CACHE_OUT_REQ     0x00000002UL      /* accept RX link-info cache out request */

#define AUTO_LEARN_EN   0xB24
#define DA_MISS_TOCPU   0xB28       /* set to 0 for fowarding DA miss frames to ETH */
#define INR_TRG_PKT_NUM 0xB2C
#define INR_TRG_TICK_NUM 0xB30

#define CPU_RD_SWDES_IND 0xB34      /* let RX logic know there is free slot for RX descriptor to write */
#define MAX_WIFI_LEN    0xB38
#define MAX_ETHER_LEN   0xB3C
#define MAX_LEN_EN      0xB40
#define DEBUG_GROUP_SEL 0xB44
#define MSDU_F_TRIG     0xB48

#define TID_SEL         0xB64
#define BSSID_CONS_CHK_EN   0xB68   /* only forward packets with matching DA&SA BSSID */
#define MMAC_DEBUG_SIGNAL	0xB6C

#define SA_AGE_BITMAP   0xB70       /* 256bits address lookup table bitmap indicate SA lookup hit  */
    #define SA_AGE_BITMAP_WIDTH     MAC_ADDR_LOOKUP_STA_TABLE_ENTRIES
    #define SA_AGE_BITMAP_STAS_PER_REGISTER    32

#define RX_MPDU_MAXSIZE 0xB90

#define RX_ACK_POLICY   0xB98
    #define RX_ACK_POLICY_AMPDU_BA      0x00000001UL    /* enable this to send BA after receiving A-MPDU (w/ normal ack policy) */
    #define RX_SNIFFER_MODE             0x00000002UL
    #define RX_ACK_POLICY_ALWAYS_ACK    0x00000004UL    /* enable this to send ACK even on RX fifo full condition */
    #define RX_PLCP_LEN_CHK             0x00000008UL
    #define WIRELESS_SEPARATION         0x00000010UL    /* toggle the bit to enable WiFi separation */
    #define RX_AMPDU_REORDER_ENABLE     0x00000020UL
	#define RX_TEMP_QUEUE_DISABLE		0x00000040UL

#define RX_LINK_INFO_CACHE_CTRL  0xB9C
    #define RX_LINK_INFO_CACHE_OUT  0x00000001UL

#define AMPDU_BMG_CTRL  0xBA0
    #define BMG_CLEAR           0x80000000UL
    #define REORDER_FLUSH       0x40000000UL
    #define REORDER_FLUSH_TYPE  0x20000000UL
    #define VARIABLE_TABLE_1_CACHE_FLUSH    0x00080000UL
    #define VARIABLE_TABLE_0_CACHE_FLUSH    0x00040000UL
    #define TSC_TABLE_1_CACHE_FLUSH         0x00020000UL
    #define TSC_TABLE_0_CACHE_FLUSH         0x00010000UL
    #define BMG_TID         0x00000F00UL
    #define BMG_RA_IDX      0x000000FFUL

#define MMAC_RXDESCR_BADDR          0xBA4
#define MMAC_RXDESCR_BADDR2         0xBA8
#define MMAC_RXDESCR_BADDR3         0xBAC
#define MMAC_RXDESCR_BADDR4         0xBB0
#define MMAC_RXDESCR_BADDR_VALID    0xBB4
	#define MMAC_RXDESCR_INIT_DONE		0x00000001UL
	#define MMAC_RXDESCR_BYPASS_MODE	0x00000002UL

#define MMAC_RXDESCR_LINK           0xBB8
#define MMAC_RXDESCR_HEAD_PTR       0xBBC
#define MMAC_RXDESCR_TAIL_PTR       0xBC0
#define MMAC_RXDESCR_COUNT          0xBC4

#define MMAC_RX_DESCR_HEAD_PTR      0xBC8
#define MMAC_RX_DESCR_TAIL_PTR      0xBCC

#define MMAC_RX_CTRL                0xBD0
    #define MMAC_RX_CTRL_STA_LOOKUP_TBL_CLEARED      0x80000000UL

#define AMPDU_RX_BITMAP_BADDR       0xBD4

#define SA_LOOKUP_CONTROL			0xBF0
#define UMAC_DEBUG_SIGNAL			0xBFC

#define LMAC_CNTL       ( LMAC_REG_OFFSET + 0x000 )     /* lower mac control register */    
    #define LMAC_CNTL_TSTART        0x00000001UL
	#define LMAC_CNTL_STAMODE		0x00000006UL
	#define LMAC_CNTL_NAV_CHK		0x00000008UL
	#define LMAC_CNTL_NOCHK_MMAC	0x00000010UL
    #define LMAC_CNTL_BEACON_ENABLE 0x00000400UL
	#define LMAC_CNTL_BEACON_FILTER 0x00000800UL
	#define LMAC_CNTL_CAL_REQ 		0x00010000UL
	#define LMAC_CNTL_CAL_GNT 		0x00020000UL

#define RTSCTS_SET      ( LMAC_REG_OFFSET + 0x004 )
    #define ERP_PROTECT             0x00000001UL
    #define CTS_SELF                0x00000002UL
    #define LMAC_FILTER_ALL_PASS    0x00000004UL
    #define DISABLE_IFS_CHECK       0x00000008UL
    #define BG_PROTECT              0x00000010UL
	#define PROTECT_MODE			0x00000060UL
		#define PROTECT_SHIFT				5
		#define PROTECT_MODE_HT_NO			0x0
		#define PROTECT_MODE_HT_NON_MEMBER	0x1
		#define PROTECT_MODE_HT_20M			0x2
		#define PROTECT_MODE_HT_MIXED		0x3
	#define NONE_GREEN_FIELD		0x00000080UL
    #define RTSCTS_MCS              0x00007F00UL
	#define FORCE_RX_RSPN_RATE		0x00008000UL
    #define LMAC_RTS_CTS_THRESHOLD  0x0FFF0000UL
    #define RTSCTS_RATE             0xF0000000UL        /* 11B or NON-HT rate */

#define BEACON_SETTING  ( LMAC_REG_OFFSET + 0x008 )
    #define BEACON_SETTING_BEACON_INTERVAL      0xFFFF0000UL    /* beacon interval in timeslot (a timeslot is typically 1024 microseconds)*/

#define DTIM_INTERVAL_REG   ( LMAC_REG_OFFSET + 0x00C )
    #define DTIM_INTERVAL_REG_DTIM_INTERVAL     0x000000FFUL
    #define DTIM_INVERVAL_REG_DTIM_COUNTER      0x0000FF00UL    /* RO: current DTIM counter */
    #define DTIM_INTERVAL_REG_SLOT_TIME         0x03FF0000UL    /* slottime, reset default to 1023 */

#define CFP_SET         ( LMAC_REG_OFFSET + 0x0010 )
#define DSSS_DEFER_SET  ( LMAC_REG_OFFSET + 0x0014 )
#define OFDM_DEFER_SET  ( LMAC_REG_OFFSET + 0x0018 )

#define AIFS_SET        ( LMAC_REG_OFFSET + 0x001C )
    #define AIFS_SET_AIFSN_VO   0xF0000000UL
    #define AIFS_SET_AIFSN_VI   0x0F000000UL
    #define AIFS_SET_AIFSN_BE   0x00F00000UL
    #define AIFS_SET_AIFSN_BK   0x000F0000UL
    #define AIFS_SET_DIFSN      0x0000000FUL

#define PHYDLY_SET      ( LMAC_REG_OFFSET + 0x0024 )
    #define PHYDLY_SET_DC_PT    0xFF000000UL
    #define PHYDLY_SET_RFRXDELY 0x00FF0000UL 
    #define PHYDLY_SET_RFTXDELY 0x0000FF00UL
    #define PHYDLY_SET_RES_TIMEOUT 0x000000FFUL

#define TXOP_THRESHOLD  ( LMAC_REG_OFFSET + 0x0028 )
	#define TXOP_TO_PKT				0x00020000UL
	#define PROTECT_LEGACY			0x00010000UL	
    #define REMAINING_TXOP_THRESHOLD  	0x00001FFFUL

#define TXOP_LIMIT      ( LMAC_REG_OFFSET + 0x002C )
    #define TXOP_LIMIT_VO   0xFF000000UL
    #define TXOP_LIMIT_VI   0x00FF0000UL
    #define TXOP_LIMIT_BE   0x0000FF00UL
    #define TXOP_LIMIT_BK   0x000000FFUL

#define BASIC_RATE_BITMAP  ( LMAC_REG_OFFSET + 0x0030 )

#define BASIC_MCS_BITMAP   ( LMAC_REG_OFFSET + 0x0034 )

#define BASIC_SET          ( LMAC_REG_OFFSET + 0x0038 )
    #define BASIC_FORMAT               0x00000003UL
    #define BASIC_NONE_HT_MODE         0x0000000CUL
    #define BASIC_CHANNEL_CH_BANDWIDTH 0x00000030UL
    #define BASIC_CHANNEL_OFFSET       0x000000C0UL
    #define BASIC_SGI                  0x00000100UL
    #define LMAC_CH_BW_CTRL_CH_OFFSET  0x00000600UL
	#define LMAC_DECIDE_CH_OFFSET	   0x00000800UL	/* enable LMAC to replace the channel offset of pre-heading */
	#define LMAC_FORCE_BW20			   0x00001000UL
	#define LMAC_OR_CCA_DIS			   0x00002000UL /* separate CCA1 and CCA0 */
	#define LMAC_CCA1_EN			   0x00004000UL /* enable the CCA1(secondary) checking*/
	#define LMAC_DECIDE_CTRL_FR	   	   0x00008000UL /* The pre-heading of initial control frame is decided by transmitted packet */	 
    #define LMAC_CH_BW_CTRL_AUTO       0x0000E800UL
    #define LMAC_CH_BW_CTRL_AUTO_MASK  0x0000F800UL

#define BEACON_TIMEOUT  ( LMAC_REG_OFFSET + 0x003C )            /* beacon TX timeout in microseconds */
    #define BEACON_TIMEOUT_VAL      0x0000FFFFUL

#define RF_TIME_CTRL    ( LMAC_REG_OFFSET + 0x0040 )

#define CCA_THRESHOLD	( LMAC_REG_OFFSET + 0x0044 )

#define BEACON_CONTROL      ( LMAC_REG_OFFSET + 0x0050 )
    #define BEACON_CONTROL_ENABLE   0x00000001UL
    #define BEACON_CONTROL_RATE_CNTL_BY_REG   0x00000002UL
    #define BEACON_CONTROL_RATE     0x00000F00UL
    #define BEACON_CONTROL_FORMAT   0x00003000UL
    #define BEACON_CONTROL_11B_SP   0x00004000UL
    #define BEACON_CONTROL_NON_SOUNDING  0x00008000UL
    #define BEACON_CONTROL_MCS      0x007F0000UL
    #define BEACON_CONTROL_SGI      0x00800000UL
    #define BEACON_CONTROL_TX_IDLE  0x01000000UL
#define BEACON_TXDSC_ADDR   ( LMAC_REG_OFFSET + 0x0054 )
#define BEACON_SWBL_BADDR   ( LMAC_REG_OFFSET + 0x0058 )
#define BEACON_TX_STATUS    ( LMAC_REG_OFFSET + 0x005C )
    #define BEACON_TX_STATUS_RESULT 0x000000FFUL

#define BEACON_SETTING2     ( LMAC_REG_OFFSET + 0x0060 )
    #define BEACON_SETTING_PRE_BEACON_INTR      0x000003FFUL    /* pre TBTT interrupt time in TU */

#define LMAC_MEDBUSY_CNT    ( LMAC_REG_OFFSET + 0x0078 )
    #define LMAC_MEDBUSY_CNT_CLEAR  0x80000000UL

#define LMAC_40MHZ_TX_REQUEST   ( LMAC_REG_OFFSET + 0x0084 )    /* 24bits counter for 40Mhz TX requests/attempts */
#define LMAC_40MHZ_TX_REJECT    ( LMAC_REG_OFFSET + 0x0088 )    /* 24bits counter for 40Mhz TX downgrade to 20Mhz TX due to seconard channel busy */

#define DEBUG_PORT      ( LMAC_REG_OFFSET + 0x0200 )

#define SEC_KEY_CTRL    ( SEC_REG_OFFSET + 0x0000 )
    #define SEC_KEY_CTRL_STA_IDX            0x00FF0000UL    /* re-key station index, or bssid index */
    #define SEC_KEY_CTRL_STA_IDX_GLOBAL     0x01000000UL    /* set to 1 for group-key, set to 0 for pair-wise key */
    #define SEC_KEY_CTRL_REKEY_SET          0x02000000UL
    #define SEC_KEY_CTRL_REKEY_STA_GKEY     0x04000000UL
    #define SEC_KEY_CTRL_HW_STATUS          0xF0000000UL
    #define SEC_KEY_CTRL_RXPN_INI_VAL       0x0000FF00UL
    #define SEC_KEY_CTRL_CIPHER_MODE        0x0000000FUL
    #define SEC_KEY_CTRL_REKEY_REQ          (SEC_KEY_CTRL_HW_STATUS | SEC_KEY_CTRL_REKEY_SET)

#define SEC_GPKEY_BA    ( SEC_REG_OFFSET + 0x0004 )

#define SEC_STA_GPKEY_BA ( SEC_REG_OFFSET + 0x0008 )

#define SECENG_CTRL     ( SEC_REG_OFFSET + 0x000C )
	#define SECENG_CTRL_BYPASS				0x00000004UL
    #define SECENG_CTRL_RSC_CHECK_OFF       0x00000008UL
    #define SECENG_CTRL_WAPI_RSC_CHECK_OFF  0x00000040UL
    #define SECENG_CTRL_STA_RSC_CHECK_OFF   0x00000080UL
    #define SECENG_CTRL_AMPDU_RSC_CHECK_OFF 0x00000200UL
	#define SECENG_CTRL_ICV_ERR_DROP		0x00010000UL
	#define SECENG_CTRL_MIC_ERR_DROP		0x00020000UL
	#define SECENG_CTRL_GKEY_RSC_ERR_DROP	0x00040000UL
	#define SECENG_CTRL_REKEY_ERR_DROP		0x00080000UL
	#define SECENG_CTRL_KEY_INVALID_DROP	0x00100000UL
	#define SECENG_CTRL_KEY_NONE_DROP		0x00200000UL
	#define SECENG_CTRL_BYPASS_DROP			0x00400000UL

#define SEC_PVKEY_BA    ( SEC_REG_OFFSET + 0x0010 )
#define SEC_PVKEY_BA2   ( SEC_REG_OFFSET + 0x0014 )
#define SEC_PVKEY_BA3   ( SEC_REG_OFFSET + 0x0018 )
#define SEC_PVKEY_BA4   ( SEC_REG_OFFSET + 0x001C )

#define SEC_CCMP_MIC_MASK   ( SEC_REG_OFFSET + 0x0020 )
    #define SEC_CCMP_FRAME_CTRL_MASK        0x0000FFFFUL    /* default 0xc78f */
    #define SEC_CCMP_SEQ_CTRL_MASK          0xFFFF0000UL    /* default 0x000f */

#define SEC_WAPI_MIC_MASK   ( SEC_REG_OFFSET + 0x0028 )
    #define SEC_WAPI_FRAME_CTRL_MASK        0x0000FFFFUL    /* default 0xc78f */
    #define SEC_WAPI_SEQ_CTRL_MASK          0xFFFF0000UL    /* default 0x000f */

#define SEC_WAPI_MIC_MASK_1   ( SEC_REG_OFFSET + 0x002C )
    #define SEC_WAPI_QOS_CTRL_MASK          0x0000FFFFUL    /* default 0xffff */

enum{
    AC_BE2_TID = 0,
    AC_BK2_TID,
    AC_BK1_TID,
    AC_BE1_TID,
    AC_VI2_TID,
    AC_VI1_TID,
    AC_VO2_TID,
    AC_VO1_TID,
};

#define FUNC_MODE   0x2090
    #define FMODE_EN                0x80000000UL
    #define FMODE_EXT_SW_HNAT       0x40000000UL
    #define FMODE_EXT_SW_WIFI       0x20000000UL
    #define FMODE_EXT_BB            0x10000000UL
    #define FMODE_BB_ONLY           0x08000000UL
    #define FMODE_AFE_ONLY          0x04000000UL
    #define FMODE_SW_TEST_MODE      0x02000000UL
    #define FMODE_EFUSE_MODE        0x01000000UL
    #define FMODE_DEBUG_EN          0x00000010UL
    #define FMODE_DEBUG_SELECT      0x0000000FUL

#define HW_RESET  0x20A0
    #define HW_RESET_WMAC         0x00000001UL
    #define HW_RESET_HNAT         0x00000002UL
    #define HW_RESET_USB          0x00000004UL
    #define HW_RESET_SWITCH       0x00000008UL
    #define HW_RESET_BB           0x00000010UL
    #define HW_RESET_SW_P0        0x00000040UL
    #define HW_RESET_SW_P1        0x00000080UL
    #define HW_RESET_WIFI_PAUSE   0x00000100UL
    #define HW_RESET_HNAT_PAUSE   0x00000200UL
    #define HW_RESET_USB_PAUSE    0x00000400UL
    #define HW_RESET_SWITCH_PAUSE 0x00000800UL
    #define HW_RESET_BB_PAUSE     0x00001000UL
    #define HW_RESET_RMII_P0      0x00010000UL
    #define HW_RESET_RMII_P1      0x00020000UL
	#define HW_RESET_DCM_BB2MAC	  0x80000000UL	/* FPGA only */	

#define GLOBAL_PS_MODE		( LMAC_REG_OFFSET + 0xE0 )
    #define PS_MODE_ENABLE			0x000003FFUL

#define GLOBAL_BEACON		( LMAC_REG_OFFSET + 0xE8 )
    #define BEACON_BACKOFF			0x000003FFUL
    #define BEACON_TX_COUNT			0x00FF0000UL
    #define BEACON_RX_COUNT			0xFF000000UL

#define GLOBAL_PRE_TBTT		( LMAC_REG_OFFSET + 0xEC )
    #define PRE_TBTT_INTERVAL_S		0x0000FFFFUL
    #define PRE_TBTT_INTERVAL		0xFFFF0000UL

#define TS_INT_STATUS		( LMAC_REG_OFFSET + 0xF0 )
#define TS_AP_INT_STATUS	( LMAC_REG_OFFSET + 0xF4 )
#ifdef TBTT_EXCEPTION_HANDLE
	#define TBTT_ER_STATUS_0		0x00FF0000UL		/* ts7:ts0 bit map */
	#define TBTT_ER_STATUS_1		0xFF000000UL		/* ts7:ts0 bit map */
#endif  // TBTT_EXCEPTION_HANDLE
#define TS_INT_MASK			( LMAC_REG_OFFSET + 0xF8 )
    #define PRE_TBTT_SW				0x000000FFUL
    #define PRE_TBTT_HW				0x0000FF00UL
    #define BEACON_TIMEOUT_PENDING	0x00FF0000UL
    #define MANUAL_EXPIRED			0xFF000000UL
#define TS_AP_INT_MASK		( LMAC_REG_OFFSET + 0xFC )
#ifdef TBTT_EXCEPTION_HANDLE
	#define TBTT_ERROR_0			0x00FF0000UL		/* ts7:ts0 bit map */
	#define TBTT_ERROR_1			0xFF000000UL		/* ts7:ts0 bit map */
#endif  // TBTT_EXCEPTION_HANDLE


#define TS0_CTRL			( LMAC_REG_OFFSET + 0x100 )
    #define BSSID_LOOKUP_INDEX		0x000000FFUL
    #define TS_ENABLE				0x00000100UL
    #define PS_MANUAL_MODE_ON		0x00000200UL
    #define PS_WAKEUP_PER_TIM		0x00000400UL
    #define PS_WAKEUP_PER_DTIM		0x00000800UL
    #define PS_DOZE_ON				0x00001000UL		/* 0: enable awake mode, 1: enable doze mode */
    #define TS_STATUS				0x00004000UL		/* 0: not in tbtt window, 1: in tbtt window */
    #define PS_STATUS				0x00008000UL		/* 0: awake, 1:doze */

#define TS0_ADJUST			( LMAC_REG_OFFSET + 0x104 )
    #define TS_INACCURACY			0x0000FFFFUL		/* Unit: us */
    #define TS_ADJUST				0xFFFF0000UL		/* Unit: us */

#define TS0_BE				( LMAC_REG_OFFSET + 0x108 )
    #define	BEACON_TIMEOUT_INTERVAL	0x0000FFFFUL		/* Unit: TU, start after TBTT */
    #define	BEACON_INTERVAL			0xFFFF0000UL		/* Unit: TU(1024us) */

#define TS0_DTIM			( LMAC_REG_OFFSET + 0x10C )
    #define	DTIM_INTERVAL			0x000000FFUL
    #define	DTIM_COUNT				0x0000FF00UL

#define TS0_O_L				( LMAC_REG_OFFSET + 0x180 )
#define TS0_O_H				( LMAC_REG_OFFSET + 0x184 )
#ifdef TBTT_EXCEPTION_HANDLE
#define TS0_NEXT_TBTT_L		( LMAC_REG_OFFSET + 0x188 )
#define TS0_NEXT_TBTT_H		( LMAC_REG_OFFSET + 0x18C )
#define TS0_MANUAL_INTERVAL	( LMAC_REG_OFFSET + 0x288 )
#define TS0_BEACON_LOSS		( LMAC_REG_OFFSET + 0x28C )
#else 	// TBTT_EXCEPTION_HANDLE
#define TS0_MANUAL_INTERVAL	( LMAC_REG_OFFSET + 0x188 )
#define TS0_BEACON_LOSS		( LMAC_REG_OFFSET + 0x18C )
#endif  // TBTT_EXCEPTION_HANDLE

#define TS_REG_LO(x)		(TS0_O_L + 0x010*x)
#define TS_REG_HI(x)		(TS0_O_H + 0x010*x)

#endif // __MAC_REGS_H__
