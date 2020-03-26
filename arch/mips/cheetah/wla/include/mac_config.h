/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_config.c
*   \brief configuration of MAC.
*   \author Montage
*/

#ifndef __MAC_CONFIG_H__
#define __MAC_CONFIG_H__

//#define BEACON_SW_TIMER				/* software timer simulate beacon interrupt */
//#define BEACON_USE_NORMAL_QUEUE
//#define CONFIG_WLA_RF_AIROHA
#define USE_W6_INTERFACE
#define WLA_DEBUG_DUMP
#define WLA_DEBUG_STATISTIC
#define WLA_WIFI2ETH
#define WLA_MMAC_RXDESCR
#define WLA_AMPDU_RX_HW_REORDER
//#define WLA_AMPDU_RX_SW_REORDER
#define WLA_PSBA_QUEUE
#define WLA_CLIENT_MODE
//#define CONFIG_WLAN_MISMATCH_DATA_TO_LOCAL
//#define CONFIG_WLAN_MISMATCH_DATA_TO_CPU
//#define ETH_PKT_ALL_TO_CPU
//#define CONFIG_WIFI2ETHER_DISABLE
//#define DEBUG_TRACE_RX_TBL_HISTORY
//#define DATA_FEAME_FORCE_TO_SW
//#define CONFIG_WLAN_HW_FORWARD_MULTICAST
//#define WBUF_IS_CACHED
#define WBUF_IP_HEADER_ALIGN
#define SW_BUF_TRACE
#define ENABLE_NAV_HONOURING
#define ENABLE_POWER_SAVING
#define NEW_BB
//#define DEBUG_MEM_CORRUPT
#define DEBUG_TXQ
#define TBTT_EXCEPTION_HANDLE
#define TX_NOAGG_BY_PSBAQ
#define FASTPATH_DESCR_COUNT	0
#define AMPDU_MAX_TRY			32

#define MAX_ADDR_TABLE_ENTRIES	256
#define MAX_DS_TABLE_ENTRIES	16
#define MAC_MAX_BSSIDS  8
#define MAX_PEER_AP		4

#define MAX_STA_DS_CACHE	23 /* 24 STA caches and 8 DS caches */

#define SW_ALLOCATED_BUF_COUNT					(32) /* buffer size should cover reorder buffer maximum window size */

#define SW_BUFFER_HEADER_POOL_SIZE				256
#define BEACON_Q_BUFFER_HEADER_POOL_SIZE        MAC_MAX_BSSIDS

#define DEF_SW_PATH_BUF_SIZE		1600   		

#ifndef CONFIG_ASIC_VER
#define CONFIG_ASIC_VER		3
#endif

#if (CONFIG_ASIC_VER == 2) && !defined(CONFIG_FPGA)
	//#define DETECT_RX_ERR_FRAME
	#define INTEL_SETUP_BA_PATCH
#endif

//#define CONFIG_MINI_WLA

#if defined(CONFIG_MINI_WLA)
	#define FASTPATH_BUFFER_HEADER_POOL_SIZE		2
	#define MAX_STA_CAP_TBL_COUNT       16             /* total elements of station capability tables */
	#define MAX_BSSIDS                  1
	#define TX_DESCRIPTOR_COUNT         64              /* numbers of continuous allocated TX descriptors */ /* TX's number is at least tiwce of RX's to handle AMSDU */
	#define RX_DESCRIPTOR_COUNT         32              /* numbers of continuous allocated RX descriptors */
	#define MMAC_RX_DESCRIPTOR_COUNT	4

	#define PSBAQ_DESCRIPTOR_COUNT		256

	#define DEFAULT_QOS_RX_MPDU_WINDOW               ( 32 )
	#define DEFAULT_LEGACY_RX_MPDU_WINDOW           ( 16 )

	#define DEF_FASTPATH_BUF_SIZE       4
	#undef WLA_WIFI2ETH

#elif defined(CONFIG_SRAM_WLA)
	#undef FASTPATH_DESCR_COUNT
	#define FASTPATH_DESCR_COUNT		32				/* number of allocated wifi to wifi descriptors */

	#define FASTPATH_BUFFER_HEADER_POOL_SIZE        (32)
	#define MAX_STA_CAP_TBL_COUNT       8             /* total elements of station capability tables */
	#define MAX_BSSIDS                  1
	#define TX_DESCRIPTOR_COUNT         64              /* numbers of continuous allocated TX descriptors */ /* TX's number is at least tiwce of RX's to handle AMSDU */
	#define SW_RET_DESCRIPTOR_COUNT     240				/* swretq's number is large enought to cover AMPDU release descripotrs */
	#define HW_RET_DESCRIPTOR_COUNT     1		

	#define RX_DESCRIPTOR_COUNT         32              /* numbers of continuous allocated RX descriptors */
	#define MMAC_RX_DESCRIPTOR_COUNT	256

	#define PSBAQ_DESCRIPTOR_COUNT		256

	#define DEFAULT_QOS_RX_MPDU_WINDOW               ( 32 )
	#define DEFAULT_LEGACY_RX_MPDU_WINDOW           ( 16 )

	#define DEF_FASTPATH_BUF_SIZE       512 //576

	#undef WLA_WIFI2ETH
	#define WLA_AMPDU_RX_HW_REORDER
	#undef WLA_AMPDU_RX_SW_REORDER

	#undef MAX_ADDR_TABLE_ENTRIES
	#define MAX_ADDR_TABLE_ENTRIES		8
	#undef MAX_DS_TABLE_ENTRIES
	#define MAX_DS_TABLE_ENTRIES		1

	#undef SW_BUFFER_HEADER_POOL_SIZE
	#define SW_BUFFER_HEADER_POOL_SIZE				288
	#undef DEF_SW_PATH_BUF_SIZE
	#define DEF_SW_PATH_BUF_SIZE		512   

#elif defined(CONFIG_CHEETAH)
	#undef FASTPATH_DESCR_COUNT
	#define FASTPATH_DESCR_COUNT		256				/* number of allocated wifi to wifi descriptors */

	#define FASTPATH_BUFFER_HEADER_POOL_SIZE        (256*3*2)
	#define MAX_STA_CAP_TBL_COUNT       64             /* total elements of station capability tables */
	#define MAX_BSSIDS                  4
	#define TX_DESCRIPTOR_COUNT         64              /* numbers of continuous allocated TX descriptors */ /* TX's number is at least tiwce of RX's to handle AMSDU */
	#define SW_RET_DESCRIPTOR_COUNT     128				/* swretq's number is large enought to cover AMPDU release descripotrs */
	#define HW_RET_DESCRIPTOR_COUNT     1		

	#define RX_DESCRIPTOR_COUNT         32              /* numbers of continuous allocated RX descriptors */
	#define MMAC_RX_DESCRIPTOR_COUNT	256

	#define PSBAQ_DESCRIPTOR_COUNT		256

	#define DEFAULT_QOS_RX_MPDU_WINDOW               ( 32 )
	#define DEFAULT_LEGACY_RX_MPDU_WINDOW           ( 16 )

	#define DEF_FASTPATH_BUF_SIZE       512 //576

	#undef WLA_WIFI2ETH
	#define WLA_AMPDU_RX_HW_REORDER
	#undef WLA_AMPDU_RX_SW_REORDER

	#undef MAX_ADDR_TABLE_ENTRIES
	#define MAX_ADDR_TABLE_ENTRIES		64
	#undef MAX_DS_TABLE_ENTRIES
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE
	#define MAX_DS_TABLE_ENTRIES		16
#else
	#define MAX_DS_TABLE_ENTRIES		4
#endif	
#else

	#define FASTPATH_BUFFER_HEADER_POOL_SIZE        (128*8*2)
	#define MAX_STA_CAP_TBL_COUNT       256             /* total elements of station capability tables */
	#define MAX_BSSIDS                  8
	#define TX_DESCRIPTOR_COUNT         64              /* numbers of continuous allocated TX descriptors */ /* TX's number is at least tiwce of RX's to handle AMSDU */
	#define RX_DESCRIPTOR_COUNT         32              /* numbers of continuous allocated RX descriptors */
	#define MMAC_RX_DESCRIPTOR_COUNT	256

	#define PSBAQ_DESCRIPTOR_COUNT		512

	#define DEFAULT_QOS_RX_MPDU_WINDOW               ( 32 )
	#define DEFAULT_LEGACY_RX_MPDU_WINDOW           ( 16 )

	#define DEF_FASTPATH_BUF_SIZE       512 //576
  
	#if defined(CONFIG_WIFI2USB)
		#undef WLA_WIFI2ETH
	#endif
#endif

#ifndef CONFIG_NET_NONCACHED_CLUSTER
	#define WBUF_IS_CACHED
#endif


#define HT_LONG_RETRY_DEFAULT 8
#define HT_SHORT_RETRY_DEFAULT 8
#define LEGACY_LONG_RETRY_DEFAULT 8
#define LEGACY_SHORT_RETRY_DEFAULT 8

#define NBUF_LEN		1536

#define PSBAQ_BCAST_TX_QUEUE_LEN	255

/* HW characterics; DO NOT CHANGE IT! */

#define MAC_TX_TEMP_DRAM_BUFFER_SIZE            ( 64 * 8 )

#define MAC_STA_CAP_TABLE_ENTRIES_PER_BADDR     64

#define MAC_ADDR_LOOKUP_STA_TABLE_ENTRIES       256

#define MAC_ADDR_LOOKUP_DS_TABLE_ENTRIES        16


#define MAX_RATE_TBL_COUNT          (MAX_BSSIDS*3)

#define MAX_QOS_TIDS                8

#define QOS_TX_STATUS_TBLS          MAX_QOS_TIDS
#define QOS_RX_STATUS_TBLS          ( MAX_QOS_TIDS + 1)

#define MAX_PS_QUEUE_LENGTH			64 //(64 * 3)	/* PS queue should not too big to consume all buffers */	
#define MAX_AMPDU_QUEUE_LENGTH		170 		/* To pass WIFI 2.4.30 case, large queue length to buffer 4 IP fragment traffics simultaneously */
	
#define EDCA_LEGACY_FRAME_TID       0

#define EDCA_HIGHTEST_PRIORITY_TID  7

#define MAX_TX_AMPDU_IDLE_TIME		10 // seconds

/* According WMM test plan table C.2 */
#define VO1_DEFAULT_DSCP_VAL		0x38
#define VO2_DEFAULT_DSCP_VAL		0x30
#define VI1_DEFAULT_DSCP_VAL		0x28
#define VI2_DEFAULT_DSCP_VAL		0x20
#define BE1_DEFAULT_DSCP_VAL		0x18
#define BE2_DEFAULT_DSCP_VAL		0x00
#define BK1_DEFAULT_DSCP_VAL		0x10
#define BK2_DEFAULT_DSCP_VAL		0x08
#define DSCP_VAL_SHIFT				2

#define VO1_DEFAULT_802_1D_VAL		0x7
#define VO2_DEFAULT_802_1D_VAL		0x6
#define VI1_DEFAULT_802_1D_VAL		0x5
#define VI2_DEFAULT_802_1D_VAL		0x4
#define BE1_DEFAULT_802_1D_VAL		0x3
#define BE2_DEFAULT_802_1D_VAL		0x0
#define BK1_DEFAULT_802_1D_VAL		0x2
#define BK2_DEFAULT_802_1D_VAL		0x1
#define PRI_1D_VAL_SHIFT			5

#define ACQ_MIN_SIZE				10
#define ACQ_TOTAL_SIZE				256

#define BEACON_BITRATE				CCK_1M

#define MAX_CONSECUTIVE_AMPDU_TX_FAILURE	15

#define WLAN_BCAST_THRESHOLD	100

#define WLA_HWADDR_AGE		3000 // 5 min
#define WLA_ACQ_CHECK_TIME	100 // 1 sec
#define WLA_BCN_MON_TIME	100 // 1 sec
#endif // __MAC_SIM_CONFIG_H__

#if !defined(CONFIG_CHEETAH_FPGA)
#define WLA_MULTICAST_PATCH
#define WLA_MULTICAST_IDX 63
#endif
