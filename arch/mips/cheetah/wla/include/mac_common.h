/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_common.c
*   \brief  wla MAC's data structure.
*   \author Montage
*/

#ifndef __MAC_COMMON_H__
#define __MAC_COMMON_H__

#include "mac_config.h"

#include "mac_regs.h"


typedef union {
    struct {
        u32 vld:1;
        u32 tosw:1;
        u32 ps:1;
        u32 wep_defkey:1;       

        u32 no_ap_isolated:1;
		u32	p2p_go:1;
		u32 eth_header:1;
		u32	reserved:6;
        u32 rx_ampdu:8;        

        u32 bssid:3;

		u32 captbl_idx  :8;
    } bit;
    u32 val;
} __attribute__((__packed__)) sta_basic_info;

typedef struct {
	sta_basic_info basic_cap;
	char addr[6];
}__attribute__((__packed__)) sta_addr_entry;

typedef union 
{
    struct
    {
        u16     ch_offset:2;
        u16     txcnt:3;
        u16     sgi:1;
        u16     format:2;
        u16     ht:1;
        u16     tx_rate:7;
    } bit;
    u16     val;
} rate_code;

typedef struct {
	/* word 1 */
    u32     valid:1;
    u32     NE:1;
    u32     BG:1;
    u32     power_saving:1;
    u32     AP:1;
    u32     AMSDU:1;
    u32     short_preamble:1;
    u32     QOS:1;
    u32     M:1;
    u32     PSMP:1;
    u32     gsn:1;
    u32     frag_size:2;
    u32     order:1;
    u32         :1;
	u32		parent_ap:1;

    u32     tx_inc:1;
    u32     rx_check_inc:2;
    u32     wep_defkey:1;
    u32     cipher_mode:4;
    u32     force_rts:1;
    u32     ess_idx:4;
    u32     bssid:3;
	/* word 2 */
    u32         :6;
    u32     RX_tbl_addr:26;
	/* word 3 */
    u32     ps_buffer_st:6;
    u32     TX_tbl_addr:26;
	/* word 4 */
    u32     glong_retry:4;
    u32     gshort_retry:4;
    u32     gack_policy:2;
    u32         :6;

    u32     MPDU_spacing:3;
    u32     fec:1;
    u32     nsd:1;
    u32     smt:1;
    u32     NESS:2;
    u32     nhtm:2;
    u32     ANT:2;
    u32     ch_offset:2;
    u32     STBC:2;
	/* word 5,6 */
    rate_code rate[4];
	/* word 7,8 */
	u16     tx_ok_rate[4]; 
	/* word 9 */
    //u32     tx_byte_cnt;
	u16		tx_try_cnt0;
	u16		tx_try_cnt1;
	/* word 10 */
    u16     tx_fail_cnt;
    //u16     tx_ok_cnt;
	u16		tx_try_cnt2;
	/* word 11 */
    u32     rx_byte_cnt;
	/* word 12 */
    u8      rx_rate;
    u8      rssi;
    u16     rx_pkt_cnt;
	/* word 13,14,15,16 */
    u32     WAPI_PN[4];
} sta_cap_tbl;
/*  
    __attribute__((__packed__)) removed
    We do not/cannot packed the sta_cap_tbl structure, since it will cause rate_config to have byte access when 16bits is desired.
    This will cause rate_config update to be none-atomic. 
 */
#define STA_CAP_STATISTIC_ALIGN_SIZE 		24


typedef struct {
    u32     tx_ok_cnt:16;
    u32         :4;
    u32     tx_sequence:12;

    u32     tx_fail_cnt:16;
    u32     tx_retry_cnt:16;

    u32     tx_byte_cnt;

    u32         :16;
    u32         :4;
    u32     rbm:2;
    u32     ack_policy:2;
    u32     long_retry:4;
    u32     short_retry:4;          
} /* __attribute__((__packed__)) */ sta_tx_status_tbl;

typedef struct
{
    u32         :1;
    u32     ap:1;
    u32     baon:1;
    u32     ps_trig_en:1;
    u32         :12;
    u32         :10;
    u32     bmg_window:6;

    u32     inba:1;
    u32     mf:1;
    u32     desti:2;
    u32     swb:1;
    u32     vld:1;
    u32     htvld:1;
    u32     header_chg:1;
    u32         :8;
    u32     rx_sequence:16;

    u32     frame_head_index:16;
    u32     frame_tail_index:16;

    u32         :16;
    u32         :3;
    u32     mpdu_pkt_len:13;

    u32     rsc[4];
} /* __attribute__((__packed__)) */ sta_rx_status_tbl;


typedef struct {
    u32     next_index:16;
    u32     ep:1;
    u32     offset_h:2;             /* offset: high bits */
    u32     len:13;
    u32     offset:6;               /* offset: low bits */
    u32     dptr:26;
} __attribute__((__packed__)) buf_header;

typedef struct {
    u32     own:1;          /* owner bit, 0:free , 1: upper MAC occupy */
    u32     eor:1;          /* end of ring */
    u32         :1;
    u32     bc:1;           
    u32     da_is_ap:1;
    u32     df:1;           /* direct forward */
    u32     swb:1;
    u32     np:1;           /* use sta cap tables next page */

    u32     secoff:1;
    u32         :2;
	u32     dis_agg:1;      /* do not enter the PSBAQ even the BA session does exist */
    u32     tid:4;

	u32     force_hw_free:1;
	u32     	:1;
    u32     header_length:6;

    u32     addr_index:8;

    u32     :2;
    u32     llc:1;
    u32     ba:1;
    u32     nd_rts:1;
    u32     dis_sn:1;
    u32     dis_duration:1;
    u32     ins_ts:1;

    u32     fack:1;
    u32     md:1;
    u32     ins_gsn:1;
    u32     ack_policy:2; // nd_ack:2;
    u32     bssid:3;

    u32     pkt_length:13;
    u32     ba_no_agg:1;     /* do not send with A-MPDU even the packet enters PSBAQ (if BA session does exist); use the sequence num in the BA session */
    u32     AMSDU:1;
    u32     chk_ps:1;

    u32     bhdr_tail_index:16;
    u32     bhdr_head_index:16;

    union 
    {
            struct 
            {
                  u32         :16;
				  u32     rate_index:8;
				  u32         :3;
                  u32     tx_failed:1;
                  u32     retry_cnt:4;
            } tx;
            struct
            {
                  u32         :22;
                  u32     ifs:2;
                  u32         :1;
                  u32     antennas:1;
                  u32     ch_offset:2;
                  u32     non_ht_mode:2;
                  u32     format:2;
            } beacon;
#if (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
            struct
            {
                  u32         :13;
                  u32     backoff_0_8:9;
                  u32     ifs:2;
                  u32     backoff_9:1;
                  u32     antennas:1;
                  u32     ch_offset:2;
                  u32     non_ht_mode:2;
                  u32     format:2;
            } atim;
#endif //  (defined(CONFIG_WLAN_IBSS_MODE) && defined(CONFIG_WLAN_ATIM_VERIFY))
    } u;
} tx_descriptor;
//} __attribute__((__packed__)) tx_descriptor;

typedef struct 
{
    tx_descriptor tx_descr;
    u32 np:1;
    u32 b2b:1;
    u32    :4;
    u32 next_pointer:26;
    u32 ifs:2;
#ifdef CONFIG_WLAN_IBSS_MODE
	u32	is_atim:1;				/* add by sw to indicate this desc is for ATIM or not*/
    u32    :3;
#else		//	CONFIG_WLAN_IBSS_MODE
    u32    :4;
#endif		//	CONFIG_WLAN_IBSS_MODE
    u32 schedule_time:26;
} __attribute__((__packed__)) ssq_tx_descr;

#define DA_HIT_ADDR		0x1
#define DA_HIT_BSSID	0x2
#define SA_HIT_ADDR		0x4
#define TA_HIT_DS		0x8

typedef struct {
    u32     own:1;          /* owner bit, 0:free , 1: upper MAC occupy */
    u32     eor:1;          /* end of ring */
    u32     drop:1;
    u32     bc:1;
    u32     udrap:1;
    u32     from_parent_ap:1;		/* client mode packet */
    u32     swb:1;
	u32     sa_idx_26:5;	
    u32     tid:4;
    
	u32     sa_idx_01:2;
    //u32     vlan_tag:1;             /* 802.1Q header exists */
    //u32     double_vlan_tag:1;      /* double vlan tagging */
    u32     header_length:6;
    u32     addr_index:8;

	u32		rssi:8;
	u32		 :4;
	u32     sa_idx_7:1;
	u32		bssid:3;

    u32     pkt_length:13;
    u32     trigger_frame:1;
    u32     ps_poll:1;
    u32     ps:1;
#define e2w_da_hit	ps

    u32     bhdr_tail_index:16;
    u32     bhdr_head_index:16;

	u32		bar:1;			/* BAR received */
	u32     ampdu:1;        /* packet extracted from A-MPDU */
	u32     amsdu:1;        /* RX packet is actually an A-MSDU */
    u32     sec_mismatch:1; /* RX plain-text packet when the TA has cipher key attached */
	u32        :2;
    u32     sec_status:9;
    u32        :1;

    u32     framedesti:2;
    u32     ftcpu:1;
    u32     hit:4;
    u32     tide:1;

    u32     bae:1;
    u32     qose:1;
    u32     bse:1;
    u32     ve:1;                               /* frame control version mismatch */
    //u32     rae:1;                              /* RA address mismatch */
	u32		wh:1;								/* keep wlan header */
    u32     bsid:1;                             /* BSSID mismatch */
    u32     sec_err:1;                          /* security error, need to check sec_status */
    u32     crc:1;                              /* CRC error */
} __attribute__((__packed__)) rx_descriptor;

#define SA_IDX_FROM_DESC(_x) ((_x->sa_idx_7 << 7) | (_x->sa_idx_26 << 2) | _x->sa_idx_01)

typedef struct {
    u32     sequence_num:12;
    u32        :2;
    u32     bank_sel:2;
    u32     next_index:16;
    u32     descr[4];
    u32     tsc[4];
} __attribute__((__packed__)) mmac_rx_descriptor;

/* RX descriptor sec_status bitfields */
#define SS_TOCPU				0x100
#define SS_BYPASS				0x040
#define SS_CIPHER_KEY_NONE		0x020
#define SS_CIPHER_KEY_INVALID	0x010
#define SS_REKEY_DROP			0x008
#define SS_DISASSOC_DROP		0x004
#define SS_TKIP_MIC_ERR			0x002
#define SS_ICV_ERR				0x001
#define SS_DROP					( SS_REKEY_DROP | SS_DISASSOC_DROP | SS_TKIP_MIC_ERR | SS_ICV_ERR )

#define REORDER_BUF_MAX_BUFSIZE    64
typedef struct {
    u32     rsc[4];
    u32     :5;
    u32     buf_size:3;
    u32     ssn:12;
    u32     head_seq_num:12;
	u32     bitmap[2];
    u32     dbg_dirty:16;
    u32     drop:16;
    u32     data[REORDER_BUF_MAX_BUFSIZE];
} __attribute__((__packed__)) ampdu_reorder_buf;

#define CIPHER_TYPE_NONE    0
#define CIPHER_TYPE_WEP40   1
#define CIPHER_TYPE_WEP104  2
#define CIPHER_TYPE_TKIP    3
#define CIPHER_TYPE_CCMP    4
#define CIPHER_TYPE_SMS4	5

#define WEP_DEF_TXKEY_ID	0
#define CIPHYER_TYPE_INVALID    CIPHER_TYPE_NONE

#define WEP_40_KEY_LEN		5
#define WEP_104_KEY_LEN		13
#define TKIP_KEY_LEN		16
#define TKIP_MICKEY_LEN		8
#define CCMP_KEY_LEN		16

#define WAPI_PN_DWORD		0x5c365c36UL
#define WAPI_PN_64BITS		0x5c365c365c365c36ULL	
#define WAPI_TX_INC_ONE     0
#define WAPI_TX_INC_TWO     1
#define WAPI_RX_CHECK_INC_EVEN    0
#define WAPI_RX_CHECK_INC_ODD     1
#define WAPI_RX_CHECK_INC         2

#define WAPI_CIPHER_KEY_LEN     16
#define WAPI_MICKEY_LEN         16

typedef union {
    struct {
        u32     cipher_type:16;
        u32         :14;
        u32     txkeyid:2; 

        u8      key[4][13];

        u32     __dummy;

        u32     iv:24;
        u32         :8;
    } __attribute__((__packed__)) wep_key;
    struct {
        u32     cipher_type:4;
        u32         :26;
        u32     txkeyid:2;
    
        u32     txmickey[2];

        u32     rxmickey[2];

        u8      key[16];

        u32     txtsc_lo;
    
        u32     txtsc_hi:16;
        u32         :16;

		u32     rxmic[2];
    
        u32     rxdata[3];
    } __attribute__((__packed__)) tkip_key;
    struct {
        u32     cipher_type:4;
        u32         :26;
        u32     txkeyid:2;
    
        u32     __dummy[4];

        u8      key[16];

        u32     txpn_lo;

        u32     txpn_hi:16;
        u32         :16;
    
        u32     __dummy2[5];
    } __attribute__((__packed__)) ccmp_key;
    struct {
        u32     cipher_type:4;
        u32         :26;
        u32     txkeyid:2;
    
        u8      mickey[16];

        u8      key[16];

        u32     txpn[4];

        u32     __dummy[3];
    } __attribute__((__packed__)) wapi_key;
} cipher_key;

typedef union {
  cipher_key group_key;
  struct {
        u32     ___keydata[14];
        u64     rsc[9];     
  }; //__attribute__((__packed__));
} group_cipher_key;

#define QMODE_PS_BIT        0x01
#define QMODE_DRAIN_BIT     0x02
#define QMODE_BA_BIT        0x04
#define QMODE_DATA_BIT      0x08

typedef struct {
    u32         :2;
    u32     win_size:6;
    u32         :2;
	u32		force_noagg:1;
    u32     max_ampdu_len:2;
    u32     max_sp:2;
    u32     dly_en:1;

    u32     tx_queue_len:16;

    u32         :29;
    u32     qmode:3;

    u32     tx_bm_lo;
    u32     tx_bm_hi;

    u32     tx_tail_ptr:16;
    u32     tx_head_ptr:16;

	u32		 :3;
    u32     bar:1;
    u32     win_start:12;
    u32     pkt_cnt:16;

    u32         :12;
    u32     timer_cnt:4;
    u32         :6;
	u32		tx_rate_idx:2;
    u32     ba_retry_cnt:8;

    u16     dq_agg_cnt;
    u16     eq_agg_cnt;
} __attribute__((__packed__)) ucast_qinfo;

typedef struct {
    u32         :16;
    u32     tx_queue_len:16;

    u32         :16;
    u32     sta_cnt:8;
    u32         :5;
    u32     qmode:3;

    u32         :32;
    u32         :32;

    u32     tx_tail_ptr:16;
    u32     tx_head_ptr:16;

    u32         :16;
    u32     pkt_cnt:16;

    u32         :32;
    u32         :32;
} __attribute__((__packed__)) bcast_qinfo;

typedef struct {
    u32     timestamp:16;
    u32     next_descr:16;
    u32     tsc[4];
    u32     mpdu_desp[4];
} __attribute__((__packed__)) psbaq_descriptor;

typedef struct {
    u32     vld:1;
    u32     qos:1;
    u32        :4;
    u32     qinfo_addr:26;

    u32        :23;
	u32		is_ap:1;
    u32     sta_idx:8;
} __attribute__((__packed__)) sta_qinfo_tbl;


typedef struct{
	u8 da[6];
	u8 sa[6];
	u16 len;
	u8 llc[6];
	u16 type;
	u8 data[0];	
} __attribute__((__packed__)) amsdu_sf;

typedef struct{
	u8 da[6];
	u8 sa[6];
	u16 type;
	u8 data[0];
} __attribute__((__packed__)) ehdr;

#define IS_GROUPCAST(_da)	(((u8 *)_da)[0] & 1)
#define IS_MULTICAST(_da)	((((u8 *)_da)[0] & 0xff) != 0xff)
#define IS_MCAST_MAC(_da)	((_da[0]==0x01) && (_da[1]==0x00) && (_da[2]==0x5e))

#define ETHER_HDR_LEN		14
#define DA_SA_LEN			12

#define IS_FROM_ETH(_v)	(_v > 0x7f)
#define ETH_TO_BSS(_v) (_v | 0x80)
#define ETH_VIFID(_v) (_v & 0xf)

typedef struct {
	u32	flag:1;				/* flag, 0:MCU own, 1:To HOST */
	u32	wh:1;				/* 1: WiFi header packet */
	u32 ap:1;				/* 0: Client mode, 1:AP mode */
	u32 bss_desc:3;			/* MAC address bit map */
	u32 fmds:1;				/* 1: Packet is from another AP */
	u32 ampdu:1;			/* AMPDU sub-frame */
	u32 amsdu:1;			/* AMSDU packet */
	u32 bcmc:1;				/* Broadcast/Multicast packet */
	u32 :1;
	u32 qos:1;				/* Qos packet */
	u32 pstr:1;				/* PM trigger frame */
	u32 ps:1;				/* PM bit */
	u32 eosp:1;				/* End of sevice period */
	u32 mdat:1;				/* More Data */
	u32 ipcsok:1;			/* IP check-sum is OK */
	u32 tcpcsok:1;			/* TCP check-sum is OK */
	u32 llc:1;				/* Packet have LLC field */
	u32 	:1;
	u32 tid:4;				/* Wifi packet's TID */
	u32 sa_idx:8;

	u32 pos:2;				/* 4-byte alignment address */
	u32 pkt_len:14;			/* Packet length */
	u32 	:4;
	u32 sn:12;				/* Sequence number */

	u32 secst:4;			/*	Bit0: ICV error/runt error
								Bit1: MIC error
								Bit2: key not valid
								Bit3: non-key */
	u32 hit:4;				/*	Bit0: BSSID match
								Bit1: TA hit in STA table
								Bit2: TA hit in DS table
								Bit3: DA hit */
	u32 offset:8;			/* Header offset */
	u32 rssi:8;				/* Signal strength */
	u32 rate:8;				/* Recv rate */
} __attribute__((__packed__)) wbuf_desc;

typedef struct {
	u32	flag:1;				/* flag, 0:MCU own, 1:To Device */
	u32	wh:1;				/* 0: Ethernet header packet, 1: WiFi header packet */
	u32 ap:1;				/* 0: Client mode, 1:AP mode */
	u32 np:1;				/* Next page */
	u32 tocpu:1;			/* Force to MCU */
	u32 ra_is_ap:1;			/* RA is AP */
	u32 wifitxdesc:1;		/* WiFi desc is carried with packet */
	u32 dis_agg:1;			/* Disable aggregation */
	u32 enterpsq:1;			/* Enter PSQ even */
	u32 secoff:1;			/* Disable encrypt */
	u32 insgsn:1;			/* Insert global sequence number */
	u32 amsd:1;				/* AMSDU packet */
	u32 ps:1;				/* PM bit */
	u32 mdat:1;				/* More Data */
	u32 eosp:1;				/* End of sevice period */
	u32 	:1;
	u32 ipcsok:1;			/* IP check-sum is OK */
	u32 tcpcsok:1;			/* TCP check-sum is OK */
	u32 	:2;
	u32 tid:4;				/* Wifi packet's TID */
	u32 np_index:8;			/* Rate table index, valid when NP = 1 */

	u32 pos:2;				/* 4-byte alignment address */
	u32 pktlength:14;		/* Packet length */
	u32 res:16;				/* Reserved */
} __attribute__((__packed__)) hbuf_desc;

#define ETH_ALIGN_SIZE	4

#endif // __MAC_COMMON_H__

