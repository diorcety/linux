/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wbuf.h
*   \brief 
*   \author Montage
*/

#ifndef __WBUF_H__
#define __WBUF_H__

#define WLAN_TX_RESERVED_LEADIND_SPACE			64
#define WLAN_TX_MIN_RESERVED_LEADIND_SPACE		4

#define WLAN_RC_BTS
//#define WLAN_RC_AMRR
#define HW_SYNC_TIME			10 /* 100ms to check hardware status per STA */

/* definition of flags */
enum {
	WBUF_FIXED_SIZE				= BIT(0),
	WBUF_RX_FRAME				= BIT(1),		/* indicate the buffer is received from wlan */
	WBUF_TX_REQ_STATUS			= BIT(2),
	WBUF_TX_NO_ACK				= BIT(3),		/* this frame does not need ACK */
	WBUF_TX_NO_SECU				= BIT(4),		/* indicate the frame does not encrypt */
	WBUF_TX_SPEC_RATE			= BIT(5),		/* force to send out in specified rate */
	WBUF_TX_BASIC_RATE			= BIT(6),		/* force to send out in basic rate */
	WBUF_TX_CHECK_PS			= BIT(7),
	WBUF_TX_INSERT_TS			= BIT(8),		/* inform HW insert Timestamp field */

	WBUF_TX_PS_NULL				= BIT(9),		/* indicate send the NULL frame with PM bit */

	WBUF_TX_REPORT_STATUS		= BIT(10),		/* indicate the wbuf is TX status */
	WBUF_TX_SUCCESS				= BIT(11),		/* report transmition is success */
#define WBUF_TX_NULL_DATA		WBUF_TX_SUCCESS	/* indicate send the NULL frame */
	
	WBUF_MGMT_FRAME				= BIT(12),		/* management frame */
	WBUF_CHAIN					= BIT(13),		/* wbuf link list */
	WBUF_EXT					= BIT(14),		/* for sample rate */	
	WBUF_TX_DIS_DURATION		= BIT(15),		/* disable hw tx duration config */
};

enum {
	WBUF_RX_DA_HIT_ADDR			= BIT(0),
	WBUF_RX_DA_HIT_BSSID		= BIT(1),
	WBUF_RX_SA_HIT_ADDR			= BIT(2),
	WBUF_RX_TA_HIT_DS			= BIT(3),
};

enum {	
	WBUF_RX_ICV_ERR				= BIT(0),
	WBUF_RX_MIC_ERR				= BIT(1),
	WBUF_RX_KEY_NOT_VALID		= BIT(2),
	WBUF_RX_NON_KEY				= BIT(3),
};

struct wbuf {
	u16 flags;
	u8	rate_idx;
	u8	sta_idx;
	/* begin of ASIC's definition */
	/* word 1 */
	u32	own:1;				/* flag, 0:MCU own, 1:To HOST */
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

	/* word 2 */
	u32 pos:2;				/* 4-byte alignment address */
	u32 pkt_len:14;			/* Packet length */
	u32 	:4;
	u32 sn:12;				/* Sequence number */

	/* word 3*/
	u32 secst:4;			/*	Bit0: ICV error/runt error
								Bit1: MIC error
								Bit2: key not valid
								Bit3: non-key */
	u32 hit:4;				/*	Bit0: BSSID match
								Bit1: TA hit in STA table
								Bit2: TA hit in DS table
								Bit3: DA hit */
	u32 data_off:8;			/* Header offset */
	u32 rssi:8;				/* Signal strength */
	u32 rate:8;				/* Recv rate */
	/* end of ASIC's definition */

	struct wbuf *wb_next;
	struct wbuf *rb_next;	/* reorder buffer link list */
};


struct wlan_rate {
	unsigned int flags;
	unsigned int perfect_tp;		/* throughput of 100% successful transmission */
	unsigned short bitrate;
	unsigned short hw_value;
};

#define RATE_FLAGS_HT_SGI20			0x1
#define RATE_FLAGS_HT_SGI40			0x2
#define RATE_FLAGS_HT_40M			0x4
#define RATE_FLAGS_HT_GF			0x8
#define RATE_FLAGS_SHORT_PREAMBLE 	0x10
#define RATE_FLAGS_INIT				0x80

enum {
	BA_IDLE_STATE = 0,
	BA_ESTABLISHING_STATE,
	BA_ESTABLISHED_STATE,
	BA_TEARDOWNING_STATE,
	BA_FORBIDDEN_STATE,
};

struct tx_ba_session {
	u8		state;
	u8 		req_num;	/* record how many times to send ADDBA or fail to send AMPDU */
	u8		dialogtoken;
	u8 		reserve;
	u16		start_seq;
	u16		win_start;
};

struct rx_ba_session {
		u16 win_start;
		u8 win_size;
		u8 stored_num;
		struct wbuf *reorder_q_head;
		struct wbuf *reorder_q_tail;
};

/* BA's command */
enum{
	BA_SESSION_INITIATION,
	BA_SESSION_TEARDOWN,
	BA_SESSION_TEARDOWN_ALL,
};

struct rate_stats{
	u32 attempts;
	u32 success;
};

struct tx_rate_info {
	u8 rate_idx;
	u8 bts_rates;
};
#define TS_STATE_INIT		BIT(0)
#define TS_STATE_SYNCED		BIT(1)

struct wsta_cb
{
	struct wsta_cb *next;
	u32		sync_time;
	u16		aid; /* STA's unique AID (1 .. 2007) or 0 if not yet assigned */
	u8 		addr_idx;
	u8 		captbl_idx;
	u8		bss_desc;
	u8		mode;

#if defined(WLAN_RC_AMRR)
	struct {
		u8	success;
		u8	recovery;
		u8	success_threshold;
		u16	total_cnt;
		u16	rate0_ok_cnt;
	}amrr;
#endif

#if defined(WLAN_RC_BTS)
#define BTS_RATE_MAX_NUM 20
	struct bts_cb{
		struct bts_rate {
			u32 timestamp;			/* Timestamp of transmited using this rate */
			u32 tp;					/* throughput */
			u16 avg_prob;			/* average probability of successful transmission */
			//u8 credit;				/* credit for sample */
			u8 rate_idx;
			u16 success;
			u16 attempts;
		}rates[BTS_RATE_MAX_NUM];

		struct tx_rate_info aux_tx_rate[4];			/* auxiliary rate set to probe at background */
		u32 sample_timestamp;
		u8 aux;
		u8 rates_len;
		u16 probation;
		u8 cur_rates[4];
	}bts;
#endif
	struct tx_rate_info cur_tx_rate[4];
	u8		rate_flags;
	//u8		max_tx_rate;
	//u8		min_tx_rate;	
	u32		supported_rates;

	void	*linked_ap;

	u32		rx_bytes;
	u32		rx_cnt;
	
	u32		tx_bytes;
	u16		tx_try_cnt[3];
	u16		tx_fail_cnt;
	u16		tx_ok_rate[4];

	u16		hw_cap_ready:1;
	u16		activity:1;
	u16		controlled_port:1;
	u16		max_ampdu_factor:2;
	u16		min_mpdu_start_spacing:3;
	u16		qos:1;
	u16		ht:1;
	u16		ps_mode:1;
	u16		reserved:5;

	u32		reorder_buffer_check_time;
	u32		tx_ampdu_teardown_time;
	u32		psbaq_freeze_time;
	u16		wds_sa_count;
	u8		max_sp_len;
	u8		apsd_trigger_deliver;

	u8		rc_allow_ampdu;
	s8      bad_ampdu_tx;
	u8		rx_rate;
	u8		rssi;
	u8		tx_ampdu_bitmap;
	u8		chip_vendor;

	struct tx_ba_session ba_tx[8];
	struct rx_ba_session *ba_rx[8];
	u16		ampdu_rx_last_seq_num[8]; /* per TID */
	u16		psbaq_dq_cnt[8]; /* per TID */

	u8		ts_synced;	/* for TBTT_EXCEPTION_HANDLE & de-auth by beacon loss */

#ifdef CONFIG_WLAN_IBSS_MODE
	u8 		*ibss_bssid;
	u8		ibss_ds_idx;
#endif
	void	*sta;
	u32		basic_rc_count;
	u32		basic_rc_flag;
	u32 	last_rx;		/* for inactivity */
};


#define APSD_AC_BE_TRIGGER		0x01
#define APSD_AC_BK_TRIGGER		0x02
#define APSD_AC_VI_TRIGGER		0x04
#define APSD_AC_VO_TRIGGER		0x08
#define APSD_AC_BE_DELIVERY		0x10
#define APSD_AC_BK_DELIVERY		0x20
#define APSD_AC_VI_DELIVERY		0x40
#define APSD_AC_VO_DELIVERY		0x80

#define APSD_AC_ALL_DELIVERY	(APSD_AC_BE_DELIVERY|APSD_AC_BK_DELIVERY|APSD_AC_VI_DELIVERY|APSD_AC_VO_DELIVERY)

enum {
	VENDOR_INTEL = 1,
};

struct wbuf *wbuf_alloc(u32 size) __attribute__ ((section(".iram")));
void wbuf_free(struct wbuf *wb) __attribute__ ((section(".iram")));

#endif
