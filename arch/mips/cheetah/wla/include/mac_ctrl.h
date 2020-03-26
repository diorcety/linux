/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file mac_ctrl.h
*   \brief
*   \author Montage
*/

#ifndef __MAC_INFO_H__
#define __MAC_INFO_H__

#include <mac_config.h>
#include <mac_common.h>
#include <wbuf.h>
#include <wlan_def.h>
#include <wla_api.h>

#define WLAN_RX_LLC_OFFSET					68

#define MAC_ACK_POLICY_NORMAL_ACK			0
#define MAC_ACK_POLICY_NOACK				1

#define MAC_IFS_DIFS						2

///#define BHDR_LOCK()				///arthur_lock()
///#define BHDR_UNLOCK()			///arthur_unlock()

#define BW20_ONLY	0
#define HT40_PLUS   1
#define HT40_MINUS  2

/* channel offset */
#define CH_OFFSET_20		0
#define CH_OFFSET_40		1
#define CH_OFFSET_20U		2
#define CH_OFFSET_20L		3

/* rate format encoding */
#define FMT_NO_HT   0
#define FMT_HT      1
#define FMT_HT_GF   2
#define FMT_11B     3


#define CCK_1M_CODE			0x0
#define CCK_2M_CODE			0x1
#define CCK_5_5M_CODE		0x2
#define CCK_11M_CODE		0x3
#define FMT_11B_SHORT_PREAMBLE 0x4

#define OFDM_6M_CODE		0xb
#define	OFDM_9M_CODE		0xf
#define OFDM_12M_CODE		0xa
#define OFDM_18M_CODE		0xe
#define OFDM_24M_CODE		0x9
#define OFDM_36M_CODE		0xd
#define OFDM_48M_CODE		0x8
#define OFDM_54M_CODE		0xc

#define MCS_0_CODE			0
#define MCS_1_CODE			1
#define MCS_2_CODE			2
#define MCS_3_CODE			3
#define MCS_4_CODE			4
#define MCS_5_CODE			5
#define MCS_6_CODE			6
#define MCS_7_CODE			7
#define MCS_8_CODE			8
#define MCS_9_CODE			9
#define MCS_10_CODE			10
#define MCS_11_CODE			11
#define MCS_12_CODE			12
#define MCS_13_CODE			13
#define MCS_14_CODE			14
#define MCS_15_CODE			15

#define MCS_32_CODE			32

#define B_RATE			0x1
#define G_RATE			0x2
#define HT_RATE			0x4


#define TPUT_DP_NUM			5
#define TPUT_DP_MAX			100
#define WMAC_WAIT_FOREVER(con, msg) \
	do {\
		unsigned int cnt = 0;\
		while((con)) {\
			cnt++; \
			if(info->wmac_polling == 2) \
				break; \
			else if(cnt == 0xffffffff) \
				continue; \
			else if(cnt == 0xffff) {\
				DBG(msg"?????????????System maybe enter Busy Loop????????????\n"); \
				if(info->wmac_polling == 1) break; \
			} \
		}\
		if(cnt >= 0xffff) DBG("Loop stop, cnt=%d\n", cnt);\
	}while(0)

#define WMAC_WAIT_FOREVER_AND_CNT(con, msg, _cnt) \
	do {\
		unsigned int cnt = 0;\
		while((con)) {\
			if(cnt == 0xffffffff) \
				continue; \
			cnt++; \
			if(cnt == 0xffff) {\
				DBG(msg"?????????????System maybe enter Busy Loop????????????\n"); \
			} \
		}\
		if(cnt >_cnt) _cnt = cnt; \
		if(cnt >= 0xffff) DBG("Loop stop, cnt=%d\n", cnt); \
	}while(0)

#define WMAC_WAIT_AND_SLEEP(con, msg) \
	do {\
		unsigned int cnt = 0;\
		while((con)) {\
			if(++cnt > 5) {\
				wla_sleep(1); \
				if((cnt&0xfff) == 0xfff) \
				{ \
					DBG(msg"??????waiting(%d)????????\n", cnt); \
					info->mac_recover = 1; \
					break; \
				} \
			} \
		}\
	}while(0)

struct wb_q {
	struct wbuf *head;
	struct wbuf *tail;
	unsigned int len;	
};

#define WBQ_PUT(q, wb) { \
	(wb)->wb_next = 0; \
	if((q)->tail == 0) \
		(q)->head = (wb); \
	else \
		(q)->tail->wb_next = (wb); \
	(q)->tail = wb; \
	(q)->len++; \
}

#define WBQ_GET(q, wb) { \
	(wb) = q->head; \
	if(wb) { \
		if(((q)->head = (wb)->wb_next) == 0) \
			(q)->tail = 0; \
		(wb)->wb_next = 0; \
		(q)->len--; \
	} \
}

#define WBQ_IS_EMPTY(q)		((q)->len == 0)

struct rx_q {
	rx_descriptor *desc_hdr;				/* array of static allocated rx_descriptors */
	unsigned char index;							/* next unhandled rx descriptor index */
	unsigned char max;								/* static allocated rx_descriptor count */
	unsigned int pkt_count;
#ifdef WLA_DEBUG_DUMP
	unsigned int dbg_ctrl;
#endif
};

struct tx_q {
	tx_descriptor *desc_hdr;				/* array of static allocated tx_descriptors */
	unsigned char windex;							/* next unhandled tx descriptor index */
	unsigned char rindex;							/* next unhandled tx descriptor index */
	unsigned char free;							/* free tx_descriptor count */
	unsigned char max;								/* static allocated tx_descriptor count */
	unsigned int pkt_count;
#ifdef WLA_DEBUG_DUMP
	unsigned int dbg_ctrl;
#endif
};

struct ret_q {
	tx_descriptor *desc_hdr;				/* array of static allocated tx_descriptors */
	unsigned char index;							/* next unhandled tx descriptor index */
	unsigned char max;								/* static allocated tx_descriptor count */
	unsigned int pkt_count;
};

struct beacon_desc {
	ssq_tx_descr desc;
	u32 total_len; /* software store beacon original total length */
};

enum {
	HW_FORWARD_DATA,
	SW_FORWARD_DATA,
};

enum {
	ACQ_BK_IDX,
	ACQ_BE_IDX,
	ACQ_VI_IDX,
	ACQ_VO_IDX,
};

#define PS_FLG_NON_SLEEP	BIT(0)

typedef struct {
    int sta_cap_tbl_count;                          /* STA station capability tables allocation count */
    int rate_tbl_count;

	short sta_cap_tbl_free_idx;						/* free STA station capability tables index */
	char *sta_cap_tbls_mem;
    sta_cap_tbl *sta_cap_tbls;                      /* STA station capability tables */
    sta_cap_tbl *rate_tbls;                         /* STA station capability tables next page for rate selection */

	sta_addr_entry *ext_sta_tbls;
	sta_addr_entry *ext_ds_tbls;
	short sta_tbl_count;
	short ds_tbl_count;
 
    short sw_buf_headers_count;                  /* total number of macframe headers in SW pool */
    short sw_allocated_buf_headers_count;               /* total number of macframe headers in SW pool used for RX linklist */

	short sw_rx_bhdr_start;						/* starting index of buffer header for software RX path */
	short sw_tx_bhdr_start;						/* starting index of buffer header for software TX path */
	short sw_tx_bhdr_end;						/* ending index of buffer header for software TX path */

    volatile short sw_tx_bhdr_head;                     /* index of free list head of buffer header for TX usage */
	volatile short sw_tx_bhdr_tail;                     /* index of free list head of buffer header for TX usage */
    short hw_buf_headers_count;                  /* static allocated macframe header in HW pool */
    buf_header *buf_headers;       				/* array of allocated buffer headers;  (set base address to hardware)*/
	char *bufs;

    int mmac_rx_descr_count;
    mmac_rx_descriptor *mmac_rx_descriptors;

	int psbaq_descr_count;
	psbaq_descriptor *psbaq_descriptors;
	
	int *reorder_buf_mapping_tbl;
	ampdu_reorder_buf *reorder_buf;
	bcast_qinfo *bcast_qinfos;
	sta_qinfo_tbl *sta_qinfo_tbls;

	group_cipher_key* group_keys;                         /* pointer to group/def keys array */
    cipher_key* private_keys;                       /* pointer to private/pair keys array */
	//sta_cipher_key *sta_group_keys;
 
	struct rx_q wl_rxq;
	struct rx_q eth_rxq;
	struct tx_q wl_txq;
	struct tx_q eth_txq;
	struct ret_q fp_retq;
	struct ret_q wl_sw_retq;
	//struct wb_q wl_tx_wbq;
	//struct wb_q eth_tx_wbq;

    u8 *hw_tx_buffer;                               /* 64 * 8 bytes dram buffer for tx hardware */
	u8 *fastpath_buf;

	rx_descriptor *fastpath_descriptors;
    int fastpath_bufsize;                           /* frame buffer size on fastpath */
    int sw_path_bufsize;

	int stop_ampdu_rx;		/* To protect STOP_AMPDU_RX procedure */
    int first_bssid;            /* XXX: used only in simulation code */

    /* runtime HW/SW shared data */

	volatile ssq_tx_descr *beacon_descriptors_list;
	volatile ssq_tx_descr *beacon_update_list;

	struct wsta_cb **wcb_tbls;	
	char eth_macaddr[6];
	/* configuration */
	unsigned short curr_center_freq;
	unsigned char curr_channel;
	char bandwidth_type;
	char bw40mhz_intolerant;

	int filter;	
	char wds_mode;	

	unsigned char last_psq_trigger;

	unsigned int int_mask;

	unsigned int slottime;
	unsigned long beacon_interval; /* beacon interval, in jiffies unit */
	unsigned char mac_is_ready;
	unsigned char mac_recover;
	unsigned int mac_recover_mode;
	unsigned int mac_recover_cnt;
	unsigned int mon_4013_hit;
	unsigned int mon_8094_hit;
	unsigned int mon_8000_hit;
	unsigned int mon_bmgctrl_hit;
	unsigned int mon_bufempty_hit;
	unsigned int mon_rxzombie_hit;
	unsigned int mon_4013_cnt;
	unsigned int mon_8094_cnt;
	unsigned int mon_8000_cnt;
	unsigned int mon_bmgctrl_cnt;
	unsigned int mon_bufempty_cnt;
	unsigned int mon_rxzombie_cnt;
	unsigned int mon_wronghead_cnt;
	unsigned int mon_zerodataofs_cnt;
	unsigned int mon_wrongflag_cnt;
	unsigned int mon_t_closest;
	unsigned int mon_t_current;
	unsigned int mon_t_delta;
	unsigned int mon_t_recover;
	unsigned int restore_lmac[12];
	unsigned int restore_umac[11];

	char mcast2ucast_mode;
    char power_saving_mode;
	
    int	txpowlimit;
	unsigned int txpowlevel;
 
	MGT_FUNCPTR	wla_mgt_handler;
	MGT_FUNCPTR	wla_eapol_handler;
    void (*wla_moniter_handler)(struct wbuf *);

	struct wsta_cb *sta_timeout_list;
	unsigned int hwaddr_age_time;

	//unsigned int macaddr345;
	char first_bss_desc;
	char bss_num;
	char ap_isolated;
	char bss_isolated;
	char ibss_exist;
	char ap_exist;
	char linked_ap_count;

	
	//unsigned char parent_ap_captbl_idx; /* capability table index of Client mode's AP */
	//char stamode_bss_desc;		/* station mode's bss desc */
	//struct wsta_cb *peer_ap; /* Record WDS peer APs */
	//struct wsta_cb *multicast_addr;
	struct peer_ap {
		unsigned char type;
		unsigned char sta_captbl;
		unsigned char peer_sta_count;
		unsigned char bss_desc;
	}linked_ap[MAX_PEER_AP];

	struct mbss_t{
		unsigned char role;
		unsigned char sta_count;
		unsigned char vif_id;
		unsigned char wds_mode;	

		struct tx_rate_t {
			unsigned char rate_captbl;
			unsigned char rate_idx;
		} tx_rate[3]; /* basic rate, fixed rate, multicast rate */

		struct peer_ap *ln_ap;	

#ifdef CONFIG_WLAN_CLIENT_PS
		unsigned char ps_tx_state;
		unsigned char ps_rx_state;
		unsigned char ps_flags;
		unsigned char ps_policy;
		unsigned int busy_timestamp;		/* unit: 10ms, os_current_time() */
#endif // CONFIG_WLAN_CLIENT_PS
		unsigned int wme_acm;
	}mbss[MAC_MAX_BSSIDS];

	unsigned char erp_protect;
	unsigned char non_gf_exist;

	unsigned char data_forward_mode; /* 0: ASIC forward data frame, 1: software forward data frame*/
	unsigned char txq_limit; /* legacy frame through PSBAQ to limit maximum descriptor usage */	
	unsigned char rate_adaption_policy; /* policy of dynamic rate adaption, 1: AMPDU's rate sticked during highest two rates */
	unsigned char fix_ac_queue_limit;
	
	unsigned char acq_len[4];

	unsigned int mat_timestamp;
	unsigned int acq_timestamp; 
	unsigned int bcn_mon_timestamp; 
	/* debug control */
	unsigned int beacon_timestamp;
	unsigned int umac_revsion;
	unsigned int beacon_tx_bitmap;
	unsigned int pre_tbtt_int;
	unsigned int ts0_int;
	unsigned int beacon_tx_miss;
	char w2e_rx_dump;
	char w2e_tx_dump;
	char w_rx_dump; 
	char w_tx_dump; 

	int ampdu_sn_gap;
#if defined(SW_BUF_TRACE)
	char *sw_buf_trace; /* trace sw buffer usage */
#endif
	char *tx_tbl_free_memory;
	char no_kick_reorder_buf; /* no kick reorder buffer if ampdu is pending */

	char rc_debug; /* show RC debug message */
	/* statistic */
	unsigned int sw_wrx_drop_pkt_cnt;
	unsigned int sw_w2e_unicast_pkt_cnt;
	unsigned int sw_w2e_unicast_pkt_bytes;
	unsigned int sw_w2e_bcast_pkt_cnt;
	unsigned int sw_e2w_unicast_pkt_cnt;
	unsigned int sw_e2w_unicast_pkt_bytes;
	unsigned int sw_e2w_bcast_pkt_cnt;

	unsigned int sw_wtx_no_bhdr;
	unsigned int sw_wtx_no_desc;
	unsigned int sw_wtx_bcast_pkt_waiting;

	unsigned int erx_last_sample_time;
	unsigned int bc_ps_queue_pending;

	unsigned int hw_buf_full;
	unsigned int rx_desc_full;
	unsigned int rx_fifo_full;
	unsigned int sw_retq_full;
	unsigned int eth_sw_desc_full;
	unsigned int eth_hw_desc_full;
#ifdef DEBUG_TPUT
	unsigned short tput_enable;
	unsigned short tput_w2e;			/* 1: wifi to eth, 0: eth to wifi */
	unsigned int tput_dp[TPUT_DP_NUM+1];
	unsigned int tput_dp_cnt[TPUT_DP_NUM];
	unsigned int tput_dp_arr[TPUT_DP_NUM][TPUT_DP_MAX];
#endif
	
	unsigned char rf_init_err;
#if defined(CONFIG_WMAC_RECOVER)
	struct wbuf *beacon_wb_addr[MAC_MAX_BSSIDS][3];
#endif
	char wmac_polling;
	unsigned int psbaq_cmd_waiting_max_cnt;
	
	//new add params
	unsigned int ts_status;
	unsigned int ts_ap_status;
	unsigned int long_retry;
	unsigned int short_retry;

	void *sc;
	
	/* For IPHONE patch, add cache to avoid the received frame too soon after assoc */ 
	unsigned char peer_cache[6]; 
} MAC_INFO;

typedef struct wif_vif {
	void *sc;						// upper interface
	unsigned short unit;
	unsigned short flags;
} wif_vif;

#define VIF_ACTIVE		0x1

#define IDX_TO_STACAP(_i)	&(((sta_cap_tbl *)info->sta_cap_tbls)[_i])
#define IDX_TO_RATETBL(_i)	&(((sta_cap_tbl *)info->rate_tbls)[_i])
#define STACAP_TO_IDX(_t)	((sta_cap_tbl *)_t - ((sta_cap_tbl *)(info->sta_cap_tbls)))
#define RATETBL_TO_IDX(_t)	((sta_cap_tbl *)_t - ((sta_cap_tbl *)(info->.rate_tbls)))

#define IDX_TO_ADDR_TBL(_i)	&(((sta_addr_entry *)info->ext_sta_tbls)[_i])
#define IDX_TO_DS_TBL(_i)	&(((sta_addr_entry *)info->ext_ds_tbls)[_i])

#define IDX_TO_PSBA_DESC(_i)	&(((psbaq_descriptor *)info->psbaq_descriptors)[_i])
#define IDX_TO_BCAST_QI(_i)		&(((bcast_qinfo *)info->bcast_qinfos)[_i])
#define IDX_TO_STA_QINFO(_i)	&(((sta_qinfo_tbl *)info->sta_qinfo_tbls)[_i])

#define IDX_TO_PRIVATE_KEY(_i)	&(((cipher_key *)info->private_keys)[_i])
#define IDX_TO_GROUP_KEY(_i)	&(((cipher_key *)info->group_keys)[_i])

/* address lookup options */
#define BY_ADDR_IDX			0x1
#define IN_DS_TBL			0x2
/* address lookup error code */
#define LOOKUP_ERR			(-1)
#define LOOKUP_WRONG_TBL	(-2)

#define PSBAQ_HAS_DATA	((QMODE_DATA_BIT << 7*4)|(QMODE_DATA_BIT << 6*4)| \
						 (QMODE_DATA_BIT << 5*4)|(QMODE_DATA_BIT << 4*4)| \
						 (QMODE_DATA_BIT << 3*4)|(QMODE_DATA_BIT << 2*4)| \
						 (QMODE_DATA_BIT << 1*4)|(QMODE_DATA_BIT))

#define COUNTER_DELTA(delta, prev_hw_counter, current_hw_counter) \
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

int set_mac_info_parameters(MAC_INFO* info);
int init_station_cap_tables(MAC_INFO* info);

int init_macframe_headers(MAC_INFO* info);

int init_transmit_descriptors(MAC_INFO* info);

short sta_cap_get_free(MAC_INFO *info);

void wmac_program_macaddr(MAC_INFO *info);

int wla_mac_start(MAC_INFO* info);
int wla_mac_stop(MAC_INFO* info);

void wla_mac_reset(MAC_INFO *info);
void mac_free_buffer_hdr(u32 head_index, u32 tail_index, u8 hw_buf);

int mac_addr_lookup_engine_find(u8 *addr, int addr_index, int *basic_cap, char flag);
int mac_addr_lookup_engine_update(MAC_INFO *info, u32 addr, u32 basic_cap, u8 flag);
int mac_addr_lookup_engine_flush(MAC_INFO *info);
void find_all_addrs_and_update(MAC_INFO* info, u8 captbl_idx, u32 basic_cap);

int wla_rxq(MAC_INFO *info, struct rx_q *rxq);
void wla_tx_frame(MAC_INFO *info, struct wbuf *wb, struct wbuf *wb_tx);
void wla_tx_frame_handler(MAC_INFO *info, struct wbuf *wb, struct wbuf *wb_tx);
#if defined(CONFIG_WIFI2USB)
void wframe_del_llc(struct wbuf *wb) __attribute__ ((section(".iram")));
void wla_usb_tx_frame(MAC_INFO *info, struct wbuf *wb) __attribute__ ((section(".iram")));
#endif
void wla_tx_done(MAC_INFO *info) __attribute__ ((section(".iram")));

void mac_disassociate(char sta_idx, char np);
void mac_stacap_cache_sync(int idx, char wait);
void mac_ratecap_cache_sync(int idx, char wait);
void mac_rx_linkinfo_cache_sync(void);

int bhdr_to_idx(MAC_INFO* info, buf_header *bhdr);
buf_header *bhdr_find_tail(MAC_INFO* info, buf_header *bhdr);
buf_header *idx_to_bhdr(MAC_INFO* info, int index);
buf_header* bhdr_get_first(MAC_INFO* info);
void bhdr_insert_tail(MAC_INFO* info, int head, int tail);

tx_descriptor* get_free_tx_descr(MAC_INFO* info, int blocking);
int mac_trigger_tx(void);

int disable_mac_interrupts(void);

int reset_mac_registers(void);
u8 bb_register_read(int bb_reg);
void bb_register_write(int bb_reg, u8 value);
int mac_addr_lookup_engine_insert(u8 *addr, int index, u32 basic_cap);
void mac_addr_lookup_engine_remove(u8 *addr);

void wla_set_channel(MAC_INFO *info);
char mac_get_dtim_counter(void);

ehdr *wframe_format(struct wbuf *wb, struct wlan_hdr *fm) __attribute__ ((section(".iram")));
void data_frame_forward(struct wbuf *wb, ehdr *ef, int from_bss) __attribute__ ((section(".iram")));

void rx_frame_dispatcher(struct wbuf *wb) __attribute__ ((section(".iram")));

void wla_set_bandwidth_type(MAC_INFO* info);

char wla_mac_init(MAC_INFO *info);
#if defined(CONFIG_WMAC_RECOVER)
void wla_mac_ethernet_forward_setup(u32 permit);
void wla_pre_clean(MAC_INFO *info);
char wla_mac_recover(MAC_INFO *info);
#endif
void wla_mac_clean(MAC_INFO* info);
void wla_mac_release(MAC_INFO* info);

/* QoS */
void wla_ac_paramters(int ac, int cw_min, int cw_max, int aifs, int txop);

/* beacon */
#if defined(CONFIG_WMAC_RECOVER)
void save_beacon_list(MAC_INFO* info, struct beacon_desc *b_desc);
void update_beacon_list(MAC_INFO* info);
#endif
void wla_beacon(MAC_INFO *info);
void wla_beacon_setup(unsigned short beacon_interval, unsigned char dtim_period);
char wla_add_beacon(MAC_INFO *info, struct wbuf *wb1,  struct wbuf *wb2,  struct wbuf *wb3);
void wla_beacon_start(MAC_INFO* info);
void wla_beacon_stop(MAC_INFO* info);
void wla_beacon_flush(MAC_INFO* info);
void wla_beacon_interrupt_setup(MAC_INFO *info, u32 bss_idx, u32 role, u32 en);
#ifdef TBTT_EXCEPTION_HANDLE
void wla_update_next_tbtt(MAC_INFO *info, u32 intr_mask);
#endif  // TBTT_EXCEPTION_HANDLE
#ifdef CONFIG_WLAN_CLIENT_PS
void client_tbtt_handle(MAC_INFO *info, u32 bss_desc);
#endif // CONFIG_WLAN_CLIENT_PS

/* rate adaption */
void wla_rc_init(MAC_INFO *info, struct wsta_cb *cb);
int check_spec_rate(MAC_INFO *info, struct wsta_cb *cb);
int wla_rc_update(MAC_INFO *info, struct wsta_cb *cb);
unsigned char get_hw_bitrate_format(int tx_rate_idx);
unsigned char get_hw_bitrate_code(int tx_rate_idx);
unsigned int get_hw_bitrate(int tx_rate_idx);
void wla_setup_tx_rate(MAC_INFO *info, short bss, int rate_index, char type, int flags);
unsigned char primary_ch_offset(u8 bw_type);
short wla_rate_encode(MAC_INFO* mac_info, unsigned char tx_rate_idx, unsigned char retry, 
		unsigned char flags);

/* serurity */
inline void mac_set_wep_group_key(cipher_key *hwkey, char *key, char cipher_type, char keyidx, char is_txkey);
inline void mac_set_wep_key(cipher_key *hwkey, char *key, char cipher_type, char keyidx, char is_txkey);
inline void mac_set_tkip_group_key(cipher_key *hwkey, char *key, char keyidx, char is_txkey);
inline void mac_set_tkip_key(cipher_key *hwkey, char *key);
inline void mac_set_ccmp_group_key(cipher_key *hwkey, char *key, char keyidx, char is_txkey);
inline void mac_set_ccmp_key(cipher_key *hwkey, char *key);

void mac_rekey_start(int index, int key_type, char cipher_type, int rsc_init_value);
void mac_rekey_done(void);
void mac_set_key(cipher_key *hwkey, char *key, char cipher_type, char keyidx, char is_txkey);

/* reordering buffer */
void mac_invalidate_ampdu_scoreboard_cache(int sta_index, int tid);
char mac_flush_all_ampdu_reorder_buf(MAC_INFO *info, int sta_index, int tid);
void mac_flush_one_ampdu_reorder_buf(int sta_index, int tid);
int mac_attach_ampdu_reorder_buf(MAC_INFO *info, int sta_index, int tid, u16 ssn, u16 bufszie);
int mac_detach_ampdu_reorder_buf(MAC_INFO* info, int sta_index, int tid);
int psbaq_set_tid_delivery_status(MAC_INFO* info, int index, int tid, char delivery_enable);

void mac_detach_psbaq_ucast_qinfo(MAC_INFO* info, int sta_index);
void psbaq_invalidate(u8 bssid, u8 captbl_idx);

void reorder_buf_release(struct rx_ba_session *ba, u16 new_start, u8 in_order);
void reorder_buf_ctrl(MAC_INFO *info, struct wbuf *wb, u16 seq, u8 tid);

unsigned int psbaq_all_status(unsigned char captbl_idx);
unsigned char psbaq_qmode(unsigned char captbl_idx, unsigned char tid);
char psbaq_qmode_is_busy(MAC_INFO *info, unsigned char captbl_idx, unsigned char tid);
struct wsta_cb *update_ps_state(struct wbuf *wb, char ps_poll);

char ds_table(char mode);
struct peer_ap *new_peer_ap(MAC_INFO *info, char type);

void ethernet_parse_and_forward(MAC_INFO *info, struct wbuf *wb, char from_bss);
//mac register

#ifdef WLA_MULTICAST_PATCH
void wla_add_multicast_addr_tbl(int bss_idx);
#endif
#endif // __MAC_INFO_H__

