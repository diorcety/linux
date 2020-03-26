/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_api.h
*   \brief	define wla control command
*   \author Montage
*/

#ifndef _WLA_API_H_
#define _WLA_API_H_

enum
{
	SET_BSS = 1,
	SET_CHANNEL,
	SET_TX_PARAMS,	
	SET_STA_CAP,
	//SET_CIPHER_KEY,
	//SET_CIPHER_KEY_INDEX,
	//DEL_CIPHER_KEY,
	SET_FRAG_THRESHOLD,
	SET_RTS_THRESHOLD,
	SET_BEACON_PARAMS,
	SET_TX_RATE,
	DEL_TX_RATE,
	SET_WDS_MODE,
	SET_RX_FILTER,
	DEL_RX_FILTER,
	SET_STA_HW_PATH,
	SET_AP_ISOLATED,
	SET_TX_MCAST2UNI,
	SET_POWER_SAVING,
	SET_SLOTTIME,
	SET_40MHZ_INTOLERANT,
	SET_FORWARD_MODE,
	SET_TXQ_LIMIT,
	SET_ACQ_LEN,
	SET_RATE_ADAPTION,
	SET_WME_ACM,
};

#define RETURN_ERR	(-1)
#define RETURN_OK	(0)
#define RETURN_SKIP	(1)

enum {
	BASIC_TX_RATE,
	FIXED_TX_RATE,
	MULTICAST_TX_RATE,
};


enum {
	BW40MHZ_SCN = 0,	/* no secondary channel is present */
	BW40MHZ_SCA = 1,	/* secondary channel is above the primary channel */
	BW40MHZ_SCB = 3,	/* secondary channel is below the primary channel */
	BW40MHZ_AUTO = 4,	/* auto select secondary channel */
};

/* commands of wla_cfg_ampdu */
enum {
	START_AMPDU_RX,
	STOP_AMPDU_RX,
	START_AMPDU_TX,
	STOP_AMPDU_TX,
	GET_AMPDU_RX_STATUS,
	GET_AMPDU_TX_STATUS,
};

/* commands of wla_cfg_forward */
enum {
	DATA_NO_FORWARD,
	DATA_CAN_FORWARD,
};

enum {
	HT_NO_PROTECT_MODE = 0,
	HT_NON_MEMBER_PROTECT_MODE,
	HT_20MHZ_PROTECT_MODE,
	HT_NON_HT_MIXED_PROTECT_MODE,
	BG_PROTECT_MODE,	
};

#if 0
enum {
	STA_MODE = 0,
	WDS_AP_MODE,
	BSS_AP_MODE,
	IBSS_MODE,
};
#endif
enum {
	KEY_TYPE_PAIRWISE_KEY = 0,
	KEY_TYPE_STA_PAIRWISE_KEY,
	KEY_TYPE_GLOBAL_KEY,
	KEY_TYPE_STA_GLOBAL_KEY,	
};

/* WDS mode */
enum {
	AP_WDS_NONE = 0,					
	AP_WDS_BRIDGE,	
	AP_WDS_REPEATER,
};

enum {
	SLOTTIME_9US = 9,
	SLOTTIME_20US = 20,
};

#define RX_FILTER_MGMT_FRAME		0x1
#define RX_FILTER_CTRL_FRAME		0x2
#define RX_FILTER_DATA_FRAME		0x4
#define	RX_FILTER_BEACON			0x8
#define RX_FILTER_PROBE_RESP		0x10
#define RX_FILTER_PROBE_REQ			0x20


#ifndef WIF_NONE
#define	WIF_NONE				0
#define WIF_STA_ROLE			0x01
#define WIF_AP_ROLE				0x02
#define WIF_IBSS_ROLE			0x03
#define WIF_P2P_CLIENT_ROLE		0x9
#define WIF_P2P_GO_ROLE			0xa
#define WIF_MAT 				0x100
#endif

#if 0
enum {
	WIF_NONE = 0,
	WIF_STA_ROLE = 0x01,
	WIF_AP_ROLE = 0x02,
	WIF_IBSS_ROLE = 0x3,

	WIF_P2P_CLIENT_ROLE = 0x9,
	WIF_P2P_GO_ROLE = 0xa,

	WIF_MAT = 0x100,
};
#endif
/* linked AP type */
enum {
	LINKED_AP = 1,
	MAT_LINKED_AP,	
	WDS_LINKED_AP,
	IBSS_STA,
};

/* bb rx counter ctrl */
enum {
	BB_RX_CNT_RESET,
	BB_RX_CNT_ENABLE,
	BB_RX_CNT_DISABLE,
};

#define	BB_RX_CHANNEL_CNT_ADDR	0x20
#define	BB_RX_BUSY_CNT_ADDR		0x23

void bb_rx_counter_ctrl(u32 op);
u32 bb_rx_counter_read(u8 addr);
u8 bb_read_noise_floor(void);


typedef void (*MGT_FUNCPTR)(struct wbuf *, void *);


int wla_cfg(unsigned int cmd, ...);
/*FIXME:Not used*/
//extern void wla_register_mgt_handler(MGT_FUNCPTR func);

struct wsta_cb *wla_get_wcb(void);
char wla_get_sta(char *sta_addr, char bss_idx, struct wsta_cb *wcb, char mode);
//New added on linux
char wla_chg_sta(char *sta_addr, char bss_idx, struct wsta_cb *wcb, char mode);
int	wla_release_sta(struct wsta_cb *cb);
int wla_post_release_sta(struct wsta_cb *wcb);

void wla_add_wif(u32 idx, u8 role, u8 vif_id);
void wla_set_ap_isolated(u32 isolated);
void wla_set_bss_isolated(u32 isolated);
void wla_start_tsync(u32 bss_idx, u32 beacon_intval, u32 dtimi_intval, u32 timeout);
void wla_set_beacon_interrupt(u32 bss_idx, u32 role, u32 en);

enum{
	NO_PROTECTION = 0,
	GF_PROTECTION = 1,
	OFDM_PROTECTION,
};
void wla_set_protection(u8 type);
#if defined(CONFIG_WMAC_RECOVER)
void wla_notify_no_ampdu(void);
void wla_recover(void);
int wla_monitor_check(unsigned int val, unsigned int *cnt, unsigned int *cnt1, char *msg);
void wla_rx_monitor(unsigned int *cnt, char *msg);
int wla_tx_monitor(void);
#endif
void wla_set_recovery(u32 recovery);
int wla_ampdu(u32 cmd, struct wsta_cb *wcb, u8 tid, u16 ssn, u16 bufsize);
void wla_forward(u32 cmd, struct wsta_cb *wcb);
void wla_tx_power(u32 level);
void wla_cipher_key(struct wsta_cb *wcb, unsigned char cipher_type, unsigned char key_type,
						char *key, char keyidx, char is_txkey);
void wla_ac_paramters(int ac, int cw_max, int cw_min, int aifs, int txop);
u32 wla_update_sta_status(struct wsta_cb *wcb, u32 current_time);
u32 wla_get_ssn(struct wsta_cb *wcb, u8 tid);
void wla_wds_bss_isolated(void);
u32 wla_get_cca(void);
void wla_moniter(void);
void wla_rf_reg(u8 num, u32 val);
void wla_rf_txvga_reg(u8 ch, u32 val);
u32 wla_get_rf_txvga_reg(u8 ch);
void wla_rf_freq_ofs_reg(u32 val);
u32 wla_get_rf_freq_ofs_reg(void);
void wla_bb_reg(char *rfc_data, u32 len);
u16 wla_sta_forward_path(char *addr);
u8 wla_mac_status(void);
void wla_set_recover(u8 val);
u32 wla_read_beacon_loss_count(u32 bss_idx);
void wla_mgt_send(struct wbuf *wb);
char wla_put_beacon(struct wbuf *wb1, struct wbuf *wb2, struct wbuf *wb3);
void wla_hw_sync(void);
void wla_add_sta_timer(struct wsta_cb *new, unsigned int timeout);
void wla_del_sta_timer(struct wsta_cb *wcb);
void wla_set_timer(void);

#endif
