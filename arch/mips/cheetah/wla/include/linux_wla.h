/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file linux_wla.h
*   \brief  Linux WLA header.
*   \author Montage
*/
/*=============================================================================+
| Header                                                                       |
+=============================================================================*/

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#if defined(CONFIG_CHEETAH_FPGA)
#define CONFIG_FPGA
#endif

#define CHAN2G(_freq)  { \
       .band = IEEE80211_BAND_2GHZ, \
       .center_freq = (_freq), \
       .hw_value = (_freq), \
       .max_power = 20, \
}

#define CHAN5G(_freq) { \
       .band = IEEE80211_BAND_5GHZ, \
       .center_freq = (_freq), \
       .hw_value = (_freq), \
       .max_power = 20, \
}

#define MAX_2GHZ_CHANNELS				16
#define MAX_5GHZ_CHANNELS				32
#define WLA_RATE_MAX					32
#define WDS_TABLE_ENTRIES				4
#define MAX_TX_POWER					15

#define VALID   1

#define LEGACY_RATE 0

/* bandwidth */
#define BW_20MHZ    0
#define BW_40MHZ    1

/* channel offset */
#define CH_OFFSET_20        0
#define CH_OFFSET_40        1
#define CH_OFFSET_20U       2
#define CH_OFFSET_20L       3
/* format encoding */
#define FMT_NO_HT   0
#define FMT_HT      1
#define FMT_HT_GF   2
#define FMT_11B     3

/* rate encoding */
#define _11B_1M     0x0
#define _11B_2M     0x1
#define _11B_5_5M   0x2
#define _11B_11M    0x3
#define _11B_SHORT_PREAMBLE     0x04

#define _OFDM_6M    0x0b
#define _OFDM_9M    0x0f
#define _OFDM_12M   0x0a
#define _OFDM_18M   0x0e
#define _OFDM_24M   0x09
#define _OFDM_36M   0x0d
#define _OFDM_48M   0x08
#define _OFDM_54M   0x0c

#define SHA1_MAC_LEN 20
#define PMK_LEN             32
#define MAX_WEP_KEYLEN  13
#define MAX_WAPI_KEYLEN 32
#define MAX_WEP_KEYS    4
#define WDS_MAX_CIPHER_KEY_LENGTH   ( MAX_WEP_KEYS * MAX_WEP_KEYLEN )

#define SKB_WB_OFFSET	4
#define HEADROOM_LEAST	SKB_WB_OFFSET + 24

#define USE_TASKLET
//#define USE_NAPI
//#define USE_WORKQUEUE

/*RC use workqueue if undef then use timer*/
#define USE_WQ_RC

#define HW_MAP(_name, _handle, _reg, _val)   \
{   \
    .name       = (_name),      \
	.handler    = (_handle),    \
	.reg		= (_reg),    \
    .val		= (_val),    \
}

/*=============================================================================+
| Struct                                                                       |
+=============================================================================*/
struct wds_key
{
     u8  cipher_type;
     u8  txkeyid;
     u8  keylen[MAX_WEP_KEYS];
     u8  key_data[WDS_MAX_CIPHER_KEY_LENGTH];
     u8  *key[MAX_WEP_KEYS];
};

struct wds_peer {
    u8  cipher;
    u8  key_idx;
    u8  key[32];
    u8 *addr[WLAN_ADDR_LEN];
    struct wsta_cb *wcb;
    struct wds_peer *next;
};

#ifdef CONFIG_CHEETAH_SMART_CONFIG

#define SMART_BASELEN	200

struct smartcfg {
	struct wsta_cb *wcb; //
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
    struct wsta_cb *wcb_ap; // 
    char aplist_stop;
#endif
	u8 bss_desc;
	u8 state;
    u8 decoder;
	u8 fixchan;			//	fixed chan section
    u8 channel_index;
	u8 cur_chan;		//	current channel
    u8 num_of_channel;
	u8 channel_below_first;	//	channel bellow or above
	u8 swupdwn;			//	need to above/below chan
	u8 macmatch;		//  mac match bit field
	u32 datalen;		//	ssid len
    u32 datacnt;                        
	u32 offset;			//	actually len = pktlen - offset
	
	u8 ta[6];

    u32 valid_pkt_count;
    u32 valid_pkt_count_prev;
    u32 signature_count;
    u32 signature_count_prev;
	unsigned long next_decode_status_check_time;

    u8 black_list_count;
    u8 black_list_channel_num;

	unsigned long total;
	unsigned long chantime;
};

enum {
	SMART_INIT = 0,
	SMART_REMAINCHAN,
	SMART_DECODE,
	SMART_DONE,
	SMART_USRSTOP,
};

enum {
	SMART_SSID = 0,
	SMART_PASS,
};
#endif

struct rf_reg {
	unsigned int down;
	unsigned int ospd;
	unsigned int uart;
	unsigned int reg0;
	unsigned int reg6;
	unsigned int reg15;
	unsigned int reg17;
};

// wla_softc
struct wla_softc {
	MAC_INFO *mac_info;
    struct list_head list;
    struct ieee80211_hw *hw; //keep
    struct device *dev;

    struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];
    struct ieee80211_channel channels_2ghz[MAX_2GHZ_CHANNELS];
    struct ieee80211_channel channels_5ghz[MAX_5GHZ_CHANNELS];
    struct ieee80211_rate rates[IEEE80211_NUM_BANDS][WLA_RATE_MAX];
    u32 cur_basic_rates;

    struct ieee80211_channel channel;
    enum nl80211_channel_type channel_type;
    u16 curr_center_freq;
    enum ieee80211_band curr_band;
	u32 primary_ch_offset;               /* channel offset setting for sending on 20Mhz-primary channel */

    unsigned long beacon_int; /* in jiffies unit */
    unsigned int rx_filter;

    int started;		//keep

    u8 dtim_period;

#if defined(ENABLE_POWER_SAVING)
    u8 aid0_tim;
#endif

    struct mutex mutex;
    int	txpowlimit;
#if defined	USE_TASKLET
    struct tasklet_struct irq_tasklet;
#elif defined USE_WORKQUEUE
    struct workqueue_struct *irq_wq;
    struct work_struct irq_work;
#elif defined USE_NAPI
	struct net_device napi_dev;
	struct napi_struct napi;
#endif
    struct tasklet_struct beacon_tasklet;

    int sta_if_count;               /* number of interfaces as a STA */
    int sta_mode_ap_index;

#if defined(FASTPATH_MULTICAST_FORWARD)
    int pending_add_mcast_sta_req;  // pending add mcast station request
    u8  mcast_sta_macaddr[ETH_ALEN];
    int mcast_sta_bssid_idx;
#endif
	//Need to refine
	//WDS
	int wds_mode;
	struct wds_peer *all_wds_peer;

	struct delayed_work hw_sync_work;
#ifdef CONFIG_CHEETAH_SMART_CONFIG	
	struct delayed_work smartcfg_work;
	struct delayed_work smartcfg_usrwork;
	struct smartcfg smrtcfg;
#endif
	u32 tx_fail_times;
	spinlock_t lock;
	unsigned int lock_counter;
	struct net_device *bssdev[4];
	struct survey_info survey[11];
	unsigned int ap_exist;
	struct rf_reg rfreg;
	struct seq_file *seq_file;
};

struct wla_vif_priv {
	u8 bcn_threshold;
	u16 aid;
	char role;
	char bss_desc;
	char cipher_type;
	char unit;
	char *name;
};

struct wla_sta_priv
{
    struct wsta_cb *wcb;
    bool assoc;
    bool sta_mode;
};

struct hw_setup {
    const char *name;
    void (*handler)(unsigned int reg, unsigned int val);
	char *reg;
	char *val;
};

//rate
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
    };
    u16     val;
} __attribute__((__packed__)) rate_config;

struct wla_rate_info
{
    u32 ratekbps;
    u32 hw_basic_rate_bitmap;

    u32 valid:1;

    u32 mandatory:1;        /* mandatory rate A/B/G */
    u32 short_preamble:1;   /* short preamble */
    u32 sgi:1;              /* short guard interval */
    u32 ht:1;               /* high throughput; HT, legacy(CCK and non_HT_OFDM) */
    u32 bw:1;
    u32 ch_offset:2;
    u32 format:2;
    u32 tx_rate:7;          /* HT: MCS rate index, legacy: R */
};

struct wla_rate_table {
    int rate_cnt;

    struct wla_rate_info info[32];
};

__attribute__((used))
static struct wla_rate_table wla_11a_ratetable = {
    8,
    {
        { 6000,  0x010, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_6M, },
        { 9000,  0x020, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_9M, },
        { 12000, 0x040, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_12M, },
        { 18000, 0x080, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_18M, },
        { 24000, 0x100, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_24M, },
        { 36000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_36M, },
        { 48000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_48M, },
        { 54000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_54M, },
    },
};

__attribute__((used))
static struct wla_rate_table wla_11g_ratetable = {
    12,
    {
        { 1000,  0x001, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_11B, _11B_1M, },
        { 2000,  0x002, VALID, 1, 1, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_11B, _11B_2M, },
        { 5500,  0x004, VALID, 1, 1, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_11B, _11B_5_5M, },
        { 11000, 0x008, VALID, 1, 1, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_11B, _11B_11M, },
        { 6000,  0x010, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_6M, },
        { 9000,  0x020, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_9M, },
        { 12000, 0x040, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_12M, },
        { 18000, 0x080, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_18M, },
        { 24000, 0x100, VALID, 1, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_24M, },
        { 36000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_36M, },
        { 48000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_48M, },
        { 54000, 0x000, VALID, 0, 0, 0, LEGACY_RATE, BW_20MHZ, 0, FMT_NO_HT, _OFDM_54M, },
    },
};

__attribute__((used))
static struct ieee80211_channel wla_channels_2ghz[] = {
    CHAN2G(2412), /* Channel 1 */
    CHAN2G(2417), /* Channel 2 */
    CHAN2G(2422), /* Channel 3 */
    CHAN2G(2427), /* Channel 4 */
    CHAN2G(2432), /* Channel 5 */
    CHAN2G(2437), /* Channel 6 */
    CHAN2G(2442), /* Channel 7 */
    CHAN2G(2447), /* Channel 8 */
    CHAN2G(2452), /* Channel 9 */
    CHAN2G(2457), /* Channel 10 */
    CHAN2G(2462), /* Channel 11 */
    CHAN2G(2467), /* Channel 12 */
    CHAN2G(2472), /* Channel 13 */
    CHAN2G(2484), /* Channel 14 */
};

__attribute__((used))
static struct ieee80211_channel wla_channels_5ghz[] = {
    CHAN5G(5180), /* Channel 36 */
    CHAN5G(5200), /* Channel 40 */
    CHAN5G(5220), /* Channel 44 */
    CHAN5G(5240), /* Channel 48 */

    CHAN5G(5260), /* Channel 52 */
    CHAN5G(5280), /* Channel 56 */
    CHAN5G(5300), /* Channel 60 */
    CHAN5G(5320), /* Channel 64 */

    CHAN5G(5500), /* Channel 100 */
    CHAN5G(5520), /* Channel 104 */
    CHAN5G(5540), /* Channel 108 */
    CHAN5G(5560), /* Channel 112 */
    CHAN5G(5580), /* Channel 116 */
    CHAN5G(5600), /* Channel 120 */
    CHAN5G(5620), /* Channel 124 */
    CHAN5G(5640), /* Channel 128 */
    CHAN5G(5660), /* Channel 132 */
    CHAN5G(5680), /* Channel 136 */
    CHAN5G(5700), /* Channel 140 */

    CHAN5G(5745), /* Channel 149 */
    CHAN5G(5765), /* Channel 153 */
    CHAN5G(5785), /* Channel 157 */
    CHAN5G(5805), /* Channel 161 */
    CHAN5G(5825), /* Channel 165 */
};

/*=============================================================================+
| Function                                                                     |
+=============================================================================*/
/*resource*/
void wla_wds_setup(struct wla_softc *sc);
void wla_free_sc(struct wla_softc *sc);
struct wbuf *wla_skb_to_wb(struct sk_buff **skb);
int wla_beacon_get(struct ieee80211_vif *vif, u8 dtim);
void modify_macaddr(struct ieee80211_vif *vif, char *macaddr);
netdev_tx_t wla_data_send(struct sk_buff *skb, struct net_device *dev);
int wla_cfg_station(struct wla_softc *sc, struct ieee80211_sta *sta, struct ieee80211_vif *vif, enum sta_notify_cmd cmd);
void wla_hal_cipher_key(struct ieee80211_sta *sta, struct ieee80211_vif *vif, bool add);
void wla_remove_station(struct wla_softc *sc, struct ieee80211_sta *sta, struct ieee80211_vif *vif);
unsigned char wla_key_chg(struct ieee80211_vif *vif, struct ieee80211_key_conf *key);
void wla_chg_tx_rate(short bss, int rate_index, char type);
unsigned char wla_cipher_chg(struct ieee80211_key_conf *key);
unsigned char wla_transfer_channel_num(unsigned short freq);
int wla_debug_init(void *wla_sc);
int wla_procfs_init(struct wla_softc *wla_sc);
int wla_test_init(void);
void wla_procfs_exit(void);
void wla_debug_exit(void);
void wla_test_exit(void);
void wla_hw_setup(void);
void wla_get_hw_mac(unsigned short idx, unsigned char *mac);
void hw_cdb_get(const char *name, unsigned char *cdb);
void hw_cdb_set(const char *name, const char *cdb);

void wla_beacon_t(struct wla_softc *sc);
int wla_kthread_init(struct wla_softc *sc);

void wla_cfg80211_ops(struct ieee80211_hw *hw);
void wla_update_survey_stats(struct wla_softc *sc, char used);
void wla_acs_reset(struct ieee80211_hw *hw);

void initkernelenv(void);
void dinitkernelenv(void);
struct file *openfile(char *path,int flag,int mode);
int writefile(struct file *fp,char *buf,int writelen); 
int readfile(struct file *fp, char *buf, int readlen);
int closefile(struct file *fp);

extern unsigned int __hw_mode;
extern unsigned int __tx_power;
extern unsigned int __wl_power_backoff;
extern unsigned int __rfchip;
extern unsigned int __ex_ldo;
extern unsigned int __sw_forward;
extern unsigned int __ap_isolated;
extern unsigned int __bss_isolated;
extern unsigned int __wds_mode;
extern unsigned int __sta_mat;
extern unsigned int __pw_save;
extern unsigned int __no_ampdu;
extern unsigned int __ampdu_tx_mask;
extern unsigned int __apid;
extern unsigned int __staid;
extern unsigned int __fix_txrate;
extern unsigned int __rate_flags;
extern unsigned int __recovery;
extern u64 __wds_peer_addr[WDS_TABLE_ENTRIES];
extern u32 __cipher_type;
extern u32 __wds_keyid;
extern u32 __uapsd_enable;
extern struct wds_key __wds_key[WDS_TABLE_ENTRIES];
#ifdef CONFIG_CHEETAH_SMART_CONFIG
extern u32 __smartcfg_enable;
extern u32 __smartcfg_dbgmask;
#endif

int wla_cdb_get_int(unsigned short index, int def_val);
unsigned int os_current_time(void);

void wm_ba_event(u8 cmd, u8 tid, u32 sta);

//int wla_forward_to_ethernet(struct wbuf *wb, ehdr *ef);
struct wbuf *copy_bhdr_to_wb(MAC_INFO* info, unsigned short head, unsigned short tail);
void wla_mgmt_process(struct wbuf *wb, void *wcb);
void wla_spin_lock(void);
void wla_spin_unlock(void);
void wla_spin_lock1(char *p);
void wla_spin_unlock1(char *p);

void wla_software_intr(int *parm);
void wla_hw_sync_work(struct work_struct *work);
#ifdef CONFIG_CHEETAH_SMART_CONFIG
void wla_chgchan_inform(struct work_struct *work);
void smart_inform_user(struct work_struct *work);
#ifdef CONFIG_CHEETAH_SMART_TO_USRSPACE 
int smart_usr_inform(struct wbuf *wb, char *ptr);
void wla_chgchan_fixed(struct smartcfg *cfg, int chan);
void smrtcfg_clear_aplist(char flag);
#endif
//void wla_chgchan_inform(unsigned long data);
void smart_start(struct smartcfg *cfg, int chansec);
void smart_stop(struct smartcfg *cfg);
#endif

extern u8 bb_rssi_decode(u8 val);
extern u8 bb_rssi_decode1(u8 val);
//extern struct sta_info *sta_info_get_by_idx(struct ieee80211_local *local, int idx,struct net_device *dev);
//extern struct sta_info *sta_info_get(struct ieee80211_local *local, const u8 *addr);
