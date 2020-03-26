#ifdef CONFIG_CHEETAH_SMART_CONFIG

//#define MIMO_DEBUG
#include "smartcfg.h"

#if !defined(USR_SPACE_TEST)
/*=============================================================================+
|                                                                              |
| Copyright 2015                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
#include <os_compat.h>
#include <linux_wla.h>
#include <mac_common.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <asm/mach-cheetah/common.h>
#include <wla_debug.h>
#include <rf.h>
#include <wla_util.h>
#include <net/regulatory.h>
#include <net/cfg80211.h>

#define SC_DBG(...) printk(KERN_NOTICE __VA_ARGS__)

#define KEEP_ADDR_LOOKUP_DATA

#define CHANNEL_HOPPING

#define CHANNEL_BLACKLIST

#define ENABLE_40MHZ_CHANNEL

#define ENABLE_MAC_RECOVERY

#define DECODE_AP_FORWARD_DATA

#define PLAY_OMNI_VOICE

extern MAC_INFO *info;
u32 __smartcfg_enable = 0;
u32 __smartcfg_dbgmask = 0;
u32 __smartcfg_filter_mode = 0;
u32 __smartcfg_voice_type = 0;
u8 *cfgdata = NULL;

static unsigned short curr_center_freq;
static unsigned char curr_channel;
static char bandwidth_type;
#define MAX_CFGDATA_SIZE 256
unsigned long cfgdata_valid[MAX_CFGDATA_SIZE / sizeof(unsigned long)];

#define SMRTCFG			"/lib/wdk/omnicfg"
#define SMRT_PATH		"PATH=/usr/sbin:/usr/bin:/sbin:/bin:/lib/wdk"
#define SMRTDONE		"done"
#define SMRUSRSTOP		"state"
#define SMRTVOICE       "voice"

#if defined(CONFIG_CHEETAH_FPGA)
#define CHGCHANDELAY                            (HZ)
#else
#define CHGCHANDELAY                            (HZ / 2)
#endif
#define SMART_DECODE_STATUS_CHECK_INTERVAL      (CHGCHANDELAY * 5)

#define MIMO_LOCK_ATTEMPT_TIME (HZ)

enum {
	S_SCAN_CHANNEL = 0,
	S_STAY_CHANNEL,
	S_DECODE,
    S_DONE,
	S_USRSTOP,
};

#ifdef PLAY_OMNI_VOICE
enum {
    S_VOICE_NONE = 0,
    S_VOICE_CFG,
};
#endif

#define SISO_DECODER 0
#define MIMO_DECODER 1

#define DISPATCH_SISO_SCAN      0x01
#define DISPATCH_SISO_DECODE    0x02
#define DISPATCH_MIMO_SCAN      0x04
#define DISPATCH_MIMO_DECODE    0x08

#define FILTER_MODE_DROP_ALL        0
#define FILTER_SISO_SCAN    DISPATCH_SISO_SCAN
#define FILTER_SISO_DECODE  DISPATCH_SISO_DECODE
#define FILTER_MIMO_SCAN    DISPATCH_MIMO_SCAN
#define FILTER_MIMO_DECODE  DISPATCH_MIMO_DECODE

#define SIGNATURE_NOT_FOUND         0       // no signature found
#define STAY_IN_THE_CHANNEL         1       // request to stay in the channel for futhur seeking
#define SIGNATURE_LOCKED            2       // found a candidate station

#define DECODE_FAILED               4
#define DECODE_IN_PROC              5
#define DECODE_DONE                 6

#define SIGNATURE_MAC_COUNT 2

#define SISO2_SAFE_BASE 10
#define SISO2_SAFE_RANGE 190
u8 smrt_chan_multicast_addr_list[SIGNATURE_MAC_COUNT][6] = {
    {0x01, 0x00, 0x5E, 0x63, 0x68, 0x6E},
    {0x01, 0x00, 0x5E, 0x6E, 0x65, 0x6C},
};

u8 smrt_cfgdata_addr[6] = {0x01, 0x00, 0x5E, 0x73, 0x69, 0x00};
u8 smrt_cfgdata_addr_ipv4[3] = {0x01, 0x00, 0x5E};
u8 smrt_cfgdata_addr_ipv6[3] = {0x33, 0x33, 0x66};
#if 1
#define SMRTCFG_DEBUG_FLAG_OPEN 0x01
#define SMRTCFG_DEBUG_FLAG_CLOSE 0x00
u32 smrtcfg_debug_flag = 0x00;
#endif 
#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
u32 sniffer_rx_count;
u32 last_sniffer_rx_count;
unsigned long next_rx_count_check_time;
#define RX_COUNT_CHECK_INTERVAL     (15 * HZ)
#endif

static inline int is_cfgdata_addr(unsigned char *mac)
{
    return ((smrt_cfgdata_addr[0]==mac[0])
            &&(smrt_cfgdata_addr[1]==mac[1])
            &&(smrt_cfgdata_addr[2]==mac[2])
            &&(smrt_cfgdata_addr[3]==mac[3])
            &&(smrt_cfgdata_addr[4]==mac[4]));
}

static inline int is_cfgdata_addr_ipv4(unsigned char *mac)
{
    return ((smrt_cfgdata_addr_ipv4[0]==mac[0])
            &&(smrt_cfgdata_addr_ipv4[1]==mac[1])
            &&(smrt_cfgdata_addr_ipv4[2]==mac[2])
            );
}
static inline int is_cfgdata_addr_ipv6(unsigned char *mac)
{
    return ((smrt_cfgdata_addr_ipv6[0]==mac[0])
            &&(smrt_cfgdata_addr_ipv6[1]==mac[1])
            &&(smrt_cfgdata_addr_ipv6[2]==mac[2])
            );
}
static inline unsigned char get_cfgdata_idx(unsigned char *mac)
{
    return mac[5];
}

#if defined(DECODE_AP_FORWARD_DATA)

int ds_captbl_idx;
static void smart_get_sta(char *sta_addr, char bss_desc, struct wsta_cb *wcb, char mode)
{
    sta_cap_tbl* captbl;
    int index, i;
    int addr_index;
    sta_basic_info bcap;
    sta_tx_status_tbl *tx_tbls;
    sta_rx_status_tbl *rx_tbls;
    ucast_qinfo *qinfos;
    struct peer_ap *ap;

    /* always allocate QOS table size, even for non-QOS STA */
    int tx_tbl_num = QOS_TX_STATUS_TBLS;
    int rx_tbl_num = QOS_RX_STATUS_TBLS;
    int mpdu_win_num = DEFAULT_LEGACY_RX_MPDU_WINDOW;

    if (!wcb)
    {
        panic("forget to assign wcb!!\n");
    }

    bcap.val = 0;
    /* find duplicated address and release it */
    index = mac_addr_lookup_engine_find(sta_addr, (int)&addr_index, 0, mode);
    if (index >= 0)
    {
        WLA_DBG(WLADEBUG, "duplicated addr and link to captbl[%d], release it!\n", index);
        mac_addr_lookup_engine_update(info, addr_index, 0, BY_ADDR_IDX);
    }

    if (0 > (index = sta_cap_get_free(info)))
        return;

    ds_captbl_idx = index;
    SC_DBG("Use captbl %d for sniffer\n", ds_captbl_idx);

    captbl = &info->sta_cap_tbls[index];

    captbl->BG = 1;
    captbl->gsn = 1;
    captbl->QOS = 0;
    captbl->nsd = 1;    // not sounding

    if (wcb->qos)
    {
        captbl->NE = 1;
        captbl->gsn = 0;
        captbl->QOS = 1;

        tx_tbl_num = QOS_TX_STATUS_TBLS;
        rx_tbl_num = QOS_RX_STATUS_TBLS;
        mpdu_win_num = DEFAULT_QOS_RX_MPDU_WINDOW;
    }

    /*	
        | tx_tbl |
        | rx_tbl |
        | tx_baps_qinfos |
    */
#ifdef CONFIG_SRAM_WLA
    tx_tbls = &all_tx_tbls[index*QOS_TX_STATUS_TBLS];
    rx_tbls = &all_rx_tbls[index*QOS_RX_STATUS_TBLS];
    qinfos =  &all_qinfos[index*QOS_TX_STATUS_TBLS];

    memset(tx_tbls, 0, sizeof(sta_tx_status_tbl)*QOS_TX_STATUS_TBLS);
    memset(rx_tbls, 0, sizeof(sta_rx_status_tbl)*QOS_RX_STATUS_TBLS);
#else
    tx_tbls = (sta_tx_status_tbl *)WLA_MALLOC(sizeof(sta_tx_status_tbl) * tx_tbl_num +
                                              sizeof(sta_rx_status_tbl) * rx_tbl_num +
                                              sizeof(ucast_qinfo) * tx_tbl_num,
                                              DMA_MALLOC_UNCACHED|DMA_MALLOC_BZERO|DMA_MALLOC_ALIGN);
    rx_tbls = (sta_rx_status_tbl*)(&tx_tbls[tx_tbl_num]);
#endif	// CONFIG_SRAM_WLA
    for (i=0; i<tx_tbl_num; i++)
    {
        tx_tbls[i].long_retry = HT_LONG_RETRY_DEFAULT;
        tx_tbls[i].short_retry = HT_SHORT_RETRY_DEFAULT;
        tx_tbls[i].ack_policy = MAC_ACK_POLICY_NORMAL_ACK;
    }

    captbl->TX_tbl_addr = (u32) PHYSICAL_ADDR(tx_tbls);
    captbl->RX_tbl_addr = (u32) PHYSICAL_ADDR(rx_tbls);

    /* these settings will be used when toggling gsn in TX descr */
    captbl->gack_policy = MAC_ACK_POLICY_NORMAL_ACK;
    captbl->glong_retry = LEGACY_LONG_RETRY_DEFAULT;
    captbl->gshort_retry = LEGACY_SHORT_RETRY_DEFAULT;

    captbl->bssid = bss_desc; /* why set it in cap-table ? */

    captbl->MPDU_spacing = wcb->min_mpdu_start_spacing;
    /* AP manager assign wcb */
    wcb->captbl_idx = index;
    info->wcb_tbls[index] = wcb;
    wla_rc_init(info, wcb); 

//  bcap.bit.bssid = bss_desc;
    bcap.bit.vld = 1;

    bcap.bit.tosw = 1;
//  bcap.bit.rx_ampdu = 255;
//  bcap.bit.eth_header = 0;
    /* The cached cap-table should write back before inserting addr-table */
//	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

    /* write back TX and RX part of stacap */
//	mac_stacap_cache_sync(index, true);
//	mac_rx_linkinfo_cache_sync();

    /* [31:8] is basic capability, [7:0] is lookup index */
    addr_index = mac_addr_lookup_engine_update(info, (int)sta_addr, (bcap.val | index), mode);

    wcb->addr_idx = addr_index;
    wcb->hw_cap_ready = 1;

    ap = 0;
    {
        /* STA MODE */
        captbl->parent_ap = 0;
        captbl->AP = 0;
        info->mbss[(u32)bss_desc].sta_count++;
    }
    wcb->linked_ap = ap;
    wcb->mode = mode;
    wcb->bss_desc = bss_desc;

//	DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));
//	mac_stacap_cache_sync(index, true);

#if defined(WLA_PSBA_QUEUE)
    /* initialize psbaq for BA-noagg/PS/AMPDU */
#ifndef CONFIG_SRAM_WLA
    qinfos = (ucast_qinfo *)&rx_tbls[rx_tbl_num];
#endif

    for (i=0; i<tx_tbl_num; i++)
    {
        memset(&qinfos[i], 0, sizeof(ucast_qinfo));
        qinfos[i].tx_head_ptr = 0xffff;
        qinfos[i].tx_tail_ptr = 0xffff;
        qinfos[i].tx_queue_len = MAX_PS_QUEUE_LENGTH;
    }

    info->sta_qinfo_tbls[index].qinfo_addr = PHYSICAL_ADDR((char *)qinfos);
    info->sta_qinfo_tbls[index].sta_idx = addr_index;
    info->sta_qinfo_tbls[index].qos = wcb->qos; 
    info->sta_qinfo_tbls[index].vld = 1;

    DCACHE_STORE((unsigned int)captbl, sizeof(sta_cap_tbl));

    /* write back TX and RX part of stacap */
    mac_stacap_cache_sync(index, true);
    mac_rx_linkinfo_cache_sync();
#endif

    return; 
}

/* Non reentrant function */
int smrt_release_sta(struct wsta_cb *wcb)
{
	MAC_INFO *info = WLA_MAC_INFO;
	sta_cap_tbl *captbl;
	int captbl_idx;

	if(wcb == 0)
		return 0;

	if(wcb->hw_cap_ready == 0)
		return 0;
	
	captbl_idx = wcb->captbl_idx;
	captbl = &info->sta_cap_tbls[captbl_idx];
	if(!captbl->valid)
	{
		WLA_DBG(WLADEBUG, "???%s(%d), captbl->valid=%d\n", __func__, __LINE__, captbl->valid);
		return 0;
	}

	//DCACHE_FLUSH((unsigned int)captbl, sizeof(sta_cap_tbl));  /* captbl should be located non-cached memory */
	captbl = (sta_cap_tbl *)NONCACHED_ADDR(captbl);

	captbl->valid = 0;

	/* write back TX and RX part of stacap */
	mac_stacap_cache_sync(captbl_idx, true);
	mac_rx_linkinfo_cache_sync(); 

	wla_lock();
	wcb->hw_cap_ready = 0;
	wcb->sta = 0;
	wla_unlock();

	return 0;
}

int is_macaddr_all_zero(u8 *addr)
{
    return !(addr[0]|addr[1]|addr[2]|addr[3]|addr[4]|addr[5]);
}

unsigned char siso_locked_ap_addr[6];
unsigned char mimo_locked_ap_addr[6];

unsigned char siso2_last_mac_addr_ipv4[6];
unsigned char siso2_last_mac_addr_ipv6[6];
unsigned char siso2_mac_addr_offset_ipv4[6];
unsigned char siso2_mac_addr_offset_ipv6[6];
unsigned int siso2_last_mac_length_ipv4;
unsigned int siso2_last_mac_length_ipv6;
unsigned int siso2_mac_length_offset_ipv4;
unsigned int siso2_mac_length_offset_ipv6;
void __register_address_to_ds_table(u8 *addr)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    struct wsta_cb *wcb;

    if(!cfg->wcb)
    {
        wcb = wla_get_wcb();
        if(wcb)
        {
            smart_get_sta(addr, 0 /*cfg->bss_desc*/, wcb, IN_DS_TBL);
            cfg->wcb = wcb;
        }
    }
    else
    {
        mac_addr_lookup_engine_update(info, (int)addr, 0xC0000000 | ds_captbl_idx, IN_DS_TBL);
    }
}

void unregister_siso_locked_ap_addr(void)
{
    if(!is_macaddr_all_zero(siso_locked_ap_addr))
    {
        if(memcmp(siso_locked_ap_addr, mimo_locked_ap_addr, ETH_ALEN))
        {
            mac_addr_lookup_engine_update(info, (u32) &siso_locked_ap_addr[0], 0x00000000, IN_DS_TBL);
            SC_DBG("Release locked AP addr %pM\n", siso_locked_ap_addr);
        }
    
        memset(siso_locked_ap_addr, 0, ETH_ALEN);
    }
}

void register_siso_locked_ap_addr(u8 *addr)
{
    unregister_siso_locked_ap_addr();

    __register_address_to_ds_table(addr);
    //mac_addr_lookup_engine_update(info, (u32) addr, 0x80000000, IN_DS_TBL);
    memcpy(siso_locked_ap_addr, addr, ETH_ALEN);

    SC_DBG("SISO locked AP addr %pM\n", addr);
}

void unregister_mimo_locked_ap_addr(void)
{
    if(!is_macaddr_all_zero(mimo_locked_ap_addr))
    {
        if(memcmp(siso_locked_ap_addr, mimo_locked_ap_addr, ETH_ALEN))
        {
            mac_addr_lookup_engine_update(info, (u32) &mimo_locked_ap_addr[0], 0x00000000, IN_DS_TBL);
            SC_DBG("Release locked AP addr %pM\n", mimo_locked_ap_addr);
        }
    
        memset(mimo_locked_ap_addr, 0, ETH_ALEN);
    }
}

void register_mimo_locked_ap_addr(u8 *addr)
{
    unregister_mimo_locked_ap_addr();

    __register_address_to_ds_table(addr);
    //mac_addr_lookup_engine_update(info, (u32) addr, 0x80000000, IN_DS_TBL);
    memcpy(mimo_locked_ap_addr, addr, ETH_ALEN);

    SC_DBG("MIMO locked AP addr %pM\n", addr);
}

#endif

void disable_sniffer_mode(void)
{
    bb_register_write(0x12, 0x05);
    bb_register_write(0x13, 0x03);

    MACREG_UPDATE32(RTSCTS_SET, 0, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_SNIFFER_MODE);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);
}

void enable_sniffer_mode(void)
{
    MACREG_UPDATE32(RTSCTS_SET, LMAC_FILTER_ALL_PASS, LMAC_FILTER_ALL_PASS);
    MACREG_UPDATE32(RX_ACK_POLICY, RX_SNIFFER_MODE, RX_SNIFFER_MODE);
    MACREG_UPDATE32(RX_ACK_POLICY, 0, RX_AMPDU_REORDER_ENABLE);

    bb_register_write(0x12, 0x05);
    bb_register_write(0x13, 0x07);
}

void smart_inform_user(struct work_struct *work)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;

    char *argv[5] = {NULL, NULL, NULL, NULL, NULL};
    char *envp[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
    unsigned int i;
#if defined(KEEP_ADDR_LOOKUP_DATA)
    sta_basic_info bcap;
    unsigned char addr[6];
#endif     

    int data_len, mode, ssid_len, pass_len;
    int err = 0;

    wla_lock();

#if defined(DECODE_AP_FORWARD_DATA)
    unregister_siso_locked_ap_addr();
    unregister_mimo_locked_ap_addr();
#endif

#if defined(KEEP_ADDR_LOOKUP_DATA)
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
    {
        mac_addr_lookup_engine_find(addr, i, &bcap.val, BY_ADDR_IDX);
        if(bcap.bit.vld)
        {
            bcap.bit.tosw = 0;
            mac_addr_lookup_engine_update(info, i, bcap.val, BY_ADDR_IDX);
        }
    }
#else
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);
#endif

#if defined(DECODE_AP_FORWARD_DATA)
    if(cfg->wcb)
    {    
        smrt_release_sta(cfg->wcb);
        wla_post_release_sta(cfg->wcb);
        cfg->wcb = NULL;
    }
#endif

    disable_sniffer_mode();

    info->curr_center_freq = curr_center_freq;
    info->curr_channel = curr_channel;
    info->bandwidth_type = bandwidth_type;

    SC_DBG("Change back to channel %d %d\n", info->curr_channel, info->bandwidth_type);
    wla_set_channel(info);
    wla_set_bandwidth_type(info);

    /* inform to user-space */
    argv[0] = SMRTCFG; 
    envp[0] = SMRT_PATH;

    if (cfg->state == S_DONE)
    {
        data_len = cfgdata[0] + 1;
        mode = cfgdata[1];
        ssid_len = cfgdata[2];
        if(data_len > (ssid_len + 3))
            pass_len = cfgdata[3 + ssid_len];
        else
            pass_len = 0;

        cfgdata[3 + ssid_len] = 0;
        cfgdata[4 + ssid_len + pass_len] = 0;

        argv[1] = SMRTDONE; 

        envp[1] = kmalloc((sizeof("MODE=") + 2), GFP_ATOMIC);
        if(envp[1]==NULL)
            err = 1;
        else
            //sprintf(envp[1], "MODE=%d", mode);
            snprintf(envp[1],(sizeof("MODE=") + 2), "MODE=%d", mode);

        SC_DBG("envp[1] %s\n", envp[1]);

        envp[2] = kmalloc((sizeof("SSID=")+ssid_len+1), GFP_ATOMIC);
        if(envp[2]==NULL)
            err = 1;
        else
            //sprintf(envp[2], "SSID=%s", &cfgdata[3]);
            snprintf(envp[2],(sizeof("SSID=")+ssid_len+1), "SSID=%s", &cfgdata[3]);

        SC_DBG("envp[2] %s\n", envp[2]);

        if(pass_len)
        {
            envp[3] = kmalloc((sizeof("PASS=")+pass_len+1), GFP_ATOMIC);
            if(envp[3]==NULL)
                err = 1;
            else
                //sprintf(envp[3], "PASS=%s", &cfgdata[4+ssid_len]);
                snprintf(envp[3],(sizeof("PASS=")+pass_len+1), "PASS=%s", &cfgdata[4+ssid_len]);

            SC_DBG("envp[3] %s\n", envp[3]);
        }
    }
    else if (cfg->state == S_USRSTOP)
    {
        argv[1] = SMRUSRSTOP;
        envp[1] = NULL;
    }

    if(cfgdata)
    {
        kfree(cfgdata);
        cfgdata = NULL;
    }

    if (blocks)
    {
        kfree(blocks);
        blocks=NULL;
    }

    if (lock_data)
    {
        free_lock_data(lock_data);
        kfree(lock_data);
        lock_data=NULL;
    }

    __smartcfg_enable = 0;

    wla_unlock();

    if(!err)
        call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);

    if (envp[1])
        kfree(envp[1]);
    if (envp[2])
        kfree(envp[2]);
    if (envp[3])
        kfree(envp[3]);

    return;
}

void smart_start(struct smartcfg *cfg, int chansec)
{
    struct wla_softc *sc = info->sc;
    unsigned char channel, bss_desc;
    unsigned int i;
#if defined(KEEP_ADDR_LOOKUP_DATA)
    sta_basic_info bcap;
#endif

    wla_lock();

    if(__smartcfg_enable)
    {
        SC_DBG("smart_start(): already started, ignored\n");
        wla_unlock();
        return;
    }

    __smartcfg_filter_mode = FILTER_MODE_DROP_ALL;

    cfgdata = kzalloc(sizeof(u8) * (MAX_CFGDATA_SIZE + 1), GFP_ATOMIC);
    if(cfgdata==NULL)
    {
        SC_DBG("smart_start(): cfgdata malloc failure\n");
        wla_unlock();
        return;
    }

    SC_DBG("Smart-config start\n");

    curr_center_freq = info->curr_center_freq;
    curr_channel = info->curr_channel;
    bandwidth_type = info->bandwidth_type;
    memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
    sniffer_rx_count = 0;
    last_sniffer_rx_count = 0;
#endif
    __smartcfg_enable = 1;

#ifdef PLAY_OMNI_VOICE
    // initial voice type with none
    __smartcfg_voice_type = S_VOICE_NONE;
#endif

#if defined(KEEP_ADDR_LOOKUP_DATA)
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
    {
        mac_addr_lookup_engine_find(NULL, i, &bcap.val, BY_ADDR_IDX);
        if(bcap.bit.vld)
        {
            bcap.bit.tosw = 1;
            mac_addr_lookup_engine_update(info, i, bcap.val, BY_ADDR_IDX);
        }
    }
#else
    for (i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
        mac_addr_lookup_engine_update(info, i, 0, BY_ADDR_IDX);
#endif

//	cancel_delayed_work(&sc->smartcfg_work);
    channel = cfg->cur_chan;
    bss_desc = cfg->bss_desc;
    memset(cfg, 0, sizeof(struct smartcfg));
#if defined(DECODE_AP_FORWARD_DATA)
    memset(siso_locked_ap_addr, 0, ETH_ALEN);
    memset(mimo_locked_ap_addr, 0, ETH_ALEN);
    memset(siso2_last_mac_addr_ipv4, 0, ETH_ALEN);
    memset(siso2_last_mac_addr_ipv6, 0, ETH_ALEN);
    memset(siso2_mac_addr_offset_ipv4, 0, ETH_ALEN);
    memset(siso2_mac_addr_offset_ipv6, 0, ETH_ALEN);

#endif

    cfg->num_of_channel = 11;
    if(cfg80211_regdomain)
    {
        SC_DBG("Regulatory domain %c%c\n", cfg80211_regdomain->alpha2[0], cfg80211_regdomain->alpha2[1]);
        if((cfg80211_regdomain->alpha2[0]=='C')&&(cfg80211_regdomain->alpha2[1]=='N'))
        {
            SC_DBG("13 channels\n");
            cfg->num_of_channel = 13;
        }
    }
    cfg->fixchan = chansec;
    if (chansec == 2)
        cfg->cur_chan = 4;
    else if (chansec == 3)
        cfg->cur_chan = 8;
    else
        cfg->cur_chan = channel;
    cfg->bss_desc=bss_desc;

    enable_sniffer_mode();

    __smartcfg_filter_mode = (FILTER_SISO_SCAN | FILTER_MIMO_SCAN);
 
    cfg->total = jiffies;
#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
    next_rx_count_check_time = jiffies + RX_COUNT_CHECK_INTERVAL;
#endif   

    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, 0);
    wla_unlock();
}

static int smrt_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len);
static int smrt2_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len);
static int siso_smart_decode(char *data, struct smartcfg *cfg,  unsigned int len)
{
    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)data;
    __le16 fc;
    int ret = DECODE_IN_PROC;

    fc = hdr->frame_control;
    if (ieee80211_is_data(fc))
    {
        if (is_multicast_ether_addr(hdr->addr3) && (!memcmp(hdr->addr2, cfg->ta, 6)))
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt_data_decode(hdr->addr3, cfg, len);
            else
                ret = smrt_data_decode(hdr->addr3, cfg, len+2);
        }

#if defined(DECODE_AP_FORWARD_DATA)
        if (is_multicast_ether_addr(hdr->addr1) && (!memcmp(hdr->addr3, cfg->ta, 6)))
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt_data_decode(hdr->addr1, cfg, len);
            else
                ret = smrt_data_decode(hdr->addr1, cfg, len+2);
        }
#endif
    }

    return ret;
}
static int siso2_smart_decode(char *data, struct smartcfg *cfg,  unsigned int len)
{
    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)data;
    __le16 fc;
    int ret = DECODE_IN_PROC;

    fc = hdr->frame_control;
    if (ieee80211_is_data(fc))
    {   
        if (is_multicast_ether_addr(hdr->addr3) && (!memcmp(hdr->addr2, cfg->ta, 6)))
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt2_data_decode(hdr->addr3, cfg, len);
            else
                ret = smrt2_data_decode(hdr->addr3, cfg, len+2);
        }

#if defined(DECODE_AP_FORWARD_DATA)
        if (is_multicast_ether_addr(hdr->addr1) && (!memcmp(hdr->addr3, cfg->ta, 6)))
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt2_data_decode(hdr->addr1, cfg, len);
            else
                ret = smrt2_data_decode(hdr->addr1, cfg, len+2);
        }
#endif
    }

    return ret;
}
static void smart_signature_check(char *mac, struct smartcfg *cfg)
{
    int i;

    for (i=0; i<SIGNATURE_MAC_COUNT; i++)
    {
        if (!memcmp(smrt_chan_multicast_addr_list[i], mac, 6))
            cfg->signature_count++;
    }
}

static int smrt_chan_chkaddr(char *mac, struct smartcfg *cfg, unsigned int len)
{
    int i;
    int ret = SIGNATURE_NOT_FOUND;

    for (i=0; i<SIGNATURE_MAC_COUNT; i++)
    {
        if (cfg->macmatch & (1<<i))
            continue;
        if (!memcmp(smrt_chan_multicast_addr_list[i], mac, 6)) // make sure channel.
        {
            SC_DBG("Signature %pM matched\n", mac);

            cfg->macmatch |= (1 << i);
            if (cfg->macmatch == (1<<SIGNATURE_MAC_COUNT)-1)   // We found channel special multicast address
            {
                ret = SIGNATURE_LOCKED;
                goto done;
            }
        }
    }

    if (cfg->macmatch)
        ret = STAY_IN_THE_CHANNEL;
    else
        ret = SIGNATURE_NOT_FOUND;

done:   
    return ret;
}

static int smrt2_chan_chkaddr(char *mac, struct smartcfg *cfg, unsigned int len)
{
    int ret = SIGNATURE_NOT_FOUND;

    if(is_cfgdata_addr_ipv4(mac))
    {
        if((!is_macaddr_all_zero(siso2_last_mac_addr_ipv4)) &&
           ((len != siso2_last_mac_length_ipv4 && memcmp(siso2_last_mac_addr_ipv4, mac, 6))&&
            (len - siso2_last_mac_length_ipv4 == mac[3]-siso2_last_mac_addr_ipv4[3])))
        {
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk("match mac, mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
            }

            memcpy(siso2_mac_addr_offset_ipv4, mac, ETH_ALEN);
            siso2_mac_length_offset_ipv4 = len;
        }
        else
        {
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk("not match mac, read mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
            }

            memcpy(siso2_last_mac_addr_ipv4, mac, ETH_ALEN);
            siso2_last_mac_length_ipv4 = len;
            goto done;
        }
        
        SC_DBG("Signature matched\n");
        ret = SIGNATURE_LOCKED;
        goto done;   
    }
    else if(is_cfgdata_addr_ipv6(mac))
    {
        if((!is_macaddr_all_zero(siso2_last_mac_addr_ipv6)) &&
           ((len != siso2_last_mac_length_ipv6 && memcmp(siso2_last_mac_addr_ipv6, mac, 6))&&
            (len - siso2_last_mac_length_ipv6 == mac[3]-siso2_last_mac_addr_ipv6[3])))
        {
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk("match mac, mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
            }

            memcpy(siso2_mac_addr_offset_ipv6, mac, ETH_ALEN);
            siso2_mac_length_offset_ipv6 = len;
        }
        else
        {
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk("not match mac, read mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
            }

            memcpy(siso2_last_mac_addr_ipv6, mac, ETH_ALEN);
            siso2_last_mac_length_ipv6 = len;
            goto done;
        }
        
        SC_DBG("Signature matched\n");
        ret = SIGNATURE_LOCKED;
        goto done;   
    }
           
    ret = SIGNATURE_NOT_FOUND;

done:   
    return ret;
}

static int smrt_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len)
{
    unsigned char idx;
    u8 byte;
    int ret = DECODE_IN_PROC;

    smart_signature_check(mac, cfg);

    if (is_cfgdata_addr(mac))
    {
        if(len >= cfg->offset)
        {
            idx = get_cfgdata_idx(mac);
            byte = (u8) (len - cfg->offset);

            //SC_DBG("Idx %d Len %d Byte %02x\n", idx, len, byte);
            cfg->valid_pkt_count++;

            if(test_bit(idx, cfgdata_valid))
            {
                if(cfgdata[idx]!=byte)
                {
                    SC_DBG("Inconsistent Data[%d]=%02x %02x, reset decoded data\n", idx, (u8) cfgdata[idx], (u8) byte);

                    memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
                    cfg->datacnt = 0;
                    cfg->datalen = 0;
                }
            }
            else
            {
                set_bit(idx, cfgdata_valid);

                cfgdata[idx] = byte;
                cfg->datacnt++;
    
                SC_DBG("Data[%d]=%02x\n", idx, (u8) cfgdata[idx]);
    
                if(idx==0)
                {
                    cfg->datalen = byte + 1;                
                    SC_DBG("Config data length = %d\n", cfg->datalen);
                }
            }
        }
        else
        {
            goto Exit;
        }
    }

    if (cfg->datalen && (cfg->datacnt == cfg->datalen))
    {
        //struct wla_softc *sc = info->sc;
        int i;

        cfg->total = (long)jiffies - (long)cfg->total;
        for(i=0;i<cfg->datalen;i++)
        {
            if(isprint(cfgdata[i]))
               SC_DBG("CFGDATA[%d] 0x%02x %c\n", i, cfgdata[i], cfgdata[i]);
            else
               SC_DBG("CFGDATA[%d] 0x%02x\n", i, cfgdata[i]);
        }

        ret = DECODE_DONE;
    }

Exit:
    return ret;
}

uint8_t Crc8(const void *vptr, int len)
{
	const uint8_t *data = vptr;
	unsigned crc = 0;
	int i, j;
	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for(i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (uint8_t)(crc >> 8);
}
static int smrt2_data_decode(unsigned char *mac, struct smartcfg *cfg, unsigned int len)
{
    unsigned char idx;
    u8 byte1, byte2;
    int ret = DECODE_IN_PROC;

    if (is_cfgdata_addr_ipv4(mac) || is_cfgdata_addr_ipv6(mac))
    {
        //DO LENGTH AND MAC[3] CHECK
        if(is_cfgdata_addr_ipv4(mac))
        {
            if(is_macaddr_all_zero(siso2_mac_addr_offset_ipv4))
            {
                //COMPARE AND STORE OFFSET
                if((!is_macaddr_all_zero(siso2_last_mac_addr_ipv4)) &&
                   ((len != siso2_last_mac_length_ipv4 && memcmp(siso2_last_mac_addr_ipv4, mac, 6))&&
                    (len - siso2_last_mac_length_ipv4 == mac[3]-siso2_last_mac_addr_ipv4[3])))
                {
                    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                    {
                        printk("match mac, mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
                    }

                    memcpy(siso2_mac_addr_offset_ipv4, mac, ETH_ALEN);
                    siso2_mac_length_offset_ipv4 = len;
                    goto Exit;
                }
                else
                {
                    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                    {
                        printk("not match mac, read mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
                    }

                    memcpy(siso2_last_mac_addr_ipv4, mac, ETH_ALEN);
                    siso2_last_mac_length_ipv4 = len;
                    goto Exit;
                }               
            }
            else if(len - siso2_mac_length_offset_ipv4 != mac[3] - siso2_mac_addr_offset_ipv4[3])
            {
                goto Exit;
            }
        }
        else if(is_cfgdata_addr_ipv6(mac))
        {
            if(is_macaddr_all_zero(siso2_mac_addr_offset_ipv6))
            {
                if((!is_macaddr_all_zero(siso2_last_mac_addr_ipv6)) &&
                   ((len != siso2_last_mac_length_ipv6 && memcmp(siso2_last_mac_addr_ipv6, mac, 6))&&
                    (len - siso2_last_mac_length_ipv6 == mac[3]-siso2_last_mac_addr_ipv6[3])))
                {
                    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                    {
                        printk("match mac, mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
                    }

                    memcpy(siso2_mac_addr_offset_ipv6, mac, ETH_ALEN);
                    siso2_mac_length_offset_ipv6 = len;
                    goto Exit;
                }
                else
                {
                    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                    {
                        printk("not match mac, read mac = %02x %02x %02x %02x %02x %02x, len = %d\n", 
                               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
                    }

                    memcpy(siso2_last_mac_addr_ipv6, mac, ETH_ALEN);
                    siso2_last_mac_length_ipv6 = len;
                    goto Exit;
                }   
            }
            else if(len - siso2_mac_length_offset_ipv6 != mac[3] - siso2_mac_addr_offset_ipv6[3])
            {
                goto Exit;
            }
        }
        idx = mac[3];
        if(idx>60)
        {
            goto Exit;
        }
        byte1 = (u8) mac[4];
        byte2 = (u8) mac[5];

        cfg->valid_pkt_count++;

        if(test_bit(idx, cfgdata_valid))
        {
            if(cfgdata[idx*2]!=byte1)
            {
                SC_DBG("Inconsistent Data[%d]=%02x %02x, reset decoded data\n", idx, (u8) cfgdata[idx*2], (u8) byte1);

                memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
                cfg->datacnt = 0;
                cfg->datalen = 0;
            }
            else if(cfgdata[(idx*2)+1]!=byte2)
            {
                SC_DBG("Inconsistent Data[%d]=%02x %02x, reset decoded data\n", idx, (u8) cfgdata[(idx*2)+1], (u8) byte2);

                memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
                cfg->datacnt = 0;
                cfg->datalen = 0;
            }
        }
        else
        {
            set_bit(idx, cfgdata_valid);

            cfgdata[idx*2] = byte1;
            cfgdata[(idx*2)+1] = byte2;
            cfg->datacnt++;

            if(idx==0)
            {
                cfg->datalen = byte1;                
                SC_DBG("Config data length = %d\n", cfg->datalen);
            }
        }      
    }

    if (cfg->datalen && (cfg->datacnt == ((cfg->datalen+2)/2)))
    {
        int i;
        uint8_t data_crc;

        cfg->total = (long)jiffies - (long)cfg->total;

        data_crc = Crc8(cfgdata, cfg->datalen);

        if(cfgdata[cfg->datalen]==data_crc)
        {
            printk("Pass crc check, crc = %02x\n", data_crc);
        }
        else
        {
            printk("Wrong crc!!, correct crc = %02x, now crc = %02x\n", cfgdata[cfg->datalen], data_crc);
            //ReDecoding until pass crc check;
            for(i=0;i<cfg->datalen;i++)
            {
                if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                {    
                    printk(KERN_INFO "%02d:%02x %c\n", i, cfgdata[i], cfgdata[i]);
                }
            }
            memset(cfgdata, 0, cfg->datacnt*2);
            memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
            cfg->datacnt = 0;
            cfg->datalen = 0;

            goto Exit;
        }
        for(i=0;i<cfg->datalen;i++)
        {
            if(isprint(cfgdata[i]))
               SC_DBG("CFGDATA[%d] 0x%02x %c\n", i, cfgdata[i], cfgdata[i]);
            else
               SC_DBG("CFGDATA[%d] 0x%02x\n", i, cfgdata[i]);
        }

        ret = DECODE_DONE;
    }
Exit:
    return ret;
}
static int siso_signature_search(char *data, struct smartcfg *cfg,  unsigned int len)
{
    struct ieee80211_hdr_3addr *hdr = (struct ieee80211_hdr_3addr *)data;
    __le16 fc;
    int ret = SIGNATURE_NOT_FOUND;

    fc = hdr->frame_control;

    /*We can check pkt len size*/

    if (!ieee80211_is_data(fc))
        return SIGNATURE_NOT_FOUND;

    /*should only handle multicast addr*/
    if (is_multicast_ether_addr(hdr->addr3))
    {
        if(len>=SISO2_SAFE_BASE && len<SISO2_SAFE_BASE+SISO2_SAFE_RANGE)
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt2_chan_chkaddr(hdr->addr3, cfg, len);
            else
                ret = smrt2_chan_chkaddr(hdr->addr3, cfg, len+2);
        }
        else
        {
            ret = smrt_chan_chkaddr(hdr->addr3, cfg, len);
        }
        if(ret == SIGNATURE_LOCKED)
        {
            memcpy(cfg->ta, hdr->addr2, ETH_ALEN);
#if defined(DECODE_AP_FORWARD_DATA)
            register_siso_locked_ap_addr(hdr->addr1);
#endif
            goto locked;
        }
    }

#if defined(DECODE_AP_FORWARD_DATA)
    if (is_multicast_ether_addr(hdr->addr1))
    {
        if(len>=SISO2_SAFE_BASE && len<SISO2_SAFE_BASE+SISO2_SAFE_RANGE)
        {
            if(ieee80211_is_data_qos(fc))
                ret = smrt2_chan_chkaddr(hdr->addr1, cfg, len);
            else
                ret = smrt2_chan_chkaddr(hdr->addr1, cfg, len+2);
        }
        else
        {
            ret = smrt_chan_chkaddr(hdr->addr1, cfg, len);
        }
        if(ret == SIGNATURE_LOCKED)
        {
            memcpy(cfg->ta, hdr->addr3, ETH_ALEN);
            register_siso_locked_ap_addr(hdr->addr2);
            goto locked;
        }
    }
#endif

    return ret;

locked:
    cfg->chantime = (long)jiffies - (long)cfg->total;

    if(ieee80211_is_data_qos(fc))
        cfg->offset = len - SMART_BASELEN;
    else
        cfg->offset = len - SMART_BASELEN + 2;

    SC_DBG("Locked STA %pM, channel:%d base_len:%d offset:%d\n",cfg->ta, cfg->cur_chan, len, cfg->offset);

    return ret;
}

void smart_stop(struct smartcfg *cfg)
{
    struct wla_softc *sc = info->sc;

    wla_lock();

    if(0==__smartcfg_enable)
    {
        SC_DBG("smart_stop(): already stopped, ignored\n");
        wla_unlock();
        return;
    }

    disable_sniffer_mode();
    __smartcfg_filter_mode = FILTER_MODE_DROP_ALL;

    cfg->state = SMART_USRSTOP;
    //cancel_delayed_work(&sc->smartcfg_work);
    cancel_delayed_work_sync(&sc->smartcfg_work);
    
    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_usrwork, HZ);

    wla_unlock();

    do
    {
        SC_DBG("Smart-config waiting stop done\n");
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(HZ);
    } while(0!=__smartcfg_enable);

    SC_DBG("Smart-config stopped\n");
}

#if defined(CHANNEL_HOPPING)
const u8 scan_channel_list_b[] = { 1, 6, 11, 2, 5, 8, 3, 9, 4, 7, 10 };
const u8 scan_channel_list_d[] = { 1, 6, 11, 2, 13, 5, 8, 3, 9, 4, 12, 7, 10 };
const u8 *scan_channel_list;
u8 num_of_channel;
#endif

int has_two_secondary_channels(int channel)
{
    if(num_of_channel==13)
        return ((channel >= 5) && (channel <= 9));
    else
        return ((channel >= 5) && (channel <= 7));
}

void wla_chgchan_inform(struct work_struct *work)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    static u8 stay_channel_state_count = 0;
    static u8 channel_below = 0;
    char *argv[5] = {NULL, NULL, NULL, NULL, NULL};
    char *envp[6] = {NULL, NULL, NULL, NULL, NULL, NULL};

    // Timeout concern

    wla_lock();

    if(cfg->num_of_channel==13)
    {
        num_of_channel = 13;
        scan_channel_list = scan_channel_list_d;
    }
    else
    {
        num_of_channel = 11;
        scan_channel_list = scan_channel_list_b;
    }

#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
    if(time_after(jiffies, next_rx_count_check_time))
    {
        if(sniffer_rx_count == last_sniffer_rx_count)
        {
            if(__smartcfg_enable)
            {
                SC_DBG("Trigger MAC recovery @ %ld\n", jiffies);
                info->mac_recover = 1;
            }
        }

        last_sniffer_rx_count = sniffer_rx_count;
        next_rx_count_check_time = jiffies + RX_COUNT_CHECK_INTERVAL;
    }
#endif

#if defined(CHANNEL_BLACKLIST)
Restart:
#endif

    if (cfg->state == S_SCAN_CHANNEL) //In this state we should change channel step by step
    {
        stay_channel_state_count = 0;
        if (!cfg->swupdwn)
        {
            if (cfg->fixchan == 1) // chan 1-4
                cfg->cur_chan=(cfg->cur_chan>=4)?1:(cfg->cur_chan+1);
            else if (cfg->fixchan == 2) // chan 5-8
                cfg->cur_chan=(cfg->cur_chan>=8)?5:(cfg->cur_chan+1);
            else if (cfg->fixchan == 3) // chan 9-11
                cfg->cur_chan=(cfg->cur_chan>=num_of_channel)?9:(cfg->cur_chan+1);
            else // all chan
            {
#if defined(CHANNEL_HOPPING)
                if(cfg->channel_index >= num_of_channel)
                    cfg->channel_index = 0;

                cfg->cur_chan = scan_channel_list[cfg->channel_index];

                cfg->channel_index++;
                if(cfg->channel_index >= num_of_channel)
                    cfg->channel_index = 0;

#else
                cfg->cur_chan=cfg->cur_chan>=num_of_channel?1:++cfg->cur_chan;
#endif

#if defined(CHANNEL_BLACKLIST)
                if(cfg->black_list_count && (cfg->black_list_channel_num==cfg->cur_chan))
                {
                    cfg->black_list_count--;
                    SC_DBG("Skip channel %d\n", cfg->cur_chan);
                    goto Restart;
                }
#endif
            }

#if defined(ENABLE_40MHZ_CHANNEL)
            if (has_two_secondary_channels(cfg->cur_chan))
                cfg->swupdwn = 2;
            else
                cfg->swupdwn = 0;
#else
            cfg->swupdwn = 0;
#endif
        }

        if ((cfg->swupdwn) && has_two_secondary_channels(cfg->cur_chan))
        {
            if(((cfg->swupdwn==2) && (cfg->channel_below_first))
                ||((cfg->swupdwn==1) && (!cfg->channel_below_first)))
            {
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                SC_DBG("Chan:%d-\n",cfg->cur_chan);
                channel_below = 1;
            }
            else
            {
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                SC_DBG("Chan:%d+\n",cfg->cur_chan);
                channel_below = 0;
            }
            cfg->swupdwn--;
        }
        else
        {
#if defined(ENABLE_40MHZ_CHANNEL)
            if (cfg->cur_chan < 5)
            {    
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                SC_DBG("Chan:%d+\n",cfg->cur_chan);
            }
            else
            {    
                wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                SC_DBG("Chan:%d-\n",cfg->cur_chan);
            }
#else
            wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCN);
            SC_DBG("Chan:%d\n",cfg->cur_chan);
#endif
        }

        goto timer;
    }
    else if (cfg->state == S_STAY_CHANNEL) //In this state we just remain on chan 
    {
        cfg->macmatch = 0;
        stay_channel_state_count++;
        if(stay_channel_state_count>4)
        {
            SC_DBG("Lock attempt failed\n");
            cfg->swupdwn = 0;
            cfg->state = S_SCAN_CHANNEL;
            __smartcfg_filter_mode = (FILTER_SISO_SCAN | FILTER_MIMO_SCAN);

#if defined(CHANNEL_BLACKLIST)
            cfg->black_list_count = 1;
            cfg->black_list_channel_num = cfg->cur_chan;
#endif
        }        
        goto timer;
    }
    else if (cfg->state == S_DECODE)
    {
#if 1
        if(time_after(jiffies, cfg->next_decode_status_check_time))
        {
            if(cfg->signature_count==cfg->signature_count_prev)
            {
                SC_DBG("Unlock the STA and try again\n");

                cfg->swupdwn = 0;
                cfg->macmatch = 0;

                memset(siso2_mac_addr_offset_ipv4, 0, ETH_ALEN);
                memset(siso2_mac_addr_offset_ipv6, 0, ETH_ALEN);
                memset(cfgdata, 0, cfg->datacnt*2);
                memset(cfgdata_valid, 0, sizeof(cfgdata_valid));
                cfg->datacnt = 0;
                cfg->datalen = 0;

                /* swap the above/below change order */
                cfg->channel_below_first = (cfg->channel_below_first) ? 0 : 1;

                cfg->state = S_SCAN_CHANNEL;
                __smartcfg_filter_mode = (FILTER_SISO_SCAN | FILTER_MIMO_SCAN);

#if defined(CHANNEL_BLACKLIST)
                cfg->black_list_count = 1;
                cfg->black_list_channel_num = cfg->cur_chan;
#endif
            }
#if defined(ENABLE_40MHZ_CHANNEL)
            else if (has_two_secondary_channels(cfg->cur_chan))
            {
                if(cfg->valid_pkt_count==cfg->valid_pkt_count_prev)
                {
                    if(channel_below)
                    {
                        wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCA);
                        SC_DBG("Chan:%d+\n",cfg->cur_chan);
                        channel_below = 0;
                    }
                    else
                    {
                        wla_cfg(SET_CHANNEL, cfg->cur_chan, BW40MHZ_SCB);
                        SC_DBG("Chan:%d-\n",cfg->cur_chan);
                        channel_below = 1;
                    }
                }
            }
#endif

            cfg->signature_count_prev = cfg->signature_count;
            cfg->valid_pkt_count_prev = cfg->valid_pkt_count;
            cfg->next_decode_status_check_time = jiffies + SMART_DECODE_STATUS_CHECK_INTERVAL;
        }
        goto timer;
#endif
    }
    wla_unlock();
    return;

timer:
    //cancel_delayed_work(&sc->smartcfg_work); /* XXX */
    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, CHGCHANDELAY);
    wla_unlock();
#ifdef PLAY_OMNI_VOICE
    if (__smartcfg_voice_type == S_VOICE_CFG)
    {
        __smartcfg_voice_type = S_VOICE_NONE;

        /* inform to user-space */
        argv[0] = SMRTCFG;
        argv[1] = SMRTVOICE;
        argv[2] = "cfg";
        envp[0] = SMRT_PATH;

        call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
    }
#endif
}

static u32 length_data=0;
static int last_frame_tid;
#define FRAME_TYPE_DONT_CARE        0
#define FRAME_TYPE_UNSUPPORT_FORMAT 1
#define FRAME_TYPE_DATA_MPDU        2
static int last_frame_type = FRAME_TYPE_DONT_CARE;

#define DECODE_DATA_LENGTH_H    (SMART_CONFIG_LENGTH_BASE + (0x3f<<SMART_CONFIG_SHIFT))
#define SAFE_LENGTH             100
#define MIMO_DECODE_PHASE_TID   0x5
#define MIMO_LOCK_PHASE_TID     0x1
#define AGGREGATE_PADDING       4

int smartcfg_dispatcher(u8 *ptr, unsigned int len, u8 *mac_addr, u8 *ap_addr, u32 flags);
int smartcfg_filter(u8 *ptr, unsigned int len, volatile rx_descriptor *rx_desc)
{
    struct wlan_ba *ba_head = (struct wlan_ba *) ptr;
    struct wlan_qoshdr *qdata = (struct wlan_qoshdr *) ptr;
    int next_frame_type;

    next_frame_type = FRAME_TYPE_DONT_CARE;
#if defined(MIMO_DEBUG)
    printk("len: %d, crc: %d, ptr[0]= %x\n", len, rx_desc->crc, ptr[0]);
    printk("type %x, subtype %x, bactl %04x, baseqctl %04x\n"
           , ba_head->hdr.type,  ba_head->hdr.subtype, ba_head->bactl, ba_head->baseqctl);
#endif
    if ((__smartcfg_filter_mode & (FILTER_MIMO_SCAN|FILTER_MIMO_DECODE)) && rx_desc->crc)
    {
        if ((__smartcfg_filter_mode&FILTER_MIMO_DECODE)
            &&(((SC_LOCK_LENGTH_BASE>>SMART_CONFIG_SHIFT) > len)
               ||(((DECODE_DATA_LENGTH_H + SAFE_LENGTH + header_len_diff)>>SMART_CONFIG_SHIFT) < len)))
        {
            return 0;
        }
        else if(__smartcfg_filter_mode&FILTER_MIMO_DECODE)
        {
            length_data=(len+4)*4;
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk(KERN_INFO "UF2 %d\n", length_data);
            }
            next_frame_type = FRAME_TYPE_UNSUPPORT_FORMAT;
        }
        else if ((__smartcfg_filter_mode&FILTER_MIMO_SCAN)
            &&(((SC_LOCK_LENGTH_BASE>>SMART_CONFIG_SHIFT) > len)
               ||(((SMART_CONFIG_LENGTH_BASE + SAFE_LENGTH)>>SMART_CONFIG_SHIFT) < len)))
        {
            return 0;
        }
        else if(__smartcfg_filter_mode&FILTER_MIMO_SCAN)
        {
            length_data=(len+4)*4;
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                printk(KERN_INFO "UF1 %d\n", length_data);
            }
            next_frame_type = FRAME_TYPE_UNSUPPORT_FORMAT;
        }

    }
    else if ((__smartcfg_filter_mode & (FILTER_MIMO_SCAN|FILTER_MIMO_DECODE))
             &&((ba_head->hdr.type==WLAN_FC_TYPE_CTRL)&&(ba_head->hdr.subtype==WLAN_FC_SUBTYPE_BA)))
    {
        u16 cur_ssn;
        u16 cur_tid;
        cur_tid=((ba_head->bactl & LE_WLAN_BA_TID)>>LE_WLAN_BA_TID_S);
        if ((__smartcfg_filter_mode&FILTER_MIMO_DECODE)
            &&((memcmp(ba_head->hdr.addr1, lock_mac_addr, WLAN_ADDR_LEN))
               ||((MIMO_DECODE_PHASE_TID!=cur_tid)
                  &&(MIMO_LOCK_PHASE_TID!=cur_tid))))
        {
            return 0;
        }
        if((__smartcfg_filter_mode&FILTER_MIMO_DECODE) && cur_tid==5)
        {
        }
        else if ((__smartcfg_filter_mode&FILTER_MIMO_SCAN)
            &&(MIMO_LOCK_PHASE_TID!=cur_tid))
        {
            return 0;
        }
        
        cur_ssn=((ba_head->baseqctl & LE_WLAN_BA_SSN_L)>>LE_WLAN_BA_SSN_L_S
                 |(ba_head->baseqctl & LE_WLAN_BA_SSN)<<LE_WLAN_BA_SSN_S)& WLAN_BA_SSN_FIELD; // Sequence Number
#if defined(MIMO_DEBUG)
        printk("state: %d, tid=%d, BA %d\n", __smartcfg_filter_mode, cur_tid, cur_ssn);
#endif
        if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
        {
            printk(KERN_INFO "state: %d, tid=%d, BA %d\n", __smartcfg_filter_mode, cur_tid, cur_ssn);
        }
        if (MIMO_LOCK_PHASE_TID==cur_tid)
        {
            if (last_frame_type!=FRAME_TYPE_DONT_CARE)
            {
                if ((SC_LOCK_LENGTH_BASE <= length_data)
                    &&((SMART_CONFIG_LENGTH_BASE + SAFE_LENGTH) >= length_data))
                {    
    
                    if(last_frame_type==FRAME_TYPE_DATA_MPDU)
                    {                    
                        smartcfg_dispatcher(ptr,length_data+AGGREGATE_PADDING,ba_head->hdr.addr1, ba_head->hdr.addr2, DISPATCH_MIMO_SCAN);
                    }
                    else
                    {
                        smartcfg_dispatcher(ptr,length_data,ba_head->hdr.addr1, ba_head->hdr.addr2, DISPATCH_MIMO_SCAN);
                    }
                }
            }
        }
        else
        {
            if (last_frame_type!=FRAME_TYPE_DONT_CARE)
            {           
                if(last_frame_type==FRAME_TYPE_DATA_MPDU)
                {
                    smartcfg_dispatcher(ptr,length_data+AGGREGATE_PADDING,ba_head->hdr.addr1, ba_head->hdr.addr2, DISPATCH_MIMO_DECODE);
                }
                else
                {  
                    smartcfg_dispatcher(ptr,length_data,ba_head->hdr.addr1, ba_head->hdr.addr2, DISPATCH_MIMO_DECODE);
                }           
            }
        }
    }
    else if ((last_frame_type!=FRAME_TYPE_DONT_CARE)
             &&(__smartcfg_filter_mode & (FILTER_MIMO_SCAN|FILTER_MIMO_DECODE))
             &&((ba_head->hdr.type==WLAN_FC_TYPE_CTRL)&&(ba_head->hdr.subtype==WLAN_FC_SUBTYPE_ACK)))
    {

#if defined(MIMO_DEBUG)
        printk(KERN_DEBUG "ACK %d\n", length_data);
#endif
        if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
        {
            printk(KERN_INFO "ACK %d\n", length_data);
        }
        if (__smartcfg_filter_mode&FILTER_MIMO_DECODE)
        {
            if(FRAME_TYPE_DATA_MPDU==last_frame_type)
            {
                if(last_frame_tid==MIMO_LOCK_PHASE_TID)
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_SCAN);
                else
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_DECODE);
            }
            else
            {
                if ((SC_LOCK_LENGTH_BASE <= length_data)
                    &&((SMART_CONFIG_LENGTH_BASE + header_len_diff - AGGREGATE_PADDING) > length_data))
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_SCAN);
                else 
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_DECODE);
            }
        }
        else if (__smartcfg_filter_mode&FILTER_MIMO_SCAN)
        {
            if(FRAME_TYPE_DATA_MPDU==last_frame_type)
            {
                if(last_frame_tid==MIMO_LOCK_PHASE_TID)
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_SCAN);
            }
            else
            {    
                if ((SC_LOCK_LENGTH_BASE <= length_data)
                   &&(SMART_CONFIG_LENGTH_BASE + SAFE_LENGTH >= length_data))
                    smartcfg_dispatcher(ptr,(length_data+AGGREGATE_PADDING),ba_head->hdr.addr1, NULL, DISPATCH_MIMO_SCAN);
            }
        }
    }
    else if ((__smartcfg_filter_mode & (FILTER_SISO_SCAN|FILTER_MIMO_SCAN|FILTER_SISO_DECODE|FILTER_MIMO_DECODE))
        && (0==rx_desc->crc))
    {
        if((qdata->subtype==WLAN_FC_SUBTYPE_QOS) && (qdata->type==WLAN_FC_TYPE_DATA))
	    {
            if(!qdata->amsdu)
            {
                if((qdata->tid==MIMO_LOCK_PHASE_TID)||(qdata->tid==MIMO_DECODE_PHASE_TID))
                {
                    length_data=((len+4)/4)*4;
                    last_frame_tid = qdata->tid;
                    next_frame_type = FRAME_TYPE_DATA_MPDU;
                }
            }
            if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
            {
                if(length_data>1000)
                {    
                    printk(KERN_INFO "\t\t\t\t\t\t\t\t\t\t\t\tnonUF %d\n", length_data);
                }
                else
                    printk(KERN_INFO "nonUF %d\n", length_data);
            }
        }

        /* for SISO decode + scan */
        smartcfg_dispatcher(ptr, len, NULL, NULL, DISPATCH_SISO_SCAN|DISPATCH_SISO_DECODE);
    }

    last_frame_type = next_frame_type;

    return 0;
}

int mimo_smart_decode(unsigned int len, struct sc_rx_block *blocks)
{
    unsigned char *ret=NULL;
    unsigned int data=0;

    if(header_len_diff > len)
        return DECODE_IN_PROC;        

    data = len - header_len_diff;

#if defined(MIMO_DEBUG)
    printk("decode data\n", data);
#endif
    ret = sc_decoder(data, blocks);
    if (ret!=NULL)
    {
#if defined(MIMO_DEBUG)
        printk("MIMO Decode Success\n");
#endif
        memcpy(cfgdata,ret,ret[0]+1);
        return DECODE_DONE;
        //cfgdata=ret;
        //return DECODE_DONE
    }
    return DECODE_IN_PROC;
}

int mimo_signature_search(unsigned int len, char *mac_addr, struct try_lock_data *lock_data)
{
    int ret_state=0;
    header_len_diff=0;

    header_len_diff = try_lock_channel(len, mac_addr, lock_data, &ret_state);
    if (header_len_diff > 0)
    {
#if defined(MIMO_DEBUG)
    printk("length diff base=%d\n", header_len_diff);
#endif
    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
    {
        printk(KERN_INFO "length diff base=%d\n", header_len_diff);
    }
        return SIGNATURE_LOCKED;
    }
    else if(ret_state==STAY_IN_THE_CHANNEL)
    {
        return STAY_IN_THE_CHANNEL;
    }
    return SIGNATURE_NOT_FOUND;
}

int smartcfg_dispatcher(u8 *ptr, unsigned int len, u8 *mac_addr, u8 *ap_addr, u32 flags)
{
    struct wla_softc *sc = info->sc;
    struct smartcfg *cfg = &sc->smrtcfg;
    int ret = 0;

    // Dispatch data for "SISO" algorithm
    if ((__smartcfg_filter_mode & FILTER_SISO_SCAN) && (flags & (FILTER_SISO_SCAN | FILTER_SISO_DECODE)))
    {
        if (__smartcfg_filter_mode & FILTER_SISO_DECODE)
        {
            if (len >= SISO2_SAFE_BASE && len < SISO2_SAFE_BASE+SISO2_SAFE_RANGE)
            {
                // Call siso2 decode algorithm
                ret = siso2_smart_decode(ptr, cfg, len);
            }
            else
            {
                // Call siso1 decode algorithm
                ret = siso_smart_decode(ptr, cfg, len);
            }

            if(ret==DECODE_DONE)
                goto decode_done;
        }
        else
        {
            ret = siso_signature_search(ptr, cfg, len);

            if(ret==STAY_IN_THE_CHANNEL)
            {
                cfg->state = S_STAY_CHANNEL;
            }
            else if(ret==SIGNATURE_LOCKED)
            {
#ifdef PLAY_OMNI_VOICE
                // Only play omni_voice in situation that never lock before
                if (!(__smartcfg_filter_mode & (FILTER_MIMO_DECODE | FILTER_SISO_DECODE)))
                {
                    __smartcfg_voice_type = S_VOICE_CFG;
                    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, HZ);
                }
#endif

                cfg->state = S_DECODE;
                cfg->decoder = SISO_DECODER;
                cfg->next_decode_status_check_time = jiffies + SMART_DECODE_STATUS_CHECK_INTERVAL;
                __smartcfg_filter_mode |= FILTER_SISO_DECODE;
            }
        }
    }

    // Dispatch data for "MIMO" algorithm
    if ((__smartcfg_filter_mode & FILTER_MIMO_SCAN) && (flags & (FILTER_MIMO_SCAN | FILTER_MIMO_DECODE)))
    {
        if(__smartcfg_filter_mode & FILTER_MIMO_DECODE)
        {
            if(memcmp(mac_addr, lock_mac_addr, 6))
                goto done;
            if (flags & DISPATCH_MIMO_SCAN)
            {
                cfg->signature_count++;
                ret = DECODE_IN_PROC;
            }
            else if(flags & DISPATCH_MIMO_DECODE)
            {
#if defined(DECODE_AP_FORWARD_DATA)
                if(ap_addr && is_macaddr_all_zero(mimo_locked_ap_addr))
                    register_mimo_locked_ap_addr(ap_addr);
#endif

                if (NULL!=lock_data)
                {
                    free_lock_data(lock_data);
                    kfree(lock_data);
                    lock_data=NULL;
                }

                if (NULL==blocks)
                {
                    blocks=(struct sc_rx_block *)kmalloc(sizeof(struct sc_rx_block),GFP_ATOMIC);
                    if (NULL==blocks)
                    {
                        printk("!ERR: blocks memory not allocate\n");
                        goto done;
                    }
                    memset(blocks, 0, sizeof(struct sc_rx_block));
                    blocks->current_rx_block = -1;
                }
                cfg->valid_pkt_count++;
                ret = mimo_smart_decode(len, blocks);
            }

            if(ret==DECODE_DONE)
                goto decode_done;
        }
        else
        {
            if(flags & DISPATCH_MIMO_SCAN)
            {
                if (NULL!=blocks)
                {
                    kfree(blocks);
                    blocks=NULL;
                    memset(lock_mac_addr,0,WLAN_ADDR_LEN);
                }
                if (NULL==lock_data)
                {
                    lock_data=(struct try_lock_data *)kmalloc(sizeof(struct try_lock_data), GFP_ATOMIC);
                    if (NULL==lock_data)
                    {
                        printk("!ERR: lock_data not allocate\n");
                        goto done;
                    }
                    memset(lock_data,0,sizeof(struct try_lock_data));
                }
    
                ret = mimo_signature_search(len, mac_addr, lock_data);

                if(ret==STAY_IN_THE_CHANNEL)
                {
                    cfg->state = S_STAY_CHANNEL;
                }
                else if(ret==SIGNATURE_LOCKED)
                {
#ifdef PLAY_OMNI_VOICE
                    // Only play omni_voice in situation that never lock before
                    if (!(__smartcfg_filter_mode & (FILTER_MIMO_DECODE | FILTER_SISO_DECODE)))
                    {
                        __smartcfg_voice_type = S_VOICE_CFG;
                        ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_work, HZ);
                    }
#endif

                    cfg->state = S_DECODE;
                    cfg->decoder = MIMO_DECODER;
                    cfg->next_decode_status_check_time = jiffies + SMART_DECODE_STATUS_CHECK_INTERVAL;
                    __smartcfg_filter_mode |= FILTER_MIMO_DECODE;
                    memcpy(lock_mac_addr, mac_addr, ETH_ALEN);
#if defined(DECODE_AP_FORWARD_DATA)
                    if(ap_addr)
                        register_mimo_locked_ap_addr(ap_addr);
                    else
                        unregister_mimo_locked_ap_addr();
#endif
                }
            }
        }
    }

done:
    return 0;

decode_done:
#ifdef PLAY_OMNI_VOICE
    __smartcfg_voice_type = S_VOICE_NONE;
#endif
    disable_sniffer_mode();
    cfg->state = S_DONE;
    __smartcfg_filter_mode = FILTER_MODE_DROP_ALL;
    ieee80211_queue_delayed_work(sc->hw, &sc->smartcfg_usrwork, HZ);
    return 0;
}

int smartcfg_process(struct rx_q *rxq)
{
    volatile rx_descriptor *rx_desc;
	volatile buf_header *bhdr=NULL;
	struct wbuf *wb, *wb1, *wb_ptr;
	u16 bhdr_head_index, bhdr_tail_index;
	u8 hw_buf;
	u16 flags;
	int count = 0;
    u8 *data;

	wla_lock();
	/* handle HW RX Queue */
	for(rx_desc = &rxq->desc_hdr[rxq->index]; (rx_desc->own == 0) && (++count <= rxq->max); 
			rx_desc = &rxq->desc_hdr[rxq->index])
	{
		/* initialize temp variables per desacriptor */
		wb = wb1 = 0;
		flags = WBUF_RX_FRAME|WBUF_FIXED_SIZE;

		bhdr_head_index = rx_desc->bhdr_head_index;
		bhdr_tail_index = rx_desc->bhdr_tail_index;

#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
		sniffer_rx_count++;
#endif

		if(!rx_desc->swb)
		{
			/* check legal range */
			if((bhdr_head_index >= info->hw_buf_headers_count)||
				(bhdr_tail_index >= info->hw_buf_headers_count))
			{
				WLA_DBG(WLAWARN, "%s(%d) Wrong head idx=%x, tail idx=%x, limit=%x\n", __func__,
					__LINE__,  bhdr_head_index, bhdr_tail_index, info->hw_buf_headers_count);
#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
				wla_rx_monitor(&info->mon_wronghead_cnt, "Wrong head");
#else
				printk("Wrong head !!!");
#endif
			}

			hw_buf = 1;
			WLA_DBG(WLAWARN, "??????HW frame's Drop cause: %x, rx_desc[%d]->pkt_length=%d\n", ((u32 *)rx_desc)[3], rxq->index, rx_desc->pkt_length);
			WLA_DUMP_BUF_16BIT((char *)rxq->desc_hdr, info->wl_rxq.max*16);
		}
		else
		{
			/* check legal range */
			if((bhdr_head_index >= info->sw_tx_bhdr_start)||
				(bhdr_head_index < info->hw_buf_headers_count)||
				(bhdr_tail_index >= info->sw_tx_bhdr_start)||
				(bhdr_tail_index < info->hw_buf_headers_count))
			{
				WLA_DBG(WLAWARN, "??????%s(%d) Wrong head idx=%x, tail idx=%x, limit=%x\n", 
					__func__, __LINE__, bhdr_head_index, bhdr_tail_index,
					info->sw_allocated_buf_headers_count);
				WLA_DBG(WLADEBUG, "rx_desc->pkt_length=%d\n", rx_desc->pkt_length);
				goto fail2;
			}

			if(rx_desc->pkt_length == 0)
			{
				/* UMAC may drop BAR with zero length to CPU */
				WLA_DBG(WLAWARN, "WARNING: zero rx_desc->pkt_length, %08x-%08x-%08x-%08x, rxq->index=%d\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], rxq->index);

				goto fail2;
			}

            hw_buf = 0;

		}

		/* Give up all tasks once MAC shutdown */
		if(!info->mac_is_ready)
			goto fail;

        if(FILTER_MODE_DROP_ALL==__smartcfg_filter_mode)
            goto fail;

		bhdr = idx_to_bhdr(info, bhdr_head_index);

        wb_ptr = dptr_to_wb((char *)(u32)bhdr->dptr);
        data = (u8 *)((int)wb_ptr + (int)get_bhdr_offset((buf_header *)bhdr));

		if((rx_desc->drop) || (rx_desc->crc))
		{
#if 0
            if((data[0]==0x88) || (data[0] == 0x08))
            {
                if((data[4] & 0x01) && (data[4]!=0xff))
                printk("(%d) %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", rx_desc->pkt_length, data[4], data[5], data[6], data[7], data[8], data[9]
                       , data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21]);
            }
#endif

            smartcfg_filter((u8 *)UNCAC_ADDR(data), (0x1fff & rx_desc->pkt_length), rx_desc);
            goto fail;
		}
        else if((data[0]==0x88) || (data[0] == 0x08))
        {
#if 0
            if((data[4] & 0x01) && (data[4]!=0xff))
            {    
                printk("(%d) %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", rx_desc->pkt_length, data[4], data[5], data[6], data[7], data[8], data[9]
                       , data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21]);
            }
#endif
            smartcfg_filter((u8 *)UNCAC_ADDR(data), (0x1fff & rx_desc->pkt_length), rx_desc);
            goto fail;
        }

		if(((bhdr->offset_h << 6) | bhdr->offset ) == 0)
		{
			WLA_DBG(WLAWARN, "Zero data offset: %08x, %08x, %08x, %08x, bhdr=%p(%08x)\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], bhdr, *(u32 *)bhdr);

#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
			wla_rx_monitor(&info->mon_zerodataofs_cnt, "Zero data offset");
#else
			printk("wla_rx_wlq():Zero data offset\n");
#endif
		}

#ifdef CONFIG_SRAM_WLA
#error not supported 
#endif
		if(wb)
		{
		}
		/* multiple buffers or Hardware buffers need to copy to one buffer */
		else if((bhdr_head_index != bhdr_tail_index) || (!rx_desc->swb))
		{
			goto fail;
		}
		/* signle buffer, just exchange buffer's address to gain performance */
		else
		{
            struct wlan_qoshdr *fm;

            wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
            wb->wb_next=0;
            wb->rb_next=0;

            if (wb->flags != WBUF_FIXED_SIZE)
            {
                WLA_DBG(WLAERROR, "wla_rx_wlq(): wrong wb->flags:%x, bhr_head=%d, index=%d\n", wb->flags, bhdr_head_index, rxq->index);
                WLA_MSG("~~~~~~~~wb=%p\n", wb);
#if defined(CONFIG_WMAC_RECOVER) && defined(ENABLE_MAC_RECOVERY)
                wla_rx_monitor(&info->mon_wrongflag_cnt, "wrong wb->flags");
#else
                printk("wla_rx_wlq(): wrong wb->flags !!!");
#endif
            }

            if (!wb->wh)
                goto fail;

            wb->data_off = wb->data_off + WLAN_TX_MIN_RESERVED_LEADIND_SPACE;

            fm = (struct wlan_qoshdr *) UNCAC_ADDR(data);
            switch (fm->type)
            {
                case WLAN_FC_TYPE_MGT:
                    {
                        if (fm->subtype == WLAN_FC_SUBTYPE_PROBE_REQ)
                        {
                            unsigned int *bhdr_ptr;
    
                            if ((wb1 = WBUF_ALLOC(0)) == 0)
                            {
                                WLA_DBG(WLAERROR, "wla_rx_wlq(): wbuf_alloc failed\n");
                                goto fail;
                            }
    
                            wb->flags |= WBUF_MGMT_FRAME;
                            if (info->wla_mgt_handler)
                            {
                                //SC_DBG("Probe request\n");
                                info->wla_mgt_handler(wb, NULL);
                            }
    
                            bhdr_ptr = (unsigned int *)bhdr;
                            bhdr_ptr[0] = (1 << 15);
                            bhdr_ptr[1] = (int)wb_to_dptr(wb1);
    #if defined(WBUF_IS_CACHED)
                            DCACHE_INVALIDATE((unsigned int)wb1, 1600);
    #endif
                        }
                        break;
                    }
                case WLAN_FC_TYPE_CTRL:
                    {
                        if (fm->subtype == WLAN_FC_SUBTYPE_BA)
                        {
                            smartcfg_filter((u8 *)fm, wb->pkt_len, rx_desc);
                        }
                        else if (fm->subtype == WLAN_FC_SUBTYPE_ACK)
                        {
                            smartcfg_filter((u8 *)fm, wb->pkt_len, rx_desc);
                        }
                        break;
                    }
                    //case WLAN_FC_TYPE_DATA:
                default:
                    break; 
            }
		}

        rxq->pkt_count++;
fail:	/* There are valid buffer headers and release to hardware. */
		mac_free_buffer_hdr(bhdr_head_index, bhdr_tail_index, hw_buf);
/* The descriptor has invalid buffer header, just return descriptor */
fail2:
		//rx_desc->bhdr_head_index = rx_desc->bhdr_tail_index = 0;
		rx_desc->own = 1;
		rxq->index = (rxq->index + 1) % rxq->max;

		MACREG_WRITE32(CPU_RD_SWDES_IND, 1);     /* kick RX queue logic as it might be pending on get free RX descriptor to write */
	}

	wla_unlock();

    return count;
}
#endif

void gen_bit_mask(unsigned int start, unsigned int end, unsigned int *mask)
{
    int block_start, block_end;
    int low, high, i;
    unsigned int reverse_mask=0;

    if(start > end)
    {
        low = end;
        high = start;
    }
    else
    {
        low = start;
        high = end;
    }
    
    for(i=0; i < SC_LOCK_PHASE_BIT_MAP_NUMS; i++)
    {
        block_start = 32*i;
        block_end = 32*(i + 1);
        mask[i] = 0;

        if(start == end)
            continue;
        else
        {
            if((low < block_end) && (high > block_start))
            {
                if(low < block_end)
                {
                    if(low <= block_start)
                        mask[i] = (unsigned int)-1;
                    else
                        mask[i] = (unsigned int)(0 - (1 << (low - block_start)));
                }
                if(high < block_end)
                {
                    reverse_mask = (1 << (high - block_start)) - 1;
                    mask[i] = mask[i] & reverse_mask;
                }
            }
        }
    }
}

struct lock_phase_ta* insert_lock_data(unsigned int data, unsigned char *addr, struct try_lock_data *lock_data)
{
    int current_pos=0;
    int i, x, y;
    struct lock_phase_ta *ta=NULL, *find=NULL, *pre=NULL;
    int observe_window = (SC_FRAME_TIME_INTERVAL + 10) * 4;
    unsigned int current_time = current_timestamp();
    unsigned int bit_mask[SC_LOCK_PHASE_BIT_MAP_NUMS];

    /* pass too old entries */
    while(lock_data->start_pos != lock_data->next_pos) // "==" : no entry
    {
#if 0
        SC_PRINT("current_time=%d, timestamp[%d]=%d, diff=%d, observe_window=%d\n",
                current_time, lock_data->start_pos, lock_data->timestamp[lock_data->start_pos],
                ((int)current_time - (int)lock_data->timestamp[lock_data->start_pos]), observe_window);
#endif
        if(((int)current_time - (int)lock_data->timestamp[lock_data->start_pos]) > observe_window)
        {
            if(++lock_data->start_pos >= SC_LOCK_PHASE_DATA_NUMS)
                lock_data->start_pos = 0;
        }
        else
            break;
    }

    /* insert data */
    lock_data->frames[lock_data->next_pos] = data;
    lock_data->timestamp[lock_data->next_pos] = current_time;
    current_pos = lock_data->next_pos;

    /* lock_data->next_pos causes the valid data num = (SC_LOCK_PHASE_DATA_NUMS - 1).
       It is acceptable.                                                                */
    if((++lock_data->next_pos) >= SC_LOCK_PHASE_DATA_NUMS)
        lock_data->next_pos = 0;
    if(lock_data->next_pos == lock_data->start_pos)
    {
        if(++lock_data->start_pos >= SC_LOCK_PHASE_DATA_NUMS)
            lock_data->start_pos = 0;
    }

#if 0
    SC_PRINT("%s(): next = %d, start=%d\n", __FUNCTION__, lock_data->next_pos, lock_data->start_pos);
#endif

    gen_bit_mask(lock_data->start_pos, lock_data->next_pos, bit_mask);
#if 0
    SC_PRINT("mask = ");
    for(i=0; i<SC_LOCK_PHASE_BIT_MAP_NUMS; i++)
        SC_PRINT("0x%08x:", bit_mask[i]);
    SC_PRINT("\n");
#endif
    ta = lock_data->ta_list;
    while(ta)
    {
        if((find == NULL) && (memcmp(ta->addr, addr, 6) == 0))
            find = ta;

        /* rebuild the bit_map */
        for(i=0; i<SC_LOCK_PHASE_BIT_MAP_NUMS; i++)
            ta->bit_map[i] &= bit_mask[i];

        pre = ta;
        ta = ta->next;
    }

    if(find == NULL)
    {
        find = (struct lock_phase_ta *)SC_MALLOC(sizeof(struct lock_phase_ta));
        if(find == NULL)
            return NULL;
        memset(find, 0, sizeof(struct lock_phase_ta));
        memcpy(find->addr, addr, 6);
        if(pre)
            pre->next = find;
        else
            lock_data->ta_list = find;
    }

    x = (current_pos)/(32);
    y = (current_pos)%(32);
    find->bit_map[x] |= (1 << y);

#if 0
    SC_PRINT("%s(): x=%d, y=%d, current_pos=%d, find->bit_map[x]=0x%08x\n",
            __FUNCTION__, x, y, current_pos, find->bit_map[x]);
#endif

    return find;
}

static int match_arr[32][3];
int lock_match(struct lock_phase_ta *ta, struct try_lock_data *lock_data)
{
    int ptr = lock_data->start_pos;
    int i, j;
    int x, y;
    int header_len=0;
    unsigned int comp_val=0;
    memset(match_arr, 0, sizeof(match_arr));

    while(ptr != lock_data->next_pos)
    {
        x = (ptr)/32;
        y = (ptr)%32;
        
        if(ta->bit_map[x] & (1 << y))
        {
            comp_val = lock_data->frames[ptr] - SC_LOCK_INTERVAL_LEN;

            for(i=0; i<32; i++)
            {
                if(match_arr[i][0] == 0)
                {
                    match_arr[i][0] = lock_data->frames[ptr];
                    break;
                }
                else
                {
                    for(j=2; j>=0; j--)
                    {
                        if((match_arr[i][j] != 0) && (match_arr[i][j] == comp_val))
                        {
                            if(j == 2)
                            {
                                header_len = match_arr[i][0] - SC_LOCK_SPEC_CHAR - SC_LOCK_LENGTH_BASE;
#if defined(MIMO_DEBUG)
                                SC_PRINT("find the match value header_len=%d (0)\n", header_len);
#endif
                                return header_len; 
                            }
                            else
                            {
                                match_arr[i][j+1] = lock_data->frames[ptr];
                            }
                        }
                    }
                }
            }
        }

        if(++ptr >= SC_LOCK_PHASE_DATA_NUMS)
            ptr = 0;
    }

#if 0
    for(i =0; i<32; i++)
    {
        SC_PRINT("ta=%02x:%02x:%02x:%02x:%02x:%02x:, comp_val=%x, match_arr[%d] = %04x:%04x:%04x\n", 
                ta->addr[0], ta->addr[1], ta->addr[2], 
                ta->addr[3], ta->addr[4], ta->addr[5], comp_val,
                i, match_arr[i][0], match_arr[i][1], match_arr[i][2]);
    }
#endif
    return 0;
}

int try_lock_channel(unsigned int data, unsigned char *addr, struct try_lock_data *lock_data, int *ret_state)
{
    int header_len=0;
    static unsigned long limit_time = 0;
    struct lock_phase_ta *ta;

    if(time_after(limit_time, jiffies))
    {
        *ret_state=STAY_IN_THE_CHANNEL;
    }
    ta = insert_lock_data(data, addr, lock_data);
    if(ta)
    {
        limit_time = jiffies + MIMO_LOCK_ATTEMPT_TIME;
        header_len = lock_match(ta, lock_data);
    }

    return header_len;  
}

int free_lock_data(struct try_lock_data *lock_data)
{
    struct lock_phase_ta *ta, *next;

    ta = lock_data->ta_list;

    while(ta)
    {
        next = ta->next;
        SC_FREE(ta);
        ta = next;
    }
    return 0;
}
unsigned char* sc_decoder(unsigned int data, struct sc_rx_block *blocks)
{
    static unsigned char pre_control_data = 0x00;
    int total_length = 0;
    struct sc_block_entry *entry;
    unsigned char *ret=NULL, *ptr=NULL;
    int block_num=0;
    int i, tmp, total_group = 0;
    int check;
    uint8_t data_crc;

    if(SMART_CONFIG_LENGTH_BASE > data)
        return NULL;

    data = data - SMART_CONFIG_LENGTH_BASE;
    
    if(data & SMART_CONFIG_LENGTH_MASK)        // not 2x2 smart config info
    {
        //SC_PRINT("%s(): not in the sc location\n", __FUNCTION__);
        return NULL;
    }

    data = data >> SMART_CONFIG_SHIFT;
    check = (blocks->block_counter[0] >> 1);

#if defined(MIMO_DEBUG)
    SC_PRINT("data %02x\n",data);
#endif
    if(pre_control_data == data)
        return NULL;
    else
        pre_control_data = data;

    if((blocks->pending_rx_data) && (data & SC_CONTROL_DATA)) // pass this data info rx
    {
        //SC_PRINT("%s(): pending rx data\n", __FUNCTION__);
        return NULL;
    }
#if defined(MIMO_DEBUG)
    SC_PRINT("accept data=%x\n",data);
#endif
    if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
    {
        printk(KERN_INFO "accept data=%x\n",data);
    }
    if(data & SC_CONTROL_DATA)  // data
    {
        if(0>blocks->current_rx_block)
            return NULL;

        entry = &blocks->block[blocks->current_rx_block];
        if(blocks->current_rx_block < SMART_CONFIG_BLOCK_NUMS)
        {
            if(entry->current_rx_count < SMART_CONFIG_BLOCK_FRAMES)
            {    
                entry->value[entry->current_rx_count] = (data & 0xf);
                entry->current_rx_count++;
                if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                {
                    printk(KERN_INFO "\t\t\tpass\n");
                }
            }
        }
    }
    else//control
    {
        block_num = (data - SC_CTL_SSID_BLOCK_0);
        if(block_num < SMART_CONFIG_BLOCK_NUMS && block_num >= 0)
        {
            SC_PRINT("find the group %d, pending_rx_data=%d, ",block_num, blocks->pending_rx_data);
            if(blocks->current_rx_block>=0)
            {
                entry = &blocks->block[blocks->current_rx_block];
                SC_PRINT("current_rx_count=%d, ", entry->current_rx_count);

                if(blocks->pending_rx_data)
                    blocks->pending_rx_data = 0;
                else if((entry->current_rx_count == SMART_CONFIG_BLOCK_FRAMES) &&
                    ((block_num == (blocks->current_rx_block + 1)) || (block_num == 0)))
                {
                    blocks->block_map |= (1 << blocks->current_rx_block);
                }
            }

            SC_PRINT("current block %d, block_map %x\n",
                     blocks->current_rx_block, blocks->block_map);

            if(blocks->block_map & 0x01)
            {
                entry = &blocks->block[0];
                total_length = (entry->value[0] << 4) | entry->value[1];
                total_length+=2;            
            }

            if(blocks->block_map & (1 << block_num))
                blocks->pending_rx_data = 1;
            else
            {
                entry = &blocks->block[block_num];
                entry->current_rx_count = 0;
            }
            
            blocks->current_rx_block = block_num;

            if(blocks->block_counter[block_num] < 255)
                blocks->block_counter[block_num]++;
            if(blocks->block_map)
            {
                for(i=(SMART_CONFIG_BLOCK_NUMS-1); i>=0; i--)
                {
                    if(blocks->block_counter[i] == 0)
                        continue;
                    // we expect that the rx group id conters should be closed

                    if((blocks->block_counter[0] > 3) && (blocks->block_counter[i] < check))
                        continue;
                    tmp = (1 << (i+1)) - 1;
                    if((tmp & blocks->block_map) == tmp)
                    {
                        total_group = i+1;
                        if(total_group >= ((total_length+7)/8))
                        {
                            ret = (unsigned char *)SC_MALLOC(
                                            SMART_CONFIG_BLOCK_BYTES * total_group);
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            /* do other control action */
        }
    }
    if(ret) // complete all blocks
    {
        ptr = ret;

        for(i=0; i < total_group; i++)
        {
            entry = &blocks->block[i];
            for(tmp=0; tmp < SMART_CONFIG_BLOCK_FRAMES; tmp += 2)
            {
                *ptr = (entry->value[tmp] << 4) | entry->value[tmp+1];
                ptr++;
            }
        }
        data_crc = Crc8(ret, total_length - 1);
        if(data_crc == ret[total_length-1])
        {
            printk("Pass crc check, crc = %02x\n", data_crc);
        }
        else
        {
            printk("Wrong crc!!, correct crc = %02x, now crc = %02x\n", ret[total_length-1], data_crc);
            //ReDecoding until pass crc check;
            for(i=0; i < total_length; i++)
            {
                if(smrtcfg_debug_flag == SMRTCFG_DEBUG_FLAG_OPEN)
                {    
                    printk(KERN_INFO "%02d:%02x %c\n", i, ret[i], ret[i]);
                }
            }
            memset(blocks, 0, sizeof(struct sc_rx_block));
            blocks->current_rx_block = -1;
            SC_FREE(ret);
            ret = NULL;
        }
#if defined(MIMO_DEBUG)
        if ((SMART_CONFIG_BLOCK_BYTES * total_group) <= ret[0])
        {
            SC_PRINT("!!!Decode length error\n");
        }
        else
        {
            SC_PRINT("ret=%02x",ret[0]);
            for(i=0; i<ret[0]; i++)
            {
                SC_PRINT(" %02x",ret[i+1]);
            }
            SC_PRINT("\n");
        }
#endif
    }

    return ret;
}

#if defined(USR_SPACE_TEST)
unsigned int* lock_stream_gen(unsigned int *stream, unsigned int base, unsigned int spec_char, unsigned int interval)
{
    stream[0] = base + spec_char;
    stream[1] = stream[0] + interval;
    stream[2] = stream[1] + interval;
    stream[3] = stream[2] + interval;

    return stream;
}

unsigned int* append_even_odd(unsigned int *stream, unsigned char data)
{
    // one byte data seperate to two data length
    *stream = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = ((data | SC_EVEN_ODD) << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;

    return stream;
}
unsigned int* append_even_control(unsigned int *stream, unsigned char data)
{
    *stream = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    return stream;
}
unsigned int* append_even(unsigned int *stream, unsigned char data)
{
    *stream = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = (data << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    return stream;
}
unsigned int* append_odd(unsigned int *stream, unsigned char data)
{
    *stream = ((data | SC_EVEN_ODD) << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = ((data | SC_EVEN_ODD) << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    *(++stream) = ((data | SC_EVEN_ODD) << SMART_CONFIG_SHIFT) + SMART_CONFIG_LENGTH_BASE;
    return stream;
}

unsigned int* sc_encoder(unsigned char *data, int data_len, int *encoded_stream_len)
{
    int i;
    unsigned char block_count=0, tmp=0;
    unsigned char *data_end = data + data_len;
    unsigned int *stream, *ptr;
    int stream_len;

  //stream_len = ((data_len << 1) + (data_len / SMART_CONFIG_BLOCK_BYTES)) << 1;  // << 1 is the same as *2
    stream_len = (((data_len << 1) + (data_len / SMART_CONFIG_BLOCK_BYTES)) << 1) + (data_len << 1) + ((data_len / SMART_CONFIG_BLOCK_BYTES) << 1);
    if((stream = (unsigned int *)malloc(stream_len*sizeof(unsigned int))) == NULL)
        return NULL;

    ptr = stream;

    while(data < data_end)
    {
      //ptr = append_even_odd(ptr, (SC_CTL_SSID_BLOCK_0 + block_count++));
        ptr = append_even_control(ptr, (SC_CTL_SSID_BLOCK_0 + block_count++));
        for(i=0; i<8; i++)
        {
            tmp = ((*data & 0xf0) >> 4) | SC_CONTROL_DATA;
          //ptr = append_even_odd(++ptr, tmp);
            ptr = append_even(++ptr, tmp);
            tmp = (*data & 0x0f) | SC_CONTROL_DATA;
          //ptr = append_even_odd(++ptr, tmp);
            ptr = append_odd(++ptr, tmp);
            if(++data == data_end)
                break;
        }
        ptr++;
    }

    *encoded_stream_len = stream_len;
    return stream;
}

bool parser_encode_opt(int argc, char* argv[], struct opt_test_encode *data)
{
    if (argc <= encode_arg_user) {
        printf("prog <prefer_header_len> <wisp(3) or station(9)> <user> <password> <frame lost ratio> <rx_loop>\n");
        printf("ex: prog 26 3 user password 10 100\n");
        printf("prefer_header_len : 0 is random from 0 ~ 100\n");
        printf("    10 : 10%% frame lost ratio (default = 0)\n");
        printf("    100 : 1000 loop to rx data, each loop has 1000 frames (default = 100)\n");
        return false;
    }
    data->prefer_header_length = atoi(argv[encode_arg_prefer_header_length]);
    data->mode = atoi(argv[encode_arg_mode]);
    data->user = argv[encode_arg_user];
    data->password = NULL;
    data->frame_lost_ratio = 0;  // default value
    data->rx_loop = 100;         // default value

    if (argc > encode_arg_pw)
        data->password = argv[encode_arg_pw];
    if(argc > encode_arg_frameloss_ratio)
        data->frame_lost_ratio = atoi(argv[encode_arg_frameloss_ratio]);
    if(argc > encode_arg_rx_loop)
        data->rx_loop = atoi(argv[encode_arg_rx_loop]);

    return true;
}

void dump(unsigned char *data, int len, int type)
{
    unsigned int *data_32 = (unsigned int *)data;

    int i;
    for(i=0; i<len; i++)
    {
        if(((i%16) == 0) && (i != 0))
            printf("\n");
        if(type == 8)
            printf("%02x ", data[i]);
        else
            printf("%04x ", data_32[i]);
    }
            
    printf("\n\n");
}
unsigned int * do_encode(struct opt_test_encode *opts, unsigned char ** plaintext, int *plaintext_len,
                         int * encoded_stream_len, int real_header_length)
{
    int user_len = (int)strlen(opts->user);
    int pw_len = (opts->password == NULL) ? 0 : (int)strlen(opts->password);
    int data_len = user_len + pw_len + 4;   // 4 is total + mode  + user_len + pw_len
    int stream_len = (data_len % 8 != 0) ? ((data_len + 8) >> 3) << 3 : data_len; // make sure stream_len is multiples of 8
    printf("%s(): data_len=%d, stream_len=%d, lost_ratio=%d\n", __FUNCTION__,
           data_len, stream_len, opts->frame_lost_ratio);

    unsigned char * stream = NULL;
    if((stream = (unsigned char *)malloc(stream_len)) == NULL)
    {
        printf("main: no resource\n");
        return NULL;
    }
    *plaintext_len = stream_len;
    // filled plaintext as [totalLength][mode][userLen][user][pwLen][password]
    // totalLength is count from [mode]
    memset(stream, 0, stream_len);
    index = 0;
    stream[index++] = (data_len-1) & 0xFF;
    stream[index++] = opts->mode & 0xFF;
    stream[index++] = user_len & 0xFF;
    memcpy(stream+index, opts->user, user_len);
    index += user_len;
    stream[index++] = pw_len & 0xFF;
    memcpy(stream+index, opts->password, pw_len);

    uint8_t data_crc8 = Crc8(stream, data_len);
    stream[index++] = data_crc8;//crc8



    *plaintext = stream;
    

    unsigned int * encoded_stream;
    if((encoded_stream = sc_encoder(stream, stream_len, encoded_stream_len)) == NULL)
    {
        printf("encode no resource\n");
        return NULL;
    }

    printf("original stream (ptr=%p, len=%d): \n", stream, stream_len);
    dump(stream, stream_len, 8);
    printf("encoded stream (ptr=%p, len=%d): \n", encoded_stream, *encoded_stream_len);
    dump((unsigned char *)encoded_stream, *encoded_stream_len, 32);

    int i;
    for (i = 0; i < (*encoded_stream_len); ++i) {
        encoded_stream[i] += real_header_length;
    }

    printf("encoded stream (after adding header length): \n");
    dump((unsigned char *)encoded_stream, *encoded_stream_len, 32);
    return encoded_stream;
}

unsigned int * gen_air_packets(struct opt_test_encode *opts)
{
    unsigned int lock_stream[4];
    srandom((int)time(0));
    int random_header_len = (opts->prefer_header_length == 0) ? (random() % 100) + 1 : opts->prefer_header_length;
    printf("random header len %d\n", random_header_len);

    /* gen stream */
    lock_stream_gen(lock_stream, SC_LOCK_LENGTH_BASE, SC_LOCK_SPEC_CHAR, SC_LOCK_INTERVAL_LEN);

    int i;
    for(i=0; i<4; i++)
    {
        lock_stream[i] += random_header_len;
    }

    printf("lock_stream = 0x%08x:0x%08x:0x%08x:0x%08x\n",
           lock_stream[0], lock_stream[1], lock_stream[2], lock_stream[3]);

    unsigned char *plaintext = NULL;
    unsigned int *ciphertext = NULL;
    int encoded_stream_len = 0;
    int plaintext_len = 0;
    ciphertext = do_encode(opts, &plaintext, &plaintext_len, &encoded_stream_len, random_header_len);
    if (NULL == ciphertext)
        goto free_resource;
#if defined(MIMO_DEBUG)
    printf("original stream (ptr=%p, len=%d): \n", plaintext, plaintext_len);
    dump(plaintext, plaintext_len, 8);
    printf("encoded stream (ptr=%p, len=%d): \n", ciphertext, encoded_stream_len);
    dump((unsigned char *)ciphertext, encoded_stream_len, 32);
#endif

    unsigned int * gen_air_stream = malloc(MAX_AIR_PACKET * sizeof(unsigned int));
    if (NULL == gen_air_stream) {
        goto free_resource;
    }

    FILE * fp = fopen("abc.txt", "wb");
    if (fp == NULL) {
        printf("failed to open abc.txt");
    }

    int lockstream_idx = 0, cipher_idx = 0;
    char buf[8];

    for (i = 0; i < MAX_AIR_PACKET; ++i)
    {
        int rand_opt = random() % 3;

        switch (rand_opt)
        {
            case 0:
                gen_air_stream[i] = ciphertext[cipher_idx++];
                cipher_idx = cipher_idx % encoded_stream_len;
                break;
            case 1:
                gen_air_stream[i] = lock_stream[lockstream_idx++];
                lockstream_idx = lockstream_idx % 4;
                break;

            default:
                gen_air_stream[i] = (random() % (1518 - random_header_len)) + random_header_len;
                break;
        }
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%d\n", gen_air_stream[i]);
        fwrite(buf, sizeof(char), strlen(buf), fp);
    }
    fclose(fp);

free_resource:
    if(plaintext)
    {
        free(plaintext);
    }
    if(ciphertext)
    {
        free(ciphertext);
    }

    return gen_air_stream;
}

void try_lock_and_decede(unsigned int * simulate_air, unsigned int size)
{
    unsigned int i = 0;
    unsigned int header_len = 0;
    unsigned char addr[6] = {0x33, 0xbb, 0x33, 0x77, 0x55, 0x66};
    struct try_lock_data lock_data;
    memset(&lock_data, 0, sizeof(struct try_lock_data));

    for (; i < size; ++i)
    {
        header_len = try_lock_channel(simulate_air[i], addr, &lock_data);
        if(header_len)
            break;
    }

    free_lock_data(&lock_data);

    if (header_len == 0)
    {
        printf("cannot lock");
        return;
    }

    struct sc_rx_block *blocks = NULL;
    if((blocks = malloc(sizeof(struct sc_rx_block))) == NULL)
    {
        return;
    }

    memset(blocks, 0, sizeof(struct sc_rx_block));
    blocks->pending_rx_data = 1;

    unsigned int sc_data = 0;
    unsigned char *ret = NULL;
    for (; i < size; ++i) {
        sc_data = simulate_air[i] - header_len;
        ret = sc_decoder(sc_data, blocks);
        if(ret != NULL)
            break;
    }

    if (ret != NULL) {
        printf("\ndecoded stream: \n");
        int plain_text_len = (int)strlen((const char*)ret);
        if (plain_text_len % 8 != 0) {
            plain_text_len = ((plain_text_len + 8) >> 3) << 3;
        }
        dump(ret, plain_text_len, 8);
    }

    if (blocks)
    {
        free(blocks);
    }
    if (ret)
    {
        free(ret);
    }
}

/*!
 *  full run unit test
 *
 *  @param opts unit_test
 */
void unit_test(struct opt_test_encode opts)
{
    unsigned int * simulate_air = gen_air_packets(&opts);
    try_lock_and_decede(simulate_air, MAX_AIR_PACKET);
}

/*!
 *  test
 *
 *  @param filepath file to read and decode
 */
void test_file(char * filepath) {
    FILE *fp = fopen(filepath, "r");
    if (fp == NULL) {
        printf("Failed to open file");
        return;
    }

    // try lock
    unsigned int header_len = 0;

    struct try_lock_data lock_data;
    memset(&lock_data, 0, sizeof(struct try_lock_data));

    unsigned char addr[6] = {0x33, 0xbb, 0x33, 0x77, 0x55, 0x66};

    char * buf = NULL;
    size_t len;
    while ((buf = fgetln(fp, &len))) {
        if (buf[len - 1] == '\n')
            buf[len - 1] = '\0';

        printf("%04d ", atoi(buf));
        header_len = try_lock_channel(atoi(buf), addr, &lock_data);
        if (header_len) {
            break;
        }
    }

    if (header_len == 0) {
        printf("\ncannot lock\n");
        return;
    }

    printf("\n\nfind the match value header_len=%d \n\n", header_len);

    struct sc_rx_block *blocks = NULL;
    if((blocks = malloc(sizeof(struct sc_rx_block))) == NULL)
        return;

    memset(blocks, 0, sizeof(struct sc_rx_block));
    blocks->pending_rx_data = 1;

    unsigned int sc_data = 0;
    unsigned char *ret = NULL;
    while ((buf = fgetln(fp, &len))) {
        if (buf[len - 1] == '\n')
            buf[len - 1] = '\0';

        if (buf[0] == 'b' || buf[0] == 'f') {
            continue;
        }

        printf("%04d ", atoi(buf));
        sc_data = atoi(buf) - header_len;
        ret = sc_decoder(sc_data, blocks);
        if(ret != NULL)
            break;
    }
    printf("\n");

    if (ret != NULL) {
        printf("\ndecoded stream: \n");
        int plain_text_len = (int)strlen((const char*)ret);
        if (plain_text_len % 8 != 0) {
            plain_text_len = ((plain_text_len + 8) >> 3) << 3;
        }
        dump(ret, plain_text_len, 8);
    }

    if (blocks)
        free(blocks);
    if (ret)
        free(ret);

    fclose(fp);

}

int main(int argc, char* argv[])
{
    if (argc == 2) {
        test_file(argv[1]);
        return 0;
    }

    struct opt_test_encode opts;
    if(false == parser_encode_opt(argc, argv, &opts))
        return 0;
    unit_test(opts);
    return 0;
}

char * fgetln(FILE *fp, size_t *len)
{
    static char *buf = NULL;
    static size_t bufsiz = 0;
    char *ptr;

    if (buf == NULL)
    {
        bufsiz = BUFSIZ;
        if ((buf = malloc(bufsiz)) == NULL)
        {
            return NULL;
        }
    }

    if (fgets(buf, bufsiz, fp) == NULL)
    {
        return NULL;
    }

    *len = 0;
    while ((ptr = strchr(&buf[*len], '\n')) == NULL)
    {
        size_t nbufsiz = bufsiz + BUFSIZ;
        char *nbuf = realloc(buf, nbufsiz);

        if (nbuf == NULL)
        {
            int oerrno = errno;
            free(buf);
            errno = oerrno;
            buf = NULL;
            return NULL;
        }
        else
        {
            buf = nbuf;
        }

        if (fgets(&buf[bufsiz], BUFSIZ, fp) == NULL)
        {
            buf[bufsiz] = '\0';
            *len = strlen(buf);
            return buf;
        }

        *len = bufsiz;
        bufsiz = nbufsiz;
    }

    *len = (ptr - buf) + 1;
    return buf;
}
 
#endif /* USR_SPACE_TEST */

#endif /* CONFIG_CHEETAH_SMART_CONFIG */
