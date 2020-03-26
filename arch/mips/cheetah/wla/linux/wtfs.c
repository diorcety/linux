#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/synclink.h>
#include <net/mac80211.h>
#include <os_compat.h>
#include <rf.h>
#include <ip301.h>
#include <rfc.h>
#include <wla_bb.h>
#include <linux_wla.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/common.h>
#include <wlan_def.h>

#include <math.h>
#include <rfc_comm.h>
#include <rfac.h>

#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/semaphore.h>

#include <linux/kthread.h>
#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
#include <asm/mach-cheetah/idb.h>
#endif

#define CMD_OK 1
#define CMD_SHOW_USAGE 2

#define MAX_CMD_STRING_LENGTH   256
#define MAX_CMD_ARGV 8

#define TEST_FILTER_MT_ACT      0x1
#define TEST_FILTER_BEACON      0x2
#define TEST_FILTER_MGT         0x4
#define TEST_FILTER_CTRL        0x8
#define TEST_FILTER_DATA        0x10
#define TEST_FILTER_DATA_BC     0x20

#define TEST_DUMP_SIMPLE        0x1
#define TEST_DUMP_PAYLOAD       0x2


static char __wt_cmd[MAX_CMD_STRING_LENGTH];

#define WT_Q_SUCCESS  0
#define WT_Q_ERROR    -1

struct wt_msgq{
    spinlock_t  lock;
    short       max_msgsz;
    char        max_qlen;
    char        qfree;
    char        *qhead;
    char        *qtail;
    char        *qbuf;
    bool        echo;
    struct semaphore   sem;
};

typedef struct { 
    //u32 start; 
    struct completion start;
    //u32 stop; 
    struct completion stop;
    s32 tx_repeat; 
    u32 tx_burst_num; 
    u32 tx_sleep_time; 
    u32 tx_len; 
    u8 tx_addr[6]; 
    u8 tx_short_preamble; 
    u8 tx_green_field;  
    u8 tx_sgi;  
 
    u8 rx_drop; 
    u8 rx_dump; 
    u32 rx_filter; 
    u32 rx_sleep_time; 
 
    u8 my_addr[6]; 
    u8 channel; 
    u8 secondary_channel; 
    u8 bss_desc; 
    u8 tx_qos;  
    u8 tx_rate; 
    u8 txpower; 
 
    u8 rx_echo; 
    u8 tx_echo; 
 
    u32 tx_count; 
    u32 rx_count; 
    u32 expect_count; 
    u32 fail_count; 
 
    struct wt_msgq *msgq_id;  
    //struct workqueue_struct *wt_wq;  
    struct wsta_cb *tx_wcb; 
 
    u8 rssi; 
 
} wla_test_cfg;

struct wt_work_struct
{
    struct work_struct _work;
    struct wbuf *rx_wb;
};

    /* default value */ 
wla_test_cfg acfg = { 
    //.start = 0, 
    //.stop = 1, 
    .tx_repeat = 0, 
    .tx_burst_num = 10, 
    .tx_sleep_time = 1, 
    .tx_len = 256, 
    .tx_addr = {0x00, 0x11, 0x11, 0x11, 0x11, 0x11}, 
    .tx_short_preamble = 0, 
    .tx_green_field = 0, 
    .tx_sgi = 0, 
 
    .rx_dump = 0, 
    .rx_echo = 0, 
    .rx_filter = 0, 
    .rx_sleep_time = 100, 
 
    .my_addr = {0x00, 0x11, 0x11, 0x11, 0x11, 0x12}, 
    .channel = 1, 
    .secondary_channel = 0, 
    .tx_qos = 1, 
    .tx_rate = 5, 
    .txpower = 0, 
 
    .msgq_id = NULL,
    //.wt_wq = NULL,
    .rssi = 0, 
}; 

wla_test_cfg *acfg_p = &acfg;
static struct task_struct *wt_tsk;
static struct work_struct *wt_rx_work;

static char g_tmp_txvga_cdb[255] = {0};

extern int get_args (const char *string, char *argvs[]);

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
int atoi(const char *cp)
{
    char *enddp;
    return simple_strtoul(cp, &enddp, 10); 
}

int hatoi(const char *cp)
{
    char *enddp;
    return simple_strtoul(cp, &enddp, 16); 
}

void os_thread_sleep(unsigned int t)
{
    msleep(0.1 * t);
}

void cdb_get_mac(unsigned short idx, unsigned char *mac)
{
    char cdb_mac[] = "xx:xx:xx:xx:xx:xx"; 
    wla_get_hw_mac(0, cdb_mac);
    memcpy(mac, wmaddr_aton(cdb_mac), sizeof(u8) * 6);
}

s8 bb_rssi_decode_mp(u8 val)
{
    // mp tool version from Jerry
    s8 lna_gain_b[4] = {0, 7, 13, 21}; //dB
    s32 lna_gain, vga_gain, tx_gain;

    lna_gain = lna_gain_b[(val >> 5) & 0x3];
    vga_gain = (val & 0x1f) * 2;
    tx_gain = lna_gain + vga_gain;
    return tx_gain;
}

char wla_rx_parser(struct wbuf *wb)
{
    struct wlan_qoshdr *fm;
    char *type, *subtype, *saddr;
    char txaddr[32];

    type = "";
    subtype = "";
    saddr = "";

    memcpy(txaddr, wmaddr_ntoa(acfg_p->tx_addr), 32);
    
    if(wb->wh)
    {
        fm = (struct wlan_qoshdr *)((int)wb + wb->data_off);
        switch(fm->type)
        {
            case WLAN_FC_TYPE_MGT:
                type = "MGT";
                
                if(fm->subtype == WLAN_FC_SUBTYPE_BEACON)
                {
                    if((acfg_p->rx_filter & TEST_FILTER_BEACON) == 0)
                        goto drop;

                    subtype = "beacon";
                    break;
                }
                else if(fm->subtype == WLAN_FC_SUBTYPE_ACTION)
                {
                    struct wlan_action_frame *act_fm = (struct wlan_action_frame *)fm;
                    if(act_fm->category == WLAN_ACTION_VENDOR)
                    {
                        if((acfg_p->rx_filter & TEST_FILTER_MT_ACT) == 0)
                            goto drop;

                        subtype = "MT_ACT";
                        break;
                    }
                    else
                        subtype = "action";
                }
                
                if((acfg_p->rx_filter & TEST_FILTER_MGT) == 0)
                    goto drop;

                if(fm->subtype == WLAN_FC_SUBTYPE_PROBE_RESP)
                    subtype = "prob resp";
                else if(fm->subtype == WLAN_FC_SUBTYPE_PROBE_REQ)
                    subtype = "prob req";

                break;
            case WLAN_FC_TYPE_CTRL:
                if((acfg_p->rx_filter & TEST_FILTER_CTRL) == 0)
                    goto drop;

                type = "CTRL";

                break;
            case WLAN_FC_TYPE_DATA:
                if((acfg_p->rx_filter & TEST_FILTER_DATA) == 0)
                    goto drop;

                type = "DATA";
                if(wb->bcmc)
                {   
                    if((acfg_p->rx_filter & TEST_FILTER_DATA_BC) == 0)
                        goto drop;
                    subtype = "BCAST";
                }
                else
                subtype = "UCAST";

                break;
        }

        saddr = wmaddr_ntoa(fm->addr2);
    }
    else
    {
        if((acfg_p->rx_filter & TEST_FILTER_DATA) == 0)
                    goto drop;

        type = "DATA";
    
        if(wb->bcmc)
        {   
            if((acfg_p->rx_filter & TEST_FILTER_DATA_BC) == 0)
                goto drop;
            subtype = "BCAST";
        }
        else
            subtype = "UCAST";

        saddr = wmaddr_ntoa(acfg_p->tx_addr);
    }

    if(acfg_p->rx_dump & TEST_DUMP_SIMPLE)
        WLA_DBG(WLADEBUG, "---- RX %s(%s) From:%s    RSSI:%d ----\n", type, subtype, saddr, bb_rssi_decode(wb->rssi));
    
    if (!strcmp(txaddr, saddr))
        acfg_p->rssi = wb->rssi;
    
    return 0;
drop:
    return 1;
}

struct wt_msgq* wt_msgq_create(short max_msg_size, char max_qlen, bool echo)
{
    struct wt_msgq *m;
    
    /* first 2 bytes indicate data length of every msg buffer */
    m = (struct wt_msgq *)kmalloc(sizeof(struct wt_msgq) + (max_msg_size+2)*max_qlen, GFP_KERNEL);
    if(m == NULL)
        return NULL;
    
    memset(m, 0, sizeof(struct wt_msgq));
    spin_lock_init(&(m->lock));
    init_MUTEX_LOCKED(&(m->sem));
    m->echo = echo;
    m->max_msgsz = max_msg_size+2;
    m->max_qlen = max_qlen;
    m->qbuf = (char *)m + sizeof(struct wt_msgq);
    m->qhead = m->qbuf;
    m->qtail = m->qbuf;
    m->qfree = max_qlen;
    //m->waiting = 0;
    
    return m;
}   

void wt_msgq_destory(struct wt_msgq *hdl)
{
    struct os_msgq *m = (struct os_msgq *) hdl;
    kfree(m);
}

int wt_msgq_post(struct wt_msgq *hdl, char *msg, int msg_size)
{
    struct wt_msgq *m = (struct wt_msgq *) hdl;

    spin_lock_bh(&(m->lock));
    if((m->qfree == 0) ||
       ((m->max_msgsz-2) < msg_size) ||
       (msg == NULL))
    {
        spin_unlock_bh(&(m->lock));
        return WT_Q_ERROR;
    }

    if(msg_size == 0) /* it means wakeup target right now */
        goto out;

    memcpy(m->qtail+2, msg, msg_size);
    *(short *)m->qtail = msg_size;
    m->qtail = m->qtail + m->max_msgsz;
    /* rewinder */
    if(m->qtail >= m->qbuf + m->max_msgsz * m->max_qlen)
        m->qtail = m->qbuf;
    m->qfree--;
out:
    spin_unlock_bh(&(m->lock));
    if(m->echo)
        up(&(m->sem));
    return WT_Q_SUCCESS;
}

int wt_msgq_wait(struct wt_msgq *hdl, char *msg, int *msg_size)
{
    struct wt_msgq *m = (struct wt_msgq *) hdl;
    int ret = WT_Q_ERROR;

    if(m->echo)
        if(down_timeout(&(m->sem), msecs_to_jiffies(100)) == -ETIME)
            return ret;

    spin_lock_bh(&(m->lock));

    if(m->qfree < m->max_qlen)
    {
        if(*(short *)m->qhead == 0)
        {
            printk("wt_msgq_wait() return zero length data!\n");
        }
        memcpy(msg, m->qhead+2, *(short *)(m->qhead));
        *msg_size = *(short *)m->qhead;
        *(short *)m->qhead = 0;
        m->qhead = m->qhead + m->max_msgsz;
        /* rewinder */
        if(m->qhead >= m->qbuf + m->max_msgsz * m->max_qlen)
            m->qhead = m->qbuf;
        m->qfree++;
        ret = WT_Q_SUCCESS;
    }

    spin_unlock_bh(&(m->lock));
    return ret;
}

bool wt_msgq_is_empty(struct wt_msgq *hdl)
{
    struct wt_msgq *m = (struct wt_msgq *) hdl;

    if(m->qfree == m->max_qlen)
        return true;
    else
        return false;
}

static void wt_rx_wq_function(struct work_struct *ws)
{
    struct wbuf *wb = NULL;
    struct wbuf *rx_wb = NULL; 
    MAC_INFO *info = WLA_MAC_INFO;
    int msg_len;
 
    while(1)
    {
        //if(acfg_p->start == 0)
        //    break;

        if(completion_done(&(acfg_p->start)))
            break;

        if(wt_msgq_wait(acfg_p->msgq_id, (char *)&rx_wb, &msg_len) == WT_Q_ERROR)
        {
            continue;
        }

        if(wla_rx_parser(rx_wb))
        {
            wbuf_free(rx_wb);
            continue;   
        }
    
        if(acfg_p->rx_dump & TEST_DUMP_PAYLOAD)
        {
            WLA_DUMP_BUF((char *)((int)rx_wb + rx_wb->data_off), rx_wb->pkt_len);
        }

        acfg_p->rx_count++;

        if(acfg_p->rx_echo)
        {
            if (!(wb=wbuf_alloc(0)))
            {
                WLA_DBG(WLADEBUG, "%s: no buf for send\n",__func__);
                wbuf_free(rx_wb);
                continue;
            }

            wb->flags |= WBUF_TX_NO_ACK;
            wb->wh = 0;
        
            wb->sta_idx = acfg_p->tx_wcb->captbl_idx;

            wb->data_off = sizeof(struct wbuf);
            wb->bss_desc = 0;
            wb->pkt_len = acfg_p->tx_len;
            wb->tid = 0;
#if 0
            /* action frame */
            fm = (struct wlan_action_frame *) ((char *)wb+wb->data_off);
            memset(fm, 0, sizeof(struct wlan_action_frame));
            fm->hdr.type = WLAN_FC_TYPE_MGT; 
            fm->hdr.subtype = WLAN_FC_SUBTYPE_ACTION; 
            memcpy(fm->hdr.addr1, acfg_p->tx_addr, 6);
            memcpy(fm->hdr.addr2, acfg_p->my_addr, 6);
            fm->category = WLAN_ACTION_VENDOR;
#else
            memcpy((char *)wb+wb->data_off, acfg_p->tx_addr, 6);
            memcpy((char *)wb+wb->data_off+6, acfg_p->my_addr, 6);
#endif
            acfg_p->tx_count++;

            wla_tx_frame_handler(info, wb, wb);

        }

        wbuf_free(rx_wb);

    }
    return;
}


void wla_test_rx_handler(struct wbuf *wb)
{
    if(WT_Q_ERROR == wt_msgq_post(acfg_p->msgq_id, (char *)&wb, sizeof(struct wbuf *)))
    {
        wbuf_free(wb);
    }
    return;
}

void wla_cdb_txvga_parser_set(char *cdb_txvga, int ch, int value)
{
    char txvga_copy[255] = {0};
    int ch_txvga[16] = {0};
    char tmp[16] = {0};
    char *token, *s;
    const char *delim = ",";
    int get_ch = 0, get_v = 0;
    int i = 0;

    //strncpy(txvga_copy, cdb_txvga, 255);
    //token = strtok(txvga_copy, ",");
    //while (token != NULL)
    s = cdb_txvga;
    while((token = strsep(&s, delim)))
    {
        sscanf(token, "%d=%d", &get_ch, &get_v);
        if (get_ch > 0)
            ch_txvga[get_ch-1] = get_v;
        //token = strtok(NULL, ",");
        get_ch = 0;
    }

    ch_txvga[ch-1] = value;
    
    memset(txvga_copy, 0, 255 * sizeof(char));
    
    for( i=0; i<16; i++)
    {
        if (ch_txvga[i] != 0)
        {
            sprintf(tmp, "%d=%d," , i+1, ch_txvga[i]);
            strcat(txvga_copy, tmp);
        }                   
    }

    strncpy(cdb_txvga, txvga_copy, 255);
}

void wla_test_help(void)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	struct seq_file *s = (struct seq_file *)sc->seq_file;
    
    seq_printf(s, "wt start (start the program)\n");
    seq_printf(s, "   stop (force to stop)\n");
    seq_printf(s, "   cfg (show all configuration)\n");
    seq_printf(s, "   chan <channel_num> <second channel: 1:Above, 3:Below>\n");
    seq_printf(s, "   txrate <1~4:CCK, 5~12:OFDM, 13~20:MCS> (fixed TX rate)\n");
    seq_printf(s, "   tx_gf <0/1> (Green Field)\n");
    seq_printf(s, "   tx_sp <0/1> (Short Preamble)\n");
    seq_printf(s, "   tx_sgi <0/1> (Short Guard Interval)\n");
    seq_printf(s, "   filter <MT:0x1, beacon:0x2, mgt:0x4, ctrl:0x8, data:0x10, bcast:0x20> (RX desired packet)\n");
    seq_printf(s, "   dump <0: no show, 0x1:simple, 0x2:payload> (dump RX packet)\n");
    seq_printf(s, "   addr <aa:bb:cc:dd:ee:ff> (peer STA's mac address)\n");
    seq_printf(s, "   stat\n");
    seq_printf(s, "   rx <drop|echo> <expect num>\n");
    seq_printf(s, "   tx <count>/repeat <payload_length> <burst> <echo>\n");
    seq_printf(s, "   bbcnt\n");
    seq_printf(s, "   rssi\n");
    seq_printf(s, "   txvga <value>/save\n");
    seq_printf(s, "   fofs <value>/save\n");
}


int wla_test(void *data)
{
    struct wbuf *wb, *rx_wb;
    int iteration, rate_flags, msg_len;
    MAC_INFO *info = WLA_MAC_INFO;
    bool echo = false;
    //struct wlan_action_frame *fm;

    //acfg_p->stop = 0;
    //acfg_p->start = 1;
    INIT_COMPLETION(acfg_p->start);
    INIT_COMPLETION(acfg_p->stop);

    wla_cfg(SET_RX_FILTER, RX_FILTER_MGMT_FRAME|RX_FILTER_CTRL_FRAME|RX_FILTER_DATA_FRAME);

    if(acfg_p->rx_filter & TEST_FILTER_BEACON)
        wla_cfg(DEL_RX_FILTER, RX_FILTER_BEACON|RX_FILTER_PROBE_RESP);
    else
        wla_cfg(SET_RX_FILTER, RX_FILTER_BEACON|RX_FILTER_PROBE_RESP);

    //ifcfg_up("wlan0");
    //wla_register_mgt_handler(0); /* force to unregister mgt handler */
    info->wla_mgt_handler = 0;
    info->wla_eapol_handler = 0;
    
    wla_tx_power(acfg_p->txpower);
    wla_cfg(SET_CHANNEL, (int)acfg_p->channel, (int)acfg_p->secondary_channel);

    rate_flags = RATE_FLAGS_HT_40M;
    if(acfg_p->tx_short_preamble)
        rate_flags |= RATE_FLAGS_SHORT_PREAMBLE;
    if(acfg_p->tx_green_field)
        rate_flags |= RATE_FLAGS_HT_GF;
    if(acfg_p->tx_sgi)
        rate_flags |= RATE_FLAGS_HT_SGI20|RATE_FLAGS_HT_SGI40;  
    
    acfg_p->tx_wcb = wla_get_wcb(); 
    acfg_p->tx_wcb->rate_flags = rate_flags;
    //wla_tx_power(acfg_p->txpower);

    //wla_cfg(SET_BEACON_PARAMS, 100, 3);

    /* fragment threshold */
    wla_cfg(SET_FRAG_THRESHOLD, 2346 - 24);

    /* RTS threshold */
    wla_cfg(SET_RTS_THRESHOLD, 2346 - 24);

    wla_set_protection(NO_PROTECTION);
    wla_set_ap_isolated(0);
    wla_set_bss_isolated(0);

    cdb_get_mac(0, acfg_p->my_addr);
#if 1
    if(acfg_p->rx_echo || acfg_p->rx_drop)
    {
        acfg_p->bss_desc = register_macaddr(STA_MACADDR, acfg_p->my_addr);
        //wla_add_wif(acfg_p->bss_desc, WIF_STA_ROLE, 0);
        wla_add_wif(acfg_p->bss_desc, WIF_AP_ROLE, 0);
    }
    else
    {
        acfg_p->bss_desc = register_macaddr(AP_MACADDR, acfg_p->my_addr);
        wla_add_wif(acfg_p->bss_desc, WIF_AP_ROLE, 0);
    }
#else
    /* WDS mode */
    acfg_p->bss_desc = register_macaddr(AP_MACADDR, acfg_p->my_addr);
    wla_add_wif(acfg_p->bss_desc, WIF_AP_ROLE, 0);
#endif
    wla_cfg(SET_TX_RATE, FIXED_TX_RATE, acfg_p->bss_desc, rate_flags, acfg_p->tx_rate);
    acfg_p->tx_wcb->qos = acfg_p->tx_qos;
#if 1
    if(acfg_p->rx_echo || acfg_p->rx_drop)
        wla_get_sta(acfg_p->tx_addr, acfg_p->bss_desc, acfg_p->tx_wcb, LINKED_AP);
    else
        wla_get_sta(acfg_p->tx_addr, acfg_p->bss_desc, acfg_p->tx_wcb, 0);
#else
    /* WDS mode */
    wla_get_sta(acfg_p->tx_addr, acfg_p->bss_desc, acfg_p->tx_wcb, WDS_LINKED_AP);
#endif


    if(acfg_p->tx_echo || acfg_p->rx_echo || acfg_p->rx_drop)
    {
        if(acfg_p->tx_echo || acfg_p->rx_echo)
            echo = true;
        if((acfg_p->msgq_id = wt_msgq_create(sizeof (struct wbuf *), 64, echo)) == 0) 
            goto exit;
        
        info->wla_moniter_handler = wla_test_rx_handler; 
    }

    acfg_p->rx_count = 0;
    acfg_p->tx_count = 0;
    acfg_p->fail_count = 0;
    if(acfg_p->tx_repeat)
        acfg_p->expect_count = 0;   
    wb = 0;
    rx_wb = 0;
    //acfg_p->start = 1;
    iteration = 0;
    while((acfg_p->tx_repeat == -1)||(iteration < acfg_p->tx_repeat))
    {
        //if(acfg_p->start == 0)
        //    break;
        if(completion_done(&(acfg_p->start)))
            break;

        if((iteration % acfg_p->tx_burst_num) == 0)
        {
            os_thread_sleep(acfg_p->tx_sleep_time);
        }

        iteration++;

        if (!(wb=wbuf_alloc(0)))
        {
            WLA_DBG(WLADEBUG, "%s: no buf for send\n",__func__);
            continue;
        }

        wb->flags |= WBUF_TX_NO_ACK;
        wb->wh = 0;

        wb->sta_idx = acfg_p->tx_wcb->captbl_idx;

        wb->data_off = sizeof(struct wbuf);
        wb->bss_desc = 0;
        wb->pkt_len = acfg_p->tx_len;
        wb->tid = 0;
#if 0
        /* action frame */
        fm = (struct wlan_action_frame *) ((char *)wb+wb->data_off);
        memset(fm, 0, sizeof(struct wlan_action_frame));
        fm->hdr.type = WLAN_FC_TYPE_MGT; 
        fm->hdr.subtype = WLAN_FC_SUBTYPE_ACTION; 
        memcpy(fm->hdr.addr1, acfg_p->tx_addr, 6);
        memcpy(fm->hdr.addr2, acfg_p->my_addr, 6);
        fm->category = WLAN_ACTION_VENDOR;
#else
        memcpy((char *)wb+wb->data_off, acfg_p->tx_addr, 6);
        memcpy((char *)wb+wb->data_off+6, acfg_p->my_addr, 6);
#endif
        acfg_p->tx_count++;
        acfg_p->expect_count++;

        wla_tx_frame_handler(info, wb, wb);

        if(acfg_p->tx_echo)
        {
            /* waiting for echo packet */
            if(wt_msgq_wait(acfg_p->msgq_id, (char *)&rx_wb, &msg_len) == WT_Q_SUCCESS)
            {
                if(wla_rx_parser(rx_wb) == 0)
                    acfg_p->rx_count++;
                wbuf_free(rx_wb);
            }
        }
    }

    //while(acfg_p->rx_echo || acfg_p->rx_drop)
    if(acfg_p->rx_echo || acfg_p->rx_drop)
    {
        wt_rx_work = kzalloc(sizeof(typeof(*wt_rx_work)), GFP_KERNEL);
        INIT_WORK(wt_rx_work, wt_rx_wq_function);
        schedule_work(wt_rx_work);
        //if(acfg_p->start == 0)
        //    break;
        wait_for_completion(&(acfg_p->start));
    }

    if(acfg_p->expect_count)
        acfg_p->fail_count = acfg_p->expect_count - acfg_p->rx_count;
exit:
    info->wla_moniter_handler = 0;
    //acfg_p->start = 0;
    complete(&(acfg_p->start));
    
    wla_release_sta(acfg_p->tx_wcb);
    os_thread_sleep(100); /* wait for sta release */
    //ifcfg_down("wlan0");
    clean_all_macaddr();
    //if(acfg_p->wt_wq)
    //{
    //    destroy_workqueue(acfg_p->wt_wq);
    //    acfg_p->wt_wq = NULL;
    //}
    if(wt_rx_work)
    {
        while(!wt_msgq_is_empty(acfg_p->msgq_id))
            os_thread_sleep(100);
        flush_scheduled_work();
        kfree(wt_rx_work);
        wt_rx_work = NULL;
    }
    if(acfg_p->msgq_id)
    {
        wt_msgq_destory(acfg_p->msgq_id);
        acfg_p->msgq_id = 0;
    }
    //kfifo_free(&wt_rx_fifo);
    printk("wla_test is Done\n");
    //acfg_p->stop = 1;
    complete(&(acfg_p->stop));
    //exit(0); 
    return 0;
}
#if 0
int wait_wt_ready(void)
{
 	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	struct seq_file *s = (struct seq_file *)sc->seq_file;
    unsigned int cnt = 0;
    
    while (cnt < 500)   // 5s
    {
        if(acfg_p->stop)
            break;
        os_thread_sleep(100);   // 10 ms
        cnt++;
    }

    if(!acfg_p->stop)
    {
        seq_printf(s, "The program not stop yet!\n");
        return 0;
    }
    return 1;
}
#endif
static int wla_test_cmd(int argc, char *argv[])
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	struct seq_file *s = (struct seq_file *)sc->seq_file;
    int ret;
    
	//seq_printf(s, "%s %d argc:%d\n", __func__, __LINE__, argc);
    
    if(argc < 1)
	{
	    seq_printf(s, "%s %d argc:%d\n", __func__, __LINE__, argc);
		return 0;
	}

    if(!strcmp("stop", argv[0]))
    {
        //acfg_p->start = 0;
        complete(&(acfg_p->start));
    }
    else if(!strcmp("start", argv[0]))
    {
        //if(acfg_p->start)
        if(!completion_done(&(acfg_p->start)))
        {
            seq_printf(s, "The program has exceuted!\n");
            return CMD_OK;
        }

        //if(!wait_wt_ready())
        if(!wait_for_completion_timeout(&(acfg_p->stop), msecs_to_jiffies(5000))) 
        {
            seq_printf(s, "The program not stop yet!\n");
            return CMD_OK;
        }

        //os_thread_create((OS_FUNCPTR)wla_test, NULL, "wla_test", 30, 8192);
        wt_tsk = kthread_create(wla_test, NULL, "wla_test");
        if (IS_ERR(wt_tsk)) {
            ret = PTR_ERR(wt_tsk);
            wt_tsk = NULL;
            goto out;
        }
        wake_up_process(wt_tsk);
out:
        // clear counter
        bb_register_write(0x80, 0xc0);
        // enable counter
        bb_register_write(0x80, 0x80);
    }
    else if(!strcmp("cfg", argv[0]))
    {
        //seq_printf(s, "start:%d\n", acfg_p->start);
        seq_printf(s, "channel:%d, secondary_channel=%d\n", acfg_p->channel, acfg_p->secondary_channel);
        //seq_printf(s, "Peer STA's address: %s\n", ether_ntoa((struct ether_addr *)acfg_p->tx_addr));
        seq_printf(s, "Peer STA's address: %s\n", wmaddr_ntoa(acfg_p->tx_addr));
        seq_printf(s, "-- RX --\n   drop:%d, echo:%d, dump:%d, expect:%d, filter:0x%08x\n", acfg_p->rx_drop, acfg_p->rx_echo,  acfg_p->rx_dump, acfg_p->expect_count, acfg_p->rx_filter);
        seq_printf(s, "-- TX --\n   echo:%d, repeat: %d, burst: %d, sleep time: %dms, length: %dBytes, rate=%d\n", acfg_p->tx_echo, acfg_p->tx_repeat, acfg_p->tx_burst_num, acfg_p->tx_sleep_time*10, acfg_p->tx_len, acfg_p->tx_rate);
        seq_printf(s, "   tx_sgi=%d, tx_green_field=%d, tx_short_preamble=%d\n", acfg_p->tx_sgi, acfg_p->tx_green_field, acfg_p->tx_short_preamble);
    }
    else if(!strcmp("chan", argv[0]))
    {
        if(argc < 2)
            return CMD_SHOW_USAGE;

        acfg_p->channel = atoi(argv[1]);
        acfg_p->secondary_channel = 0;

        if(argc >= 3)
        {
            acfg_p->secondary_channel = atoi(argv[2]);
        }
    }
    else if(!strcmp("txrate", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;
        
        acfg_p->tx_rate = atoi(argv[1]);
    }
    else if(!strcmp("tx_sgi", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        acfg_p->tx_sgi = atoi(argv[1]);
    }
    else if(!strcmp("tx_gf", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        acfg_p->tx_green_field = atoi(argv[1]);
    }
    else if(!strcmp("tx_sp", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        acfg_p->tx_short_preamble = atoi(argv[1]);
    }
    else if(!strcmp("addr", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        memcpy(acfg_p->tx_addr, wmaddr_aton(argv[1]), 6);
    }
    else if(!strcmp("tx", argv[0]))
    {
        if(argc < 2)
            return CMD_SHOW_USAGE;

        if(!strcmp("repeat", argv[1]))
            acfg_p->tx_repeat = 0xffffffff;
        else
            acfg_p->tx_repeat = atoi(argv[1]);

        if(argc >= 3)
        {
            acfg_p->tx_len = atoi(argv[2]);
        }

        if(argc >= 4)
        {
            acfg_p->tx_burst_num = atoi(argv[3]);
        }

        if(argc >= 5)
        {
            acfg_p->tx_echo = 1;
        }

        acfg_p->rx_drop = 0;
        acfg_p->rx_echo = 0;
    }
    else if(!strcmp("dump", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        acfg_p->rx_dump = atoi(argv[1]);
    }
    else if(!strcmp("filter", argv[0]))
    {
        if(argc != 2)
            return CMD_SHOW_USAGE;

        acfg_p->rx_filter = atoi(argv[1]);
    }
    else if(!strcmp("rx", argv[0]))
    {
        if(!strcmp("echo", argv[1]))
        {
            acfg_p->rx_echo = 1;
        }
        else
            acfg_p->rx_drop = 1;

        acfg_p->tx_repeat = 0; /* disable tx */
        if(argc >= 3)
        {
            acfg_p->expect_count = atoi(argv[2]);
        }
    }
    else if(!strcmp("stat", argv[0]))
    {
        seq_printf(s, "Total: %d\nPass: %d\nFail: %d\nTX: %d\nRX: %d\n", acfg_p->expect_count, acfg_p->rx_count, acfg_p->fail_count, acfg_p->tx_count, acfg_p->rx_count);
    }
    else if(!strcmp("bbcnt", argv[0]))
    {
        unsigned long value;

        // disable counter
        bb_register_write(0x80, 0x0);
        // read ok counter high byte
        value = (bb_register_read(0x89) << 8);
        // read ok counter low byte
        value |= bb_register_read(0x8a);

        seq_printf(s, "%ld\n", value);
    }
    else if(!strcmp("rssi", argv[0]))
    {
        seq_printf(s, "%d\n", bb_rssi_decode_mp(acfg_p->rssi));
        // reset rssi after user get value
        acfg_p->rssi = 0;
    }
    else if(!strcmp("txvga", argv[0]))
    {
        u32 reg;
        char tmp_cdb[255] = {0};
        
        reg = ip301_spi_read(0x7);
        //printf("0x%x\n", reg);    

        if(argc >= 2)
        {
            if(!strcmp("save", argv[1]))
            {
                //reg = wla_get_rf_txvga_reg(acfg_p->channel-1);

                //seq_printf(s, "ch%d %d\n", acfg_p->channel, reg);
                //cdb_set($hw_ch_txvga, tmp_cdb);
                hw_cdb_set("txvga", g_tmp_txvga_cdb);
                
                hw_cdb_get("txvga", tmp_cdb);
                if(strcmp(tmp_cdb, g_tmp_txvga_cdb))
                    seq_printf(s, "Update txvga fail!\n");
            }
            else
            {
                u32 vga = 0;
                if(!strcmp("default", argv[1]))
                {
                    wla_tx_power(acfg_p->txpower);
                    wla_rf_txvga_reg(acfg_p->channel-1, vga);
                    //cdb_set_array($hw_ch_txvga, acfg_p->channel, &vga);
                }
                else
                {
                    vga = atoi(argv[1]);
                    reg = (reg & 0x301ff) | ((vga << 9) & 0xfe00);
                    ip301_spi_write(0x7, reg);
                    wla_rf_txvga_reg(acfg_p->channel-1, vga);

                    if(strlen(g_tmp_txvga_cdb) == 0)
                        hw_cdb_get("txvga", g_tmp_txvga_cdb);
                    wla_cdb_txvga_parser_set(g_tmp_txvga_cdb, acfg_p->channel, vga);
                }
            }                                     
        }
        else                                      
        {
            reg = ip301_spi_read(0x7);            
            reg = (reg & 0xfe00) >> 9;            
            seq_printf(s, "%d\n", reg);                  
        }
    }  
    else if(!strcmp("fofs", argv[0]))             
    {  
        u32 reg;
        char tmp[16] = {0};                                  

        //printf("0x%x\n", reg);                  
                                                  
        if(argc >= 2)                            
        {
            if(!strcmp("save", argv[1]))          
            {                                     
                char tmp_cdb[16] = {0};                                  
                
                reg = wla_get_rf_freq_ofs_reg();                   
                reg = reg & 0x1f;
                sprintf(tmp, "%x", reg);            
                hw_cdb_set("freq_ofs", tmp);

                hw_cdb_get("freq_ofs", tmp_cdb);
                if(hatoi(tmp) != hatoi(tmp_cdb))
                    seq_printf(s, "Update txvga fail!\n");
            }
            else
            {
                u32 freq_ofs = 0;                 
                
                reg = ip301_spi_read(0x11);       
                freq_ofs = atoi(argv[1]);
                reg = (reg & 0xffffffe0) | (freq_ofs & 0x1f);      
                ip301_spi_write(0x11, reg);
                wla_rf_freq_ofs_reg(freq_ofs);
            }
        }
        else
        {
            reg = ip301_spi_read(0x11);
            reg = reg & 0x1f;
            //printf("%d\n", reg);    
            seq_printf(s, "%d\n", reg);
        }
    }
    else
        return CMD_SHOW_USAGE;


    return CMD_OK;
}

struct idb_command idb_wt_cmd =
{
    .cmdline = "wt",
    .help_msg = "wt                         wla test command", 
    .func = wla_test_cmd,
};

static int wt_fops_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(__wt_cmd, 0, 300);

	if (count > 0 && count < 299) {
		if (copy_from_user(__wt_cmd, buffer, count))
			return -EFAULT;
		__wt_cmd[count-1]='\0';
	}
	return count;

}

static int wt_fops_show(struct seq_file *s, void *priv)
{
 	MAC_INFO *info = WLA_MAC_INFO;
	struct wla_softc *sc = info->sc;
	int	argc ;
	char* argv[MAX_CMD_ARGV];
    int ret;

    //mutex_lock(&data->mutex);
	sc->seq_file = s;
	argc = get_args((const char *)__wt_cmd, argv);
	ret = wla_test_cmd(argc, argv);
    if(ret == CMD_SHOW_USAGE)
        wla_test_help();
	sc->seq_file = NULL;
    //mutex_unlock(&data->mutex);
	//argc = get_args((const char *)data, argv);
    // do something
    
    return 0;
}

static int wt_fops_open(struct inode *inode, struct file *file)
{
    int res;
	
    res = single_open(file, wt_fops_show, NULL);
    return res;
}

static const struct file_operations wt_fops = {
    .open       = wt_fops_open,
	.read       = seq_read,
	.write		= wt_fops_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#endif

int wla_test_init(void)
{
#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
	struct proc_dir_entry *res;
    
	register_idb_command(&idb_wt_cmd);
	
    res = create_proc_entry("wt", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->proc_fops = &wt_fops;

    init_completion(&(acfg_p->start));
    complete(&(acfg_p->start));
    init_completion(&(acfg_p->stop));
    complete(&(acfg_p->stop));
#endif
    return 0;
}

void wla_test_exit(void)
{
#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
	unregister_idb_command(&idb_wt_cmd);
	remove_proc_entry("wt", NULL);
#endif
}
