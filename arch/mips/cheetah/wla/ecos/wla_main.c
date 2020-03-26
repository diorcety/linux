/*=============================================================================+
|                                                                              |
| Copyright 2013                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_main.c
*   \brief  wlan accelerator driver.
*   \author Montage
*/


/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/drv_api.h>
#include <netdev.h>
#include <eth_drv.h>

//#include <cdb_api.h>
#include <ecos_wla.h>
#include <wla_api.h>
#include <wla_debug.h>
#include <ether.h>
#include <wla_bb.h>
#include <wm_msg.h>
#include <common.h>
#include <rf.h>

#include  <sys/ioctl.h>
static bool wla_drv_init(struct cyg_netdevtab_entry *pnde);
static struct wla_dev *wla_dev_init(short unit, void *sc);
static unsigned int wla_isr(cyg_vector_t vec, cyg_addrword_t data) __attribute__ ((section(".iram")));
static void wla_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data) __attribute__ ((section(".iram")));

static int wla_control(struct eth_drv_sc *sc, unsigned long cmd, void *args, int arglen);
static void wla_deliver(struct eth_drv_sc *sc) __attribute__ ((section(".iram")));
static void wla_data_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
		int sg_len, int total_len, unsigned long key);
static void wla_start(struct eth_drv_sc *sc, unsigned char *eaddr, int flags);
static void wla_stop(struct eth_drv_sc *sc);
static int wla_cansend(struct eth_drv_sc *sc);
static unsigned short wla_attach(struct eth_drv_sc *sc);

void wla_software_intr(int *parm);
extern void cta_eth_setmac(unsigned char unit, unsigned char *mac);
extern void wm_receive_frame(struct wbuf *wb, void *wcb);

/*=============================================================================+
| Variables
+=============================================================================*/
struct wla_dev my_wla_dev;
struct wla_dev *my_wla = &my_wla_dev;

#define	PREPEND_SIZE	(sizeof(struct wbuf)+2)

ETH_DRV_SC(wla_sc0, &my_wla_dev, "wlan0",
           wla_start, wla_stop, wla_control, wla_cansend, wla_data_send,
           0, wla_deliver, wla_deliver, wla_attach);

NETDEVTAB_ENTRY(wla_netdev0, "wla0", wla_drv_init, &wla_sc0); 

/*=============================================================================+
| Functions
+=============================================================================*/
static unsigned short wla_attach(struct eth_drv_sc *sc)
{
	return 0;
}

static bool wla_drv_init(struct cyg_netdevtab_entry *pnde)
{
	struct eth_drv_sc *sc;
	struct wla_dev *dev;

	sc = (struct eth_drv_sc *)(pnde->device_instance);
	
	WLA_DBG(WLADEBUG, "%s: pnde=%x, sc=%x\n", __func__,
			(unsigned int) pnde, (unsigned int) sc);
	
	if ((dev = wla_dev_init(0, (void *)sc)) == 0)
	{
		WLA_DBG(WLADEBUG, "wla_dev_init: failed hw!\n");
		return false;
	}

	(sc->funs->eth_drv->init)(sc, NULL);
	return true;
}

static unsigned int wla_isr(cyg_vector_t vec, cyg_addrword_t data)
{
	/* disable all interrupts */
	MACREG_WRITE32(MAC_INT_MASK_REG, MAC_INT_FULL_MASK); 
 
	cyg_drv_interrupt_acknowledge(vec);
	return CYG_ISR_CALL_DSR;
}

static void wla_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
	unsigned int status, ts_status, ts_ap_status;
	struct eth_drv_sc *sc = (struct eth_drv_sc *)data;
	MAC_INFO *info = WLA_MAC_INFO;
	
	status = MACREG_READ32(MAC_INT_STATUS);
	//MACREG_WRITE32(MAC_INT_CLR, status & ~MAC_INT_TXRX_MASK);

	if((status & MAC_INT_PRETBTT) && ~(info->int_mask & MAC_INT_PRETBTT))
	{
		info->pre_tbtt_int++;

    	ts_status = MACREG_READ32(TS_INT_STATUS);
		MACREG_WRITE32(TS_INT_STATUS, ts_status);
		ts_ap_status = MACREG_READ32(TS_AP_INT_STATUS);
		MACREG_WRITE32(TS_AP_INT_STATUS, ts_ap_status);

		//printd("time=%d, ts_status=%x, ts_ap=%x\n", os_current_time(), ts_status, ts_ap_status);
		
		/* TS of PRE_TBTT_SW */
		if(ts_status)
		{
			//info->ts0_int++;
			timeout((OS_FUNCPTR)wla_software_intr, (void *)ts_status, 0);
		}
			
#ifdef TBTT_EXCEPTION_HANDLE
		/* TBTT ERROR EXCEPTION */
		if(ts_ap_status & (TBTT_ER_STATUS_0 | TBTT_ER_STATUS_1))
		{
			wla_update_next_tbtt(info, ts_ap_status);
		}
#endif // TBTT_EXCEPTION_HANDLE
	}
	if(status & MAC_INT_HW_BUF_FULL)
	{
		info->hw_buf_full++;
	}
	if(status & MAC_INT_RX_DESCR_FULL)
	{
		info->rx_desc_full++;
	}
	if(status & MAC_INT_RX_FIFO_FULL)
	{
		info->rx_fifo_full++;
	}
	if(status & MAC_INT_SW_RTURNQ_FULL)
	{
		info->sw_retq_full++;
	}
#if defined(CONFIG_ARTHUR) && defined(ARTHUR_WIFI2ETH)
	if(status & MAC_INT_ETH_DESCR_FULL)
	{
		int val;
		val = MACREG_READ32(ETH_RX_INTR_STATUS);
		if(val & ETH_RX_INTR_STATUS_WAIT_SW_DESCR)
			info->eth_sw_desc_full++;
		if(val & ETH_RX_INTR_STATUS_WAIT_HW_DESCR)
			info->eth_hw_desc_full++;
	}
#endif
	if(status & MAC_INT_TXRX_MASK)
	{
		sc->state |= ETH_DRV_NEEDS_DELIVERY;
    	wakeup_netdrv(NET_TX|NET_RX);
	}

	MACREG_WRITE32(MAC_INT_CLR, status & ~MAC_INT_TXRX_MASK);

	if(info->mac_is_ready)
	{
		/* enable non tx/rx interrupts */
		if(MAC_INT_TXRX_MASK & status)
			MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask | MAC_INT_TXRX_MASK);
		else
			MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask);
	}
}

static struct wla_dev *wla_dev_init(short unit, void *sc)
{
	MAC_INFO *info;

	WLA_DBG(WLADEBUG, "%s()\n", __func__); 	
	memset(my_wla , 0, sizeof(struct wla_dev));

	info = WLA_MAC_INFO;
	info->sc = (void *)my_wla;

	/* initialize RF */
	if((info->rf_init_err = rf_init()))
		return NULL;

	my_wla->mutex = os_mutex_create();
	my_wla->wla_lock_count = 0;

	wla_mac_reset(info);

	if(wla_mac_init(info) < 0)
		return NULL;

	bb_init();

	/* register IRQ */
	my_wla->irq = IRQ_WIFI;	
	cyg_drv_interrupt_create(my_wla->irq, 0, (CYG_ADDRWORD)sc,
		wla_isr, wla_dsr, &my_wla->wla_int_handle, &my_wla->wla_int_obj);

	cyg_drv_interrupt_attach(my_wla->wla_int_handle);
	cyg_drv_interrupt_configure(my_wla->irq, 1, 0 );

	my_wla->flags = DEV_INIT; 
	return my_wla;
}

void wla_eapol_frame(struct wbuf *wb, void *ef)
{
	MAC_INFO *info = WLA_MAC_INFO; 
	struct wsta_cb *wcb = info->wcb_tbls[wb->sa_idx];

	wm_receive_frame(wb, wcb);	
}

static void wla_start(struct eth_drv_sc *sc, unsigned char *eaddr, int flags)
{
	wif_vif *vif = sc->driver_private;

	if(!(vif->flags & VIF_ACTIVE))
	{
		vif->flags |= VIF_ACTIVE;
		WLA_DBG(WLADEBUG, "Start WLAN VIF(%d)!\n", vif->unit);

		if(!(my_wla->flags & DEV_ACTIVE))
		{
			MAC_INFO *info = WLA_MAC_INFO;
			WLA_DBG(WLADEBUG, "Start WLAN DEV..\n");

			wla_lock();

			info->wla_mgt_handler = wm_receive_frame;
			info->wla_eapol_handler = wla_eapol_frame;

			//wla_mac_reset(info);
			info->bss_num = 0;

			if(wla_mac_start(info) < 0)
			{
				wla_unlock();
				return;
			}
			
			if(my_wla->mac_info.curr_channel)
			{
				wla_set_channel(info);
				wla_set_bandwidth_type(info);
			}

			wla_unlock();

			info->mac_is_ready = true;
			cyg_drv_interrupt_unmask(my_wla->irq);
			my_wla->flags |= DEV_ACTIVE;
		}
	}

	return;
}

static void wla_stop(struct eth_drv_sc *sc)
{
	wif_vif *vif = sc->driver_private;

	if(vif->flags & VIF_ACTIVE)
	{
		unsigned int i;

		vif->flags &= ~VIF_ACTIVE;
		WLA_DBG(WLADEBUG, "Stop WLAN VIF(%d)!\n", vif->unit);

		/* Don't shutdown WLA, if other VIFs are active.*/
		for(i=0; i<MBSS_MAX_NUM; i++)
		{
			if(my_wla->vifs[i].flags & VIF_ACTIVE)
				return;
		}
	
		if(my_wla->flags & DEV_ACTIVE)
		{
			MAC_INFO *info = WLA_MAC_INFO;
	
			my_wla->flags &= ~DEV_ACTIVE;
			WLA_DBG(WLADEBUG, "Stop WLAN DEV..\n");

			info->wla_mgt_handler = 0;
			info->wla_eapol_handler = 0;

			info->mac_is_ready = false;

			cyg_drv_interrupt_mask(my_wla->irq);

			untimeout((OS_FUNCPTR)wla_beacon, info);

			wla_lock();

			wla_mac_stop(info);
			wla_mac_clean(info);
			//wla_mac_reset(info);
			//wla_mac_release(info);

			wla_unlock();
		}
	}
}

#if defined(CONFIG_WMAC_RECOVER)
void wla_recover(void)
{
	unsigned int i;
	wif_vif *vif;

	for(i=0; i<MBSS_MAX_NUM; i++)
	{
		vif = &my_wla->vifs[i];

		if(!vif->sc)
			continue;
		if(!(vif->flags & VIF_ACTIVE))
			continue;
		vif->flags &= ~VIF_ACTIVE;
		WLA_DBG(WLADEBUG, "Stop WLAN VIF(%d)!\n", vif->unit);
	}
	if(my_wla->flags & DEV_ACTIVE)
	{
		MAC_INFO *info = &my_wla->mac_info;
	
		my_wla->flags &= ~DEV_ACTIVE;
		WLA_DBG(WLADEBUG, "Stop WLAN DEV..\n");

		/* Prohibit ethernet to wifi */
		wla_mac_ethernet_forward_setup(0);
#if (FASTPATH_DESCR_COUNT > 0)
		MACREG_UPDATE32(ERR_EN, 0, FASTPATH_WIFI_TO_WIFI);
#endif
		MACREG_WRITE32(DA_MISS_TOCPU, 1);

		info->mac_is_ready = false;
		info->mac_recover = true;
		cyg_drv_interrupt_mask(my_wla->irq);
#if defined(ARTHUR_WIFI2ETH)
		w2e_interrupt_mask();
#endif
		untimeout((OS_FUNCPTR)wla_beacon, info);

		/* Save LMAC setting */
		for(i=0; i<12; i++)
		{
			info->restore_lmac[i] = MACREG_READ32(LMAC_CNTL+(i<<2));
		}
		wla_lock();

		wla_mac_stop(info);
		/* Resotre beacon buffer */
		save_beacon_list(info, (struct beacon_desc *)info->beacon_descriptors_list);
		wla_beacon_flush(info);
		wla_mac_reset(info);

		wla_unlock();

		WLA_DBG(WLADEBUG,"%s cnt(%d)\n", __func__, ++info->mac_recover_cnt);

		wla_lock();

		/* MAC recover */
		wla_mac_recover(info);

		if(wla_mac_start(info) < 0)
		{
			wla_unlock();
			return;
		}
		
		if(my_wla->mac_info.curr_channel)
		{
			wla_set_channel(info);
			wla_set_bandwidth_type(info);
		}
		/* Without cache sync for address table, target can not receive frames once __no_ampdu is set */
		{
			char addr[6];
			int bcap = 0;

			/* Cache sync for DS address table */
			for(i=0; i < MAX_DS_TABLE_ENTRIES; i++)
				mac_addr_lookup_engine_find(addr, i, &bcap, IN_DS_TBL|BY_ADDR_IDX);
			/* Cache sync for staion address table */
			for(i=0; i < MAX_ADDR_TABLE_ENTRIES; i++)
				mac_addr_lookup_engine_find(addr, i, &bcap, BY_ADDR_IDX);
			mac_rx_linkinfo_cache_sync();
		}

		wla_unlock();

		info->mac_recover = false;
		info->mac_is_ready = true;
		cyg_drv_interrupt_unmask(my_wla->irq);
#if defined(ARTHUR_WIFI2ETH)
		w2e_interrupt_unmask();
#endif
		my_wla->flags |= DEV_ACTIVE;

		MACREG_WRITE32(DA_MISS_TOCPU, 0);
#if (FASTPATH_DESCR_COUNT > 0)
		MACREG_UPDATE32(ERR_EN, FASTPATH_WIFI_TO_WIFI, FASTPATH_WIFI_TO_WIFI);
#endif
		/* Permit ethernet to wifi */
		wla_mac_ethernet_forward_setup(1);

		/* In linux, RX delba will block by mac is not ready. */ 
		/* Prohibit AMPDU */
		wla_notify_no_ampdu();
		udelay(1000000);
		wla_cfg(SET_BEACON_PARAMS, 100, 3);

		/* Restore LMAC setting */
		for(i=0; i<12; i++)
		{
			MACREG_WRITE32(LMAC_CNTL+(i<<2), info->restore_lmac[i]);
		}
		for(i=0; i<info->bss_num; i++)
		{
			struct mbss_t *wif;
		
			wif = &info->mbss[i];
		
			if(wif->role != WIF_AP_ROLE)
				continue;
			/* separate the interrupt & beacon setting */
			wla_set_beacon_interrupt(i, wif->role, 1);
		}
		update_beacon_list(info);
	}
	for(i=0; i<MBSS_MAX_NUM; i++)
	{
		vif = &my_wla->vifs[i];

		if(!vif->sc)
			continue;
		if(vif->flags & VIF_ACTIVE)
			continue;
		vif->flags |= VIF_ACTIVE;
		WLA_DBG(WLADEBUG, "Start WLAN VIF(%d)!\n", vif->unit);
	}
}
#endif

static int wla_cansend(struct eth_drv_sc *sc)
{
	int free = 0;

	if (my_wla->flags & DEV_ACTIVE)
		free = 1;
		
	return free;
}


static void wla_data_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
		int sg_len, int total_len, unsigned long key)
{
	struct wbuf *wb;
	char *startp;
	struct mbuf *m = (struct mbuf *)key;
	int wb_cpy=0;
	char from_bss;
	MAC_INFO *info = WLA_MAC_INFO;

	if(!info->mac_is_ready)
	{
		WLA_DBG(WLAWARN, "%s(), mac is not ready!\n", __func__);
		eth_drv_free_key((void *)key);
		return;
	}
	if(!((m->m_flags & M_EXT) && (sg_len == 1)))
		wb_cpy = 1;
	else if(PREPEND_SIZE > (m->m_data - m->m_ext.ext_buf))
		wb_cpy = 1;

	if(wb_cpy)
	{
		if (!(wb=wbuf_alloc(0)))
		{
			WLA_DBG(ERROR, "%s: no buf for send\n",__func__);
			eth_drv_free_key((void *)key);
			return;
		}
		/* copy to new one cluster and free original data */
		/* +2 => ether header len 14 byte, 14+2=16 let address be multiple of 4 (ip alignment) */
		startp = (char *)wb+sizeof(struct wbuf)+2;	
		eth_drv_sg_copy(startp, NBUF_LEN, sg_list, sg_len);
		eth_drv_free_key((void *)key);
		
		/* +2 => ether header len 14 byte, 14+2=16 let address be multiple of 4 (ip alignment) */
		wb->data_off = sizeof(struct wbuf)+2;
	}
	else
	{
		/* 
			Transfer mbuf to wbuf directly to avoid memcpy.
			We assume Cluster has enough offset to store wbuf header 
		*/
		wb = (struct wbuf *)m->m_ext.ext_buf;
		memset(wb, 0, sizeof(struct wbuf));
		wb->flags = WBUF_FIXED_SIZE; 
		startp = (char *)m->m_data;
		wb->data_off = m->m_data - m->m_ext.ext_buf;
	  	MBUF_FREE(m, mbtypes[m->m_type]);
	}
	wb->pkt_len = total_len;
	from_bss = ETH_TO_BSS(((wif_vif *)(sc->driver_private))->unit);

	ethernet_parse_and_forward(&my_wla->mac_info, wb, from_bss);
}

static void wla_deliver(struct eth_drv_sc *sc)
{
	volatile unsigned int status;
	MAC_INFO *info = WLA_MAC_INFO;

	status = MACREG_READ32(MAC_INT_STATUS);
	MACREG_WRITE32(MAC_INT_CLR, status & MAC_INT_TXRX_MASK);     		/* write 1 to clear */

	//WLA_DBG(DEBUG, "%s. status=%x\n", __func__, status);

	if(status & (MAC_INT_SW_RTURNQ|MAC_INT_SW_RTURNQ_FULL))
	{
		wla_tx_done(info);
	}
	if(status & (MAC_INT_SW_RX|MAC_INT_RX_DESCR_FULL))
	{
		wla_rxq(info, &info->wl_rxq);
	}

	if(info->mac_is_ready)
	{
		/* enable tx/rx interrupts */
		MACREG_WRITE32(MAC_INT_MASK_REG, info->int_mask); 
	}
}

static int wla_control(struct eth_drv_sc *sc, unsigned long cmd, void *args, int arglen)
{
	switch(cmd)
	{
#ifdef SIOCGPHY
    case SIOCGPHY:
		{
			struct ifreq *ifr= (struct ifreq *)args;
			int *ret = (unsigned int *)ifr->ifr_ifru.ifru_data;
			wif_vif *vif= (wif_vif *)(sc->driver_private);

			if(ret && vif)
			{
				MAC_INFO *info = WLA_MAC_INFO;
				int bss_desc = 0xff;
				short i, j;

				*ret = 0;

				for(i=0, j=info->bss_num; (j>0) && (i<MAC_MAX_BSSIDS); i++)
				{
					if(info->mbss[i].role == 0)
						continue;
					j--;
					if(info->mbss[i].vif_id == vif->unit)
					{
						bss_desc = i;
						break;
					}	
				}
				if(bss_desc == 0xff)
					break;

				if(info->mbss[bss_desc].role == WIF_AP_ROLE)
					*ret = 1;
				else if(info->mbss[bss_desc].role == WIF_STA_ROLE)
				{
					struct peer_ap *ln_ap = (struct peer_ap *)info->mbss[bss_desc].ln_ap;

					if(ln_ap)
					{
						struct wsta_cb *cb = info->wcb_tbls[ln_ap->sta_captbl];

						if(cb && cb->controlled_port)
						{
							*ret = 1;
						}
					}
				}
			}
		}
		break;
#endif
#ifdef SIOCDEVCTRL
	case SIOCDEVCTRL:
		{
			struct ifreq *ifr= (struct ifreq *)args;
			char *ifn_clone = (char *)ifr->ifr_ifru.ifru_data;
			eth_drv_sc_copy(sc, ifn_clone);
		}
		break;
#endif
#ifdef SIOCMACCTRL
	case SIOCMACCTRL:
		{
			struct ifreq *ifr=(struct ifreq *)args;
			io_mac *mac=(io_mac *)ifr->ifr_ifru.ifru_data;
			sc->driver_private = &my_wla->vifs[mac->unit];
			if(mac->type == PHYSICAL_MACADDR)
			{
				cta_eth_setmac(mac->unit, mac->mac);
			}
			bcopy(mac->mac, &sc->sc_arpcom.ac_enaddr, ETHER_ADDR_LEN);
			/* configure myself's mac address in cheetah macaddr tables */
			register_macaddr(mac->type, mac->mac);

		}
		break;
#endif
#ifdef SIOCVIFCTRL
	case SIOCVIFCTRL:
		{
			struct ifreq *ifr=(struct ifreq *)args;
			io_vif *wif=(io_vif *)ifr->ifr_ifru.ifru_data;
			wif_vif *vif=&my_wla->vifs[wif->unit];
			vif->sc = (void *)wif->sc;
			vif->unit = (unsigned short)wif->unit;
		}
		break;
#endif
	default:
		return -1;
	}
	return 0;
}

