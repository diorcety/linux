/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file txrx.c
*   \brief  wla TX/RX functions.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <wla_debug.h>
#include <wla_util.h>
#ifdef CONFIG_LED_WIFI
#include <led_api.h>
#endif
#include <wla_mat.h>
#include <wla_api.h>
#include <wla_bb.h>
#include <asm/mach-cheetah/idb.h>

/*=============================================================================+
| Variables
+=============================================================================*/
//struct macaddr_pool maddr_tables = { 0 };

#if defined(DEBUG_TXQ)
tx_descriptor debug_txq[256];
int debug_txq_index= 0;
#endif
/*=============================================================================+
| Functions
+=============================================================================*/

#ifdef CONFIG_WLAN_CLIENT_PS

/*!-----------------------------------------------------------------------------
 * function: check_tim()
 *
 *      \brief	check AID in maps. reference from ieee80211_check_tim()
 * 		\param 	
 *			
 *      \return	
 +----------------------------------------------------------------------------*/
bool check_tim(struct wlan_ie_tim *tim, u8 tim_len, u16 aid)
{
	u8 mask;
	u8 index, indexn1, indexn2;

	if((!tim) || (tim_len < sizeof(*tim)))
		return false;

	aid &= 0x3fff;
	index = aid / 8;
	mask  = 1 << (aid & 7);

	indexn1 = tim->bitctl & 0xfe;
	indexn2 = tim_len + indexn1 - 4;

	if (index < indexn1 || index > indexn2)
		return false;

	index -= indexn1;

	return !!(tim->bitmap[index] & mask);
}



/*!-----------------------------------------------------------------------------
 * function: power_saving_trigger()
 *
 *      \brief	send trigger frame or wakeup
 *		\param 	
 *			
 *      \return	
 +----------------------------------------------------------------------------*/
void power_saving_trigger(MAC_INFO *info, u8 *bssid, u8 bss_desc, u8 sta_idx, u16 aid, u8 is_qos, u8 policy)
{
	struct wbuf *wb=NULL;
	struct mbss_t *mbss = &info->mbss[bss_desc];
	u32 len=0, total_len=0;
	
	if (!(wb=wbuf_alloc(0)))
	{
		WLAN_DBG("%s: no buf for send\n",__func__);
		return;
	}

	//wb->flags |= WBUF_TX_NO_SECU|WBUF_TX_BASIC_RATE|WBUF_TX_CHECK_PS;	
	wb->flags |= WBUF_TX_NO_SECU|WBUF_TX_BASIC_RATE;
	wb->sta_idx = sta_idx;
	wb->data_off = sizeof(struct wbuf);
	wb->wh = 1;
	wb->ap = 0;
	wb->bss_desc = bss_desc & 0x7;
	wb->tid = 7;

	memset((u8 *)((u32)wb + wb->data_off), 0, sizeof(struct wlan_qoshdr));

	//printd("%s() : policy =0x%x\n", __FUNCTION__, policy);

	/* ---
		NOTE: (1) If all ac are legacy or uapsd, the TIM IE PVB bit indicates all ac's frame.
		      (2) If 4 ac are mixed with legacy & uapsd, the PVB bit only indicates the legacy ac.
	   --- */
	if(policy & PS_POLICY_PSPOLL)
	{
		struct wlan_ctrl_pspoll_hdr *pspoll = (struct wlan_ctrl_pspoll_hdr *)((u32)wb + wb->data_off);
		len = sizeof(struct wlan_ctrl_pspoll_hdr);

		pspoll->type = WLAN_FC_TYPE_CTRL;
		pspoll->subtype = WLAN_FC_SUBTYPE_PS_POLL;
		pspoll->from_ds = 0;
		pspoll->to_ds = 1;
		pspoll->aid = htows(aid);

		wb->flags |= WBUF_TX_DIS_DURATION;

		memcpy(pspoll->bssid, bssid, WLAN_ADDR_LEN);
		memcpy(pspoll->ta, maddr_tables.m[bss_desc].addr, WLAN_ADDR_LEN);
	}
	else	/* PSnonPOLL & UAPSD */
	{
		if(policy & PS_POLICY_PSnonPOLL)
		{
			wla_ps(bss_desc, PS_STATE_AWAKE, 1, 1);
			wla_ps(bss_desc, PS_STATE_AWAKE, 0, 1);
		}
		else	/* uapsd */
		{
			wla_lock();
			mbss->ps_flags |= PS_FLG_NON_SLEEP;
			wla_unlock();
		}

		struct wlan_hdr *fm = (struct wlan_hdr *)((u32)wb + wb->data_off);

		if(is_qos)
		{
			len = sizeof(struct wlan_qoshdr);
			fm->subtype = WLAN_FC_SUBTYPE_QOS_NULL;
			wb->qos = 1;
		}
		else
		{
			len = sizeof(struct wlan_hdr);
			fm->subtype = WLAN_FC_SUBTYPE_NULL;
		}

		fm->type = WLAN_FC_TYPE_DATA;
		fm->from_ds = 0;
		fm->to_ds = 1;

		memcpy(fm->addr1, bssid, WLAN_ADDR_LEN);
		memcpy(fm->addr2, maddr_tables.m[bss_desc].addr, WLAN_ADDR_LEN);
		memcpy(fm->addr3, bssid, WLAN_ADDR_LEN);

		wb->flags |= (WBUF_TX_CHECK_PS|WBUF_TX_NULL_DATA);	
	}

	wb->pkt_len = len;
	
	total_len = wb->pkt_len + wb->data_off;
	FLUSH_TX((char *)wb, total_len);
	//diag_dump_buf(wb, wb->data_off+wb->pkt_len);
	
	wla_tx_frame_handler(info, wb, wb);
}

/*!-----------------------------------------------------------------------------
 * function: check_ps_pending_rx()
 *
 *      \brief	check tim map to send trigger frame
 *		\param 	
 *			
 *      \return	
 +----------------------------------------------------------------------------*/
static void check_ps_pending_rx(MAC_INFO *info, struct wlan_qoshdr *fm, u32 len)
{
	sta_basic_info bcap;

	if(mac_addr_lookup_engine_find(fm->addr2, 0, (int *)&bcap, IN_DS_TBL) >= 0)
	{
		struct mbss_t *mbss = &info->mbss[bcap.bit.bssid];
		if((mbss->role == WIF_STA_ROLE) && (mbss->ps_rx_state > PS_STATE_AWAKE))
		{
			struct wsta_cb *wcb = info->wcb_tbls[bcap.bit.captbl_idx];
			struct wlan_ie_tim *tim=NULL;
			struct wlan_probe_resp_frame *resp;
			struct wlan_ie_generic *one=NULL;
			char *pos;
			int left;
			u8 hit=0;

			resp = (struct wlan_probe_resp_frame *)fm;
			pos = (char *)&resp[1];
			left = len - sizeof(struct wlan_probe_resp_frame);
			
			while(left >= 2) 
			{
				one = (struct wlan_ie_generic *)pos;
				
				if(one->id == WLAN_ELEMID_TIM) 
				{
					tim = (struct wlan_ie_tim *)one;
					break;
				}
				else
				{
					left -= one->len + 2;
					pos += one->len + 2;
				}
			}

			if(tim == NULL)
			{
				WLA_DBG(WLAWARN, "can't find tim ie\n");
				return;
			}

			/* check that STA's AID in bitmap */
			hit = check_tim(tim, tim->len+2, wcb->aid);
			/* FIXME: don't need to check that AP will send bc/mc frame? 	*/
			/* 		  hit |= !!(tim_ie->bitctl & 0x01); 					*/

			if(hit)
			{
				if(mbss->ps_rx_state == PS_STATE_SLEEP)
					wla_ps(bcap.bit.bssid, PS_STATE_DOZE, 1, 1);
				power_saving_trigger(info, fm->addr2, bcap.bit.bssid, bcap.bit.captbl_idx, wcb->aid, wcb->qos, mbss->ps_policy);
			}
			else if(!(mbss->ps_flags & PS_FLG_NON_SLEEP)) 
			{
				/* PS_FLG_NON_SLEEP: UAPSD AC hasn't rx EOSP */

				if(mbss->ps_rx_state == PS_STATE_DOZE)
					wla_ps(bcap.bit.bssid, PS_STATE_SLEEP, 0, 1);
				if(mbss->ps_tx_state == PS_STATE_DOZE)
					wla_ps(bcap.bit.bssid, PS_STATE_SLEEP, 1, 1);
			}
			
			/* FIXME : should consider the scan process 			*/
			/* 		   filter is deleted in client_tbtt_handle() 	*/
			if(mbss->ps_rx_state > PS_STATE_AWAKE)
				wla_cfg(SET_RX_FILTER, RX_FILTER_BEACON);
		}
	}
}

/*!-----------------------------------------------------------------------------
 * function: check_ps_more_bit()
 *
 *      \brief	check more bit to send trigger frame
 *              check eosp to re-ennter sleep state
 *		\param 	
 *			
 *      \return	
 +----------------------------------------------------------------------------*/
static void check_ps_more_bit_and_eosp(MAC_INFO *info, struct wbuf *wb)
{
	u8 bss_desc = wb->bss_desc;
	u8 sta_idx = wb->sa_idx;
	u8 addr[6];
	struct mbss_t *mbss = &info->mbss[bss_desc];
	struct wsta_cb *wcb = info->wcb_tbls[sta_idx];

#if 0
	printd("%s() : bss_desc=%d, sta_idx=%d, mdat=%d, eosp=%d, ps_state=%d\n", __FUNCTION__, bss_desc, sta_idx, wb->mdat, wb->eosp, mbss->ps_rx_state);
	diag_dump_buf(wb, wb->data_off+wb->pkt_len);
#endif

	if((mbss->role == WIF_STA_ROLE) && wcb && (mbss->ps_rx_state > PS_STATE_AWAKE))
	{
		if(wb->mdat)
		{
			if(mac_addr_lookup_engine_find(addr, wb->sta_idx, 0, (IN_DS_TBL|BY_ADDR_IDX)) >= 0)
				power_saving_trigger(info, addr, bss_desc, sta_idx, wcb->aid, wcb->qos, mbss->ps_policy);
		}
		else if(wb->eosp)	/* uapsd end of service */
		{
			wla_lock();
			mbss->ps_flags &= ~PS_FLG_NON_SLEEP;
			wla_unlock();
		}
	}
}
#endif // CONFIG_WLAN_CLIENT_PS

#ifdef CONFIG_TX_MCAST2UNI
/*!-----------------------------------------------------------------------------
 * function: mcast2ucast()
 *
 *      \brief	multicast to unicast converter
 *		\param 	i: bss desc index 
 *				wb: original wbuf 
 *      \return	true or false
 +----------------------------------------------------------------------------*/
static int mcast2ucast(struct wbuf *wb, int from_bss)
{
	MAC_INFO *info = &my__dev.mac_info;
	struct wbuf *wb1, *wb_tx, wb2;
	struct wsta_cb *wcb;
	int i, bcap, datap;
	char addr[6];
	sta_cap_tbl *captbl;

	for(i=0; i<info->sta_cap_tbl_count; i++)
	{
		captbl = &info->sta_cap_tbls[i];
		/* find out valid station */
		if(!captbl->valid)
			continue;
		if(captbl->bssid != wb->bss_desc)
			continue;
		bcap = 0;

		/* multicast come from station and match it's self */
		if((!IS_FROM_ETH(from_bss)) && (i == wb->sta_idx))
			continue;
		/* first element of wcb do not use to compare */
		if(!(wcb = info->wcb_tbls[i]))
			continue;
		/* clone wbuf to send */
		if(((wb1=WBUF_COPY(wb, &wb2)) == 0))
			continue;
		/* get station MAC address */
		mac_addr_lookup_engine_find(addr, wcb->addr_idx, &bcap, BY_ADDR_IDX);
		wb1->flags &= ~WBUF_TX_BASIC_RATE;
		wb1->bss_desc = captbl->bssid;
		wb1->sta_idx = wcb->captbl_idx;
		wb_tx = WBUF_COPY_TXBUF(wb, wb1);
		datap = WBUF_COPY_DATAP(wb, wb1);
		memcpy((char *)datap, addr, WLAN_ADDR_LEN);
		wla_tx_frame_handler(info, wb1, wb_tx);
	}

	return 1;
}
#endif

static int wla_chk_ctrlport(struct wbuf *wb, struct wsta_cb *wcb)
{
	ehdr *ef;

	if(wcb->controlled_port)
		return 1;
	
	if(!wb->wh) {

		ef = (ehdr *)((int)wb + wb->data_off);
		
		if(ef->type == ETHERTYPE_EAPOL) {
			/*For EAPOL encrypt or not.*/
			unsigned short *p =(unsigned short *)((char *)ef + 19);
			if(*p&(0x8)) {
				wb->flags |= WBUF_TX_NO_SECU;
			//	idb_print("!!!!! [%s] EAPOL NO SEC\n",__func__);
			}
			//else
			//	idb_print("!!!!! [%s] EAPOL SEC\n",__func__);
			return 1;
		}

		if(ef->type == ETHERTYPE_WAPI)
			return 1;
	}

	return 0;
}

/*!-----------------------------------------------------------------------------
 * function: resp_deauth_frame()
 *
 *      \brief	Once received frame from non-associated STA, reject it with 
				deauth frame.
 *		\param 	
 *			
 *      \return	void
 +----------------------------------------------------------------------------*/
static void resp_deauth_frame(struct wbuf *wb, struct wlan_qoshdr *fm)
{
	MAC_INFO *info = WLA_MAC_INFO;
	struct wlan_deauth_frame *deauth;
	char *bssid;

	WLA_DBG(WLADEBUG, "RX non-associated STA's frame, addr=%s\n", wmaddr_ntoa((char *)fm->addr2)); 

	switch(fm->type)
	{
 		case WLAN_FC_TYPE_CTRL:
 			bssid = fm->addr1;
 			break;
 		case WLAN_FC_TYPE_DATA:
 			if(fm->from_ds)
 				bssid = fm->addr2;
 			else
				bssid = fm->addr1;
			break;
		default:
			WLA_DBG(WLADEBUG, "Could not deauth this frame!\n");
			wbuf_free(wb);
			return;
	}

	wb->flags = WBUF_FIXED_SIZE|WBUF_TX_BASIC_RATE|WBUF_MGMT_FRAME;

	/* It can let dst memory does not overlay with src memory */
	wb->data_off += 24;
	wb->wh = 1;

	deauth = (struct wlan_deauth_frame *)((int)wb + wb->data_off);
	memset(deauth, 0 , sizeof(*deauth)); 
	BUILD_MGT_FRAME(&deauth->hdr, WLAN_FC_SUBTYPE_DEAUTH, fm->addr2, fm->addr1, bssid);
	deauth->reasoncode = htows(WLAN_REASON_CLASS3_FRAME_FROM_NONASSOC_STA);
	wb->pkt_len = sizeof(*deauth);


	DCACHE_STORE((unsigned int)wb, WLAN_TX_RESERVED_LEADIND_SPACE + wb->pkt_len);

	wla_tx_frame_handler(info, wb, wb);
}

/*!-----------------------------------------------------------------------------
 * function: wframe_format()
 *
 *      \brief	transfer wlan header of data frame to ethernet header
 *				(DA+SA+ethertype+payload)
 *		\param 	wb: original wbuf
 *				fm: wlan header pointer 
 *      \return	ETHERNET header pointer
 +----------------------------------------------------------------------------*/
ehdr *wframe_format(struct wbuf *wb, struct wlan_hdr *fm)
{
	ehdr *ef;
	char da[WLAN_ADDR_LEN];

	ef = (ehdr *)(wb_to_llc(wb) + 8 - ETHER_HDR_LEN);
	if(wb->hit & WBUF_RX_TA_HIT_DS)
	{
		if(wb->ap == 0)
		{
			/* I play client role. */
			memcpy(ef->sa, fm->addr3, WLAN_ADDR_LEN);
			memcpy(ef->da, fm->addr1, WLAN_ADDR_LEN);
		}
		else
		{
			struct wlan_hdr_addr4 *fm4 = (struct wlan_hdr_addr4 *)fm;
			/* I play AP role and RX WDS frame */
			memcpy(ef->sa, fm4->addr4, WLAN_ADDR_LEN);
			memcpy(ef->da, fm4->addr3, WLAN_ADDR_LEN);
		}
	}
	else
	{
		/* I play AP role. */
		/* backup addr2, addr3 */
		memcpy(da, fm->addr3, WLAN_ADDR_LEN);
		memcpy(ef->sa, fm->addr2, WLAN_ADDR_LEN);
		memcpy(ef->da, da, WLAN_ADDR_LEN);
	}

	wb->pkt_len = (int)fm + wb->pkt_len - (int)ef;
	wb->data_off = (int)ef - (int)wb;

	wb->wh = 0;

	return ef;
}

/*!-----------------------------------------------------------------------------
 * function: ethernet_tx_basicrate()
 *
 *      \brief	ARP, DHCP, EAPOL and WAI force to basic rate
 *		\param
 *				
 *      \return
 +----------------------------------------------------------------------------*/
static void ethernet_tx_basicrate(struct wbuf *wb, ehdr *ef)
{
#ifndef IPPROTO_UDP 
#define IPPROTO_UDP             17 
#endif
	switch(ef->type)
	{
		case ETHERTYPE_IP:
		{
			struct ip *iph = (struct ip *)ef->data;
			struct udphdr *udp = (struct udphdr *)((unsigned int)iph + iph->ip_hl*4);
			if((iph->ip_p == IPPROTO_UDP)&&
			  ((udp->uh_dport==67)||(udp->uh_dport==68)))
			{
				wb->flags |= WBUF_TX_SPEC_RATE;
			}
			break;
		}
		case ETHERTYPE_ARP:
		case ETHERTYPE_EAPOL:
			wb->flags |= WBUF_TX_SPEC_RATE;
			break;
#if defined(CONFIG_WLA_WAPI) || defined(CONFIG_WAPI)
		case ETHERTYPE_WAPI:
			wb->flags |= (WBUF_TX_SPEC_RATE|WBUF_TX_NO_SECU);
			break;
#endif
	}
}

/*!-----------------------------------------------------------------------------
 * function: eth_parse_and_forward()
 *
 *      \brief	fill in wbuf header of ethernet frame
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void ethernet_parse_and_forward(MAC_INFO *info, struct wbuf *wb, char from_bss)
{
	int idx;
	char *datap;

	datap = (char *)wb + wb->data_off;

	if(((idx = mac_addr_lookup_engine_find(datap, 0, 0, 0)) < 0) && (idx == LOOKUP_WRONG_TBL))
	{
		if((idx = mac_addr_lookup_engine_find(datap, 0, 0, IN_DS_TBL)) < 0)
		{
			if((datap[0] & 0x1) == 0)
				WLA_DBG(WLAWARN, "unicast DA is not found in address table\n");
			wb->bss_desc = info->first_bss_desc;	
		}
	}

	if(idx >= 0)
	{
		wb->sta_idx = idx;
		wb->bss_desc = info->sta_cap_tbls[idx].bssid;
		wb->hit |= WBUF_RX_DA_HIT_ADDR;
		if(info->mbss[wb->bss_desc].role & WIF_STA_ROLE)  
			wb->ap = 0;
	}

	/* Use IP header to find out DSCP priority, otherwise priority = 0 */
	if((*(unsigned short *)&datap[12] == 0x0800) && ((datap[14] & 0xf0) == 0x40))
		wb->tid = (datap[15] & 0xe0) >> 5;
	else
		wb->tid = 0;
	/* To check ACM bit, if ACM bit = 1, downgrade UP to BE */
	if(info->mbss[wb->bss_desc].wme_acm & BIT(wb->tid))
		wb->tid = 0;

	WLA_TIMESTAMP_TRACE(!info->tput_w2e, 2);

	data_frame_forward(wb, (ehdr *)datap, (int)from_bss);

	return; 
}

/*!-----------------------------------------------------------------------------
 * function: data_frame_forward()
 *
 *      \brief	Forwarding core function. Input frames from wlan/eth2wifi/ethernet and forward.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void data_frame_forward(struct wbuf *wb, ehdr *ef, int from_bss)
{
	MAC_INFO *info = WLA_MAC_INFO;
	int i;
	struct wsta_cb *wcb;
	unsigned char vif_id;
	int datap;
	struct wbuf *wb_tx, wb2;

	from_bss &= 0xff;

	ethernet_tx_basicrate(wb, ef);
	if(IS_FROM_ETH(from_bss))
		vif_id =  ETH_VIFID(from_bss);
	else
		vif_id = info->mbss[from_bss].vif_id;

	/* DA is a station of BSS */
	if((wb->hit & WBUF_RX_DA_HIT_ADDR)
#ifdef WLA_MULTICAST_PATCH
		&& (wb->sta_idx != 255)
#endif
		)
	{
		/* DA hit frame should bind with DA's bss descriptor */

		wcb = info->wcb_tbls[wb->sta_idx];
		
		/*Check This*/
		if(!wcb) {
			idb_print("[%s]:WCB NULL, sta_idx:%d max:%d\n",__func__,wb->sta_idx,info->sta_cap_tbl_count);
			goto free_wb;
		}
		/* Check port is authorized */
		if(!wla_chk_ctrlport(wb,wcb))
			goto free_wb;

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
		if(wcb->mode == MAT_LINKED_AP)
			mat_handle_insert(wb, ef, maddr_tables.m[wcb->bss_desc].addr);
#endif

		//WLA_DBG(WLADEBUG, "Forward Hit DA's data frame\n");
		WLA_TIMESTAMP_TRACE(!info->tput_w2e, 2);

		wla_tx_frame_handler(info, wb, wb);

		WLA_TIMESTAMP_TRACE(!info->tput_w2e, 3);

		WLA_TIMESTAMP_DELTA_CAL(!info->tput_w2e);
		return;
	}
	else if(!IS_FROM_ETH(from_bss) && (wb->hit & WBUF_RX_DA_HIT_BSSID))
	{
		WLA_DATA_FRAME_ETH_FORWARD(wb, ef);
		return;
	}
	/* clone broadcast/multicast frame and forward */
	else // ((IS_GROUPCAST(ef->da)) || !(wb->flags & WBUF_RX_DA_HIT_ADDR))
	{
		struct wbuf *wb1=NULL;
		//struct wsta_cb *peer_ap;
		struct peer_ap *ap;
		short j = info->linked_ap_count;

		/* Send unicast frame to every AP and wait response to learn his address */
		for(i = 0;(j > 0) && (i < MAX_PEER_AP); i++)
		{
			ap = &info->linked_ap[i];
			if(ap->type == 0)
				continue;
			j--;
			/* Don't forward to source AP */
			if((wb->hit & WBUF_RX_TA_HIT_DS) && (wb->sa_idx == ap->sta_captbl))
				continue;
			/* WAN/LAN should have different VIF ID */
			if(info->mbss[ap->bss_desc].vif_id != vif_id)
				continue;

			wcb = info->wcb_tbls[ap->sta_captbl]; 
			/* Check port is authorized */
			if(!wla_chk_ctrlport(wb,wcb))
				continue;
	
			if((wb1=WBUF_COPY(wb, &wb2)) == 0)
				continue;

			
			wb_tx = WBUF_COPY_TXBUF(wb, wb1);
			datap = WBUF_COPY_DATAP(wb, wb1);
			wb1->sta_idx = ap->sta_captbl;
			wb1->bcmc = 0;
			wb1->bss_desc = ap->bss_desc;

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
			if(ap->type == MAT_LINKED_AP)
			{
				mat_handle_insert(wb1, (ehdr *)datap, maddr_tables.m[ap->bss_desc].addr);
			}
#endif
			wla_tx_frame_handler(info, wb1, wb_tx);
		}

		/* 
			Forward broadcast/multicast frames to every BSS,
			1. only send to WDS under bridge mode
			2. from eth2wifi
			3. from wirelss and No AP isolated, 
		*/	
		if((IS_GROUPCAST(ef->da)) && (IS_FROM_ETH(from_bss) || !info->ap_isolated))
		{
			short j = info->bss_num;
			
			/* broadcast to every BSS */
			for(i=0; (j>0) && (i<MAC_MAX_BSSIDS); i++)
			{
				if(info->mbss[i].role == 0)
					continue;
				j--;
				/* sta_count should be zero when bss play Client mode */
				/* Under WDS bridge mode, No STA could be associated */
				if(info->mbss[i].sta_count == 0)
					continue;
				/* WAN/LAN should have different VIF ID */
				else if(info->mbss[i].vif_id != vif_id)
					continue;
				else if(!IS_FROM_ETH(from_bss))
				{
					/* wifi to wifi frames be denied according bss_isolated */
					if(info->bss_isolated && (from_bss != i))
						continue;
					/* If source BSS has only one STA, the frame should not reflect to itself. */ 
					else if((from_bss == i) && (info->mbss[i].sta_count <= 1))
						continue;
				}
				/* Do not clone to last bss */
				else if(i == (info->first_bss_desc+info->bss_num-1))
				{
					wb1 = wb;
					wb = 0;
				}

#ifdef CONFIG_TX_MCAST2UNI
				if(!info->mcast2ucast_mode)
					goto m2u_end;
				if(!IS_MCAST_MAC(ef->da))
					goto m2u_end;
				if(wb1 && (wb==0))
					wb = wb1;
				wb->bss_desc = i;	
				if(mcast2ucast(wb, from_bss))
					continue;
m2u_end:
#endif
				/* protect resource if transmitting frames with lowest rate */
				if(info->sw_wtx_bcast_pkt_waiting > WLAN_BCAST_THRESHOLD)
				{
					if(wb1 && (wb==0))
						wb = wb1;

					continue;
				}
				
				if(wb && ((wb1=WBUF_COPY(wb, &wb2)) == 0))
					continue;

				wb_tx = WBUF_COPY_TXBUF(wb, wb1);
				wb1->flags |= WBUF_TX_BASIC_RATE;
				wb1->bcmc = 1;
				wb1->bss_desc = i; /*FIXME: should indicate da_idx */
				wla_tx_frame_handler(info, wb1, wb_tx);
#ifdef CONFIG_WLAN_HW_FORWARD_MULTICAST
				if(IS_MULTICAST(ef->da) && (mac_addr_lookup_engine_find(ef->da, 0, 0, 0) < 0))
				{
					sta_basic_info bcap;
					int addr_index;
						
					bcap.val = 0;
					bcap.bit.bssid = i;
					bcap.bit.vld = 1;
	
					addr_index = mac_addr_lookup_engine_update(info, (int)ef->da, 
								(bcap.val | info->mbss[i].tx_rate.multicast_rate_captbl), 0);
					WLA_DBG(WLAWARN, "Add multicast addr:%s, addr_index=%d\n", ether_ntoa((struct ether_addr *)ef->da), addr_index);
				}	
#endif
			}
		}

	}

	/* All DA miss frames will forward to ethernet */
	if(IS_FROM_ETH(from_bss))
	{
		if(!wb)
			return;

		goto free_wb;
	}
	else
	{
		/* To Ethernet frame needs bss descriptor to decide vlan */
		wb->bss_desc = from_bss; 
	}

	/* Others forward to ethernet */
#ifdef WLA_DEBUG_STATISTIC
	if(!IS_GROUPCAST(ef->da))
	{
		info->sw_w2e_unicast_pkt_cnt++;
		info->sw_w2e_unicast_pkt_bytes += wb->pkt_len;
	}
	else
		info->sw_w2e_bcast_pkt_cnt++;
#endif
#ifdef WLA_DEBUG_DUMP
	if(info->w2e_tx_dump)
	{
		WLA_DBG(WLADEBUG, "wifi -> eth: wb=%p, wb->data_off=%d, wb->pkt_len=%d\n", wb, wb->data_off, wb->pkt_len);
		WLA_DUMP_BUF((char *)ef, wb->pkt_len);
	}
#endif

	WLA_TIMESTAMP_TRACE(info->tput_w2e, 4);

#if defined(CONFIG_WIFI2USB)
	wla_usb_tx_frame(info, wb);
#elif defined(WLA_WIFI2ETH)
	w2e_tx_frame(info, wb);
#else
	WLA_DATA_FRAME_ETH_FORWARD(wb, ef);
#endif

	WLA_TIMESTAMP_TRACE(info->tput_w2e, 5);
	WLA_TIMESTAMP_DELTA_CAL(info->tput_w2e);

	return;

free_wb:

	WBUF_FREE(wb);
}

/*!-----------------------------------------------------------------------------
 * function: wrx_eframe_handler()
 *
 *      \brief	Handle Ethernet format frames. It should be data frame and to be 
				forward.	
 *		\param	struct wbuf *wb 
 *      \return	void
 +----------------------------------------------------------------------------*/
void wrx_eframe_handler(struct wbuf *wb)
{
	ehdr *ef;
	struct wsta_cb *wcb;
	MAC_INFO *info = WLA_MAC_INFO;

	if(info->filter & RX_FILTER_DATA_FRAME)
		goto data_drop;

#ifdef WLA_DEBUG_DUMP
	if(info->w_rx_dump == 0x2)
	{
		WLA_DBG(WLADEBUG, "WIFI DATA RX: wb->hit = 0x%x\n", wb->hit);
		WLA_DUMP_BUF((char *)((int)wb + wb->data_off), wb->pkt_len);
	}
#endif

	/* 
		1. Data frame's SA MUST hit when I play AP role.
		2. Data frame's TA MUST hit when I play client role. 
	 */
	if((wb->hit & (WBUF_RX_SA_HIT_ADDR|WBUF_RX_TA_HIT_DS)) == 0)
	{
		/* No hit frames should not be transfer to ethernet format */ 
		WLA_DBG(WLADEBUG, "It should not happen, %s(%d)!!, wb->hit=%x,wb->pkt_len=%d\n", __func__, __LINE__,  wb->hit, wb->pkt_len);
		WLA_DUMP_BUF((char *)((int)wb + wb->data_off), wb->pkt_len);
		goto data_drop;
	}

	ef = (ehdr *)((int)wb + wb->data_off);


	if((wcb = update_ps_state(wb, 0)) == 0) 
		goto data_drop;

#ifdef CONFIG_WLAN_CLIENT_PS
	check_ps_more_bit_and_eosp(info, wb);
#endif // CONFIG_WLAN_CLIENT_PS

	if(wb->pkt_len < 30)
	{
		WLA_DBG(WLADEBUG, "short PKT LEN:%d\n", wb->pkt_len);
		WLA_DUMP_BUF((char *)ef,  wb->pkt_len);
	}

	/* For WDS AP, we only receive from DS and to DS frame */ 
	if((wcb->mode == WDS_LINKED_AP) && !wb->ap)
		goto data_drop;

	/*
		Once data frame with SA equal to our macaddr, that means 
		1. When I play client role, root AP forward my broadcast frame.
		2. When I play AP role, it is illege frame.
	*/
	if(memcmp(maddr_tables.m[(int)wcb->bss_desc].addr, ef->sa, 6) == 0) 
	{
		goto data_drop;
	}

	/* 	
		MIC failure to do counter measure process 
	*/
	if(wb->secst & WBUF_RX_MIC_ERR)
	{
		if(info->wla_mgt_handler) 
		{
			info->wla_mgt_handler(wb, (void *)wcb);
			return;
		}
		goto data_drop;
	}

	/* My frames */
	/* We expected only EAPOL packet targeted to BSSID */
	/* FIXME: the client frame without WBUF_RX_DA_HIT_BSSID on lynx platform */
	if((wb->hit & WBUF_RX_DA_HIT_BSSID) || (wb->ap == 0))
	{
		/*	Marvell's AMPSDU may let both addr1 and addr3 equal to bssid. 
			We will get wrong WBUF_RX_DA_HIT_BSSID */
		if(memcmp(maddr_tables.m[(int)wcb->bss_desc].addr, ef->da, 6) != 0) 
		{
			wb->hit &= ~WBUF_RX_DA_HIT_BSSID;
		}

		/* EAPOL frames to EAP FSM */
		if((ef->type == ETHERTYPE_EAPOL) || (ef->type == ETHERTYPE_WAPI))
		{
			if(info->wla_eapol_handler) 
			{
				//idb_print("GOT EAPOL FRAME\n");
				info->wla_eapol_handler(wb, ef);
				return;
			}
		}
	}

	/* Frame needs to forward will fallback to here */
	if(!wla_chk_ctrlport(wb,wcb))
	{
		WLA_DBG(WLADEBUG, "Data frame is not authorized\n");
		goto data_drop;
	}
	

	if(wb->hit & WBUF_RX_TA_HIT_DS)
	{
		int addr_index;

		/* learn STAs from AP */
		if((wb->hit & WBUF_RX_SA_HIT_ADDR) == 0)
		{
			sta_basic_info bcap;

			/* Double checking to avoid burst incoming NON-HIT frames may 
				create multiple same entries */
			if(mac_addr_lookup_engine_find(ef->sa, 0, 0, 0) < 0)
			{
				/* copy basic capability of DS to new addr entry */
				mac_addr_lookup_engine_find(0, wcb->addr_idx, (int *)&bcap, IN_DS_TBL|BY_ADDR_IDX);
				/* add for fresh SA */ 
				/* [31:8] is basic capability, [7:0] is lookup index */
				addr_index = mac_addr_lookup_engine_update(info, (int)ef->sa, 
								(bcap.val | wb->sa_idx), 0);

				wcb->wds_sa_count++;
				/* allocate wds_sa to trace age time ? */

				WLA_DBG(WLADEBUG, "Add sta of AP, addr index=%d, addr=%s\n", addr_index, wmaddr_ntoa((char *)ef->sa));
			}
		}

#ifdef CONFIG_WLA_UNIVERSAL_REPEATER
		if(wcb->mode == MAT_LINKED_AP)
		{
			if(wb->bcmc == 0)
			{
				/* unicast frame under client mode */
				/* MAC Address Translate: replace the da of the frame from the Root AP */
				if(mat_handle_lookup(wb, ef) == 0)
				{
					addr_index = mac_addr_lookup_engine_find(ef->da, 0, 0, 0);			
					if(addr_index >= 0)
					{
						wb->sta_idx = addr_index;
						wb->hit |= WBUF_RX_DA_HIT_ADDR;
						wb->hit &= ~WBUF_RX_DA_HIT_BSSID;
					}	
					else
					{
						wb->hit &= ~(WBUF_RX_DA_HIT_BSSID | WBUF_RX_DA_HIT_ADDR);
					}

					/*WLA_DBG(WLADEBUG, "After mac_addr_lookup_engine_find, wb->hit=0x%x, wb->sta_idx=%d\n",
						wb->hit, wb->sta_idx);	*/
				}
			}
#ifdef CONFIG_MAT_NO_FLUSH_ENTRY_FROM_DHCP
			else
			{
				mat_insert_by_dhcp(ef);
			}
#endif
		}
#endif //  CONFIG_WLA_UNIVERSAL_REPEATER
	}

	/* Should this code move into wla_tx_frame() and wla_eth_tx_frame() ? */
	/* splite wbuf chain and transmit individually */
	if(wb->flags & WBUF_CHAIN)
	{
		struct wbuf *wb1;
		struct wbuf wb_header;

		wb_header = *wb;

		while(wb)
		{
			wb1 = wb->wb_next;
			/* only clone word 0,1 and hit from first frame */
			((int *)wb)[0] = ((int *)&wb_header)[0];
			((int *)wb)[1] = ((int *)&wb_header)[1];
			wb->hit = wb_header.hit;
			wb->wb_next = 0;
			wb->flags &= ~WBUF_CHAIN;
			ef = (ehdr *)((int)wb + wb->data_off);	
			data_frame_forward(wb, ef, (int)info->sta_cap_tbls[wcb->captbl_idx].bssid);
			wb = wb1;
		}
	}
	else
	{
		WLA_TIMESTAMP_TRACE(info->tput_w2e, 3);	
		data_frame_forward(wb, ef, (int)info->sta_cap_tbls[wcb->captbl_idx].bssid);
	}

	return;

data_drop:
	
	WBUF_FREE(wb);
	return;
}

/*!-----------------------------------------------------------------------------
 * function: rx_frame_dispatcher()
 *
 *      \brief	Classify wlan frame into mgmt/control/data frame, then call properly handler respectively.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void rx_frame_dispatcher(struct wbuf *wb)
{
	struct wlan_qoshdr *fm;
	MAC_INFO *info = WLA_MAC_INFO;

#ifdef CONFIG_LED_WIFI
	led_toggle(LED_WIFI);
#endif
#ifdef WLA_DEBUG_DUMP
	if(info->w_rx_dump == 0x1)
	{
		WLA_DBG(WLADEBUG, "WIFI RX: wb->flags = 0x%x\n", wb->flags);
		WLA_DUMP_BUF((char *)((int)wb + wb->data_off), wb->pkt_len);
	}
#endif

	if(!wb->wh)
	{
		wrx_eframe_handler(wb);
		return;
	}

	fm = (struct wlan_qoshdr *)((int)wb + wb->data_off);

	switch(fm->type)
	{
		case WLAN_FC_TYPE_MGT:
		{
			if(info->filter & RX_FILTER_MGMT_FRAME)
					break;

			if(fm->subtype == WLAN_FC_SUBTYPE_BEACON)
			{
				if(info->filter & RX_FILTER_BEACON)
					break;
#ifdef CONFIG_WLAN_CLIENT_PS
				check_ps_pending_rx(info, fm, wb->pkt_len);
#endif // CONFIG_WLAN_CLIENT_PS
			}
#if 0
			else if(fm->subtype == WLAN_FC_SUBTYPE_PROBE_REQ)
			{
				if(info->filter & RX_FILTER_PROBE_REQ)
					break;
			}
#endif
			else if(fm->subtype == WLAN_FC_SUBTYPE_PROBE_RESP)
			{
				/* If use probe req to keep alive, must rx probe resp */
#ifdef CONFIG_MONTAGE_HW_REPORT_TX_ACK
				if(info->filter & RX_FILTER_PROBE_RESP)
					break;
#endif
			}

			wb->flags |= WBUF_MGMT_FRAME;

			//WLA_DBG(WLADEBUG, "RX: MGT frame\n");
			//WLA_DUMP_BUF(fm, 128);
			if(info->wla_mgt_handler)
			{
				info->wla_mgt_handler(wb, NULL);
				return;
			}

			break;
		}
		case WLAN_FC_TYPE_CTRL:
		{
			if(info->filter & RX_FILTER_CTRL_FRAME)
					break;

			if(fm->subtype == WLAN_FC_SUBTYPE_CF_END)
				WLA_DBG(WLADEBUG, "RX: CF-END\n");
			else if(fm->subtype == WLAN_FC_SUBTYPE_PS_POLL)
			{
				WLA_DBG(WLADEBUG, "RX: PS-POLL\n");
				if((wb->hit & WBUF_RX_SA_HIT_ADDR) == 0)
				{
					/* UMAC may lose SA_HIT even if STA is exist. Try to lookup again. */
					if(mac_addr_lookup_engine_find(fm->addr2, 0, 0, 0) < 0)
					{
						/* Receives illege frame, we should deauth him to avoid it attacks me */ 
						resp_deauth_frame(wb, fm);
						return;
					}

				}
				else
					update_ps_state(wb, 1);
			}
			else if(fm->subtype == WLAN_FC_SUBTYPE_BAR)
			{
#if defined(WLA_AMPDU_RX_SW_REORDER)
				struct wlan_ba_req *bar = (struct wlan_ba_req *)fm;	
				struct wsta_cb *cb;
				struct rx_ba_session *ba;

				WLA_DBG(WLADEBUG, "RX: BAR\n");

				cb = info->wcb_tbls[wb->sa_idx];
				ba = cb->ba_rx[wtohs(bar->barctl) >> WLAN_BAR_TID_S];
				if(ba)
					reorder_buf_release(ba, wtohs(bar->barseqctl) >> WLAN_BAR_SEQ_START_S, 0);
				else
					WLA_DBG(WLADEBUG, "NO BA exist???\n");
#endif
				WBUF_FREE(wb);
				return;
			}
			else
			{
#if defined(CONFIG_CHEETAH_SMART_CONFIG) && 0
				smart_state((char *)fm, wb->pkt_len);
#endif	
				//WLA_DBG(WLADEBUG, "RX: ctrl frame, length=%d\n", wb->pkt_len);
				//WLA_DUMP_BUF((char *)fm,  wb->pkt_len);
			}
			break;
		}
		case WLAN_FC_TYPE_DATA:
		{
			if(info->filter & RX_FILTER_DATA_FRAME)
					break;

			//WLA_DUMP_BUF(fm,  wb->pkt_len);
			if((wb->hit & (WBUF_RX_SA_HIT_ADDR|WBUF_RX_TA_HIT_DS)) == 0)
			{
				/* Receives illege frame, we should deauth him to avoid it attacks me */ 
				if(memcmp(fm->addr2, info->peer_cache, 6))
				{
					resp_deauth_frame(wb, fm);
					return;
				}
			}

			/* drop null frame */
			if((fm->subtype == WLAN_FC_SUBTYPE_NULL) || 
				(fm->subtype == WLAN_FC_SUBTYPE_QOS_NULL))
			{
				//WLA_DBG(WLADEBUG, "RX: NULL DATA frame\n");
				update_ps_state(wb, 0);

#ifdef CONFIG_WLAN_CLIENT_PS
				/* For UAPSD EOSP handle */
				check_ps_more_bit_and_eosp(info, wb);
#endif // CONFIG_WLAN_CLIENT_PS
				break;
			}

			/* I expected No frame fall in here, except of frame that UMAC can not transfer format. */
			WLA_DBG(WLADEBUG, "??? unexpected data frame\n");

			wframe_format(wb, (struct wlan_hdr *)((int)wb + wb->data_off));
			wrx_eframe_handler(wb);
			return; 
		}
		default:
			break; 
	}

	WBUF_FREE(wb);
}

struct wbuf *amsdu_slice(MAC_INFO* info, unsigned short head, unsigned short tail)
{
	struct wbuf *wb, *wb1, wb_header, *old_wb;
	buf_header *bhdr;
	int sf_len, remaining, copy_len;
	amsdu_sf *sf;
	ehdr *ef;
	char *dst, *src, *start;

	bhdr = idx_to_bhdr(info, head);
	
	/* we use cached address to speed up memcpy */
	start = (char *)CACHED_ADDR(bhdr->dptr) + ((bhdr->offset_h << 6) | bhdr->offset);
	src = (char *)CACHED_ADDR(bhdr->dptr) + WLAN_RX_LLC_OFFSET;
	remaining = bhdr->len - ((int)src - (int)start);
//	DCACHE_INVALIDATE((unsigned int)src, remaining);

	wb1 = old_wb = 0;
	wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
	wb->wb_next = 0;
	/* copy wdesc to first wb */
	wb_header = *wb;

	/* First subframe's IP header does not align with 4 Bytes. 
		We must copy it to new wb to avoid alignment exception. */

	sf = (amsdu_sf *)src;
	sf_len = sf->len + ETHER_HDR_LEN;
	if(sf_len > 1524)
	{
		WLA_DBG(WLAERROR, "wrong subframe length:%d\n", sf_len);
		goto err;
	}

	/* extract subframe to new wbuf */
	while(1)
	{
		//WLA_DBG(WLADEBUG, "remaining=%d, bhdr=%x\n", remaining, bhdr);
		if((remaining < ETHER_HDR_LEN) && (bhdr_to_idx(info, bhdr) == tail))
			break; 

		if((wb=(struct wbuf *)WBUF_ALLOC(0)) == 0)
		{
			WLA_DBG(WLAERROR, "%s: WBUF_ALLOC failed\n", __func__);
			goto err;
		}

		if(wb1 == 0)
		{
			wb1 = wb;
			*wb1 = wb_header;
		}
		/* shift 2 Bytes for IP alignment */
		dst = (char *)CACHED_ADDR(wb)+WLAN_TX_RESERVED_LEADIND_SPACE + 2;
		sf = (amsdu_sf *)dst;
		sf_len = (sf_len + 0x3) & ~(0x3);

		/* First, copy subframe's header to new wbuf and get subframe length */
		/* remaining data is not enough for getting subframe length */
		if(remaining < ETHER_HDR_LEN)
		{
			memcpy(dst, src, remaining);
			dst += remaining;

			bhdr = idx_to_bhdr(info, bhdr->next_index);
			/* we use cached address to speed up memcpy */
			src = (char *)CACHED_ADDR(bhdr->dptr) + ((bhdr->offset_h << 6) | bhdr->offset);
			remaining = bhdr->len;
//			DCACHE_INVALIDATE((unsigned int)src, remaining);

			copy_len = ETHER_HDR_LEN - ((int)dst - (int)sf);
			memcpy(dst, src, copy_len);

			dst += copy_len;
			src += copy_len;
			remaining -= copy_len;
		}
		else
		{
			memcpy(dst, src, ETHER_HDR_LEN);
			dst += ETHER_HDR_LEN;
			src += ETHER_HDR_LEN;
			remaining -= ETHER_HDR_LEN;
		}

		sf_len = sf->len + ETHER_HDR_LEN;
		if(sf_len > 1524)
		{
			WLA_DBG(WLAERROR, "wrong subframe length:%d\n", sf_len);
			goto err;
		}

		wb->pkt_len = sf_len - 8; /* exclude padding and LLC header */ 

		/* Each AMSDU subframe's length is a multiple of 4 octets, except the last */
		sf_len = (sf_len + 0x3) & ~(0x3);
		sf_len -= ETHER_HDR_LEN;

		/* copy subframe's body to wbuf */
		while(sf_len > 0)
		{
			if(remaining <= 0)
			{
				if(bhdr_to_idx(info, bhdr) != tail)
				{
					bhdr = idx_to_bhdr(info, bhdr->next_index);
					/* we use cached address to speed up memcpy */
					src = (char *)CACHED_ADDR(bhdr->dptr) + ((bhdr->offset_h << 6) | bhdr->offset);
					remaining = bhdr->len;
//					DCACHE_INVALIDATE((unsigned int)src, remaining);
				}
				else
				{
					WLA_DBG(WLADEBUG, "%s(), remaining is samller than sf'len=%d\n", __func__, sf_len);
					goto err;
				}
			} 

			if(sf_len > remaining)
				copy_len = remaining;
			else
				copy_len = sf_len;
			//WLA_DBG(WLADEBUG, "copy_len=%d\n", copy_len);
			memcpy(dst, src, copy_len);

			dst += copy_len;
			src += copy_len;
			sf_len -= copy_len;
			remaining -= copy_len;
		}

		/* remove LLC header */
		ef = (ehdr *)((int)sf + sizeof(amsdu_sf) - ETHER_HDR_LEN);
		memcpy(ef->sa, sf->sa, WLAN_ADDR_LEN);
		memcpy(ef->da, sf->da, WLAN_ADDR_LEN);
//		DCACHE_FLUSH((unsigned int)ef, wb->pkt_len);

		wb->data_off = (char *)ef - (char *)wb;
		wb->wh = 0;

		//WLA_DUMP_BUF((char *)((int)wb + wb->data_off), wb->pkt_len);
		//WLA_DBG(WLADEBUG, "wb->pkt_len=%d\n", wb->pkt_len);
		if(old_wb)
		{
			old_wb->wb_next = wb;
			old_wb->flags |= WBUF_CHAIN;
		}
		old_wb = wb;
	}
	return wb1;

err:
	if (wb1)
	{
		WBUF_FREE(wb1);
	}

	return NULL;
}

inline wbuf_desc *dptr_to_wdesc(char *dptr)
{
	return (wbuf_desc *)(NONCACHED_ADDR(dptr));
}

/*!-----------------------------------------------------------------------------
 * function: wla_rxq()
 *
 *      \brief Poll received descriptor from wlan/eth rx queue.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
int wla_rxq(MAC_INFO *info, struct rx_q *rxq)
{
    volatile rx_descriptor *rx_desc;
	volatile buf_header *bhdr=NULL;
	struct wbuf *wb, *wb1;
	u16 bhdr_head_index, bhdr_tail_index;
	u8 hw_buf;
	u16 flags;
	int count = 0;

#if defined(CONFIG_CHEETAH_SMART_CONFIG) && 1
    if (__smartcfg_enable)
		return smartcfg_process(rxq);
#endif

#if 0
	/* Moniter descriptors is enough? */
	if(rxq->desc_hdr[(rxq->index - 1) % rxq->max].own == 0)
	{
		WLA_DBG(WLAWARN, "W_rxq overrun!\n");
	}
#endif
	wla_lock();
	/* handle HW RX Queue */
	for(rx_desc = &rxq->desc_hdr[rxq->index]; (rx_desc->own == 0) && (++count <= rxq->max); 
			rx_desc = &rxq->desc_hdr[rxq->index])
	{
		WLA_TIMESTAMP_TRACE(info->tput_w2e, 0);

		/* initialize temp variables per desacriptor */
		wb = wb1 = 0;
		flags = WBUF_RX_FRAME|WBUF_FIXED_SIZE;

		bhdr_head_index = rx_desc->bhdr_head_index;
		bhdr_tail_index = rx_desc->bhdr_tail_index;

		if(!rx_desc->swb)
		{
			/* check legal range */
			if((bhdr_head_index >= info->hw_buf_headers_count)||
				(bhdr_tail_index >= info->hw_buf_headers_count))
			{
				WLA_DBG(WLAWARN, "%s(%d) Wrong head idx=%x, tail idx=%x, limit=%x\n", __func__,
					__LINE__,  bhdr_head_index, bhdr_tail_index, info->hw_buf_headers_count);
#if defined(CONFIG_WMAC_RECOVER)
				wla_rx_monitor(&info->mon_wronghead_cnt, "Wrong head");
#else
				panic("Wrong head !!!");
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

		bhdr = idx_to_bhdr(info, bhdr_head_index);

		if(rx_desc->drop)
		{
#if defined(CONFIG_CHEETAH_SMART_CONFIG) && 0
			if(!rx_desc->crc)
			{
				struct wbuf *wb_ptr = dptr_to_wb((char *)(u32)bhdr->dptr);
				u8 *data = (u8 *)((int)wb_ptr + (int)get_bhdr_offset((buf_header *)bhdr));

				if(!smart_state(data, (0x1fff & rx_desc->pkt_length)))
					goto fail;
			}
#endif
			if(rx_desc->crc || rx_desc->ve || rx_desc->bsid || rx_desc->sec_err)
			{
				WLA_DBG(WLAWARN, "CRC/SEC/VE/BSSID error, %x\n", ((u32 *)rx_desc)[3]);
					//WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)bhdr->dptr) + (int)get_bhdr_offset((buf_header *)bhdr)), rx_desc->pkt_length);
				info->sw_wrx_drop_pkt_cnt++;
				goto fail;
			}
			
			if(rx_desc->bc)	
			{
				WLA_DBG(WLAWARN, "BC but DROP!!!\n");
				WLA_DBG(WLAWARN, "Drop error frame: %x, %x, %x, %x\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3]);
			}
			else if(rx_desc->qose)
			{
				WLA_DBG(WLAWARN, "QOS Mismatch!!!\n");
			}
			else if(rx_desc->sec_mismatch)
			{
				WLA_DBG(WLAWARN, "SEC Mismatch!!!\n");
#if defined (CONFIG_WLA_WAPI) || defined (CONFIG_WAPI)
				{
				/* FIXME: patch for the case that receiving un-encrypted WAI frame */
				struct wbuf *wb_ptr = dptr_to_wb((char *)(u32)bhdr->dptr);
				u8 *data = (u8 *)((int)wb_ptr + (int)get_bhdr_offset((buf_header *)bhdr));
				ehdr *ef = (ehdr *)(data + 2);
			
				/* Note: Assume that the error frame is always replaced with ethernet header */
				if((wb_ptr->wh == 0) && (ef->type == ETHERTYPE_WAPI))
				{
					if(memcmp(maddr_tables.m[rx_desc->bssid].addr, ef->da, 6) == 0)
					{
						wb_ptr->data_off += 2;
					}
				}
				}
#endif
			}
			else if(!(rx_desc->hit & (SA_HIT_ADDR|TA_HIT_DS)))
			{
			}
			else if(rx_desc->ftcpu)
			{

			}	
			else
			{
				WLA_DBG(WLAWARN, "Drop error frame: %x, %x, %x, %x\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3]);
				WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr) + (int)get_bhdr_offset((buf_header *)bhdr)), rx_desc->pkt_length);
	
			}


		}

		if(rx_desc->pkt_length < 14 || bhdr->len < 14)
		{
			WLA_DBG(WLAERROR, "wla_rx_wlq(): wrong rx_desc->pkt_length:%x, bhdr->len=%d, %08x-%08x-%08x-%08x, rxq->index=%d\n", rx_desc->pkt_length, bhdr->len, ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], rxq->index);
			goto fail;
		}
		else if((rx_desc->pkt_length > 1560) && !rx_desc->amsdu)
		{
			WLA_DBG(WLAWARN, "pkt_length is too long:%d, drop it!!!\n", rx_desc->pkt_length);
			goto fail;
		}

		if(((bhdr->offset_h << 6) | bhdr->offset ) == 0)
		{
			WLA_DBG(WLAWARN, "Zero data offset: %08x, %08x, %08x, %08x, bhdr=%p(%08x)\n", ((u32 *)rx_desc)[0], ((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], ((u32 *)rx_desc)[3], bhdr, *(u32 *)bhdr);

#if defined(CONFIG_WMAC_RECOVER)
			wla_rx_monitor(&info->mon_zerodataofs_cnt, "Zero data offset");
			goto fail;
#else
			panic("wla_rx_wlq():Zero data offset\n");
#endif
		}

		
		/* detect buffer info does not match with descriptor */ 
		{
			u16 wdesc_tail_idx = *(u16 *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr)+32);

			if(wdesc_tail_idx != bhdr_tail_index)
			{
				/* We has known sometime PS-Poll has wrong wb, drop it */
				if(rx_desc->ps_poll)
				{
					WLA_MSG("PSPOLL but wrong wdesc\n");
				}
				else
				{
				WLA_DBG(WLAERROR, "!!Drop pkt_length:%x, %08x-%08x-%08x-%08x, index=%d\n"
							,rx_desc->pkt_length, ((u32 *)rx_desc)[0], 
							((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], 
							((u32 *)rx_desc)[3], rxq->index);
				WLA_MSG("-------wdesc:\n");
				WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr)+4), 32);
				WLA_MSG("-------frame:\n");
				WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr) + 
					(int)get_bhdr_offset((buf_header *)bhdr)), 32);
				}
				goto fail;
			}
		}

		if(!rx_desc->wh && !rx_desc->hit)
		{
			WLA_DBG(WLAERROR, "!!detect NO hit but eth hdr:%x, %08x-%08x-%08x-%08x, index=%d\n"
							,rx_desc->pkt_length, ((u32 *)rx_desc)[0], 
							((u32 *)rx_desc)[1], ((u32 *)rx_desc)[2], 
							((u32 *)rx_desc)[3], rxq->index);
				WLA_MSG("-------wdesc:\n");
				WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr)+4), 32);
				WLA_MSG("-------frame:\n");
				WLA_DUMP_BUF((char *)((int)dptr_to_wb((char *)(unsigned int)bhdr->dptr) + 
					(int)get_bhdr_offset((buf_header *)bhdr)), 32);
		}

		if(rx_desc->amsdu)
		{
			//WLA_DBG(WLAWARN, "A-MSDU received, len=%d\n", rx_desc->pkt_length);
			wb = amsdu_slice(info, bhdr_head_index, bhdr_tail_index);
			if(wb == 0)
			{
				goto fail;
			}
		}

#ifdef CONFIG_SRAM_WLA
		if(!wb)
		{
			/* FIXME: need to modify copy_bhdr_to_wb() */
			wb = copy_bhdr_to_wb(info, bhdr_head_index, bhdr_tail_index);
			if(wb == 0)
			{
				goto fail;
			}
		}
#else
		if(wb)
		{
		}
		/* multiple buffers or Hardware buffers need to copy to one buffer */
		else if((bhdr_head_index != bhdr_tail_index) || (!rx_desc->swb))
		{
			/* FIXME: need to modify copy_bhdr_to_wb() */
			wb = copy_bhdr_to_wb(info, bhdr_head_index, bhdr_tail_index);
			if(wb == 0)
			{
				goto fail;
			}
		}
		/* signle buffer, just exchange buffer's address to gain performance */
		else
		{
			unsigned int *bhdr_ptr;

			if((wb1 = WBUF_ALLOC(0)) == 0)
			{
				WLA_DBG(WLAERROR, "wla_rx_wlq(): wbuf_alloc failed\n");
				goto fail;
			}

			wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
			wb->wb_next=0;
			wb->rb_next=0;
///
#if 0
			if(addr_is_invalid(wb))
			{
				WLA_MSG("bhdr=%x, idx=%d\n", bhdr, bhdr_to_idx(info, (buf_header *)bhdr));
				panic("wla_rx_wlq(): wrong rx buffer:0x%x\n", bhdr->dptr);
			}
#endif
			if(wb->flags != WBUF_FIXED_SIZE)
			{
				WLA_DBG(WLAERROR, "wla_rx_wlq(): wrong wb->flags:%x, bhr_head=%d, index=%d\n", wb->flags, bhdr_head_index, rxq->index);
				WLA_DUMP_BUF_16BIT((char *)rxq->desc_hdr, info->wl_rxq.max*16);
				WLA_MSG("~~~~~~~~wb=%p\n", wb);
				WLA_DUMP_BUF_16BIT(((char *)wb)-16, 128);
#if defined(CONFIG_WMAC_RECOVER)
				wla_rx_monitor(&info->mon_wrongflag_cnt, "wrong wb->flags");
				WBUF_FREE(wb1);
				wb->flags = WBUF_FIXED_SIZE;
				wb=0;
				goto fail;
#else
				panic("wla_rx_wlq(): wrong wb->flags !!!");
#endif
			}
			wb->data_off = wb->data_off + WLAN_TX_MIN_RESERVED_LEADIND_SPACE;

			/* caution: !!! the operation should be atomic */
			bhdr_ptr = (unsigned int *)bhdr;
			bhdr_ptr[0] = (1 << 15);
			bhdr_ptr[1] = (int)wb_to_dptr(wb1);
#if defined(WBUF_IS_CACHED)
			DCACHE_INVALIDATE((unsigned int)wb1, 1600);
#endif
		}
#endif	// CONFIG_SRAM_WLA

		//wb->bss_desc = rx_desc->bssid;  /* FIXME: enable for client mode power saving test */

		wb->flags |= flags;
		if(rx_desc->hit & DA_HIT_ADDR)
			wb->sta_idx = rx_desc->addr_index;

		rxq->pkt_count++;
fail:	/* There are valid buffer headers and release to hardware. */
		mac_free_buffer_hdr(bhdr_head_index, bhdr_tail_index, hw_buf);
/* The descriptor has invalid buffer header, just return descriptor */
fail2:
		//rx_desc->bhdr_head_index = rx_desc->bhdr_tail_index = 0;
		rx_desc->own = 1;
		rxq->index = (rxq->index + 1) % rxq->max;

		MACREG_WRITE32(CPU_RD_SWDES_IND, 1);     /* kick RX queue logic as it might be pending on get free RX descriptor to write */

		/* sent the frame/packet to the upper layer */
		if(wb)
		{
			WLA_TIMESTAMP_TRACE(info->tput_w2e, 1);

#if defined(WLA_AMPDU_RX_SW_REORDER)
			if(wb->ampdu)
			{
				reorder_buf_ctrl(info, wb, wb->sn, wb->tid);
			}
			else
#endif
			{
                if(info->wla_moniter_handler)
                    info->wla_moniter_handler(wb);
                else
				    rx_frame_dispatcher(wb);
			}
		}
	}

	wla_unlock();
#if 0
	if(count >= rxq->max)
		WLA_DBG(WLADEBUG, "count=%d\n", count);
#endif
		return count;
}

/*!-----------------------------------------------------------------------------
 * function: wla_tx_frame()
 *
 *      \brief Wlan frame's hard transmit function. Fill TX descriptor of UMAC and 
 *			   kick off transmit.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void wla_tx_frame(MAC_INFO *info, struct wbuf *wb, struct wbuf *wb_tx)
{
	buf_header *bhdr;
	tx_descriptor tx_desc, *desc, *tx_desc_p;
	unsigned int w0;

	if(!info->mac_is_ready)
	{
		WLA_DBG(WLAWARN, "%s(), mac is not ready!\n", __func__);
		WBUF_FREE(wb);
		return;
	}

#if defined(CONFIG_CHEETAH_SMART_CONFIG)
	if(__smartcfg_enable)
	{
		WBUF_FREE(wb);
		return;
	}
#endif

	/* Make sure TXQC has enough retq size for desscriptor release */
	if((info->wl_txq.pkt_count - info->wl_sw_retq.pkt_count) > info->wl_sw_retq.max)
	{
		WLA_DBG(WLAWARN, "%s(), sw retq is not enough for xmit?\n", __func__);
		WBUF_FREE(wb);
		return;
	}

	if((bhdr = bhdr_get_first(info)) == 0)
	{
		WLA_DBG(WLAWARN, "%s(), NO bhdr to send!\n", __func__);
		WBUF_FREE(wb);
		info->sw_wtx_no_bhdr++;
		return;
	}

	/* fill in bhdr */
	bhdr->next_index = 0;
	bhdr->dptr = (int)wb_to_dptr(wb_tx);
	fill_bhdr_offset(bhdr, wb->data_off);
	bhdr->ep = 1;
	bhdr->len = wb->pkt_len;
#if defined(WBUF_IS_CACHED)
	DCACHE_STORE(wb, wb->data_off);
	DCACHE_STORE(((unsigned int)wb_tx + wb->data_off), wb->pkt_len);
#endif

#ifdef WLA_DEBUG_DUMP
	if(info->w_tx_dump)
	{
		WLA_DUMP_BUF((char *)((int)wb_tx + wb->data_off), wb->pkt_len);
	}
#endif

	tx_desc_p = &tx_desc;
	memset((char *)&tx_desc, 0, sizeof(tx_descriptor));
	if(wb->flags & WBUF_TX_DIS_DURATION)
		tx_desc.dis_duration = 1; /* HW duration insertion */

	if(wb->flags & WBUF_TX_INSERT_TS)
		tx_desc.ins_ts = 1;

	tx_desc.swb = 1;
	/* so far we only support data frame with encryption */
	if(wb->flags & (WBUF_TX_NO_SECU|WBUF_MGMT_FRAME))
		tx_desc.secoff = 1;

	if(wb->bcmc)
	{
		tx_desc.bc = 1;
		info->sw_wtx_bcast_pkt_waiting++;
	}

	tx_desc.pkt_length = wb->pkt_len;

	if(wb->wh == 0)
	{
		tx_desc.df = 0;
#if defined(CONFIG_CHEETAH)
		/* Only data frame use ethernet format */
		tx_desc.llc = 1;
		tx_desc.header_length = 12;
#endif
	}
	else
	{
		tx_desc.df = 1;
		if(wb->pkt_len <= 24)
			tx_desc.header_length = wb->pkt_len;
		else if((wb->flags & WBUF_MGMT_FRAME) || !(wb->qos))
			tx_desc.header_length = 24;
		else if(wb->qos)
			tx_desc.header_length = 26;
		else
		{
			WLA_DBG(WLAERR, "Not expected TX frame:\n");
			WLA_DUMP_BUF((char *)((int)wb_tx + wb->data_off), wb->pkt_len);
		}
	}

	/* FIXME: Eric should remove bssid on TX's descriptor */
	tx_desc.bssid = wb->bss_desc;
	tx_desc.tid = wb->tid;

	tx_desc.addr_index = wb->sta_idx;


	/* fill in rate */
	if(wb->flags & (WBUF_TX_BASIC_RATE|WBUF_TX_SPEC_RATE))
	{
		if(wb->bcmc || (wb->flags & WBUF_MGMT_FRAME))
		{
			/* management and groupcast frame shall use global sequence */
			tx_desc.ins_gsn = 1; /* not assigned QoS-TID, use global sequence number */
			tx_desc.np = 1; /* NP use global sequence number */
 			tx_desc.u.tx.rate_index = info->mbss[wb->bss_desc].tx_rate[BASIC_TX_RATE].rate_captbl;
		}
		else
		{
			struct wsta_cb *wcb = info->wcb_tbls[wb->sta_idx];
			if(wcb && (wb->flags & WBUF_TX_SPEC_RATE))
			{
				wcb->basic_rc_count++;
				check_spec_rate(info, wcb);
			}
			tx_desc.ba_no_agg = 1;	/* to PSBA queue and no aggregation, not use global sequence number */
			tx_desc.chk_ps = 1;
		}
	}
	if(wb->flags & WBUF_TX_NULL_DATA)
	{
		tx_desc.dis_sn = 1; /* not assigned QoS-TID, use global sequence number, for null frame */
	}
	if(wb->flags & WBUF_TX_CHECK_PS)
	{
		/* Using rate table to send, it can force to enqueue PSBAQ, for action(management) frame */
		tx_desc.chk_ps = 1;
	}
	
	/* BC/MC need NO ACK */
	if(wb->flags & WBUF_TX_NO_ACK)
	{
		tx_desc.fack = 1;					/* force to use ack policy specified in TX descr */
		tx_desc.ack_policy = MAC_ACK_POLICY_NOACK;
	}
	else
	{
		tx_desc.fack = 1;					/* force to use ack policy specified in TX descr */
		tx_desc.ack_policy = MAC_ACK_POLICY_NORMAL_ACK;
	}

	tx_desc.bhdr_head_index = bhdr_to_idx(info, bhdr);
	bhdr = bhdr_find_tail(info, bhdr);
	tx_desc.bhdr_tail_index = bhdr_to_idx(info, bhdr);

/* for debug */
#if 0
	if((tx_desc.dis_agg == 0) && (tx_desc.np == 0))
	{
		WLA_DBG(WLAWARN, "TX agg and tid:%d, %x, %x, %x, %x\n",tx_desc.tid, ((u32 *)tx_desc_p)[0], ((u32 *)tx_desc_p)[1], ((u32 *)tx_desc_p)[2], ((u32 *)tx_desc_p)[3]);

		WLA_DUMP_BUF((char *)((int)wb_tx + wb->data_off), wb->pkt_len);
	}
#endif
	
	wla_lock();

	desc = &info->wl_txq.desc_hdr[info->wl_txq.windex];
	/* tx descriptor is not enough, drop the frame */	
	if(desc->own == 1)
	{
		int i = bhdr_to_idx(info, bhdr);
		WLA_DBG(WLAWARN, "TX desc is not enough\n");
		bhdr_insert_tail(info, i, i);
		wla_unlock();
		WBUF_FREE(wb);
		info->sw_wtx_no_desc++;
		return;
	}
	info->wl_txq.windex = (info->wl_txq.windex + 1) % info->wl_txq.max;

	w0 = ((unsigned int *)desc)[0] & 0x40000000; /* get eor bit */
	w0 |= 0x80000000; /* enable own bit */

	
	((unsigned int *)desc)[3] = ((unsigned int *)tx_desc_p)[3];
	((unsigned int *)desc)[2] = ((unsigned int *)tx_desc_p)[2];
	((unsigned int *)desc)[1] = ((unsigned int *)tx_desc_p)[1];
	((unsigned int *)desc)[0] = ((unsigned int *)tx_desc_p)[0] | w0;

#if defined(DEBUG_TXQ)
	memcpy(&debug_txq[debug_txq_index], desc, 16);
	debug_txq_index = (debug_txq_index+1)%256;
#endif
	/* trigger TX */
	MACREG_WRITE32(TXQ_TRIGGER, 1);

	info->wl_txq.pkt_count++;
	wla_txfail_chk();
	wla_unlock();
}

/*!-----------------------------------------------------------------------------
 * function: wla_tx_done()
 *
 *      \brief Recycle buffers after wlan frame transmit.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void wla_tx_done(MAC_INFO *info)
{
    volatile tx_descriptor* tx_desc;
	buf_header* bhdr;
	struct wbuf *wb;
	int free;
	struct wsta_cb *wcb;
    int count=0;
	u16 head, tail;

	wla_lock();	
	/* handle SW TX return Queue */
	while((info->wl_sw_retq.desc_hdr[info->wl_sw_retq.index].own == 0) && (++count <=  info->wl_sw_retq.max)) 
	{
		//WLA_DBG(WLADEBUG, "%s()\n", __func__);
		tx_desc = &info->wl_sw_retq.desc_hdr[info->wl_sw_retq.index];
		head = tx_desc->bhdr_head_index;
		tail = tx_desc->bhdr_tail_index;

		if((head < info->sw_tx_bhdr_start)||	
			(head >= info->sw_tx_bhdr_end))
		{
			WLA_DBG(WLAERROR, "wla_tx_done(): wrong tx frame index:, %08x-%08x-%08x-%08x, rxq->index=%d\n", ((u32 *)tx_desc)[0], ((u32 *)tx_desc)[1], ((u32 *)tx_desc)[2], ((u32 *)tx_desc)[3], info->wl_sw_retq.index);
			WLA_DUMP_BUF_16BIT((char *)info->wl_sw_retq.desc_hdr, info->wl_sw_retq.max*16);
			goto next;
		}

		if(head != tail)
		{
			WLA_DBG(WLAERROR, "wla_tx_done(): fragment packet tramsitted, we did not do that!!!\n");
			/* force tail equal to head to workaround */
			tail = head;
		}
#if defined(SW_BUF_TRACE)
		/* Bypass duplicated bhdr to free */
		if(info->sw_buf_trace[head - info->sw_rx_bhdr_start] == 0)
		{
			WLA_DBG(WLAERROR, "!!!Detected duplicated bhdr in retq!!!, head=%d\n", head);

			goto next;
		}
#endif

		info->wl_sw_retq.pkt_count++;
		if(tx_desc->bc)
			info->sw_wtx_bcast_pkt_waiting--;

		bhdr = idx_to_bhdr(info, head);
		wb = dptr_to_wb((char *)(unsigned int)bhdr->dptr);
#if 0//defined(WLAN_RC_BTS)
		if(wb->flags & WBUF_SAMPLE_FRAME)
		{
			rc_handle_sample_frame(info, wb);
		}
		else
#endif 

		wcb = info->wcb_tbls[wb->sta_idx];
		if(!wb->bcmc && ((wb->flags & (WBUF_TX_SPEC_RATE|WBUF_MGMT_FRAME))==WBUF_TX_SPEC_RATE) &&
			wcb && wcb->basic_rc_count)
		{
			wcb->basic_rc_count--;
			check_spec_rate(info, wcb);
		}

		free = 0;
		/* Give up all tasks once MAC shutdown */
		if(!info->mac_is_ready)
		{
			free = 1;
			goto fail;
		}

		if(wb->flags & WBUF_TX_REQ_STATUS)
		{
			wb->flags |= WBUF_TX_REPORT_STATUS;
			if(!tx_desc->u.tx.tx_failed)
				wb->flags |= WBUF_TX_SUCCESS;
			else
				wb->flags &= ~WBUF_TX_SUCCESS;
#ifdef CONFIG_WLAN_CLIENT_PS
			if(wb->flags & WBUF_TX_PS_NULL)
			{
				/* set the tx/rx ps state to PS_DOZE */
				wla_ps(wb->bss_desc, PS_STATE_DOZE, 1, 1);
				wla_ps(wb->bss_desc, PS_STATE_DOZE, 0, 1);
				free = 1;
			}
			else
#endif // CONFIG_WLAN_CLIENT_PS
			{
				if(info->wla_mgt_handler)
				{
					info->wla_mgt_handler(wb, NULL);
				}
				else
					free = 1;
			}
		}
		else 
			free = 1;

fail:
		if(free)
		{
			WBUF_FREE(wb);
		}
	
		bhdr_insert_tail(info, head, tail);
next:
		tx_desc->own = 1;
		info->wl_sw_retq.index = (info->wl_sw_retq.index + 1) % info->wl_sw_retq.max;
		MACREG_WRITE32(GETRECYCQ_TRIG, 1);
	}
	wla_unlock();	
}

#if defined(WLA_AMPDU_RX_SW_REORDER)
/*!-----------------------------------------------------------------------------
 * function: reorder_buf_release()
 *
 *      \brief	flush buffers before new start sequence 
 *		\param 	captbl: STA's HW capability table
 *				ba: BA session
 *				new_start: new start sequence number
 *				in_order:
 *      \return	void
 +----------------------------------------------------------------------------*/
void reorder_buf_release(struct rx_ba_session *ba, u16 new_start, u8 in_order)
{
	struct wbuf *wb;
	u32 i = 0;
//if(ba->reorder_q_head)
//	WLA_DBG(WLADEBUG, "@@@%s(%d), ba->reorder_q_head->sn=%d, new_start=%d, in_order=%d\n", __func__, __LINE__, ba->reorder_q_head->sn, new_start, in_order);
	while(ba->reorder_q_head && WLAN_SEQ_LESS(ba->reorder_q_head->sn, new_start))
	{
		if(in_order && (ba->win_start != ba->reorder_q_head->sn))
		{
//				WLA_DBG(WLADEBUG, "~~ba->win_start=%d != ba->reorder_q_head->sn=%d\n", ba->win_start, ba->reorder_q_head->sn);
				break;
		}
	
		wb = ba->reorder_q_head;
///
#if 0
if((wb->rb_next != 0) && addr_is_invalid(wb->rb_next))
#endif		
if((wb->rb_next != 0))
		panic("????reorder_buf_release(), invalid wb->rb_next\n");
		ba->reorder_q_head = wb->rb_next;
		wb->rb_next = 0;
		ba->win_start = WLAN_SEQ_INC(wb->sn);
		if(--ba->stored_num == 0)
		{
			ba->reorder_q_tail = 0;
//			WLA_DBG(WLADEBUG, "@@@flush all buffer!!!\n");
		}
		/* pass to upper layer */
		rx_frame_dispatcher(wb);
	
		i++;
	}
	//WLA_DBG(WLADEBUG, "@@@release %d frames\n", i);	

}

/*!-----------------------------------------------------------------------------
 * function: reorder_buf_ctrl()
 *
 *      \brief All AMPDUs should get in this function to reorder.
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void reorder_buf_ctrl(MAC_INFO *info, struct wbuf *wb, u16 seq, u8 tid)
{
	struct wsta_cb *cb;
	struct rx_ba_session *ba;
	struct wbuf *wb1, *wb2;
	u16 wstart;

	cb = info->wcb_tbls[wb->sa_idx];
	ba = cb->ba_rx[tid];

	if(!ba)
	{
		WLA_DBG(WLAWARN, "NO BA session!!\n");
		goto drop_it;
	}
//WLA_DBG(WLADEBUG, "@@%s(%d), AMPDU: seq=%d\n", __func__, __LINE__, seq);

	if(ba->win_start == 0xffff)
		ba->win_start = seq;

	wstart = ba->win_start;
	/* frame sequence is older than WinStart, drop it */
	if(WLAN_SEQ_LESS(seq, wstart))
	{
		/* drop frame */
		WLA_DBG(WLAWARN, "??????Out of date with sequence=%d, wstart=%d\n", seq, wstart);
		goto drop_it;
	}
	/* frame sequence exceeds WinEnd, shift window and release older frames */
	else if(!WLAN_SEQ_LESS(seq, wstart + ba->win_size))
	{
		wstart = WLAN_SEQ_INC(WLAN_SEQ_SUB(seq, ba->win_size));
		
		WLA_DBG(WLAWARN, "@@@!!!!!!!!!outside of window, new wstart=%d\n", wstart);
		reorder_buf_release(ba, wstart, 0);
	}

	/* frame sequence is in the window */
	/* if the frame is we expected sequence number, release it immediately */	
	if(seq == ba->win_start)
	{
		//WLA_DBG(WLADEBUG, "@@@expected sn\n");
		ba->win_start = WLAN_SEQ_INC(seq);
		/* pass to upper layer */
		rx_frame_dispatcher(wb);
	}
	else
	{
		if(ba->reorder_q_tail == 0)
		{
			/* reorder buffer has nothing */
			//WLA_DBG(WLADEBUG, "@@@push into empty queue\n");
			ba->reorder_q_head = ba->reorder_q_tail = wb;
		}
		else if(WLAN_SEQ_LESS(seq, WLAN_SEQ_INC(ba->reorder_q_tail->sn)))
		{
			wb2 = 0;	
			for(wb1 = ba->reorder_q_head; wb1; wb1 = wb1->rb_next)
			{
				/* check duplicated frame */
				if(wb1->sn == seq)
				{
					WLA_DBG(WLADEBUG, "duplicated frame: %d\n", seq);
					goto drop_it;
				}
				else if(WLAN_SEQ_LESS(seq, wb1->sn))
				{
					if(wb2)
						wb2->rb_next = wb;
					else
						ba->reorder_q_head = wb;
					wb->rb_next = wb1;
					
					//WLA_DBG(WLADEBUG, "@@@******push in before wb1->sn=%d\n", wb1->sn);
					break; 
				}
				wb2 = wb1;
			}
		}
		else
		{
			//WLA_DBG(WLADEBUG, "@@@push in tail, tail sn=%d\n", ba->reorder_q_tail->sn);
			ba->reorder_q_tail->rb_next = wb;
			ba->reorder_q_tail = wb;
		}

		wb->sn = seq;
		ba->stored_num++;
	}
	
	if(ba->reorder_q_head && (ba->win_start == ba->reorder_q_head->sn))
	{
		/* release buffer in order of increasing number */
		reorder_buf_release(ba, WLAN_SEQ_INC(ba->reorder_q_tail->sn), 1);
	}

	return;

drop_it:
	WBUF_FREE(wb);
	return; 
}
#endif

/*!-----------------------------------------------------------------------------
 * function: wla_tx_frame_handler()
 *
 *      \brief pre-check tx condition including power saving/offchannel scan
 *		\param 
 *      \return	void
 +----------------------------------------------------------------------------*/
void wla_tx_frame_handler(MAC_INFO *info, struct wbuf *wb, struct wbuf *wb_tx)
{
#ifdef CONFIG_WLAN_CLIENT_PS
	struct mbss_t *mbss = &info->mbss[wb->bss_desc];
	
	if((mbss->ps_tx_state > PS_STATE_AWAKE) && (mbss->role == WIF_STA_ROLE))
	{
		u8 set_pm=1;

		if(mbss->ps_tx_state == PS_STATE_SLEEP)
		{
			wla_ps(wb->bss_desc, PS_STATE_DOZE, 1, 1);
			wla_ps(wb->bss_desc, PS_STATE_DOZE, 0, 1);
			wla_busy_timestamp(wb->bss_desc, os_current_time(), 1);
		}
		else //if(mbss->ps_tx_state == PS_DOZE)
		{
			/* FIXME: may any more efficient wakeup algorithm & 
			   		  avoid wakeup too early (cause test case failed) 	*/
			if(!(wb->wh))	// only consider data path flow
			{
				u32 now = os_current_time();
				u32 pre = wla_busy_timestamp(wb->bss_desc, 0, 0);

				if((now - pre) < 5)	// 5 = 50 ms
				{
					wla_ps(wb->bss_desc, PS_STATE_AWAKE, 1, 1);
					wla_ps(wb->bss_desc, PS_STATE_AWAKE, 0, 1);
					set_pm = 0;
				}
				else
				{
					wla_busy_timestamp(wb->bss_desc, now, 1);
				}
			}
		}
#if 1
		if((set_pm && wb->wh) && (wb == wb_tx))
		{
			struct wlan_hdr *fm = (struct wlan_hdr *)((u32)wb + wb->data_off);
			fm->pwr_mgt = 1;
			FLUSH_TX((char *)wb, wb->data_off+wb->pkt_len);
		}
#endif
	}
#endif // CONFIG_WLAN_CLIENT_PS

	/* FIXME: queue wb here & return for offchannel ps */

	wla_tx_frame(info, wb, wb_tx);
}
