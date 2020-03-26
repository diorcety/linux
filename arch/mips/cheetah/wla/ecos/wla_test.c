/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file arthur_test.c
*   \brief  test arthur wlan driver.
*   \author Montage
*/

#include <os_compat.h>
#include <stdio.h>
#include <stdlib.h>
#include <wlan_def.h>
#include <wbuf.h>

#include <mac_ctrl.h>
#include <wla_api.h>
#include <wla_debug.h>
#include <cli_api.h>
#include <os_api.h>
#include <lib/lib_ifcfg.h>
#include <common.h>
#include <cdb_api.h>

typedef struct {
	u32 start;
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
	u32 rx_sleep_time;

	u8 my_addr[6];
	u8 channel;
	u8 secondary_channel;
	u8 bss_desc;
	u8 tx_qos; 
	u8 tx_rate;
	u8 txpower;
	
	struct wsta_cb *tx_wcb;
} wla_test_cfg;;

/* default value */
wla_test_cfg acfg = {
	.start = 0,
	.tx_repeat = 1,
	.tx_burst_num = 10,
	.tx_sleep_time = 1,
	.tx_len = 256,
	.tx_addr = {0x00, 0x11, 0x11, 0x11, 0x11, 0x11},
	.tx_short_preamble = 0,
	.tx_green_field = 0,
	.tx_sgi = 0,

	.rx_dump = 0,
	.rx_drop = 0,
	.rx_sleep_time = 100,

	.channel = 1,
	.secondary_channel = 0,
	.tx_qos = 1,
	.tx_rate = 5,
	.txpower = 0,
};

wla_test_cfg *acfg_p = &acfg;

int wla_test(void *data)
{
	struct wbuf *wb;
	int iteration, rate_flags;
	MAC_INFO *info = WLA_MAC_INFO;

	wla_cfg(SET_RX_FILTER, RX_FILTER_MGMT_FRAME|RX_FILTER_CTRL_FRAME|RX_FILTER_DATA_FRAME);
	wla_cfg(SET_RX_FILTER, RX_FILTER_BEACON|RX_FILTER_PROBE_RESP);

	ifcfg_up("wlan0");
	//wla_register_mgt_handler(0); /* force to unregister mgt handler */
	info->wla_mgt_handler = 0;
	info->wla_eapol_handler = 0;

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
	wla_tx_power(acfg_p->txpower);

	//wla_cfg(SET_BEACON_PARAMS, 100, 3);

	/* fragment threshold */
	wla_cfg(SET_FRAG_THRESHOLD, 2346 - 24);

	/* RTS threshold */
	wla_cfg(SET_RTS_THRESHOLD, 2346 - 24);

	wla_set_protection(NO_PROTECTION);
	wla_set_ap_isolated(0);
	wla_set_bss_isolated(0);

	cdb_get_mac(0, acfg_p->my_addr);
	acfg_p->bss_desc = register_macaddr(AP_MACADDR, acfg_p->my_addr);
	wla_add_wif(acfg_p->bss_desc, WIF_AP_ROLE, 0);

	wla_cfg(SET_TX_RATE, FIXED_TX_RATE, acfg_p->bss_desc, rate_flags, acfg_p->tx_rate);
	acfg_p->tx_wcb->qos = acfg_p->tx_qos;
	wla_get_sta(acfg_p->tx_addr, acfg_p->bss_desc, acfg_p->tx_wcb, 0);

	acfg_p->start = 1;
	iteration = 0;
	while((acfg_p->tx_repeat == -1)||(iteration < acfg_p->tx_repeat))
	{
		if(acfg_p->start == 0)
			break;

		if((iteration % acfg_p->tx_burst_num) == 0)
		{
			os_thread_sleep(acfg_p->tx_sleep_time);
		}

		iteration++;

		if (!(wb=wbuf_alloc(0)))
		{
			WLA_DBG(WLADEBUG, "%s: no buf for send\n",__func__);
			return 0;
		}

		wb->flags |= WBUF_TX_NO_ACK;
		wb->wh = 0;

		wb->sta_idx = acfg_p->tx_wcb->captbl_idx;

		wb->data_off = sizeof(struct wbuf);
		wb->bss_desc = 0;
		wb->pkt_len = acfg_p->tx_len;
		wb->tid = 0;

		memcpy((char *)wb+wb->data_off, acfg_p->tx_addr, 6);
		memcpy((char *)wb+wb->data_off+6, acfg_p->my_addr, 6);


		wla_tx_frame_handler(info, wb, wb);

	}

	while(acfg_p->rx_drop || acfg_p->rx_dump)
	{
		if(acfg_p->start == 0)
			break;

		os_thread_sleep(acfg_p->rx_sleep_time);
	}

	acfg_p->rx_dump = 0;
	acfg_p->rx_drop = 0;
	acfg_p->start = 0;

	wla_release_sta(acfg_p->tx_wcb);

	ifcfg_down("wlan0");
	clean_all_macaddr();
	printf("wla_test is Done\n");
	exit(0);	
	return 0;
}

int wla_test_cmd(int argc, char* argv[])
{

	if(2 > argc)
		return CLI_SHOW_USAGE;


	if(!strcmp("stop", argv[1]))
	{
		acfg_p->start = 0;
	}
	else if(!strcmp("start", argv[1]))
	{
		if(acfg_p->start)
		{
			printf("The program has exceuted!\n");
			return CLI_OK;
		}
		
		os_thread_create((OS_FUNCPTR)wla_test, NULL, "wla_test", 7, 8192);
	}
	else if(!strcmp("cfg", argv[1]))
	{
		printf("start:%d\n", acfg_p->start);
		printf("channel:%d, secondary_channel=%d\n", acfg_p->channel, acfg_p->secondary_channel);
		printf("-- RX --\n   drop:%d\n", acfg_p->rx_drop);
		printf("-- TX --\n   repeat: %d, burst: %d, sleep time: %dms, length: %dBytes, rate=%d\n", acfg_p->tx_repeat, acfg_p->tx_burst_num, acfg_p->tx_sleep_time*10, acfg_p->tx_len, acfg_p->tx_rate);
		printf("   tx_sgi=%d, tx_green_field=%d, tx_short_preamble=%d\n", acfg_p->tx_sgi, acfg_p->tx_green_field, acfg_p->tx_short_preamble); 
	}
	else if(!strcmp("chan", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->channel = atoi(argv[2]);
	}
	else if(!strcmp("txrate", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->tx_rate = atoi(argv[2]);
	}
	else if(!strcmp("tx_sgi", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->tx_sgi = atoi(argv[2]);
	}
	else if(!strcmp("tx_gf", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->tx_green_field = atoi(argv[2]);
	}
	else if(!strcmp("tx_sp", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->tx_short_preamble = atoi(argv[2]);
	}
	else if(!strcmp("2chan", argv[1]))
	{
		if(argc != 3)
			return CLI_SHOW_USAGE;

		acfg_p->secondary_channel = atoi(argv[2]);
	}
	else if(!strcmp("tx", argv[1]))
	{
		if(argc < 3)
			return CLI_SHOW_USAGE;

		if(!strcmp("repeat", argv[2]))
			acfg_p->tx_repeat = 0xffffffff;
		else
			acfg_p->tx_repeat = atoi(argv[2]);

		if(argc >= 4)
		{
			acfg_p->tx_len = atoi(argv[3]);
		}

		if(argc >= 5)
		{
			acfg_p->tx_burst_num = atoi(argv[4]);
		}
	}
	else if(!strcmp("rx", argv[1]))
	{
		acfg_p->rx_drop = 1;
	}
	else
		return CLI_SHOW_USAGE;

	return CLI_OK;
}

CLI_CMD(wt, wla_test_cmd, "wt start (start the program)\n"
							 "   stop (force to stop)\n"
							 "   cfg (show all configuration)\n"
							 "   chan <channel_num>\n"
							 "   2chan <second channel> (0:None, 1:Above, 3:Below)\n"
							 "   txrate <1~4:CCK, 5~12:OFDM, 13~20:MCS> (fixed TX rate)\n"
							 "   tx_gf <0/1> (Green Field)\n"
							 "   tx_sp <0/1> (Short Preamble)\n"
							 "   tx_sgi <0/1> (Short Guard Interval)\n"
							 "   rx drop\n"
							 "   tx <count>/repeat <payload_length> <burst>"
							, 0);

