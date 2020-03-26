/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file cta_switch.c
*   \brief  Cheetah switch APIs
*   \author Montage
*/

#ifdef CONFIG_CHEETAH_ETH
/*=============================================================================+
| Included Files
+=============================================================================*/
#ifdef __ECOS
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <cyg/infra/diag.h>
#include <cta_switch.h>
#include <cli_api.h>
#include <cta_eth.h>
#include <gpio_api.h>
#include <delay.h>
#include <os_api.h>
#else
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/mach-cheetah/cta_eth.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/common.h>
#include <asm/mach-cheetah/cta_switch.h>
#include <asm/mach-cheetah/idb.h>
#endif

/*=============================================================================+
| Define
+=============================================================================*/
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define CLA					aligned(32)
#if (defined(CONFIG_SRAM_SIZE) && (CONFIG_SRAM_SIZE >= 128))
#define IN_SRAM				section(".sram")
#else
#define IN_SRAM
#endif
char switch_buf[SWITCH_BUF_SIZE*SWITCH_BUF_HEADER_COUNT] __attribute__((CLA, IN_SRAM));
char switch_buf_headers[sizeof(buf_header)*SWITCH_BUF_HEADER_COUNT] __attribute__((CLA, IN_SRAM));

#ifndef CONFIG_FPGA
/* SYS: 60.95, 64, 80, 120, 150.59, 160 MHz */
static unsigned int bus_div[] = { 0x7fc, 0x7f0, 0x7c0, 0x780, 0x688, 0x3c0 };
static unsigned int bus_tbl[] = { 60950000, 64000000, 80000000, 120000000, 150590000, 160000000};
#endif
/*=============================================================================+
| Variables
+=============================================================================*/
							/*
							   local: bit[0]=1, ephy 802.3 protocol, harmless to ip101g
									  bit[9]=0, ephy not support 100base-T4, harmless to ip101g
									  bit[14]=0, an capability, harmless to ip101g and ephy
							 */
							/*
							   Min_ifg_p2: The value should set 22 due to its own 2 cycle latency,
							               shrink this value to balace throughput when you insert VLAN.
											Min_ifg_p2 = 96 bit times/period of mii clock
							 */
#ifdef CONFIG_FPGA
static u16 min_ifg_p2[4] = { 10, 19, 10, 10 };
#else
static u16 min_ifg_p2[4] = { 10, 10, 10, 10 };
#endif
static SWITCH_INFO switch_info =
{
	func:0x98,
	tclk:0x0,
	rclk:0x0,
	boost:0x1,
	local:{ [0]=0x45e1, [1]=0x45e1 },
	mon_hit:0,
	mon_cnt:0,
	mon_bufempty_hit:0,
	mon_bufempty_cnt:0,
	mon_p1rxzombie_hit:0,
	mon_p1rxzombie_cnt:0,
	mon_p2rxzombie_hit:0,
	mon_p2rxzombie_cnt:0,
	mon_p2bufempty_hit:0,
	mon_p2bufempty_cnt:0,
	mon_crc:0,
	mon_phy_p20x17:0,
	mon_phy_p20x15:0,
	mon_phy_p40x14:0,
	mon_phy_anerr:0
};
static SWITCH_INFO *info = &switch_info;
											/* Tdma wait cycle = (System/mii clock frequency) * 8 */
static u16 tdma_wait_p01[][2] = {	{20, 196},			/* ASIC: 60.95M */
									{21, 205},			/* ASIC: 64M */
									{26, 256},			/* ASIC: 80M */
									{39, 384},			/* ASIC: 120M */
									{49, 482},			/* ASIC: 150M */
									{52, 512},			/* ASIC: 160M */
									{20, 200} };		/* FPGA: 62.5M */
static u16 tdma_wait_p2[][4] = {	{20, 6, 20, 10},	/* ASIC: 60.95M */
									{21, 7, 21, 10},	/* ASIC: 64M */
									{26, 8, 26, 13},	/* ASIC: 80M */
									{39, 12, 39, 20},	/* ASIC: 120M */
									{49, 15, 49, 25},	/* ASIC: 150.59M */
									{52, 17, 52, 26},	/* ASIC: 160M */
									{20, 16, 20, 10} };	/* FPGA: 62.5M */
extern int link_status[2];

/*=============================================================================+
| Function Prototypes
+=============================================================================*/

/*=============================================================================+
| Functions
+=============================================================================*/
#ifdef __ECOS
extern int printf(const char *fmt, ...);
extern int an_cmd(short phy, u16 val, u16 cap);
#else
void idb_dump_buf(u32 *p, u32 s);
#define	printd(fmt, args...)					printk(fmt, ##args)
#define	printf(fmt, args...)					printk(fmt, ##args)
#define	diag_dump_buf_16bit(p, len)	idb_dump_buf((u32 *)p, (u32)len)
#endif

/*!-----------------------------------------------------------------------------
 *        cta_mac_reset_sw_port:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_mac_reset_sw_port(u8 idx)
{
	u32 val, adapter;

	if(idx==2)
	{
		val = HW_RESET_HNAT;
		adapter = 0;
	}
	else
	{
		val = ((idx)?HW_RESET_SW_P1:HW_RESET_SW_P0);
		adapter = ((idx)?HW_RESET_RMII_P1:HW_RESET_RMII_P0);
	}

	/* After RMII2MII adapter reset, also need to reset MII */
	if(adapter)
	{
		MACREG_UPDATE32(HW_RESET, 0, adapter);
		MACREG_UPDATE32(HW_RESET, adapter, adapter);
	}
	MACREG_UPDATE32(HW_RESET, 0, val);
	MACREG_UPDATE32(HW_RESET, val, val);
}

/*!-----------------------------------------------------------------------------
 *        cta_mac_reset_sw_core:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_mac_reset_sw_core(void)
{
	u8 i;

	/* Do not change this reset sequence */
	/* Provide clk to HNAT and SWITCH module */
	MACREG_UPDATE32(HW_RESET, 0, HW_RESET_HNAT_PAUSE|HW_RESET_SWITCH_PAUSE);
	/* Port Reset */
	for(i=0; i<2; i++)
	{
		cta_mac_reset_sw_port(i);
	}
	/* Core Reset */
	MACREG_UPDATE32(HW_RESET, 0, HW_RESET_SWITCH);
	udelay(1000);
	MACREG_UPDATE32(HW_RESET, HW_RESET_SWITCH, HW_RESET_SWITCH);
}
/*!-----------------------------------------------------------------------------
 *        cta_switch_port_attach:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_port_attach(u8 idx, u8 vlan, u8 calibr)
{
	u32 rofs, val, mii;
	u8 hnat;

	if(idx == PORT_WLAN_IDX)
		goto p_enable;

	hnat = (idx==PORT_HNAT_IDX)?1 : 0;
	rofs = idx * 0x100;

	/* letency = (system clock / MII clock) * 8 */
	val = MAX_FORWARD_LEN<<16;
	if(hnat)
	{
		mii = info->boost;
		val |= (tdma_wait_p2[info->sysclk][mii]-1);
	}
	else
	{
		mii = 0;
		val |= (tdma_wait_p01[info->sysclk][mii]-1);
	}
	SWREG_WRITE32(rofs|EMAC_TXRX_MISC, val);
	SWREG_WRITE32(rofs|EMAC_RX_CFG, RX_USE_LU_VLAN|RM_VLAN|TIDMAP_TOS);

	val = (PREAMBLE_BYTE_SIZE<<12)|USE_DESC_VID|INS_VLAN;
	val |= (info->func & CAP_TXRETRY)?AUTO_RETRY : 0;
	if(hnat)
	{
		/* Shrink interframe gap to balance flow on RX and TX side if TX insert VLAN tag */
		val |= (min_ifg_p2[info->boost]<<16);
		val |= PAYLOAD_FLOWCTRL;
	}
	else
	{
		val |= (INTERFRAME_GAP<<16);
		if(!calibr)
		{
			val |= (info->status[idx] & PHY_FDX)?0 : COLIISION_DROP16;
		}
	}
#ifndef CONFIG_FPGA
	val |= TX_DEBUG;
#endif
	SWREG_WRITE32(rofs|EMAC_TX_CFG, val);

	/* Pause frame source address */
	if(hnat==0)
	{
		u8 *mac = &info->addr[idx][0];
		SWREG_WRITE32(rofs|EMAC_PAUSE_MACADDR_H, ((mac[0] << 8)&0xff00) | (mac[1]&0xff));
		SWREG_WRITE32(rofs|EMAC_PAUSE_MACADDR_L, (mac[2] << 24) | ((mac[3] << 16)&0xff0000) | ((mac[4] << 8)&0xff00) | (mac[5]&0xff));
	}

	/* initialize TID mapping table per port*/
	SWREG_WRITE32(rofs|EMAC_TID_TRANS_TBL_4_7,
							((0xe0 << 24) | (0xc0 << 16) | (0xa0 << 8) | (0x80 << 0)));
	SWREG_WRITE32(rofs|EMAC_TID_TRANS_TBL_0_3,
							((0x60 << 24) | (0x40 << 16) | (0x20 << 8) | (0x0 << 0)));

	val = ((IP_OFFSET>>2)<<12)|TXEN|RXEN;
	if(hnat)
	{
		/* Select P2 MII clk 0:25M, 1:1/2H, 2:25M, 3:50M */
		MACREG_UPDATE32(RMII_MII, 0, P2_PHYMODE_CLK);
		MACREG_UPDATE32(RMII_MII, (info->boost<<4), (info->boost<<4));
	}
	else
	{
		/* Support pause frame depends on advertisement */
		val |= (info->local[idx] & PHY_CAP_PAUSE)?ETH_FLOWCTRL : 0;
		if(calibr)
			val |= BITRATE_100M;
		else
		{
			val |= (info->status[idx] & PHY_100M)?BITRATE_100M : BITRATE_10M;
			val |= (info->status[idx] & PHY_FDX)?0 : HALF_DUPLEX;
		}
#ifdef CONFIG_FPGA
		val |= (info->tclk & (1 << idx))?TXCLK_INV : 0;
		val |= (info->rclk & (1 << idx))?RXCLK_INV : 0;
#else
		/* FIXME: on board external phy must invert tx/rx clock ??? */
		if((info->phyid[idx]==PHYID_ICPLUS_IP101G) && (idx == 1))
			val |= (TXCLK_INV|RXCLK_INV);
		else if(info->phyid[idx]==PHYID_MONT_EPHY)
			val |= RXCLK_INV;
#endif

		/* New CRS scheme */
		/*	bit[29:24]:	upper_bound
			bit[21:16]:	100M CRS initial value
			bit[13:8]:	10M CRS initial value
			bit[7:0]:	txen initial value */
		val |= CRS_SCHEME;
		if(info->phyid[idx]==PHYID_MONT_EPHY)
		{
#ifdef CONFIG_FPGA
			SWREG_WRITE32(rofs|EMAC_IFG, 0x19060104);
#else
			/* FIXME: txen=920ns */
			SWREG_WRITE32(rofs|EMAC_IFG, 0x18060104);
#endif
		}
		else
		{
#ifdef CONFIG_FPGA
			SWREG_WRITE32(rofs|EMAC_IFG, 0x16030101);
#else
			/* FIXME: txen=920ns */
			SWREG_WRITE32(rofs|EMAC_IFG, 0x16030102);
#endif
		}
	}
	SWREG_WRITE32(rofs|EMAC_CFG, val);
p_enable:
	val = (1 << idx);
	val = (val << 16)|((vlan << idx)<< 8)|val;
	SWREG_UPDATE32(SW_PORT_CFG, val, val);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_port_detach:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_port_detach(u8 idx)
{
	u32 rofs, val;

	/* clear link status */
	link_status[idx] = 0;
	/* disable port map to trigger release TX queue */
	val = (1 << idx);
	SWREG_UPDATE32(SW_PORT_CFG, 0, (val << 16)|(val << 8)|val);
	do
	{
		val = SWREG_READ32(SW_FC_COUNT0 + (idx * 4));
	} while(val);
	rofs = idx * 0x100;
	/* Double check TXDMA state machine fall to zero */
	do
	{
		val = SWREG_READ32(rofs|EMAC_FSM) & (TX_FSM|TXDMA_FSM);
	} while(val);
	/* soft reset MAC to avoid MAC fall in zombie mode */
	cta_mac_reset_sw_port(idx);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_port_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_port_ctrl(u8 idx, int link)
{
	if(!(info->func & CAP_ACTIVE))
		return 1;

	/* update hnat port capability here */
	if(info->bitmap & PORT_HNAT)
		cta_switch_port_attach(PORT_HNAT_IDX, 1, 0);

	if(link)
	{
		if(!(info->bitmap & (1<<idx)))
			return 1;
		info->partner[idx] = cta_eth_mdio_rd(idx, 5);
		info->status[idx] = phy_status(idx);
		cta_switch_port_attach(idx, 0, 0);
	}
	else
	{
		if(!(info->bitmap & (1<<idx)))
			return 1;
		cta_switch_port_detach(idx);
		info->partner[idx] = cta_eth_mdio_rd(idx, 5);
		info->status[idx] = phy_status(idx);
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_txrx_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_txrx_ctrl(u8 idx, u8 tclk, u8 rclk)
{
	if(tclk)
		info->tclk |= (1 << idx);
	else
		info->tclk &= ~(1 << idx);

	if(rclk)
		info->rclk |= (1 << idx);
	else
		info->rclk &= ~(1 << idx);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_sclk_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_sclk_ctrl(void)
{
	u32 sys, clk, i;

	/* Config timer for 1ms period, clk cycle = 1ms/(1/system clock) */
#ifdef CONFIG_FPGA
	sys = i = 0;
	info->sysclk = 6;
	clk = 0xf423;
#else
	/* default */
	info->sysclk = 4;
	clk = bus_tbl[info->sysclk]/1000;

	sys = ANAREG(CLKDIV) & 0xfff;
	for(i=0; i < sizeof(bus_div)/sizeof(bus_div[0]); i++)
	{
		if(sys == bus_div[i])
		{
			info->sysclk = i;
			clk = bus_tbl[i]/1000;
		}
	}
#endif
	SWREG_WRITE32(SW_TIMER, clk);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_mac_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_mac_ctrl(int idx, u8 *mac)
{
	memcpy(&info->addr[idx], mac, 6);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_8021q_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_8021q_ctrl(u32 val)
{
	int vid, dot1q, port, ofs;

	/* Set vlan table of switch, active port's bitmap */
	vid = (val & MASK_8021Q_VID) >> 16;
	dot1q = (val & MASK_8021Q_IDX) >> 8;
	port = (val & MASK_PORT_BMAP);

	if(port & PORT_HNAT)
		info->bitmap |= PORT_HNAT;
	if(port & PORT_WLAN)
		info->bitmap |= PORT_WLAN;
	if(port & PORT_ETH1)
		info->bitmap |= PORT_ETH1;
	if(port & PORT_ETH0)
		info->bitmap |= PORT_ETH0;

	ofs = dot1q<<2;
	SWREG_WRITE32(SW_VLAN_GROUP0+ofs, (port<<16)|vid);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_ethernet_forward_to_wifi:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_ethernet_forward_to_wifi(u32 permit)
{
	u32 i, ofs, rofs;

	if(permit)
	{
		/* Reinit switch core */
		cta_mac_reset_sw_core();
		cta_switch_buf_init();
		cta_switch_reg_init();

		/* Restore core setting */
		for(i=0; i<20; i++)
		{
			ofs = i<<2;
			SWREG_WRITE32(SW_PORT_CFG+ofs, info->reg_core[i]);
		}
		/* Reinit P0 and P1 */
		for(i=0; i<2; i++)
		{
			if(!(info->bitmap & (1<<i)))
				continue;
			cta_switch_port_attach(i, 0, 0);
		}
		/* Enable P2 RX */
		SWREG_UPDATE32(0x200|EMAC_CFG, RXEN, RXEN);
	}
	else
	{
		/* Save core setting */
		for(i=0; i<20; i++)
		{
			ofs = i<<2;
			info->reg_core[i] = SWREG_READ32(SW_PORT_CFG+ofs);
		}
		/* Disable P0, P1 and P2 RX */
		for(i=0; i<3; i++)
		{
			rofs = i * 0x100;
			SWREG_UPDATE32(rofs|EMAC_CFG, 0, RXEN);
		}
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_bssid_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_bssid_ctrl(u32 val)
{
	int vid, bssid, sll, ofs, msk;

	/* Set bssid table mapping to vlan id */
	vid = (val & MASK_8021Q_VID) >> 16;
	bssid = (val & MASK_BSSID_IDX) >> 8;

	sll = (bssid%2)?16:0; 									/* shift or not */
	msk = (bssid%2)?0xffff0000:0xffff; 						/* mask MSB or LSB */
	ofs = (bssid/2)<<2;										/* offset */

	SWREG_UPDATE32(SW_BSSID2VLAN0_1+ofs, vid<<sll, msk);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_pvid_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_pvid_ctrl(u32 val)
{
	int vid, pvid, sll, ofs, msk;

	/* Set port's vlan id */
	vid = (val & MASK_8021Q_VID) >> 16;
	pvid = (val & MASK_PVID_IDX) >> 8;

	sll = (pvid%2)?16:0; 									/* shift or not */
	msk = (pvid%2)?0xffff0000:0xffff; 						/* mask MSB or LSB */
	ofs = (pvid/2)<<2;										/* offset */

	SWREG_UPDATE32(SW_PORT_VID0_1+ofs, vid<<sll, msk);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_state_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_state_ctrl(u32 val)
{
	/* Start or stop switch */
	if(val)
	{
		info->func |= CAP_ACTIVE;
		if(info->bitmap & PORT_HNAT)
			cta_switch_port_attach(PORT_HNAT_IDX, 1, 0);
		if(info->bitmap & PORT_WLAN)
			cta_switch_port_attach(PORT_WLAN_IDX, 0, 0);
	}
	else
	{
		info->func &= ~CAP_ACTIVE;
		/* make sure MAC is disable */
		if(info->bitmap & PORT_ETH1)
			cta_switch_port_detach(PORT_ETH1_IDX);
		if(info->bitmap & PORT_ETH0)
			cta_switch_port_detach(PORT_ETH0_IDX);
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_brg_write_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_brg_write_state(u8 val)
{
	if(val)
		info->func |= CAP_BRIDGE_STATE;
	else
		info->func &= ~CAP_BRIDGE_STATE;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_brg_read_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_brg_read_state(void)
{
	return ((info->func & CAP_BRIDGE_STATE)?1:0);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_brg_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_brg_state(void)
{
	return ((info->func & CAP_BRIDGE)?1:0);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_pp_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_pp_state(void)
{
	return ((info->func & CAP_POLLPHY)?1:0);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_work_mode:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_work_mode(int mode)
{
	if((mode < MODE_ETH_DBG) || (mode > MODE_SW_MAX))
	{
		//printd("%s(%d) bad working mode(%d)\n", __func__, __LINE__, mode);
		return 1;
	}
	info->mode = mode;
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_port_idx:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_switch_port_idx(int unit)
{
	int phy;

	switch(info->mode)
	{
	case MODE_ETH_DBG:
		phy = (unit)?PORT_ETH0_IDX : PORT_ETH1_IDX;
		break;
	case MODE_AP:
	case MODE_WIFI_RT:
	case MODE_WISP:
	case MODE_REPEATER:
	case MODE_BRIDGE:
	case MODE_SMART:
	case MODE_3G:
#ifdef CONFIG_WLA_P2P
	case MODE_P2P:
#endif
		phy = PORT_ETH1_IDX;
		break;
	default:
		/* MODE_PURE_AP shouldn't bring up ethernet */
		printd("%s(%d) bad working mode(%d)\n", __func__, __LINE__, info->mode);
		return -1;
	}
	return phy;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_phyid:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u32 cta_switch_phyid(int idx)
{
	if((idx<0) || (idx>=PORT_HNAT_IDX))
		return 0;
	return info->phyid[idx];
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_phybcap:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u16 cta_switch_phybcap(int idx)
{
	return (info->local[idx] & (PHY_CAP_PAUSE|
								PHY_CAP_100F|PHY_CAP_100H|
								PHY_CAP_10F|PHY_CAP_10H));
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_phyctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u16 cta_switch_phyctrl(int idx)
{
	u16 cap, ctl=0;

	cap = info->local[idx];
	if(cap & PHY_CAP_AN)
		ctl |= PHYR_CTL_AN;
	if(cap & (PHY_CAP_100F|PHY_CAP_10F))
		ctl |= PHYR_CTL_DUPLEX;
	if(cap & (PHY_CAP_100F|PHY_CAP_100H))
		ctl |= PHYR_CTL_SPEED;
	return ctl;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
s8 cta_switch_init(int advert)
{
	u16 phy;
	u32 val=0;

	info->bitmap = 0;
	/* Clear P0/P1 clock mode */
	MACREG_UPDATE32(RMII_MII
						, 0
						, P1_MII_MODE|P1_RMII_OUT_REFCLK|P1_PHYMODE_CLK|P1_ADAPTER_MODE|
						  P0_MII_MODE|P0_RMII_OUT_REFCLK|P0_PHYMODE_CLK|P0_ADAPTER_MODE);

	/* Config P0 */
#if defined(CONFIG_P0_RMII)
	cta_switch_led_init(0);
    GPREG(PINMUX) |= EN_ETH0_FNC;		//Turn on P0
#endif
#ifndef CONFIG_P0_EXT_PHY
    GPREG(PINMUX) &= ~EN_ETH0_FNC;		//Turn off P0
#endif
#if defined(CONFIG_P0_RMII_CLKOUT)
	val = P0_RMII_OUT_REFCLK|P0_PHYMODE_RMII|P0_ADAPTER_RMII;
#elif defined(CONFIG_P0_RMII_CLKIN)
	val = P0_PHYMODE_RMII|P0_ADAPTER_RMII;
#elif defined(CONFIG_P0_MII_PHY)
	val = P0_MII_PHY|P0_RMII_OUT_REFCLK|P0_PHYMODE_MII_100M|P0_ADAPTER_MII;
#elif defined(CONFIG_P0_MII_MAC)
	val = P0_MII_MAC|P0_ADAPTER_MII;
#endif
	MACREG_UPDATE32(RMII_MII, val, val);

	/* Config P1 */
#if defined(CONFIG_P1_RMII)
	cta_switch_led_init(1);
    GPREG(PINMUX) |= EN_ETH1_FNC;		//Turn on P1
#endif
#ifndef CONFIG_P1_EXT_PHY
    GPREG(PINMUX) &= ~EN_ETH1_FNC;		//Turn on P1 as internal phy
#endif
#if defined(CONFIG_P1_RMII_CLKOUT)
	val = P1_RMII_OUT_REFCLK|P1_PHYMODE_RMII|P1_ADAPTER_RMII;
#elif defined(CONFIG_P1_RMII_CLKIN)
	val = P1_PHYMODE_RMII|P1_ADAPTER_RMII;
#elif defined(CONFIG_P1_MII_PHY)
	val = P1_MII_PHY|P1_RMII_OUT_REFCLK|P1_PHYMODE_MII_100M|P1_ADAPTER_MII;
#elif defined(CONFIG_P1_MII_MAC)
	val = P1_MII_MAC|P1_ADAPTER_MII;
#endif
	MACREG_UPDATE32(RMII_MII, val, val);

	cta_mac_reset_sw_core();
	cta_switch_buf_init();
	cta_switch_reg_init();

	/* FIXME: Enalbe Wlan lookup table avoid switch DA lookup fail once wifi not bring up.
		Do not enable again if it was done, otherwise maybe flush wifi setting.
	*/
	lookup_table_done();

	/* delay for read phy id, DON'T remove */
	udelay(100000);
	if(!advert)
		return 0;

	/* Phy default advertisement */
	for(phy=0; phy<PORT_HNAT_IDX; phy++)
	{
		info->phyid[phy] = cta_eth_phy_init(phy);
		an_cmd(phy, 1, info->local[phy]);
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_buf_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
s8 cta_switch_buf_init(void)
{
	int i;
	info->switch_buf_size = SWITCH_BUF_SIZE;
	info->switch_buf_headers_count = SWITCH_BUF_HEADER_COUNT;
	/* Reset HW buffer */
	info->switch_buf = (char *)NONCACHED_ADDR(switch_buf);
	memset(info->switch_buf, 0, sizeof(switch_buf));
	/*
		chain buffer headers from element[0] to element[info->switch_buf_headers_count-1]
		this linklist is used for HW as buffer to store & forward frame in switch
	*/
	info->switch_buf_headers = (buf_header *)NONCACHED_ADDR(switch_buf_headers);
	for (i=0;i<info->switch_buf_headers_count;i++)
	{
		info->switch_buf_headers[i].next_index = i + 1;
		info->switch_buf_headers[i].offset = 0;
		info->switch_buf_headers[i].ep = 0;

		info->switch_buf_headers[i].dptr = (u32) &info->switch_buf[i * info->switch_buf_size];
	}
	info->switch_buf_headers[i-1].next_index = 0;
	info->switch_buf_headers[i-1].ep = 1;
	/*
		Enable buffer header process
	*/
	SWREG_WRITE32(SW_HWBUF_HDR_BADDR, PHYSICAL_ADDR(info->switch_buf_headers));
	/* the link list head & tail index are simply ( 0  , total element of macframe headers - 1 ) */
	SWREG_WRITE32(SW_HWBUF_HDR_HT, ((u16)info->switch_buf_headers_count - 1));
	/* Set the buffer sizes of HW pool */
	SWREG_WRITE32(SW_HWBUF_SIZE, info->switch_buf_size);
	/* JH asked to disable DMA_COPY to check whether reorder buffer halt issue is disappeared. */
	//SWREG_WRITE32(SW_HWBUF_POOL_CTRL, SW_P3DMA_ENABLE|SW_HWBUF_INIT_DONE);
	SWREG_WRITE32(SW_HWBUF_POOL_CTRL, SW_HWBUF_INIT_DONE);

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_reg_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
s8 cta_switch_reg_init(void)
{
	/*
		Set buffer flow control threshold, reserved more hw buffer for port0 and port1
		avoid ether port couldn't get hw buffer while wifi router mode.
	*/
	SWREG_WRITE32(SW_HWBUF_FC1, 0x04040404);
	SWREG_WRITE32(SW_HWBUF_FC2, 0x00160000);
	cta_switch_sclk_ctrl();
	/*
		Enable queue manager and config private threshold of descriptor,
		if not enable this, it's easy to run out descriptor and send out pause frame
		if flow control of each port is enabled

		-----------------------------------------------
		| (32) | (32) | (32) | (32) |    (64)    |(14)|
		-----------------------------------------------
		                                      fc_rej  fc_drop
	*/
	SWREG_WRITE32(SW_FC_PUBLIC, 0x004c0040);
	SWREG_WRITE32(SW_FC_PRIVATE0_1, 0x200020);
	SWREG_WRITE32(SW_FC_PRIVATE2_3, 0x200020);
	SWREG_WRITE32(SW_FC_CTRL, 0xf);

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_led_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_led_init(int idx)
{
	u32 rofs, val;

	rofs = idx * 0x100;
	val = SWREG_READ32(rofs|EMAC_LED);
	val |= (LED_POLARITY|LED_MODE);
	SWREG_WRITE32(rofs|EMAC_LED, val);
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_led_action:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_switch_led_action(int idx, u16 link)
{
	u32 rofs, val;

	rofs = idx * 0x100;
	val = SWREG_READ32(rofs|EMAC_LED);
	if(link)
	{
		val &= ~(LED_POLARITY|LED_MODE);
		SWREG_WRITE32(rofs|EMAC_LED, val);
	}
	else
	{
		val |= (LED_POLARITY|LED_MODE);
		SWREG_WRITE32(rofs|EMAC_LED, val);
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_monitor:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u8 cta_switch_monitor(void)
{
	u8 rc = 0;
	u32 val, status;

	if(info->func & CAP_MONITOR)
	{
		/* monitor P1 */
		if(info->status[1] & PHY_LINK) {

			SWREG_WRITE32(SW_DEBUG_MODE, 0x76);
			status = SWREG_READ32(SW_DEBUG_STATUS);
			val = (status & 0x0ff00000) >> 20;

			if((status & 0xffff) == 0xc60f)
			{
				info->mon_hit++;
				if(info->mon_hit >= 10)
				{
					info->mon_hit = 0;
					info->mon_cnt++;
					rc = 1;
				}
			}
			else if((val == 0x12) || (val == 0x15))
			{
				info->mon_p1rxzombie_hit++;
				if(info->mon_p1rxzombie_hit >= 10)
				{
					info->mon_p1rxzombie_hit = 0;
					info->mon_p1rxzombie_cnt++;
					rc = 1;
				}
			}
			else
			{
				info->mon_hit = 0;
				info->mon_p1rxzombie_hit = 0;
			}
		}

		/* monitor P2 */
		SWREG_WRITE32(SW_DEBUG_MODE, 0x98);
		status = SWREG_READ32(SW_DEBUG_STATUS);
		val = (status & 0x0ff00000) >> 20;

		if((val == 0x12) || (val == 0x15))
		{
			info->mon_p2rxzombie_hit++;
			if(info->mon_p2rxzombie_hit >= 10)
			{
				info->mon_p2rxzombie_hit = 0;
				info->mon_p2rxzombie_cnt++;
				rc = 1;
			}
		}
		else
		{
			info->mon_p2rxzombie_hit = 0;
		}
		val = (status & 0x000f0000) >> 16;
		if(val == 0x9)
		{
			info->mon_p2bufempty_hit++;
			if(info->mon_p2bufempty_hit >= 10)
			{
				info->mon_p2bufempty_hit = 0;
				info->mon_p2bufempty_cnt++;
				rc = 1;
			}
		}
		else
		{
			info->mon_p2bufempty_hit = 0;
		}

		/* monitor descriptor */
		SWREG_WRITE32(SW_DEBUG_MODE, 0xba);
		val = SWREG_READ32(SW_DEBUG_STATUS) & 0xff;

		if(val == 0x91)
		{
			info->mon_bufempty_hit++;
			if(info->mon_bufempty_hit >= 10)
			{
				info->mon_bufempty_hit = 0;
				info->mon_bufempty_cnt++;
				rc = 1;
			}
		}
		else
		{
			info->mon_bufempty_hit = 0;
		}

		/* monitor crc */
		val = SWREG_READ32(0x100|EMAC_STATISTIC_2);
		if((val&0xffff) > 200) //crc err
		{
			printk(KERN_INFO "crcerr register value  0x%x we need reset the phy\n",val&0xffff);
			/*reset phy*/
			rc = 2;
		}

	}
	else
	{
		info->mon_hit = 0;
		info->mon_bufempty_hit = 0;
		info->mon_p1rxzombie_hit = 0;
		info->mon_p2rxzombie_hit = 0;
		info->mon_p2bufempty_hit = 0;
	}
	return rc;
}

u8 cta_switch_monitor_status(void)
{
	if (info->func & CAP_MONITOR)
		return 1;
	else
		return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_crc_monitor:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u8 cta_switch_crc_monitor(int idx)
{
	u32 rofs, val;

	if(info->func & CAP_MONITOR)
	{
		rofs = idx * 0x100;
		val = SWREG_READ32(rofs|EMAC_STATISTIC_2) & CRC_ERR_CNT;
		if(val)
		{
			info->mon_crc++;
			SWREG_WRITE32(rofs|EMAC_STATISTIC_2, CRC_ERR_CLR);
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_inc_phy_p20x17:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u32 cta_switch_inc_phy_p20x17(void)
{
	return ++info->mon_phy_p20x17;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_inc_phy_p20x15:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u32 cta_switch_inc_phy_p20x15(void)
{
	return ++info->mon_phy_p20x15;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_inc_phy_p40x14:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u32 cta_switch_inc_phy_p40x14(void)
{
	return ++info->mon_phy_p40x14;
}

/*!-----------------------------------------------------------------------------
 *        cta_switch_inc_phy_anerr:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
u32 cta_switch_inc_phy_anerr(void)
{
	return ++info->mon_phy_anerr;
}

/*!-----------------------------------------------------------------------------
 *        cta_dump_reg1:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_dump_reg1(char *str, u32 *ptr, u32 cmd)
{
	u32 loop, i, ofs;
	if((cmd==SR0) || (cmd==SR16) || (cmd==S_NONE))
		loop = PORT_HNAT_IDX;
	else
		loop = PORT_WLAN_IDX;

	if((cmd==SL0) || (cmd==SR0))
		ofs = 0;
	else if((cmd==SL8) || (cmd==SR8))
		ofs = 8;
	else
		ofs = 16;
	printf("%s", str);
	for(i=0; i<=loop; i++)
	{
		switch(cmd)
		{
		case SL0:
		case SL8:
		case SL16:
			printf("\t%s", (*ptr&((1 << i)<<ofs))?"X" : "");
			continue;
		case SR0:
		case SR16:
			printf("\t%10d", (*ptr++>>ofs)&0xffff);
			continue;
		case S_NONE:
			printf("\t%10d", (*ptr++)&0x3fffffff);
			continue;
		}
	}
	printd("\n");
}

/*!-----------------------------------------------------------------------------
 *        cta_dump_reg2:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static u32 cta_dump_reg2(char *str1, char *str2, u32 reg, u32 flag)
{
	u32 val;
	val = SWREG_READ32(reg);
	printf("%s%d", str1, val&0xffff);
	if(flag)
		printd("\n");
	printf("%s%d", str2, val>>16);
	if(flag)
		printd("\n");
	return val;
}

/*!-----------------------------------------------------------------------------
 *        cta_cfg_func:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_cfg_func(int argc, char *argv[], u8 val)
{
	if(argc!=3)
		return CLI_SHOW_USAGE;
	if(!strcmp(argv[2], "on"))
		info->func |= val;
	else if(!strcmp(argv[2], "off"))
		info->func &= ~val;
	else
		return CLI_SHOW_USAGE;
	return CLI_OK;
}

/*!-----------------------------------------------------------------------------
 *        cta_cfg_phy:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_cfg_phy(int argc, char *argv[], u16 phy, u16 cap)
{
	if(argc!=4)
		return CLI_SHOW_USAGE;
	if(!strcmp(argv[3], "on"))
		info->local[phy] |= cap;
	else if(!strcmp(argv[3], "off"))
		info->local[phy] &= ~cap;
	else
		return CLI_SHOW_USAGE;
	return CLI_OK;
}

/*!-----------------------------------------------------------------------------
 *        cta_an:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_an(void)
{
	u16 phy;
	switch(info->mode)
	{
	case MODE_ETH_DBG:
		for(phy=0; phy<PORT_HNAT_IDX; phy++)
		{
			an_cmd(phy, 0, 0);
		}
		break;
	case MODE_AP:
	case MODE_WIFI_RT:
	case MODE_WISP:
	case MODE_REPEATER:
	case MODE_BRIDGE:
		{
			phy = PORT_ETH1_IDX;
			an_cmd(phy, 0, 0);
		}
		break;
	default:
		printd("%s(%d) bad working mode(%d)\n", __func__, __LINE__, info->mode);
		break;
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_cmd_func:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_cmd_func(int argc, char *argv[], u8 val)
{
	int ret;

	if((val==CAP_BRIDGE) && (info->mode!=MODE_ETH_DBG))
	{
		printd("%s(%d) brg can't active in working mode(%d)\n", __func__, __LINE__, info->mode);
		return CLI_ERROR;
	}
	if((ret=cta_cfg_func(argc, argv, val)))
		return ret;
	/* do not restart an after setup monitor mode */
	if(val==CAP_MONITOR)
		return CLI_OK;

	if((val==CAP_BRIDGE) && (info->func & CAP_BRIDGE))
	{
		info->bitmap = 0xb;
		SWREG_WRITE32(SW_PORT_CFG, 0xb000b);
		SWREG_WRITE32(((PORT_HNAT_IDX * 0x100)|EMAC_CFG), 0x0);
	}

	cta_an();
	return CLI_OK;
}

/*!-----------------------------------------------------------------------------
 *        cta_cmd_phy:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_cmd_phy(int argc, char *argv[], u16 phy, u16 cap)
{
	int ret;

	if((ret=cta_cfg_phy(argc, argv, phy, cap)))
		return ret;

	an_cmd(phy, (info->local[phy] & cap), cap);
	return CLI_OK;
}

struct s_stat
{
	int diff;
	int deq;
};

struct s_stat s_stat_comm;
struct s_stat *s_stat_comm_ptr=&s_stat_comm;
struct s_stat stat_loop[10];
u32 stat_loop_idx=0;
/*!-----------------------------------------------------------------------------
 *        cta_stat_common:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
struct s_stat *cta_stat_common(int dump)
{
	u32 i, j, rofs, offset;
	u32 s[11][3];
	if(dump)
		printf("\t\t\tP0\t\tP1\t\tP2\n");
#ifdef CONFIG_FPGA
	for(j=0; j<10; j++)
#else
	for(j=0; j<7; j++)
#endif
	{
		offset = EMAC_STATISTIC_0 + (j * 4);
		for(i=0; i<PORT_WLAN_IDX; i++)
		{
			rofs = i * 0x100;
			s[j][i] = SWREG_READ32(rofs|offset);
		}
	}
	/* Counter for pause frame "start" */
	for(i=0; i<PORT_WLAN_IDX; i++)
	{
		rofs = i * 0x100;
		s[j][i] = SWREG_READ32(rofs|EMAC_PAUSE_MACADDR_H);
	}
	if(dump)
	{
		cta_dump_reg1("acbusy(b)", s[0], SR16);
		cta_dump_reg1("acbusy(u)", s[0], SR0);
		cta_dump_reg1("unsize\t", s[1], SR16);
		cta_dump_reg1("ovsize\t", s[1], SR0);
		cta_dump_reg1("ovrun\t", s[2], SR16);
		cta_dump_reg1("crcerr\t", s[2], SR0);
		cta_dump_reg1("enqcnt\t", s[3], S_NONE);
		cta_dump_reg1("rexmit(p)", s[4], SR16);
		cta_dump_reg1("unrun\t", s[4], SR0);
		cta_dump_reg1("collision", s[5], SR16);
		cta_dump_reg1("drop16\t", s[5], SR0);
		cta_dump_reg1("deqcnt\t", s[6], S_NONE);
#ifdef CONFIG_FPGA
		cta_dump_reg1("rxdv\t", s[7], S_NONE);
		cta_dump_reg1("crsdv\t", s[8], S_NONE);
		cta_dump_reg1("txen\t", s[9], S_NONE);
		cta_dump_reg1("pause\t", s[10], SR16);
#else
		cta_dump_reg1("pause\t", s[7], SR16);
#endif
	}

	for(i=0; i<PORT_WLAN_IDX; i++)
	{
		rofs = i * 0x100;
		SWREG_WRITE32(rofs|EMAC_STATISTIC_0, ACBUSY_MCAST_CLR|ACBUSY_UCAST_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_1, UNSIZE_CLR|OVERSIZE_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_2, OVERRUN_CLR|CRC_ERR_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_3, RX_ENQ_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_4, UNRUN_PKT_CLR|UNRUN_CNT_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_5, JAM_CLR|COLIISION_DROP16_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_6, TX_DEQ_CLR);
#ifdef CONFIG_FPGA
		SWREG_WRITE32(rofs|EMAC_STATISTIC_7, RXDV_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_8, CRSDV_CLR);
		SWREG_WRITE32(rofs|EMAC_STATISTIC_9, TX_EN_CLR);
#endif
		SWREG_UPDATE32(rofs|EMAC_PAUSE_MACADDR_H, PAUSE_CLR, PAUSE_CLR);
	}
	s_stat_comm_ptr->diff=(s[6][2] &TX_DEQ_CNT)-(s[3][2] & RX_ENQ_CNT);
	s_stat_comm_ptr->deq=s[6][2] & TX_DEQ_CNT;
	return s_stat_comm_ptr;
}

#ifdef __ECOS
/*!-----------------------------------------------------------------------------
 *        cta_stat_loop:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_stat_loop(void)
{
	struct s_stat *ptr;
	if(stat_loop_idx<10)
	{
		ptr=cta_stat_common(0);
		memcpy(&stat_loop[stat_loop_idx++], ptr, sizeof(struct s_stat));
		printd(".");
		timeout((OS_FUNCPTR)cta_stat_loop, NULL, 200);
	}
	else
	{
		printd("\n");
		stat_loop_idx=0;
	}
}
#endif

/*!-----------------------------------------------------------------------------
 *        cta_stat_dump:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_stat_dump(void)
{
	int i;
	struct s_stat *ptr;
	int diff, deq;

	stat_loop_idx=0;
	for(i=0; i<10; i++)
	{
		ptr=&stat_loop[i];
		diff=ptr->diff;
		deq=ptr->deq;
		if(deq)
			printd("(%d) pkt loss(%d) deq(%d) loss_rate(%d)\n", i, diff, deq, (diff*100)/deq);
		else
			printd("(%d) pkt loss(%d) deq(%d)\n", i, diff, deq);
	}
}

/*!-----------------------------------------------------------------------------
 *        sw_debug_cmd:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int sw_debug_cmd(int argc, char *argv[])
{
	if(2 > argc)
		return CLI_SHOW_USAGE;
	if(!strcmp(argv[1], "info"))
	{
		u32 val, i;
		char buf[16];

		val = SWREG_READ32(SW_PORT_CFG);
		printf("[vlan]\n");
		printf("\t\tP0\tP1\tP2\tP3\n");
		cta_dump_reg1("enable\t", &val, SL0);
		cta_dump_reg1("tagging\t", &val, SL8);
		printf("pvid\t");
		cta_dump_reg2("\t", "\t", SW_PORT_VID0_1, 0);
		cta_dump_reg2("\t", "\t", SW_PORT_VID2_3, 0);
		printf("\ntable\n");
		for(i=0; i<8; i++)
		{
			val = SWREG_READ32(SW_VLAN_GROUP0 + (i*4));
			sprintf(buf, "\t(%d)", val&0xffff);
			cta_dump_reg1(buf, &val, SL16);
		}
		printd("\n");
		printf("[misc]\n");
		printf("buf_headers_count=%d\n", info->switch_buf_headers_count);
		printf("buf_headers=%x\n", (unsigned int)info->switch_buf_headers);
		printf("buf=%x\nbuf_size=%d\n", (unsigned int)info->switch_buf, info->switch_buf_size);
		printf("mode=%d\n", info->mode);
		printf("ac=%d\n", (info->func & CAP_ACTIVE)?1:0);
		printf("br=%d\n", (info->func & CAP_BRIDGE)?1:0);
		printf("pp=%d\n", (info->func & CAP_POLLPHY)?1:0);
		printf("retry=%d\n", (info->func & CAP_TXRETRY)?1:0);
		printf("p2boost=%d\n", info->boost);
		printf("\n[monitor]\n");
		printf("mon_mode=%d\n", (info->func & CAP_MONITOR)?1:0);
		printf("mon_cnt=%u\n", info->mon_cnt);
		printf("mon_bufempty_cnt=%u\n", info->mon_bufempty_cnt);
		printf("mon_p1rxzombie_cnt=%u\n", info->mon_p1rxzombie_cnt);
		printf("mon_p2rxzombie_cnt=%u\n", info->mon_p2rxzombie_cnt);
		printf("mon_p2bufempty_cnt=%u\n", info->mon_p2bufempty_cnt);
		printf("mon_crc=%u\n", info->mon_crc);
		printf("mon_phy_dsptrl=%u\n", info->mon_phy_p20x17);
		printf("mon_phy_dsprdidx=%u\n", info->mon_phy_p20x15);
		printf("mon_phy_pcsseed=%u\n", info->mon_phy_p40x14);
		printf("mon_phy_anerr=%u\n", info->mon_phy_anerr);
	}
	else if(!strcmp(argv[1], "phy"))
	{
		u16 phy;
		if(argc!=2)
			return CLI_SHOW_USAGE;

		for(phy=0; phy<PORT_HNAT_IDX; phy++)
		{
			printf( "p%d:\n"
					"\tAdvertised link modes:\t\t%s%s%s%s\n"
					"\tAdvertised pause frame:\t\t%s\n"
					"\tAdvertised auto-negotiation:\t%s\n"
					,phy
					,(info->local[phy] & PHY_CAP_10H)?"10baseT/Half ":""
					,(info->local[phy] & PHY_CAP_10F)?"10baseT/Full ":""
					,(info->local[phy] & PHY_CAP_100H)?"100baseT/Half ":""
					,(info->local[phy] & PHY_CAP_100F)?"100baseT/Full ":""
					,(info->local[phy] & PHY_CAP_PAUSE)?"Yes":"No"
					,(info->local[phy] & PHY_CAP_AN)?"Yes":"No");

			if(info->status[phy] & PHY_LINK)
			{
				printf( "\tLink partner link modes:\t%s%s%s%s\n"
						"\tLink partner pause frame:\t%s\n"
						"\tLink partner auto-negotiation:\t%s\n"
						,(info->partner[phy] & PHY_CAP_10H)?"10baseT/Half ":""
						,(info->partner[phy] & PHY_CAP_10F)?"10baseT/Full ":""
						,(info->partner[phy] & PHY_CAP_100H)?"100baseT/Half ":""
						,(info->partner[phy] & PHY_CAP_100F)?"100baseT/Full ":""
						,(info->partner[phy] & PHY_CAP_PAUSE)?"Yes":"No"
						,(info->status[phy] & PHY_ANC)?"Yes":"No");
			}
			printf( "\tSpeed:\t\t%s\n"
					"\tDuplex:\t\t%s\n"
					"\tLink detected:\t%s\n"
					"\n"
					,(info->status[phy] & PHY_100M)?"100Mb/s":"10Mb/s"
					,(info->status[phy] & PHY_FDX)?"Full":"Half"
					,(info->status[phy] & PHY_LINK)?"Yes":"No");
		}
	}
	else if(!strcmp(argv[1], "stat"))
	{
		if(argc!=2)
			return CLI_SHOW_USAGE;
		cta_stat_common(1);
	}
#ifdef __ECOS
	else if(!strcmp(argv[1], "dbg"))
	{
		if(argc!=2)
			return CLI_SHOW_USAGE;
		stat_loop_idx=0;
		cta_stat_loop();
		timeout((OS_FUNCPTR)cta_stat_dump, NULL, 2300);
	}
#endif
	else if(!strcmp(argv[1], "desc"))
	{
		printd("total=%d\n", SWREG_READ32(SW_ADD_DESC_AMOUNT));
		printd("avail=%d\n", SWREG_READ32(SW_DESC_INFO0));
		cta_dump_reg2("fetch=", "free=", SW_DESC_INFO1, 1);
		cta_dump_reg2("head=", "tail=", SW_DESC_CUR_HT, 1);
	}
	else if (!strcmp(argv[1], "bufhdr"))
	{
		diag_dump_buf_16bit(info->switch_buf_headers, info->switch_buf_headers_count*8);
	}
	else if(!strcmp(argv[1], "qm"))
	{
		u32 p0, p1, p2, p3, val, num;
		val = SWREG_READ32(SW_FC_PRIVATE0_1);
		num = val & 0xffff;
		num += val >> 16;
		val = SWREG_READ32(SW_FC_PRIVATE2_3);
		num += val & 0xffff;
		num += val >> 16;
		num = MIN(num, SW_DESC_NUMBER);
		printd("[private]=%d\n", num);
		p0 = SWREG_READ32(SW_FC_COUNT0);
		p1 = SWREG_READ32(SW_FC_COUNT1);
		p2 = SWREG_READ32(SW_FC_COUNT2);
		p3 = SWREG_READ32(SW_FC_COUNT3);
		printd("[private dequeue]\np0=%d, p1=%d, p2=%d, p3=%d\n", p0, p1, p2, p3);
		val = SWREG_READ32(SW_FC_ENQ0);
		p0 = val & 0xffff;
		p1 = val >> 16;
		val = SWREG_READ32(SW_FC_ENQ1);
		p2 = val & 0xffff;
		p3 = val >> 16;
		printd("[private enqueue]\np0=%d, p1=%d, p2=%d, p3=%d\n", p0, p1, p2, p3);
		val = SWREG_READ32(SW_ADD_DESC_AMOUNT);
		val -= num;
		printd("[public]=%d\n", val);
		cta_dump_reg2("[public threshold]\nreject=", ", drop=", SW_FC_PUBLIC, 0);
		printd("\n");
	}
	else if(!strcmp(argv[1], "br"))
	{
		return cta_cmd_func(argc, argv, CAP_BRIDGE);
	}
	else if(!strcmp(argv[1], "pp"))
	{
		return cta_cmd_func(argc, argv, CAP_POLLPHY);
	}
	else if(!strcmp(argv[1], "retry"))
	{
		return cta_cmd_func(argc, argv, CAP_TXRETRY);
	}
	else if(!strcmp(argv[1], "mon"))
	{
		return cta_cmd_func(argc, argv, CAP_MONITOR);
	}
	else if(!strcmp(argv[1], "p2boost"))
	{
		int i = 0;
		if(argc!=3)
			return CLI_SHOW_USAGE;
		sscanf(argv[2],"%d",&i);
		info->boost = i & 0x3;
		cta_an();
	}
	else if(argc==4)
	{
		int i = 0;
		sscanf(argv[1],"%d",&i);
		if((i<0) || (i>1))
			return CLI_SHOW_USAGE;
		if(!strcmp(argv[2], "autoneg"))
		{
			return cta_cmd_phy(argc, argv, i, PHY_CAP_AN);
		}
		if(!strcmp(argv[2], "flowctl"))
		{
			return cta_cmd_phy(argc, argv, i, PHY_CAP_PAUSE);
		}
		else if(!strcmp(argv[2], "fulldpx"))
		{
			return cta_cmd_phy(argc, argv, i, (info->status[i] & PHY_100M)?(PHY_CAP_100F|PHY_CAP_10F) : PHY_CAP_10F);
		}
		else if(!strcmp(argv[2], "100base"))
		{
			return cta_cmd_phy(argc, argv, i, (info->status[i] & PHY_FDX)?(PHY_CAP_100F|PHY_CAP_100H) : PHY_CAP_100H);
		}
	}
	return CLI_OK;
}

#ifdef __ECOS
CLI_CMD(sw, sw_debug_cmd, "sw\n"
								"\tinfo: basic info\n"
								"\tphy: phy status\n"
								"\tstat: tx/rx statistic\n"
								"\tdbg: tx/rx statistic loop\n"
								"\tdesc: hw descriptor\n"
								"\tbufhdr: hw buffer header\n"
								"\tqm: queue manager status\n"
								"\tbr [on/off]: p0-p1 brige configuration w/o p2\n"
								"\tpp [on/off]: polling phy\n"
								"\tretry [on/off]: tx retry\n"
								"\tmon [on/off]: monitor\n"
								"\tp2boost [0-3]: p2 clk, 0:25M, 1:1/2H, 2:25M, 3:50M\n"
								"\tport[0-1] autoneg [on/off]: auto negotiation\n"
								"\tport[0-1] flowctl [on/off]: flow control\n"
								"\tport[0-1] fulldpx [on/off]: full duplex mode\n"
								"\tport[0-1] 100base [on/off]: 100base TX mode\n"
								, 0);
#else
void idb_dump_buf(u32 *p, u32 s)
{
	int i;
	while ((int)s > 0) {
		for (i = 0;  i < 4;  i++) {
			if (i < (int)s/4) {
				printk("%08X ", p[i] );
			} else {
				printk("         ");
			}
		}
		printk("\n");
		s -= 16;
		p += 4;
	}
}

static int proc_cta_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;

	p += sprintf(p, "use dump command instead\n");
	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

extern int get_args (const char *string, char *argvs[]);
static int proc_cta_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[300];
	int rc;
	int argc;
	char * argv[8] ;

	if (count > 0 && count < 299) {
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
		argc = get_args( (const char *)buf , argv );
		rc=sw_debug_cmd(argc, argv);
		if(rc == CLI_SHOW_USAGE)
		{
			printk("%s", "sw\n"
								"\tinfo: basic info\n"
								"\tphy: phy status\n"
								"\tstat: tx/rx statistic\n"
								"\tdesc: hw descriptor\n");
			printk("%s", 
								"\tbufhdr: hw buffer header\n"
								"\tqm: queue manager status\n"
								"\tbr [on/off]: p0-p1 brige configuration w/o p2\n"
								"\tpp [on/off]: polling phy\n"
								"\tretry [on/off]: tx retry\n"
								"\tmon [on/off]: monitor\n");
			printk("%s", 
								"\tp2boost [0-3]: p2 clk, 0:25M, 1:1/2H, 2:25M, 3:50M\n"
								"\tport[0-1] autoneg [on/off]: auto negotiation\n"
								"\tport[0-1] flowctl [on/off]: flow control\n"
								"\tport[0-1] fulldpx [on/off]: full duplex mode\n"
								"\tport[0-1] 100base [on/off]: 100base TX mode\n");
		}
	}
	return count;
}


static int __init init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *res;

	res = create_proc_entry("sw", S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->read_proc = proc_cta_read;
	res->write_proc = proc_cta_write;
#endif

	return 0;
}

static void __exit fini(void)
{
#ifdef CONFIG_PROC_FS
	remove_proc_entry("sw", NULL);
#endif
}

module_init(init);
module_exit(fini);

EXPORT_SYMBOL(cta_switch_init);
#endif
#endif
