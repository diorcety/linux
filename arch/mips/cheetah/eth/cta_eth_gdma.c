/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
#define REMOVE_HNAT
//#define TRIAL_RX_CKSUM_OFFLOAD
#define ENABLE_TCPUDP_V4_CKSUM		// TODO: support NETIF_F_IPV6_CSUM
    #define TCPUDP_V4_CKSUM_VERBOSE
#define ENABLE_SGIO
    //#define ENABLE_SGIO_JUST_COPY_SKB
#define ENABLE_TSO

/* 
*   \file cta_eth.c
*   \brief  Cheetah ethernet driver.
*   \author Montage
*/
/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
#include <linux/irqreturn.h>
#endif
#include <asm/delay.h>
#include <asm/mach-cheetah/cta_eth.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/common.h>
#include <asm/mach-cheetah/cta_hnat.h>
#include <asm/mach-cheetah/cta_switch.h>
#include <linux/if_vlan.h>
#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
static struct dentry *eth_debug_dir = NULL;

#if 1
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/icmp.h>

#include <asm/io.h>
#include <asm/mach-cheetah/gdma.h>
#endif

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define hal_delay_us(n)				udelay(n)
#define diag_printf					printk
#define cyg_drv_interrupt_unmask(n)	enable_irq(n)
#define cyg_drv_interrupt_mask(n)	disable_irq(n)
#ifndef SA_INTERRUPT
#define SA_INTERRUPT				IRQF_DISABLED
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
#ifndef IRQ_HANDLED
#define IRQ_HANDLED
#define irqreturn_t					void
#endif
#endif


#define ETH_DBG

#if defined(CONFIG_CHEETAH_FPGA)
#define MCR_SMI_DIV				0x0d
#else
#define MCR_SMI_DIV				0x50
#endif
#define MCR_INIT	((MCR_SMI_DIV<<MCR_SMI_S) | MCR_BWCTRL_TRIGGER|MCR_BWCTRL_DROP| \
					MCR_BYBASS_LOOKUP|	/* bypass lookup */ \
					MCR_TXDMA_RETRY_EN|MCR_TX_FLOWCTRL|MCR_LINKUP|MCR_FLOWCTRL)
					//MCR_TXDMA_RETRY_EN|MCR_TX_FLOWCTRL|MCR_LINKUP)

#define MDROPS_INIT				0x1e0100fb
#define MIM_INIT				0x33
#define MRXCR_INIT				0x8201004 //defulat=0x8321004
#define ML3_INIT				0x071505dc

#ifdef CONFIG_NET_CLUSTER_NONCACHED
#define DC_FLUSH_TX(a,l) \
{ \
	if ((((unsigned long)a) & 0xa0000000) != 0xa0000000) \
	{ \
		HAL_DCACHE_STORE(a,l); \
	} \
}

#define DC_FLUSH_RX(a,l)
#define PHYTOVA(a)				nonca_addr(a)
#define DESC_W1_TO_VBUF(w1) 	nonca_addr((w1)&0xfffffc0)
#define DESC_W1_TO_VPKT(w1) 	nonca_addr((w1<<4)>>4)
#else

#define DC_FLUSH_TX(a,l)		HAL_DCACHE_STORE(a,l)
#define DC_FLUSH_RX(a,l)		HAL_DCACHE_FLUSH(a,l)
#define PHYTOVA(a)				ca_addr(a)
#define DESC_W1_TO_VBUF(w1) 	ca_addr((w1)&0xfffffc0)
#define DESC_W1_TO_VPKT(w1) 	ca_addr((w1<<4)>>4)
#endif

#ifndef HAL_READ_ETHMAC
#define HAL_READ_ETHMAC 		cdb_get_mac
#endif

#define NUM_MBUF	 			(NUM_RX_DESC+24+2)

#define DESC_SZ					(NUM_RX_DESC*sizeof(rxdesc_t) \
								+ NUM_TX_DESC*sizeof(txdesc_t) + NUM_HW_DESC*sizeof(rxdesc_t))

#define CTA_ETH_IRQ		IRQ_MAC

#ifdef ETH_DBG
#define DBG_ERR					__BIT(0)
#define DBG_INFO				__BIT(1)
#define DBG_INIT				__BIT(2)
#define DBG_INITMORE			__BIT(3)
#define DBG_TX					__BIT(4)
#define DBG_TXMORE				__BIT(5)
#define DBG_RX					__BIT(6)
#define DBG_RXMORE				__BIT(7)
#define DBG_IOCTL				__BIT(8)
#define DBG_ISR					__BIT(9)
#define DBG_DSR					__BIT(10)
#define DBG_DELIVER				__BIT(11)
#define DBG_PHY					__BIT(12)
#define DBG_TXMORE2				__BIT(13)
#define DBG_RXMORE2				__BIT(14)
#define DBG_HNAT				__BIT(15)

#define DBG_MASK_INIT			(DBG_ERR)
#define DBG_MASK 				cta_eth_debug_lvl
#define DBG_INFO_MASK			(DBG_ERR)

#define DBGPRINTF(_type, fmt, ...) \
	do { \
		if ((_type) & DBG_INFO_MASK) \
			diag_printf(fmt, ## __VA_ARGS__); \
	} while(0)

#else

#define DBGPRINTF(_type, fmt, ...) \
	do { } while(0)
#endif

#ifndef CYG_ASSERT
#define CYG_ASSERTC(test)		do {} while(0)
#define CYG_ASSERT(test,msg)	do {} while(0)
#endif

/* the SMI control is seperated from HWNAT in IP3280 */
#define GPIO_SMI_DIV			0x20b8
#define GPIO_SMI_CR				0x20b4 // offset from MAC's base

#define RX_CACHE_FLUSH_SIZE		0x620 //1568
#define MAX_PKT_DUMP			64

#define HWD_RDY					(1<<31)
#define HWD_WR					(1<<28)
#define HWD_IDX_S				18		// which entry (0~23)
#define HWD_HWI_S				16		//which half-word (0~3)

#define PAUSE_BB		(1 << 12)
#define PAUSE_SW		(1 << 11)
#define PAUSE_USB		(1 << 10)
#define PAUSE_HNAT		(1 << 9)
#define PAUSE_WIFI		(1 << 8)

#define SWRST_BB		(1 << 4)
#define SWRST_SW		(1 << 3)
#define SWRST_USB		(1 << 2)
#define SWRST_HNAT		(1 << 1)
#define SWRST_WMAC		(1 << 0)
/*=============================================================================+
| Variables                                                                    |
+=============================================================================*/
/* Global variables */
static const struct ethtool_ops cta_ethtool_ops;
const static char *eth_ethdrv_version = "0.0.1";

cta_eth_t cta_eth;
cta_eth_t *pcta_eth = 0;
int cta_eth_debug_lvl = DBG_MASK_INIT;
extern struct cta_vlancmd_table cta_vlancmd_table[16];

struct vlan_group *vlgrp;

static bufdesc_t g_txbd[NUM_TX_DESC] __attribute__ ((aligned (0x8)));
// descriptor must be aligned to 0x20 to optimize performance
static unsigned char g_desc[DESC_SZ] __attribute__ ((aligned (0x20)));

struct net_device *ldev;
struct net_device *wdev;
int link_status[2] = { 0, 0 };
static const u8 broadcast_addr[ETH_ALEN] =  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

extern int get_args(const char *string, char *argvs[]);
/*=============================================================================+
| Function Prototypes                                                          |
+=============================================================================*/
/* Interface function for uppler layer */
//#define IN_SEC_IRAM			__attribute__ ((section(".iram")))
#define IN_SEC_IRAM


#define	USED_DSR
//#define USED_VLAN
//#define BRIDGE_FILTER
#define ETH_DEV_INIT			1

#ifdef BRIDGE_FILTER
static char mymac[6];
int cta_eth_filter_mode;
#endif

struct tasklet_struct   tasklet;    /* tasklet for dsr */
if_cta_eth_t * if_cta_eth_g[MAX_IF_NUM];

static void cta_eth_rxint(rxctrl_t *prxc) IN_SEC_IRAM;
static void cta_eth_deliver(struct net_device *sc) IN_SEC_IRAM;
static irqreturn_t cta_eth_isr( int int_num, void *dev_id, struct pt_regs *regs ) IN_SEC_IRAM;
void cta_eth_txint(txctrl_t *ptxc, int wait_free) IN_SEC_IRAM;
int cta_set_mac(struct net_device *dev, void *addr);
int eth_drv_recv1(struct net_device *dev, unsigned char * data,  int total_len, char **newbuf, void *hw_nat, unsigned int w0, unsigned int w1) IN_SEC_IRAM;
char * eth_drv_getbuf(void);
void eth_drv_freebuf(char *buf);

static int eth_start(struct net_device *dev);
static int eth_stop( struct net_device *dev );
static int eth_send(struct sk_buff *skb, struct net_device *dev ) IN_SEC_IRAM;
static int eth_ioctl( struct net_device *dev, struct ifreq *ifr, int cmd );

int cta_hnat_sdmz_rx_check(if_cta_eth_t *pifcta_eth, char *buf, char *pkt, unsigned int w0, unsigned int w1, int len);
void cta_hnat_sdmz_tx_check(char *hwadrp);

void cta_eth_flowctrl(int onoff);

static int cta_eth_get_settings(struct net_device *dev, struct ethtool_cmd *cmd);
static int cta_eth_set_settings(struct net_device *dev, struct ethtool_cmd *cmd);
/*=============================================================================+
| Extern Function/Variables                                                    |
+=============================================================================*/

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/

/*!-----------------------------------------------------------------------------
 *        cta_eth_mdio:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_mdio(short phy,short reg,unsigned short *pval,int wr)
{
	unsigned long v;
	int internal=0;

#ifdef CONFIG_P0_EXT_PHY
	if (phy == 0)
		internal = 0;
	else
#endif
#ifdef CONFIG_P1_EXT_PHY
	if (phy == 1)
		internal = 0;
	else
#endif
		internal = 1;
#ifdef CONFIG_P0_AS_ADDR2
	/*
		Ephy will listen address 0 as broadcast address,
		revise address avoid violation
	*/
	if (phy == 0)
		phy = 2;
#endif
	// arthur add bit29, but it's harmless to cheetah
	if(internal)
		phy ^= MPHY_EXT;
	v = MPHY_TR | (phy << MPHY_PA_S) | (reg << MPHY_RA_S);
	if(wr)
		v |= MPHY_WR | *pval;
	ETH_REG32(GPIO_SMI_CR) = v;
	/* delay for h/w, DON'T remove */
	hal_delay_us(1);
	for(;;)
	{
		if(MPHY_TR & (v=ETH_REG32(GPIO_SMI_CR)))
		{
			break;
		}
	}
	if (!wr)
		*pval=(v&0xffff);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_mdio_rd:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned short cta_eth_mdio_rd(short phy,short reg)
{
	unsigned short rd;
	cta_eth_mdio(phy,reg,&rd,0);
	return rd;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_mdio_wr:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_mdio_wr(short phy,short reg,unsigned short val)
{
	cta_eth_mdio(phy,reg,&val,1);
	if (DBG_INFO_MASK & DBG_PHY)
	{
		unsigned short rd;

		rd=cta_eth_mdio_rd(phy,reg);
		DBGPRINTF(DBG_PHY, "phy%02x,reg%02x,wr:%04x\n", phy,(int)reg,(unsigned int)val);
		if (rd!=val)
			diag_printf("cta_eth_mdio_wr(%02x,%02x,%04x)==> not match, rd:%x\n\n",
						phy, (int)reg, (unsigned int) val, (unsigned int)rd);
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_reset:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_phy_reset(unsigned short phy, unsigned int id)
{
	unsigned short val;

	DBGPRINTF(DBG_PHY, "%s: \n", __func__);
	val = cta_eth_mdio_rd(phy, 0);
	cta_eth_mdio_wr(phy, 0, val | 0x8000);

#if defined(CONFIG_CHEETAH_FPGA)
	if(id==PHYID_MONT_EPHY)
	{
		val = phy_status(phy);
		if(val & PHY_ANC)
		{
			DBGPRINTF(DBG_PHY, "%s: EPHY error: phy_status(%04x)\n", __func__, val);
		}
	}
#endif
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_isolate:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_phy_isolate(unsigned short phy, unsigned int isolate)
{
	/* Electrically isolate PHY from RMII. */
	unsigned short val;

	val = cta_eth_mdio_rd(phy, 0);
	if(isolate)
		cta_eth_mdio_wr(phy, 0, val | 0x0400);
	else
		cta_eth_mdio_wr(phy, 0, val & ~0x0400);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_synchronization:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_phy_synchronization(unsigned short phy)
{
	/*
	^...Avoid rx crc error once link partner is RTL8168B/8111B family
	^...PCI-E Gigabit Ethernet NIC
	 */
	/* Swap to page 2 */
	cta_eth_mdio_wr(phy, 31, 2);
	/* Restart timing synchronize and equalizer for 100BASE-TX */
	cta_eth_mdio_wr(phy, 16, (cta_eth_mdio_rd(phy, 16)|0x8000));
	hal_delay_us(3000);			/* delay for h/w, DON'T remove */
	cta_eth_mdio_wr(phy, 16, (cta_eth_mdio_rd(phy, 16)&0x7fff));
	/* Swap to page 0 */
	cta_eth_mdio_wr(phy, 31, 0);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_monitor:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
#if 0
void cta_eth_phy_monitor(unsigned short phy, unsigned int id, unsigned short cap)
{
	if(id!=PHYID_MONT_EPHY)
		return;

	pcta_eth->ephy.anc_fail_rst = 0;
	if(cap & PHY_LINK)
	{
		unsigned int j;
		/* 20131211 */
		if(cap & PHY_100M)
		{
			volatile unsigned short p20x17, p20x15, p40x14;
			int p20x17_ok=0, p20x15_ok=0, p40x14_ok=0;
			/* Swap to page 2 */
			cta_eth_mdio_wr(phy, 31, 2);
			/* Read 0x17 reg */
			p20x17=cta_eth_mdio_rd(phy, 23);
			if(((p20x17 > 0x0000) && (p20x17 < 0x00ff)) || ((p20x17 > 0xff00) && (p20x17 < 0xffff)))
			{
				p20x17_ok=1;
				DBGPRINTF(DBG_PHY, "%s: EPHY check: page(2) reg(0x17) ok, val(%08x)\n",
							__func__, p20x17);
			}
			else
			{
				DBGPRINTF(DBG_PHY, "%s: EPHY error: page(2) reg(0x17) fail!!, val(%08x) cnt(%d)\n",
							__func__, p20x17, ++pcta_eth->ephy.dsp_p20x17_fail);
			}

			p20x15=cta_eth_mdio_rd(phy, 21);
			p20x15 &= ~0x7e00;		/* bit[14:9]: clear index */
			p20x15 |= 0x0800;		/* bit[14:9]: index=4 */
			cta_eth_mdio_wr(phy, 21, p20x15);
			/* Read 0x15 reg */
			p20x15=cta_eth_mdio_rd(phy, 21);
			p20x15 &= 0x00ff;		/* bit[7:0]: snr value */
			if(((p20x15 > 0x0010) && (p20x15 < 0x007f)))
			{
				p20x15_ok=1;
				DBGPRINTF(DBG_PHY, "%s: EPHY check: page(2) reg(0x15) ok, val(%08x)\n",
							__func__, p20x15);
			}
			else
			{
				DBGPRINTF(DBG_PHY, "%s: EPHY error: page(2) reg(0x15) fail!!, val(%08x) cnt(%d)\n",
							__func__, p20x15, ++pcta_eth->ephy.dsp_p20x15_fail);
			}

			/* 20131213 */
			/* Wait 1ms */
			for(j=0; j<0x8000; j++);
			/* Swap to page 4 */
			cta_eth_mdio_wr(phy, 31, 4);
			/* Read 0x14 reg */
			p40x14=cta_eth_mdio_rd(phy, 20);
			/* Wait 800ms */
			for(j=0; j<0x17e0000; j++);
			/* Read 0x14 reg 2nd */
			p40x14=cta_eth_mdio_rd(phy, 20);
			/* Check bit[15:14] */
			if(0x8000 == (p40x14 & 0xc000))
			{
				p40x14_ok=1;
				DBGPRINTF(DBG_PHY, "%s: EPHY check: page(4) reg(0x14) ok, val(%08x)\n",
							__func__, p40x14);
			}
			else
			{
				DBGPRINTF(DBG_PHY, "%s: EPHY error: page(4) reg(0x14) fail!!, val(%08x) cnt(%d)\n",
							__func__, p40x14, ++pcta_eth->ephy.dsp_p40x14_fail);
			}

			/* Swap to page 0 */
			cta_eth_mdio_wr(phy, 31, 0);
			if((!p20x17_ok) || (!p20x15_ok) || (!p40x14_ok))
			{
				DBGPRINTF(DBG_PHY, "%s: EPHY error: DSP fail, reset EPHY(%d)\n",
							__func__, ++pcta_eth->ephy.dsp_fail);
				cta_eth_phy_reset(phy, id);
				return;
			}
		}
		pcta_eth->ephy.rst_unlock = 1;
	}
	else
	{
		if(pcta_eth->ephy.rst_unlock == 1)
			cta_eth_phy_reset(phy, id);
	}
}
#endif
int cta_eth_phy_monitor(unsigned short phy, unsigned int id, unsigned short cap)
{
	if(id!=PHYID_MONT_EPHY)
		return 0;

	pcta_eth->ephy.anc_fail_rst = 0;
	if(cap & PHY_LINK)
	{
		unsigned int j;
		/* 20131211 */
		if(cap & PHY_100M)
		{
			volatile unsigned short p40x14;
			int p40x14_ok;

			/* 20131213 */
			/* Wait 1ms */
			for(j=0; j<0x8000; j++);
			/* Swap to page 4 */
			cta_eth_mdio_wr(phy, 31, 4);
			/* Read 0x14 reg */
			p40x14=cta_eth_mdio_rd(phy, 20);
#if 0
			/* Wait 800ms */
			for(j=0; j<0x17e0000; j++);
			/* Read 0x14 reg 2nd */
#else
			for(j=0; j<0x8000; j++);
#endif
			p40x14=cta_eth_mdio_rd(phy, 20);
			/* Check bit[15:14] */
			if(0x8000 == (p40x14 & 0xc000))
				p40x14_ok=1;
			else
				p40x14_ok=0;

			/* Swap to page 0 */
			cta_eth_mdio_wr(phy, 31, 0);
			if(!p40x14_ok)
			{
				DBGPRINTF(DBG_ERR, "%s: EPHY error: descriambler seed don't load done, reset EPHY(%d)\n",
							__func__, ++pcta_eth->ephy.dsp_p40x14_fail);
				cta_eth_phy_reset(phy, id);
				return 1;
			}
		}
	}
	return 0;
}

#if defined(CONFIG_CHEETAH_FPGA)

//#define POLLING_TIMER
#if defined(POLLING_TIMER)
#define PHY_POLLING_INTERVAL        ( HZ )
static struct timer_list phy_polling_timer;
#endif

#endif

/*!-----------------------------------------------------------------------------
 *        cta_eth_hwreset:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_hwreset(void)
{
	DBGPRINTF(DBG_INIT, "cta_eth_hwreset\n");
	ETH_REG32(MSRST) = 0xfff;
	ETH_REG32(MSRST) = 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_txd_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_txd_init(txctrl_t *ptxc)
{
	/* Initialize the transmit buffer descriptors. */
	DBGPRINTF(DBG_INIT, "ifcta_eth: init %d txd at %08x\n", ptxc->num_txd,  (unsigned int) ptxc->ptxd);
	/* zero all tx desc */
	memset((void *) ptxc->ptxd, 0, ptxc->num_txd * sizeof(txdesc_t));
	/* Init buffer descriptor  */
	memset((void *) ptxc->ptxbuf, 0, ptxc->num_txd * sizeof(bufdesc_t));
	
	/*  Mark end of ring at the last tx descriptor  */
	ptxc->ptxd[ptxc->num_txd-1].w0 = DS0_END_OF_RING;
	
	ptxc->num_freetxd = ptxc->num_txd;
	ptxc->tx_head = ptxc->tx_tail = 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_rxd_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_rxd_init(rxctrl_t *prxc)
{
	/* Initialize the receive buffer descriptors. */
	int i;
	rxdesc_t *prxd;

	DBGPRINTF(DBG_INIT, "ifcta_eth: init %d rxd at %08x\n",
			prxc->num_rxd, (unsigned int) prxc->prxd);

	prxd = prxc->prxd;
	for (i = prxc->num_rxd; i>0; i--, prxd++)
	{
		prxd->w1 = 0;
		prxd->w0 = DS0_RX_OWNBIT;
	}
	(--prxd)->w0 = DS0_END_OF_RING|DS0_RX_OWNBIT;

	prxc->num_freerxd = prxc->num_rxd;
	prxc->rx_head = prxc->rx_tail = 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_txd_free:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_txd_free(txctrl_t *ptxc)
{
	int i;
	void *buf;
	bufdesc_t *ptxbuf = &ptxc->ptxbuf[0];

	/* free buffer */
	for(i=0; i < ptxc->num_txd; i++,ptxbuf++)
	{
		if ((buf=ptxbuf->buf))
		{
			eth_drv_freebuf(buf);
		}
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_rx_start:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_rx_start(void)
{
	ETH_REG32(MCR) |= (MCR_RXDMA_EN | MCR_RXMAC_EN);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_tx_start:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_tx_start(void)
{
	ETH_REG32(MCR) |= (MCR_TXDMA_EN | MCR_TXMAC_EN);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_hwd_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_hwd_init(void)
{
	unsigned short i,j;
	unsigned long x;

	for(i=0;i<24;i++)
	{
		for (j=0; j < 4; j++)
		{
			while (HWD_RDY & ETH_REG32(HWDESC));
			x = HWD_RDY|HWD_WR |((unsigned long)i<<HWD_IDX_S) | ((unsigned long)j<<HWD_HWI_S) ;
			if (j==0)
			{
				x |= ((unsigned long)DS0_RX_OWNBIT >>16)  |
					((i==23)?  (DS0_END_OF_RING >>16) : 0 ) ;
			}
			//diag_printf("x=%08x\n", x);
			ETH_REG32(HWDESC) = x;
		}
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_hwinit:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_hwinit(void)
{
	DBGPRINTF(DBG_INIT, "%s\n", __func__);

	/*  descriptor base  */
	MREG(MHWDB) = (unsigned int) pcta_eth->hw.prxd;
	MREG(MTXDB) = (unsigned int) pcta_eth->tx.ptxd;
	MREG(MSWDB) = (unsigned int) pcta_eth->rx.prxd;
	MREG(MAC_RXCR) = 0x08401004;

#if defined(CONFIG_CHEETAH_FPGA)
	MREG(MFP) = 0xcffff;
#else
	//ETH_REG32(MFP) = 0x2dffff;
	MREG(MFP) = 0x10ffff;
#endif

	ETH_REG32(MVL01) = 0x10000fff;
	ETH_REG32(MVL23) = 0x10040003;
	ETH_REG32(MVL45) = 0x10060005;
	ETH_REG32(MVL67) = 0x10080007;

	MREG(MCR)	= MCR_INIT;
	MREG(MRXCR) = MRXCR_INIT;
	MREG(MAUTO) = 0x81ff0030;
	MREG(MDROPS)= MDROPS_INIT;
	MREG(MMEMB) = 0;
	MREG(ML3)	= ML3_INIT;
	MREG(MTMII) = 0xc0000000;		//bit[31] 1:send nibble pause to switch if run out software desc.
									//bit[30] 1:turn on TMII TX machanism.

	/* Clear all pending interrupts */
	MREG(MIC)	= MREG(MIS);
	MREG(MPKTSZ)= 0x00fa05fe;		//lookup serch time, unit:clk.
	MREG(MTW) 	= 0x00000f08;		//tx retry number

	/*  Enable interrupt  */
	// enable in cta_eth_if_start
	//ETH_REG32(MIM) &= ~MIM_INIT;

	DBGPRINTF(DBG_INIT, "\nInit switch, LAN(VID 4095):p%d, WAN(VID 0): p%d\n",
							PORT_ETH1_IDX, PORT_ETH0_IDX);

	/* New switch architecture*/
	cta_switch_init(1);

	/* HW NAT initialize */
	cta_hnat_init();
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_buf_free:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_eth_buf_free(void)
{
	unsigned long cb,nb,*p;

	if (0 ==(cb = ETH_REG32(MBUFB)))
	{
		DBGPRINTF(DBG_INIT, "%s MBUF=0,can't feee\n",__func__);
		return -1;
	}
	for (;cb;)
	{
		if (3 & (unsigned long)cb )
		{
			DBGPRINTF(DBG_INIT, "%s unaligned BUF=%08lx,can't free\n",__func__, cb);
			break;
		}
		p = (unsigned long*)nonca_addr((cb<<8)>>8);
		nb = *p; // next
		eth_drv_freebuf((void*)(PHYTOVA((cb<<8)>>8)-BUF_HW_OFS)) ;
		cb = nb;
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_buf_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_eth_buf_init(void)
{
	unsigned long first, cb, nb;
	int i;

	if (!(cb = first = (unsigned long)eth_drv_getbuf()))
	{
		goto no_buf;
	}
	ETH_REG32(MBUFB) = virtophy(first)+BUF_HW_OFS ;
	for (i = 0; i < NUM_MBUF-1; i++)
	{
		if (!(nb = (unsigned long)eth_drv_getbuf()))
		{
			goto free;
		}
		/* current buf's head point to next buf's head */
		*(unsigned long *)(nonca_addr(cb)+BUF_HW_OFS) = virtophy(nb)+BUF_HW_OFS;
		/* saved the pointer (MSB 16bits) of buffer */
		//*(unsigned char*)(nonca_addr(cb)+BUF_MSB_OFS) = ((unsigned int)cb)>>24 ;
		cb = nb ;
	}
	*(unsigned long *)(nonca_addr(cb)+BUF_HW_OFS) = 0;
	ETH_REG32(MLBUF) = virtophy(cb)+BUF_HW_OFS;
	DBGPRINTF(DBG_INIT, "%s: MBUF=%x, MLBUF=%x\n", __func__, ETH_REG32(MBUFB), ETH_REG32(MLBUF));

	return 0;

free:
	cta_eth_buf_free();
no_buf:
	DBGPRINTF(DBG_INIT, "%s:no buf\n",__func__);
	return -1;
}

/*!-----------------------------------------------------------------------------
 *        cta_smi_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_smi_init(void)
{
#if (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
	MREG(GPIO_SMI_DIV) = MCR_SMI_DIV << MCR_SMI_S;
#endif
	DBGPRINTF(DBG_PHY,"SMI_CR=%x\n", (MAC_BASE+GPIO_SMI_CR));
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_init(void)
{
	/*
		This function attaches the Ethernet interface to the OS. It also calls various driver
		initialization functions to allocate drivers resources and initialize the device.
	*/
	pcta_eth = &cta_eth;

	DBGPRINTF(DBG_INIT, "%s\n", __func__);

	cta_smi_init(); // !!!!! What is this

#if !defined(__ECOS) && (CHEETAH_VERSION_CODE >= CHEETAH_VERSION(2,0))
	/* reset HNAT & Switch */
	preempt_disable();

	/* to reset Switch, Wifi/HNAT/Switch clocks need to be turn on */
	/* turn on WIFI clock */
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) & (~0x100UL));
	/* turn on HNAT clock */
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) & (~0x200UL));
	/* turn on Switch clock */
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) & (~0x800UL));

	/* reset HNAT */
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) | 0x2UL);
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) & (~0x2UL));
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) | 0x2UL);

	/* reset Switch */
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) | 0x8UL);
	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) & (~0x8UL));

	mdelay(10);

	ETH_REG32(HW_RESET) = (ETH_REG32(HW_RESET) | 0x8UL);
	preempt_enable();
#endif

	/* Make sure the uncached memory area is not cached!! */
	HAL_DCACHE_FLUSH(g_desc, DESC_SZ);

	/* Reset switch hardware */
	cta_eth_hwreset(); // NEED MODIFY ???

	//cta_eth_phyinit();
	pcta_eth->tx.ptxd = (txdesc_t *) nonca_addr((unsigned int)g_desc);
	pcta_eth->rx.prxd = (rxdesc_t *)(pcta_eth->tx.ptxd + NUM_TX_DESC);
	pcta_eth->hw.prxd = (rxdesc_t *)(pcta_eth->rx.prxd + NUM_RX_DESC);
	
	pcta_eth->tx.ptxbuf = &g_txbd[0];
	pcta_eth->tx.num_txd = NUM_TX_DESC;
	pcta_eth->rx.num_rxd = NUM_RX_DESC;
	pcta_eth->hw.num_rxd = NUM_HW_DESC;

	if (cta_eth_buf_init())
		return -1;

	cta_eth_txd_init(&pcta_eth->tx);
	cta_eth_rxd_init(&pcta_eth->rx);
	cta_eth_rxd_init(&pcta_eth->hw);
	cta_eth_hwd_init();

	pcta_eth->vector = CTA_ETH_IRQ;

#ifdef	USED_DSR
	tasklet_init(&tasklet, (void *)cta_eth_deliver, 0);
#endif
	/* request non-shared interrupt */
	if(request_irq(pcta_eth->vector, (irq_handler_t)&cta_eth_isr,
					SA_INTERRUPT, "ETHER", pcta_eth) != 0)
	{
		diag_printf( "%s: irq %d request fail\n", __func__, pcta_eth->vector );
		return -ENODEV;
	}
	/* create IRQ handler with IRQ disabled, mimic the eCos behavior */
	disable_irq(pcta_eth->vector);
	/* Initialize hardware */
	cta_eth_hwinit();

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_if_start:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_if_start(int unit)
{
	if_cta_eth_t *pifcta_eth;
	unsigned short if_bit;

	pifcta_eth = pcta_eth->if_array[unit];
#if defined(CONFIG_CHEETAH)
	if(unit == 0)
		pifcta_eth->phy_addr = PORT_ETH1_IDX;
	else
		pifcta_eth->phy_addr = PORT_ETH0_IDX;
#else
	if(unit == 0)
		pifcta_eth->phy_addr = 0;
	else
		pifcta_eth->phy_addr = 4;
#endif
	CYG_ASSERTC(pifcta_eth != 0);

	if_bit = 1 << unit;

	DBGPRINTF(DBG_INIT, "if321x_if_start(%d)\n", unit);
	if(pcta_eth->act_ifmask & if_bit)
		return;

	if(pcta_eth->act_ifmask == 0)
	{
		pcta_eth->sc_main = pifcta_eth->sc;
		//TODO , add initial code

		/*  Enable interrupt  */
		ETH_REG32(MIM) &= ~MIM_INIT;
		cyg_drv_interrupt_unmask(pcta_eth->vector);
	}
	pcta_eth->act_ifmask |= if_bit;
	DBGPRINTF(DBG_INIT, "if321x_if_start: act_ifmask=%x\n", pcta_eth->act_ifmask);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_rx_stop:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_rx_stop(rxctrl_t *prxc)
{
	int i;
	rxdesc_t *prxd;
	void *bufp;

	ETH_REG32(MCR) &= ~(MCR_RXDMA_EN | MCR_RXMAC_EN);
	for(i = 0; i < prxc->num_rxd; i++)
	{
		prxd = prxc->prxd + i;
		if(((DS0_RX_OWNBIT | DS0_TX_OWNBIT) >> 16) & *(unsigned short *) prxd)
		{
			prxd->w0 &= ~(DS0_RX_OWNBIT | DS0_TX_OWNBIT);
			prxd->w0 |= DS0_DROP_OF_PACKET;
			if(prxd->w1&0xffffff)
			{
				bufp = (void*)DESC_W1_TO_VBUF(prxd->w1);
				eth_drv_freebuf(bufp);
			}
		}
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_tx_stop:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_tx_stop(txctrl_t *ptxc)
{
	int i;
	txdesc_t *ptxd;

	ETH_REG32(MCR) &= ~(MCR_TXDMA_EN | MCR_TXMAC_EN);
	for(i = 0; i < ptxc->num_txd; i++)
	{
		ptxd = ptxc->ptxd + i;
		if(((DS0_RX_OWNBIT | DS0_TX_OWNBIT) >> 16) & *(unsigned short *) ptxd)
		{
			ptxd->w0 &= ~(DS0_RX_OWNBIT | DS0_TX_OWNBIT);
			ptxd->w0 |= DS0_DROP_OF_PACKET;
		}
	}
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_if_stop:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_if_stop(int unit)
{
	if_cta_eth_t *pifcta_eth;
	unsigned short if_bit;

	pifcta_eth = pcta_eth->if_array[unit];
	if_bit = 1 << unit;

	DBGPRINTF(DBG_INIT, "cta_eth_if_stop(%d) ifp=%x act_ifmask=%x\n",
				unit, (int) pifcta_eth, (int) pcta_eth->act_ifmask);
	CYG_ASSERTC(pifcta_eth != 0);
	if (!(pcta_eth->act_ifmask & if_bit))
		return;

	if ((pcta_eth->act_ifmask &= ~if_bit) == 0)
	{
		/* Disable interrupt */
		ETH_REG32(MIM)|= MIM_INIT;
		cyg_drv_interrupt_mask(pcta_eth->vector);
		if (0)
		{
			cta_eth_rx_stop(&pcta_eth->rx);
			cta_eth_tx_stop(&pcta_eth->tx);
			cta_eth_txd_free(&pcta_eth->tx);
		}
		diag_printf("Don't free NOW!\n");
		pcta_eth->sc_main = 0;
		return;
	}
	DBGPRINTF(DBG_INIT, "cheetah_if_stop: return act_ifmask=%x\n", pcta_eth->act_ifmask);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_add_if:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_add_if(int unit, if_cta_eth_t *pifcta_eth)
{
	int status = -1;

	if(unit <0 || unit >= MAX_IF_NUM)
		return -1;

	if(pcta_eth->if_array[unit] == 0)
	{
		pcta_eth->if_array[unit] = pifcta_eth;
		pifcta_eth->to_vlan = ((unit|0x8) << 8);
		pcta_eth->if_cnt++;
		status = 0;
	}

	return status;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_del_if:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_del_if(int unit)
{
	if_cta_eth_t *pifcta_eth;

	if(unit < 0 || unit >= MAX_IF_NUM)
		return -1;

	DBGPRINTF(DBG_INIT, "cta_eth_del_if(%d)\n", unit);
	pifcta_eth = pcta_eth->if_array[unit];
	CYG_ASSERTC(pifcta_eth != 0);
	pcta_eth->if_array[unit] = 0;

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_isr:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static irqreturn_t cta_eth_isr( int int_num, void *dev_id, struct pt_regs *regs)
{
	/* device interrupt handler */
//	struct net_device *dev = (struct net_device *)dev_id;
	ETH_REG32(MIM)|= MIM_INIT;
#ifdef	USED_DSR
	{
		tasklet_hi_schedule(&tasklet);
		return IRQ_HANDLED;
	}
#else
	cta_eth_deliver(0);
	return IRQ_HANDLED;
#endif
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_rxint:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_rxint(rxctrl_t *prxc)
{
	/* The receive interrupt service handler. */
	register unsigned int w0, w1;
	unsigned char *nbufp, *bufp;
	unsigned char *pkt;
	rxdesc_t *prxd;
	short frame_len=0;
	if_cta_eth_t *pifcta_eth;
	int loop;

	prxd = prxc->prxd;

	DBGPRINTF(DBG_RXMORE, "rxint, rxc=%x\n", (unsigned int) prxc);

	prxd = &prxc->prxd[prxc->rx_head];
	w0 = prxd->w0;
	w1 = prxd->w1;
	loop=0;

	while (!(w0 & (DS0_RX_OWNBIT)) && (loop++ < (NUM_RX_DESC/1)))
	{
		DBGPRINTF(DBG_RXMORE, "rxint, DS0 w0=%x w1=%x\n",  w0, w1);
		pkt = 0;
		/* For statistics, locate the interface first */
		DBGPRINTF(DBG_RXMORE, "vl=%x\n", RX_VIDX(w0));
		if ((pifcta_eth = pcta_eth->if_array[0]) == 0)
		{
			/* The port has been released. */
			/* Drop the packet  */
			prxd->w1=((w1>>6)<<6) + BUF_HW_OFS;
			goto next;
		}

		if(w0 & DS0_DROP_OF_PACKET)
		{
			if(w0 & (DS0_EIPCS|DS0_EL4CS))
			{
				prxc->rx_err++;
				prxc->rx_errcode[RX_ERROR(w0)]++;
				/*  Drop the packet  */
				prxd->w1=((w1>>6)<<6) + BUF_HW_OFS;
				goto next;
			}
			else
			{
#if 0
				diag_printf("Drop!!!, cause:%x\n", RX_ERROR(w0));
				bufp = (unsigned char*)nonca_addr(w1);
				bufp = (unsigned char*)((0xa<<28) | ((((unsigned int)bufp)<<4)>>4));
				diag_dump_buf_16bit(bufp,64);
#endif
			}
		}

		frame_len = RX_FRAMELEN(w0);
		prxc->rx_bytecnt += frame_len;
#if 0
		/* check broadcast or multicast packet */
		if(w0 & DS0_ISBC)
			prxc->rx_bcpkt++;
		else if(w0 & DS0_ISMC)
			prxc->rx_mcpkt++;
#endif
		prxc->rx_pktcnt++;
		/* Indicate the packet to the upper layer */
		if((pifcta_eth->flags & PRIVIFFG_ACTIVE)
			&& (0!=(nbufp=(char*)eth_drv_getbuf())))
		{
			pkt = (unsigned char*)DESC_W1_TO_VPKT(w1);
			bufp =  (unsigned char*)DESC_W1_TO_VBUF(w1);
			DC_FLUSH_RX(nbufp, RX_CACHE_FLUSH_SIZE);
			nbufp += BUF_HW_OFS ;
			prxd->w1 = (unsigned long)nbufp;
		}
		else
		{
			prxd->w1=((w1>>6)<<6) + BUF_HW_OFS;
		}
next:
		/* Release the buffer to hardware */
		if (++prxc->rx_head == prxc->num_rxd)
		{
			prxd->w0 = DS0_RX_OWNBIT | DS0_END_OF_RING;
			prxc->rx_head = 0;
		}
		else
			prxd->w0 = DS0_RX_OWNBIT;

		/* dispatch to upper layer */
		if(pkt != 0)
		{
			int hw_nat;
			//diag_dump_buf_16bit(bufp+0x40,64);
			//printf("hash[0]:%x, hash[1]:%x\n", *(int *)(bufp+4), *(int *)(bufp+12));

			if((hw_nat = cta_hnat_rx_check(bufp, pkt, w0, w1)) < 0)
			{
				eth_drv_freebuf(bufp);
				goto next_desc;
			}
			if(cta_hnat_sdmz_rx_check(pifcta_eth, bufp, pkt, w0, w1, frame_len) == 0)
				eth_drv_recv1(pifcta_eth->sc, pkt, (short)frame_len,
							(char**)&bufp, (void *)hw_nat, w0, w1); //4byte VLAN
		}
next_desc:
		prxd = &prxc->prxd[prxc->rx_head];
		w0 = prxd->w0;
		w1 = prxd->w1;
	}

	DBGPRINTF(DBG_RXMORE, "rxint end, prxc=%x\n", (unsigned int)prxc);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_txint:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_txint(txctrl_t *ptxc, int wait_free)
{
	/* The transmit interrupt service handler. */
	volatile txdesc_t *ptxd;
	bufdesc_t *ptxbuf;
	unsigned int tx_ctrl;

	DBGPRINTF(DBG_TXMORE, "txint, txc=%x\n", (unsigned int) ptxc);

	ptxd = ptxc->ptxd;
	ptxbuf = ptxc->ptxbuf;
	tx_ctrl = ptxd[ptxc->tx_tail].w0;

	if(wait_free && (ptxd[ptxc->tx_tail].w0 & DS0_TX_OWNBIT))
	{
		int i = 0;
		int j;
		diag_printf("txbusy %s(%d) tail=%d head=%d\n", __func__, __LINE__, ptxc->tx_head, ptxc->tx_tail);
		for (j=0;j < ptxc->num_txd;j++)
		{
				if((j&3)==0)
					diag_printf("\n");
				diag_printf("%08x-%08x ", ptxd[j].w0, ptxd[j].w1 );
		}
		while((ptxd[ptxc->tx_tail].w0&DS0_TX_OWNBIT))
		{
			i++;
		}
		diag_printf("i=%d\n", i);
	}
	while((!(ptxd[ptxc->tx_tail].w0 & DS0_TX_OWNBIT)) && (ptxc->num_freetxd < ptxc->num_txd))
	{

		DBGPRINTF(DBG_TX, "TD: %d,%x,%x", ptxc->tx_tail,ptxd[ptxc->tx_tail].w0, ptxd[ptxc->tx_tail].w1);
		DBGPRINTF(DBG_TX, ",%x\n", (unsigned int)ptxbuf[ptxc->tx_tail].buf);
		if (ptxbuf[ptxc->tx_tail].buf != 0)
		{
			ptxc->tx_ok++;

			if ((unsigned int)ptxbuf[ptxc->tx_tail].buf >= 0x80000000)
			{
#ifdef	USED_DSR
				dev_kfree_skb(ptxbuf[ptxc->tx_tail].buf);
#else
				dev_kfree_skb_irq(ptxbuf[ptxc->tx_tail].buf);
#endif
			}

			ptxbuf[ptxc->tx_tail].sc = 0;
			ptxbuf[ptxc->tx_tail].buf = 0;
		}

		ptxc->num_freetxd++;
		if (++ptxc->tx_tail == ptxc->num_txd)
			ptxc->tx_tail = 0;

		//tx_ctrl = ptxd[ptxc->tx_tail].w0;
	}
	DBGPRINTF(DBG_TXMORE, "txint end, txc=%x\n", (unsigned int) ptxc);
}

/*!-----------------------------------------------------------------------------
 *        eth_rx_mode:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void eth_rx_mode(struct net_device *dev)
{
	/* received include address un-matched */
	if(dev->flags & IFF_PROMISC)
		ETH_REG32(MDROPS) &= ~(1<<7);
	return;
}

/*!-----------------------------------------------------------------------------
 *        vid_to_vidx:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static unsigned short vid_to_vidx(unsigned short vid)
{
	unsigned short idx;

	if (vid == 0xfff)
		return 0;

	else
		return 1;
	for(idx=8; idx < 16; idx++)
	{
		if((vid|0x1000) == cta_vlancmd_table[idx].vid)
			break;
	}
	return (idx-8);
}

/*!-----------------------------------------------------------------------------
 *        eth_vlan_rx_add_vid:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void eth_vlan_rx_add_vid(struct net_device *dev, unsigned short vid)
{
	struct vlan_group *vg;
	struct net_device *vdev;

	if_cta_eth_t *pifcta_eth;
	pifcta_eth =(if_cta_eth_t *)netdev_priv(dev);
	vg = pifcta_eth->vg;

	vdev = vlan_group_get_device(vg, vid);
	if(!vdev || !pcta_eth)
		return;
	/* Keep original ndo_set_mac_address */
	if(!pcta_eth->ndo_set_mac_address)
	{
		pcta_eth->ndo_set_mac_address = vdev->netdev_ops->ndo_set_mac_address;
	}
	/* Swap to ours */
	vdev->netdev_ops->ndo_set_mac_address = cta_set_mac;
	vdev->ethtool_ops = &cta_ethtool_ops;
	vdev->priv_flags |= IFF_ETH_DEV;

	if(vid == 4095)
		ldev = vdev;
	if(vid == 0)
		wdev = vdev;
}

/*!-----------------------------------------------------------------------------
 *        eth_vlan_rx_register:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void eth_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	if_cta_eth_t *pifcta_eth;
	pifcta_eth =(if_cta_eth_t *)netdev_priv(dev);
	pifcta_eth->vg = grp;
}

/*!-----------------------------------------------------------------------------
 *        eth_vlan_rx_kill_vid:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void eth_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid)
{
	;
}

/*!-----------------------------------------------------------------------------
 *        eth_start:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int eth_start(struct net_device *dev)
{
	if_cta_eth_t *pifcta_eth;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
#else
	pifcta_eth = (if_cta_eth_t *)dev->priv;
#endif
	CYG_ASSERTC(pifcta_eth->sc == dev);

	DBGPRINTF(DBG_INFO, "eth_start(%d)\n", pifcta_eth->unit);

	if(!(pifcta_eth->flags & PRIVIFFG_ACTIVE))
	{
		pifcta_eth->flags |= PRIVIFFG_ACTIVE;
		cta_eth_if_start(pifcta_eth->unit);
	}

	cta_eth_rx_start();
	cta_eth_tx_start();

	memcpy(pifcta_eth->macaddr, dev->dev_addr, 6);
#if 0
	cta_eth_setmac(pifcta_eth->unit, pifcta_eth->macaddr);
#endif

    netif_start_queue (dev);

	DBGPRINTF(DBG_INFO, "eth_start(%d): done\n", pifcta_eth->unit);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        eth_stop:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int eth_stop( struct net_device *dev )
{
	if_cta_eth_t *pifcta_eth;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
#else
	pifcta_eth = (if_cta_eth_t *)dev->priv;
#endif

	CYG_ASSERTC(pifcta_eth->sc == sc);
	DBGPRINTF(DBG_INFO, "eth_stop(%d)\n", pifcta_eth->unit);

	netif_stop_queue(dev);
	if(pifcta_eth->flags & PRIVIFFG_ACTIVE)
	{
		pifcta_eth->flags &= ~PRIVIFFG_ACTIVE;
		cta_eth_if_stop(pifcta_eth->unit);
	}

	DBGPRINTF(DBG_INFO, "eth_stop(%d): done\n", pifcta_eth->unit);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        eth_ioctl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int eth_ioctl( struct net_device *dev, struct ifreq *ifr, int cmd)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	if_cta_eth_t *pdp= (if_cta_eth_t*) netdev_priv(dev);
#else
	if_cta_eth_t *pdp= (if_cta_eth_t*)dev->priv;
#endif
	struct mii_ioctl_data *data_ptr = (struct mii_ioctl_data *) &(ifr->ifr_data);
	//DBGPRINTF(DBG_INFO, "eth_ioctl(%d):cmd=%x ifr=%x\n", pifcta_eth->unit, cmd, (int)ifr);
	switch(cmd)
	{
	case SIOCGMIIPHY:
		data_ptr->phy_id = pdp->phy_addr & 0x1f;
		break;
	case SIOCGMIIREG:
		cta_eth_mdio(pdp->phy_addr,data_ptr->reg_num & 0x1f,(&(data_ptr->val_out)),0);
		break;
	case SIOCSMIIREG:
		cta_eth_mdio(pdp->phy_addr,data_ptr->reg_num & 0x1f,&(data_ptr->val_out),1);
		break;
	case SIOCSIFHWADDR:
		printk("GOT YOU\n");
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_set_mac:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_set_mac(struct net_device *dev, void *addr)
{
	unsigned int mac_hi, mac_lo;
	struct sockaddr *p = addr;
	unsigned char *mac = p->sa_data;
	int i;

	if(!is_valid_ether_addr(mac))
		return -EADDRNOTAVAIL;
	/* Not vlan net device */
	if((dev->priv_flags & IFF_802_1Q_VLAN) == 0)
	{
		for(i = 0; i < 6; ++i)
		{
			dev->dev_addr[i] = mac[i];
		}
		return 0;
	}
	/* Execute original hook funtion */
	if(pcta_eth->ndo_set_mac_address)
	{
		pcta_eth->ndo_set_mac_address(dev, addr);
	}

	/* To do: sync with switch flow ctrl mac */
	mac_hi = (mac[0] << 8) | mac[1] ;
	mac_lo = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];

	if(dev == wdev)
	{
		ETH_REG32(MSA10) = mac_hi;
		ETH_REG32(MSA11) = mac_lo;
		register_macaddr(1,mac);
	}
	else if(dev == ldev)
	{
		ETH_REG32(MSA00) = mac_hi;
		ETH_REG32(MSA01) = mac_lo;
		register_macaddr(1,mac);
	}

	return 0;
}

#ifdef BRIDGE_FILTER
/*!-----------------------------------------------------------------------------
 *        cta_eth_rx_filter:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_eth_rx_filter(char *data)
{
	static char all_ones[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	/* broadcast */
	if(!memcmp(data, all_ones, 6))
		return 0;
	/* multicast */
	if(data[0] == 0x01)
		return 0;
	/* myself, TODO: partial match */
	if(!memcmp(mymac, data, 5))
		return 0;
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_filter:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_eth_filter(struct sk_buff *skb, char *data, int tx)
{
	if( cta_eth_filter_mode == 0) //skip?
		return 0;

	if(tx)
	{
		return 0;
		/* if not from our self, drop */
		if(!memcmp(mymac, data+6, 5 ))
			return 0;
	}
	else
	{
#ifdef USED_VLAN
		/* DHCP discover/request from WAN port, drop */
		if ((*(data+15) == 0x01) && (*(data+27) == 0x11) && (*(data+39) == 0x44))
			goto drop;
#endif
		if(!cta_eth_rx_filter(data))
			return 0;
	}
drop:
	dev_kfree_skb_any(skb);
	return 1;
}
#endif

/// @cond DOXYGEN
/*=============================================================*
 *  cta_eth_cansend:
 *=============================================================*/

/// @endcond
/*=============================================================*
 *  cheetah_ethernet_forward_skb_to_wifi:
 *=============================================================*/
 /*!
 * \return None
 */
void cheetah_ethernet_forward_skb_to_wifi (struct sk_buff *skb, struct net_device *dev, unsigned short vid )
{
	struct net_device *swap_dev;
	//struct sk_buff *skb2;
	if(vid == 0xfff)
	{
#if 0
	{
		int j;
		int dlen = (skb->len > MAX_PKT_DUMP)? MAX_PKT_DUMP : skb->len;
	
		printk(">>> %s skb->data=%x , len=%x", __func__, (int)skb->data, skb->len);
		for(j=0;j<dlen;j++)
		{
			if (0==(j&0x1f))
				printk("\n%03x:", j);
			printk(" %02x",skb->data[j]&0xff);
		}
		printk("\n");
	}
#endif	
		if_cta_eth_t *pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
		// broadcast forward to wlan
		if((0==compare_ether_addr(skb->data, broadcast_addr)))
		{
			if(pifcta_eth->if_swap)
			{
				//skb2 = skb_copy(skb, GFP_ATOMIC);
				swap_dev = pifcta_eth->if_swap;
				memmove((char *)skb->data+4 ,(char *)skb->data, 12); //remove vlan
				skb_pull(skb,4);
				printk("Swap interface name %s len:%d\n",swap_dev->name,skb->len);
				skb->dev = swap_dev; //wlan0
				dev_queue_xmit(skb); //let mac80211 to build its WiFi header
			}
		}
	}
}

#if defined(ENABLE_SGIO)

static int cta_eth_sgio_csum(struct sk_buff *skb, struct net_device *dev, u16 *sgio_csum)
{
    int length;

    int i, n;
    skb_frag_t *frag;

    struct iphdr *iph;
    //struct sk_buff *nskb;
    //struct tcphdr *th;

    int ret;
    short idx[MAX_SKB_FRAGS+1];
    int nr_frags = skb_shinfo(skb)->nr_frags;
    int first_chunk_size;
    dma_descriptor *pdma;

    *sgio_csum = 0xffff;

    if(nr_frags > MAX_SKB_FRAGS)
	goto drop_tx;


    if(skb->len > 1518)
    {
	printk("OZ\n");
	goto drop_tx;
    }

#if 1
    length = skb->len;
    printk("=> skb %p, skb data %p, total_len %d\n", skb, skb->data, skb->len);
    for(i=0;i<skb_shinfo(skb)->nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];
	printk("=> frag %d %p %p %d\n", i, (void*)page_address(frag->page) + frag->page_offset, frag, frag->size);
	length -= frag->size;
    }
    printk("=> first frag length %d\n", length);
#endif

    /* get first chunk size */
    first_chunk_size = skb->len;
    for(i=0;i<nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];
	first_chunk_size -= frag->size;
    }

    if (skb->protocol == htons(ETH_P_IP))
    {
	iph = ip_hdr(skb);
    
	if (iph->protocol == IPPROTO_TCP)
	{
	    if (skb->ip_summed == CHECKSUM_PARTIAL)
	    {
		if(first_chunk_size < ((skb->csum_start - skb_headroom(skb)) + skb->csum_offset))
		{
		    /* TCP checksum addr is not in first frag. ! */
		    printk("TODO: TCP checksum addr is not in first frag\n");
		    goto drop_tx;
		}

		ret = gdma_get_free_descr(idx, nr_frags+1);
		if(0 > ret)
		    goto drop_tx;

		dma_cache_wback((unsigned long) skb->data, first_chunk_size);

		pdma = &pdma_descr[idx[0]];

		pdma->cksum_init = 1;
		pdma->cksum_initval = 0;
		pdma->cksum_offset = 0;

		pdma->src_addr = PHYSICAL_ADDR(&skb->head[skb->csum_start]);
		pdma->operation_length = pdma->cksum_length = first_chunk_size - (skb->csum_start - skb_headroom(skb));

		pdma->operation = GDMA_OP_CSUM;
		pdma->ctrl = GDMA_CTRL_GO_POLL;

		gdma_kick();

		for(i=0;i<nr_frags;i++)
		{
		    frag = &skb_shinfo(skb)->frags[i];

		    dma_map_single(&dev->dev, (void*)page_address(frag->page) + frag->page_offset, frag->size, DMA_TO_DEVICE);

		    pdma = &pdma_descr[idx[i+1]];
	    
		    pdma->cksum_init = 0;
		    pdma->cksum_offset = 0;

		    pdma->src_addr = PHYSICAL_ADDR(((void *)page_address(frag->page) + frag->page_offset));
		    pdma->operation_length = pdma->cksum_length = frag->size;

		    pdma->operation = GDMA_OP_CSUM;
		    pdma->ctrl = GDMA_CTRL_GO_POLL;

		    gdma_kick();

		    first_chunk_size -= frag->size;
		}

		while(pdma_descr[idx[nr_frags]].ctrl != GDMA_CTRL_DONE_SKIP)
                {
		    for(n=0;n<64;n++) __asm__ volatile("nop;nop;nop");
		    //printk("c");
		}

		*sgio_csum = ~pdma_descr[idx[nr_frags]].cksum_result;
		printk(" SGIO TCP hardware partial CKSUM %04x\n", *sgio_csum);

		for(i=0;i<(nr_frags+1);i++)
		{
		    pdma_descr[idx[i]].sw_inuse = 0;
		}
	    }
	}
    }

    return 0;

drop_tx:
    return 0;
}


int cta_eth_sgio_send(struct sk_buff *skb, struct net_device *dev)
{
    txctrl_t *ptxc = &pcta_eth->tx;
    volatile txdesc_t *ptxd_start;
    unsigned short tx_head, tx_curr;
    unsigned char vidx=0;

    int i, n;
    skb_frag_t *frag;

    struct iphdr *iph;

    int txd_count = skb_shinfo(skb)->nr_frags + 1;

    int ret;
    short idx[MAX_SKB_FRAGS+1];
    int nr_frags = skb_shinfo(skb)->nr_frags;
    int first_chunk_size;
    dma_descriptor *pdma;
    volatile dma_descriptor *pending_dma = NULL;

    if(skb->len > 1518)
    {
	printk("OZ\n");
	goto drop_tx;
    }

    //printk(" %d ", nr_frags);
    if(nr_frags > MAX_SKB_FRAGS)
	goto drop_tx;

    /* get first chunk size */
    first_chunk_size = skb->len;
    for(i=0;i<nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];
	first_chunk_size -= frag->size;
    }

    if (skb->protocol == htons(ETH_P_IP))
    {

	if (skb->ip_summed == CHECKSUM_PARTIAL)
	{
	    if(first_chunk_size < ((skb->csum_start - skb_headroom(skb)) + skb->csum_offset))
	    {
		/* TCP checksum addr is not in first frag. ! */
		printk("TODO: TCP checksum addr is not in first frag\n");
		goto drop_tx;
	    }

	    ret = gdma_get_free_descr(idx, nr_frags+1);
	    if(0 > ret)
		goto drop_tx;

	    dma_cache_wback((unsigned long) skb->data, first_chunk_size);

	    pdma = &pdma_descr[idx[0]];

	    pdma->cksum_init = 1;
	    pdma->cksum_initval = 0;
	    pdma->cksum_offset = 0;

	    pdma->src_addr = PHYSICAL_ADDR(&skb->head[skb->csum_start]);
	    pdma->operation_length = pdma->cksum_length = first_chunk_size - (skb->csum_start - skb_headroom(skb));

	    pdma->operation = GDMA_OP_CSUM;
	    pdma->ctrl = GDMA_CTRL_GO_POLL;

	    gdma_kick();

	    for(i=0;i<nr_frags;i++)
	    {
		frag = &skb_shinfo(skb)->frags[i];

		dma_map_single(&dev->dev, (void*)page_address(frag->page) + frag->page_offset, frag->size, DMA_TO_DEVICE);

		pdma = &pdma_descr[idx[i+1]];
	
		pdma->cksum_init = 0;
		pdma->cksum_offset = 0;

		pdma->src_addr = PHYSICAL_ADDR(((void *)page_address(frag->page) + frag->page_offset));
		pdma->operation_length = pdma->cksum_length = frag->size;

		pdma->operation = GDMA_OP_CSUM;
		pdma->ctrl = GDMA_CTRL_GO_POLL;

		gdma_kick();
	    }

	    pending_dma = &pdma_descr[idx[nr_frags]];

	    #if 0
	    while(pdma_descr[idx[nr_frags]].ctrl != GDMA_CTRL_DONE_SKIP)
		printk("c");

	    *sgio_csum = ~pdma_descr[idx[nr_frags]].cksum_result;
	    printk(" SGIO TCP hardware partial CKSUM %04x\n", *sgio_csum);

	    for(i=0;i<(nr_frags+1);i++)
	    {
		pdma_descr[idx[i]].sw_inuse = 0;
	    }
	    #endif
	}
	else
	{
	    iph = ip_hdr(skb);
	    
	    if (iph->protocol == IPPROTO_TCP)
	    {	    
		printk(" SGIO TCP full csum not support\n");
		goto drop_tx;
	    }
	    else if (iph->protocol == IPPROTO_UDP)
	    {
		printk(" SGIO UDP full csum not support\n");
		goto drop_tx;
	    }
	    else
	    {
		printk(" SGIO NOT IP, not support\n");
		goto drop_tx;
	    }
	}
    }

    preempt_disable();

    if (ptxc->num_freetxd < txd_count)
    {
	preempt_enable();
	goto drop_skb_ok;
    }

    tx_head = ptxc->tx_head;
    ptxc->tx_head = (ptxc->tx_head + txd_count) % ptxc->num_txd;

    ptxc->num_freetxd -= txd_count;
    ptxc->tx_pktcnt++;

    preempt_enable();

    dev->trans_start = jiffies;
    skb->dev = dev;

    /* fill the head descriptor last, to avoid a race-condition */

    tx_curr = tx_head;
    for(i=0;i<skb_shinfo(skb)->nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];

	tx_curr = (tx_curr + 1) % ptxc->num_txd;

	ptxd_start = &ptxc->ptxd[tx_curr];
	ptxd_start->w1 = virtophy( (page_address(frag->page) + frag->page_offset) );

	if(i == (skb_shinfo(skb)->nr_frags - 1))
	{
	    /* last frag */
	    //ptxc->ptxbuf[tx_curr].sc = dev;
	    ptxc->ptxbuf[tx_curr].buf = (void *) skb;
	    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | (frag->size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF) | ((0x8|vidx)<<8);
	}
	else
	{
	    ptxc->ptxbuf[tx_curr].buf = (void *) 0;
	    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | DS0_M | (frag->size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF) | ((0x8|vidx)<<8);
	}
    }

    ptxd_start = &ptxc->ptxd[tx_head];
    ptxc->ptxbuf[tx_head].buf = (void *) 0;

    if(pending_dma)
    {
	while(pending_dma->ctrl != GDMA_CTRL_DONE_SKIP)
	{
	    for(n=0;n<64;n++) __asm__ volatile("nop;nop;nop");
	    //printk("c");
	}

	*((__sum16 *) NONCACHED_ADDR(&skb->head[skb->csum_start + skb->csum_offset])) = ~pending_dma->cksum_result;
	//dma_cache_wback((unsigned long) &skb->head[skb->csum_start + skb->csum_offset], 2);

	for(i=0;i<(nr_frags+1);i++)
	{
	    pdma_descr[idx[i]].sw_inuse = 0;
	}
    }

    ptxd_start->w1 = virtophy(skb->data);
    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | DS0_M | (first_chunk_size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF) | ((0x8|vidx)<<8);

    /*  Kick the hardware to start transmitting  */
    ETH_REG32(MTT) = 1;

    return NETDEV_TX_OK;

drop_tx:
    return NETDEV_TX_BUSY;

drop_skb_ok:
    ptxc->tx_drop++;
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static int ___cta_eth_sgio_send(struct sk_buff *skb, struct net_device *dev)
{
    txctrl_t *ptxc = &pcta_eth->tx;
    volatile txdesc_t *ptxd_start;
    unsigned short tx_head, tx_curr;
    unsigned char vidx=0;

    int i;
    skb_frag_t *frag;

    __wsum csum_val = 0;
    void *csum_addr;
    int csum_len;
    __sum16 tcp_checksum;

    //__sum16 sgio_csum;

    struct iphdr *iph;
    //struct sk_buff *nskb;
    //struct tcphdr *th;

    int txd_count = skb_shinfo(skb)->nr_frags + 1;
    int nr_frags = skb_shinfo(skb)->nr_frags;
    int first_chunk_size;

    if(skb->len > 1518)
    {
	printk("OZ\n");
	goto drop_tx;
    }

#if 0
    length = skb->len;
    printk("=> skb %x, skb data %x, total_len %d\n", skb, skb->data, skb->len);
    for(i=0;i<skb_shinfo(skb)->nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];
	printk("=> frag %d %x %x %d\n", i, (void*)page_address(frag->page) + frag->page_offset, frag, frag->size);
	length -= frag->size;
    }
    printk("=> first frag length %d\n", length);
#endif

    /* get first chunk size */
    first_chunk_size = skb->len;
    for(i=0;i<nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];
	first_chunk_size -= frag->size;
    }

#if 1
    if (skb->protocol == htons(ETH_P_IP))
    {
	iph = ip_hdr(skb);
    
	if (iph->protocol == IPPROTO_TCP)
	{
	    if (skb->ip_summed == CHECKSUM_PARTIAL)
	    {
		if(first_chunk_size < ((skb->csum_start - skb_headroom(skb)) + skb->csum_offset))
		{
		    /* TCP checksum addr is not in first frag. ! */
		    printk("unsupported SKB frag\n");
		    goto drop_tx;
		}

		csum_addr = &skb->head[skb->csum_start];
		csum_len = first_chunk_size - (skb->csum_start - skb_headroom(skb));
		csum_val = csum_partial(csum_addr, csum_len, csum_val);

		if(csum_len % 2)
		{
		    /* only the last frag is allowed to have odd numbers of data */
		    printk("SKB first frag has odd number in length\n");
		    goto drop_tx;
		}

		for(i=0;i<skb_shinfo(skb)->nr_frags;i++)
		{
		    frag = &skb_shinfo(skb)->frags[i];
		    csum_addr = page_address(frag->page) + frag->page_offset;
		    csum_len = frag->size;

		    if((csum_len % 2) && (i != (skb_shinfo(skb)->nr_frags - 1)))
		    {
			/* only the last frag is allowed to have odd numbers of data */
			printk("SKB frag has odd number in length\n");
			goto drop_tx;
		    }
		    else
		    {
			csum_val = csum_partial(csum_addr, csum_len, csum_val);
		    }
	
		    dma_map_single(&dev->dev, (void*)page_address(frag->page) + frag->page_offset, frag->size, DMA_TO_DEVICE);
		}
	
		tcp_checksum = csum_fold(csum_val);

		//cta_eth_sgio_csum(skb,dev,&sgio_csum);
		//printk("SGIO tcp_checksum %04x %04x\n", tcp_checksum, sgio_csum);

		*((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]) = tcp_checksum;
		dma_map_single(&dev->dev,(void *)skb->data, first_chunk_size, PCI_DMA_TODEVICE);
	    }
	    else
	    {
		goto drop_tx;
	    }
	}
	else
	{
	    goto drop_tx;
	}
    }
#endif

    preempt_disable();

    if (ptxc->num_freetxd < txd_count)
    {
	preempt_enable();
	goto drop_skb_ok;
    }

    tx_head = ptxc->tx_head;
    ptxc->tx_head = (ptxc->tx_head + txd_count) % ptxc->num_txd;

    ptxc->num_freetxd -= txd_count;
    ptxc->tx_pktcnt++;

    preempt_enable();

    dev->trans_start = jiffies;
    skb->dev = dev;

    /* fill the head descriptor last, to avoid a race-condition */

    tx_curr = tx_head;
    for(i=0;i<skb_shinfo(skb)->nr_frags;i++)
    {
	frag = &skb_shinfo(skb)->frags[i];

	tx_curr = (tx_curr + 1) % ptxc->num_txd;

	ptxd_start = &ptxc->ptxd[tx_curr];
	ptxd_start->w1 = virtophy( (page_address(frag->page) + frag->page_offset) );

	if(i == (skb_shinfo(skb)->nr_frags - 1))
	{
	    /* last frag */
	    //ptxc->ptxbuf[tx_curr].sc = dev;
	    ptxc->ptxbuf[tx_curr].buf = (void *) skb;
	    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | (frag->size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF | ((0x8|vidx)<<8));
	}
	else
	{
	    ptxc->ptxbuf[tx_curr].buf = (void *) 0;
	    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | DS0_M | (frag->size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF | ((0x8|vidx)<<8)) ;
	}
    }

    ptxd_start = &ptxc->ptxd[tx_head];
    ptxc->ptxbuf[tx_head].buf = (void *) 0;

    ptxd_start->w1 = virtophy(skb->data);
    ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING) | DS0_M | (first_chunk_size<<DS0_LEN_S) | (DS0_TX_OWNBIT |DS0_DF | ((0x8|vidx)<<8));

    /*  Kick the hardware to start transmitting  */
    ETH_REG32(MTT) = 1;

    return NETDEV_TX_OK;

drop_tx:
    return NETDEV_TX_BUSY;

drop_skb_ok:
    ptxc->tx_drop++;
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}
#endif

#if defined(ENABLE_TCPUDP_V4_CKSUM)

static inline volatile dma_descriptor *gdma_init_csum(u32 addr, u16 length, u16 initval)
{
    short idx;
    int ret;

    ret = gdma_get_free_descr(&idx, 1);
    if(0 > ret)
	return NULL;

    pdma_descr[idx].cksum_init = 1;
    pdma_descr[idx].cksum_initval = initval;
    pdma_descr[idx].cksum_offset = 0;
    pdma_descr[idx].cksum_length = length;

    pdma_descr[idx].src_addr = PHYSICAL_ADDR(addr);
    pdma_descr[idx].operation_length = length;
    pdma_descr[idx].operation = GDMA_OP_CSUM;

    pdma_descr[idx].ctrl = GDMA_CTRL_GO_POLL;

    gdma_kick();

    //while(pdma_descr[idx].ctrl != GDMA_CTRL_DONE_SKIP)
    //printk(".");
    //pdma_descr[idx].sw_inuse = 0;

    return &pdma_descr[idx];
}

//static inline void tcpudp_done_csum(struct sk_buff *skb, dma_descriptor *pdma)
static inline void tcpudp_done_csum(struct sk_buff *skb, dma_descriptor *pdma, u16 *sgio_cksum)
{
    struct tcphdr *th;
    struct udphdr *uh;
    struct iphdr *iph;
    int tcp_len;
    int udp_len;

    iph = ip_hdr(skb);

    while(pdma->ctrl != GDMA_CTRL_DONE_SKIP)
	printk(".");

    if(skb->ip_summed == CHECKSUM_PARTIAL)
    {
	//printk("ORGC %x\n",*((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]));
	*((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]) = ~pdma->cksum_result;
	//printk("NEWC %x\n",*((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]));
	if(sgio_cksum)
	{
	    *sgio_cksum = ~pdma->cksum_result;
	    printk("CSUM partial %04x ", *sgio_cksum);
	}

	dma_cache_wback((unsigned long) &skb->head[skb->csum_start + skb->csum_offset], 2);
    }
    else
    {
	if(iph->protocol == IPPROTO_TCP)
	{
	    th = tcp_hdr(skb);
	    tcp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

	    //printk("ORGTC %x, cksum_result %x\n", th->check, pdma->cksum_result);
	    th->check = csum_tcpudp_magic(iph->saddr, iph->daddr, tcp_len, IPPROTO_TCP, csum_partial(th,th->doff << 2, pdma->cksum_result));
	    //printk("NEWTC %x\n", th->check);
	    if(sgio_cksum)
	    {
		*sgio_cksum = th->check;
		printk("CSUM TCP %04x ", *sgio_cksum);
	    }

	    dma_cache_wback((unsigned long) &th->check, 2);
	}
	else if (iph->protocol == IPPROTO_UDP)
	{
	    uh = udp_hdr(skb);
	    udp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

	    //printk("ORGUC %x, cksum_result %x\n", uh->check, pdma->cksum_result);
	    uh->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len, IPPROTO_UDP, csum_partial(uh, sizeof(struct udphdr), pdma->cksum_result));
	    //printk("NEWUC %x\n", uh->check);
	    if(sgio_cksum)
	    {
		*sgio_cksum = uh->check;
		printk("CSUM UDP %04x ", *sgio_cksum);
	    }

	    dma_cache_wback((unsigned long) &uh->check, 2);
	}
    }

    pdma->sw_inuse = 0;
}

static inline dma_descriptor *tcpudp_init_csum(struct sk_buff *skb)
{
    struct iphdr *iph;
    struct tcphdr *th;
    struct udphdr *uh;
    int tcp_len;
    int udp_len;
    //__sum16 checksum;
    u8 *transport_header;

    volatile dma_descriptor *pdma = NULL;

    iph = ip_hdr(skb);

    if(skb->ip_summed == CHECKSUM_PARTIAL)
    {
	return (dma_descriptor *) gdma_init_csum((u32) &skb->head[skb->csum_start], (skb->len - (skb->csum_start - skb_headroom(skb))), 0);
    }

    transport_header = skb_transport_header(skb);
    if(transport_header==NULL)
	return NULL;

    if(iph->protocol == IPPROTO_TCP)
    {
	th = (struct tcphdr *) transport_header;

	tcp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

	pdma = gdma_init_csum((u32) ((unsigned char *)th) + (th->doff << 2), tcp_len - (th->doff << 2), 0);

	th->check = 0;
    }
    else if (iph->protocol == IPPROTO_UDP)
    {
	uh = (struct udphdr *) transport_header;

        udp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

	pdma = gdma_init_csum((u32) ((unsigned char *)uh) + sizeof(struct udphdr), udp_len - sizeof(struct udphdr), 0);

	uh->check = 0;
    }

    return (dma_descriptor *) pdma;
}

#if 0
u16 gdma_csum(u32 addr, u16 length, u16 initval)
{
    short idx;
    int ret;

    ret = gdma_get_free_descr(&idx, 1);
    if(0 > ret)
	return 0xffff;

    pdma_descr[idx].cksum_init = 1;
    pdma_descr[idx].cksum_initval = initval;
    pdma_descr[idx].cksum_offset = 0;
    pdma_descr[idx].cksum_length = length;

    pdma_descr[idx].src_addr = PHYSICAL_ADDR(addr);
    pdma_descr[idx].operation_length = length;
    pdma_descr[idx].operation = GDMA_OP_CSUM;

    pdma_descr[idx].ctrl = GDMA_CTRL_GO_POLL;

    gdma_kick();

    while(pdma_descr[idx].ctrl != GDMA_CTRL_DONE_SKIP)
	printk(".");

    pdma_descr[idx].sw_inuse = 0;

    return pdma_descr[idx].cksum_result;
}

static inline void tcpudp_csum(struct sk_buff *skb)
{
    struct iphdr *iph;
    struct tcphdr *th;
    struct udphdr *uh;
    int tcp_len;
    int udp_len;
    __sum16 tcp_checksum;
    __sum16 udp_checksum;
    u8 *transport_header;

    iph = ip_hdr(skb);

    transport_header = skb_transport_header(skb);
    //if(transport_header==NULL)
    //	transport_header = &iph[(be16_to_cpu(iph->ihl) << 2)]; 

#if 1 // check this later, strange broadcast packet to WAN port with no transport header ptr
    if ((iph->protocol == IPPROTO_TCP) && (NULL==tcp_hdr(skb)))
    {
	#if 0
	int k;
	printk("NULL TCP HDR, skb %x, handle it!!!\n", skb);
	printk("------------------\n");
	for(k=0;k<skb->len;k++)
	{
	    printk(" %02x", skb->data[k]);
	}
	printk("------------------\n");
	#endif

	return;
    }

    if ((iph->protocol == IPPROTO_UDP) && (NULL==udp_hdr(skb)))
    {
	#if 0
	int k;
	printk("NULL UDP HDR, skb %x, handle it!!!\n", skb);
	printk("------------------\n");
	for(k=0;k<skb->len;k++)
	{
	    printk(" %02x", skb->data[k]);
	}
	printk("------------------\n");
	#endif

	return;
    }
#endif

    if(iph->protocol == IPPROTO_TCP)
    {
	th = (struct tcphdr *) transport_header;

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
        tcp_checksum = th->check;
        printk("#TCP ORG# SKB CKSUM:%x, TCP CKSUM:%x, skb_len:%d\n", skb->csum, tcp_checksum, skb->len);
#endif

        if (skb->ip_summed == CHECKSUM_PARTIAL)
        {
#if 0
            // partial csum is completed; we only do the left part indicated by csum_start & csum_offset
#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#TCP PAR# csum_start %d, headroom %d, offset %d\n", skb->csum_start, skb_headroom(skb), skb->csum_offset);
#endif

            tcp_checksum = csum_fold(csum_partial(&skb->head[skb->csum_start], (skb->len - (skb->csum_start - skb_headroom(skb))), 0));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#TCP PAR# SW TCP CKSUM:%x\n", tcp_checksum);
#endif
#endif
	    tcp_checksum = ~gdma_csum(&skb->head[skb->csum_start], (skb->len - (skb->csum_start - skb_headroom(skb))), 0);
#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#TCP PAR# HW TCP CKSUM:%x\n", tcp_checksum);
#endif
            *((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]) = tcp_checksum;

#if 0 // the full re-compute method
            tcp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);
            skb->csum = csum_partial(((unsigned char *)th) + (th->doff << 2), tcp_len - (th->doff << 2), 0);

            th->check = 0;

            th->check = csum_tcpudp_magic(iph->saddr, iph->daddr, tcp_len, IPPROTO_TCP, csum_partial(th,th->doff << 2, skb->csum));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#TCP FULL# SKB CKSUM:%x, TCP CKSUM:%x\n", skb->csum, th->check);
#endif
#endif
	    DC_FLUSH_TX((void *) &skb->head[skb->csum_start + skb->csum_offset], 2);
        }
	else
        {
            tcp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            skb->csum = csum_partial(((unsigned char *)th) + (th->doff << 2), tcp_len - (th->doff << 2), 0);

	    printk("#TCP FULL# SKB CKSUM: %x %x\n", skb->csum, gdma_csum(((unsigned char *)th) + (th->doff << 2), tcp_len - (th->doff << 2), 0));
#endif
	    skb->csum = gdma_csum(((unsigned char *)th) + (th->doff << 2), tcp_len - (th->doff << 2), 0);

            th->check = 0;

            th->check = csum_tcpudp_magic(iph->saddr, iph->daddr, tcp_len, IPPROTO_TCP, csum_partial(th,th->doff << 2, skb->csum));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#TCP FULL# SKB CKSUM:%x, SW TCP CKSUM:%x\n", skb->csum, th->check);
#endif
	    DC_FLUSH_TX(&th->check, 2);
        }
    }
    else if (iph->protocol == IPPROTO_UDP)
    {
	uh = (struct udphdr *) transport_header;

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
        udp_checksum = uh->check;
        printk("#UDP ORG# SKB CKSUM:%x, UDP CKSUM:%x, skb_len:%d\n", skb->csum, udp_checksum, skb->len);
#endif

        if (skb->ip_summed == CHECKSUM_PARTIAL)
        {
#if 0
            // partial csum is completed; we only do the left part indicated by csum_start & csum_offset
            udp_checksum = csum_fold(csum_partial(&skb->head[skb->csum_start], (skb->len - (skb->csum_start - skb_headroom(skb))), 0));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#UDP PAR# SW UDP CKSUM:%x\n", udp_checksum);
#endif
#endif

	    udp_checksum = ~gdma_csum(&skb->head[skb->csum_start], (skb->len - (skb->csum_start - skb_headroom(skb))), 0);
#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#UDP PAR# HW UDP CKSUM:%x\n", udp_checksum);
#endif

            *((__sum16 *) &skb->head[skb->csum_start + skb->csum_offset]) = udp_checksum;

#if 0
            udp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

            skb->csum = csum_partial(((unsigned char *)uh) + sizeof(struct udphdr), udp_len - sizeof(struct udphdr), 0);

            uh->check = 0;

            uh->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len, IPPROTO_UDP, csum_partial(uh, sizeof(struct udphdr), skb->csum));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#UDP FULL# SKB CKSUM:%x, UDP CKSUM:%x\n", skb->csum, uh->check);
#endif
#endif
	    DC_FLUSH_TX((void *) &skb->head[skb->csum_start + skb->csum_offset], 2);
        }
	else
        {
            udp_len = be16_to_cpu(iph->tot_len) - (be16_to_cpu(iph->ihl) << 2);

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            skb->csum = csum_partial(((unsigned char *)uh) + sizeof(struct udphdr), udp_len - sizeof(struct udphdr), 0);

	    printk("#UDP FULL# SKB CKSUM: %x %x\n", skb->csum, gdma_csum(((unsigned char *)uh) + sizeof(struct udphdr), udp_len - sizeof(struct udphdr), 0));
#endif
	    skb->csum = gdma_csum(((unsigned char *)uh) + sizeof(struct udphdr), udp_len - sizeof(struct udphdr), 0);

            uh->check = 0;

            uh->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len, IPPROTO_UDP, csum_partial(uh, sizeof(struct udphdr), skb->csum));

#if defined(TCPUDP_V4_CKSUM_VERBOSE)
            printk("#UDP FULL# SKB CKSUM:%x, UDP CKSUM:%x\n", skb->csum, uh->check);
#endif
	    DC_FLUSH_TX(&uh->check, 2);
        }
    }
    else if (iph->protocol == IPPROTO_ICMP)
    {
#if defined(TCPUDP_V4_CKSUM_VERBOSE)
        printk("#ICMP# SKB CKSUM:%x, ICMP CKSUM:%x\n", skb->csum, icmp_hdr(skb)->checksum);
#endif
    }
    else
    {
#if defined(TCPUDP_V4_CKSUM_VERBOSE)
        printk("Not supported checksum offload protocol %x\n", iph->protocol);
#endif
    }
}
#endif

#endif

/*!-----------------------------------------------------------------------------
 *        eth_send:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int eth_send(struct sk_buff *skb, struct net_device *dev )
{
	/*	This function transmits the given frame into the DMA engine of the
		Ethernet controller. If the transmit data path is enable, the frame
		will be transmit at once.
		sc: the pointer for the soft control of Ethernet driver.
		sg_list: the packet which is specified via a ¡§scatter-gather¡¨ list.
		sg_len: number of packet scatter in the list.
		total_len: the packet total length.
		key: the packet address for upper layer rele
	*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	if_cta_eth_t *pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
#else
	if_cta_eth_t *pifcta_eth = (if_cta_eth_t *) dev->priv;
#endif
	txctrl_t *ptxc = &pcta_eth->tx;
	volatile txdesc_t *ptxd_start;
	int total_len = skb->len; 
	struct sk_buff *nskb;
	unsigned char vidx=0;
	unsigned short vid ;
	semi_cb_t *semi_cb = SKB2SEMICB(skb);

	unsigned short tx_head;
#if defined(ENABLE_TCPUDP_V4_CKSUM)
	dma_descriptor *pdma = NULL;
#endif

#if 1
	int sgio = 0;
	u16 sgio_cksum;
	u16 sgio_cksum1;
#endif

#if defined(ENABLE_SGIO)
	if(skb_shinfo(skb)->nr_frags)
	{
	#if defined(ENABLE_SGIO_JUST_COPY_SKB)
	    nskb = skb_copy(skb, GFP_KERNEL | GFP_DMA);
	    if(!nskb)
		    goto drop_tx;

	    cta_eth_sgio_csum(skb, dev, &sgio_cksum1);

	    dev_kfree_skb(skb);
	    skb = nskb;

	    sgio = 1;
	#else
	    //if(NETDEV_TX_OK == ___cta_eth_sgio_send(skb, dev))
	    if(NETDEV_TX_OK == cta_eth_sgio_send(skb, dev))
	    {
		//printk("SGIO OK\n");
		return NETDEV_TX_OK;
	    }
	    else
	    {
printk("SGIO FAILED\n");
		nskb = skb_copy(skb, GFP_KERNEL | GFP_DMA);
		if(!nskb)
			goto drop_tx;
		dev_kfree_skb(skb);
		skb = nskb;
	    }
	#endif
	}
#endif

	CYG_ASSERT(pifcta_eth->sc == dev, "Invalid device pointer");

	DBGPRINTF(DBG_TX, "eth_send(%d) key=%08x, len=%d\n",
			 pifcta_eth->unit, (unsigned int)skb, total_len);

#ifdef BRIDGE_FILTER
	if ( cta_eth_filter(skb, skb->data, 1) )
			return 0;
#endif
	if (DBG_TXMORE2 & DBG_MASK)
	{
		int j;
		int dlen = (skb->len > MAX_PKT_DUMP)? MAX_PKT_DUMP : skb->len;
	
		printk(">>> %s skb->data=%x , len=%x", __func__, (int)skb->data, skb->len);
		for(j=0;j<dlen;j++)
		{
			if (0==(j&0x1f))
				printk("\n%03x:", j);
			printk(" %02x",skb->data[j]&0xff);
		}
		printk("\n");
	}

#if defined(ENABLE_TCPUDP_V4_CKSUM)
	dma_cache_wback((unsigned long) skb->data, skb->len);
	if(skb->protocol == htons(ETH_P_IP))
	    pdma = tcpudp_init_csum(skb);
#endif

	preempt_disable();
	if (ptxc->num_freetxd == 0 || total_len > 1518)
	{
		preempt_enable();
		goto drop_tx;
	}

	tx_head = ptxc->tx_head;

	--ptxc->num_freetxd;
	if (++ptxc->tx_head == ptxc->num_txd)
		ptxc->tx_head = 0;

	ptxc->tx_pktcnt ++;

	preempt_enable();

	if (skb_vlan_tag_present(skb)>=0)
	{	
		vid = skb_vlan_tag_get(skb);
		vidx = vid_to_vidx(vid);
	}

#if !defined(REMOVE_HNAT)
	if(!IS_HWMOD(semi_cb))
	{
		cta_hnat_sdmz_tx_check(skb->data);
		cta_hnat_learn_mac(skb->data, 1, vidx);
	}
#endif

#if defined(ENABLE_TCPUDP_V4_CKSUM)
    if(pdma)
    {
	if(sgio)
	    tcpudp_done_csum(skb, pdma, &sgio_cksum);
	else
	    tcpudp_done_csum(skb, pdma, NULL);
    }
    if(sgio)
    {
	printk("C(%04x) (%04x)\n", sgio_cksum, sgio_cksum1);
	if(sgio_cksum!=sgio_cksum1)
	{
	    printk("ERROR!!!\n");
	}
    }
#else
    DC_FLUSH_TX((void *)skb->data, skb->len);
#endif

    dev->trans_start = jiffies;
	skb->dev = dev;
#ifdef TX_LEN_PADDING
	if (total_len < ETH_MIN_FRAMELEN)
	{
		/* 
		 * XXX:
		 */
		sg_list[sg_len - 1].len += ETH_MIN_FRAMELEN - total_len;
		total_len = ETH_MIN_FRAMELEN;
	}
#endif

		ptxd_start = &ptxc->ptxd[tx_head];

		ptxc->ptxbuf[tx_head].sc = dev;
		ptxc->ptxbuf[tx_head].buf = (void *) skb;

		/*  Use one buffers per descriptor  */
		if(IS_HWMOD(semi_cb))
		{
			ptxd_start->w1 = semi_cb->w1;
			ptxd_start->w0 = semi_cb->w0 |((0x8|vidx)<<8) | (ptxd_start->w0 & DS0_END_OF_RING) ;
		}
		else
		{
			ptxd_start->w1 = virtophy(skb->data);
			ptxd_start->w0 = (ptxd_start->w0 & DS0_END_OF_RING)
						| (skb->len<<DS0_LEN_S)
			 			| (DS0_TX_OWNBIT |DS0_DF) 
						| ((0x8|vidx)<<8);
		}
		/*  Kick the hardware to start transmitting  */
		ETH_REG32(MTT) = 1;

	/* Currently do not use local forwarding */
	//cheetah_ethernet_forward_skb_to_wifi(skb,dev,vid);

	return 0;

drop_tx:
	/* No tx resource for this frame */
	DBGPRINTF(DBG_TX, "eth_send(%d) drop frame %08lx\n",
			 pifcta_eth->unit, (unsigned long)skb);
	/* Drop the frame now */
	ptxc->tx_drop++;
	return -EBUSY;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_deliver:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_deliver(struct net_device *sc)
{
	unsigned int int_status;

	int_status = ETH_REG32(MIS);
	/* Ack all interrupts */
	ETH_REG32(MIC) = int_status;

	DBGPRINTF(DBG_DELIVER, "cta_eth_deliver. istatus=%x\n",
				int_status);
	do{
		if(int_status & MIS_SQE)
		{
			//DBGPRINTF(DBG_INIT, "SQE\n\n");
			ETH_REG32(MAUTO)|=(1<<2);
		}
		if(int_status & MIS_HQE)
		{
			//DBGPRINTF(DBG_INIT, "HQE\n\n");
		}

		if(int_status & MIE_BE)
		{
			//DBGPRINTF(DBG_INIT, "BE\n\n");
			ETH_REG32(MAUTO)|=(1<<0);
		}

		if(int_status & MIS_TX)
		{
			cta_eth_txint(&pcta_eth->tx, 0);
		}

		if(int_status & MIS_RX)
		{
			cta_eth_rxint(&pcta_eth->rx);
		}
	} while(0);
#ifdef	USED_DSR
	/*
		in case of something pending to do,
		re-schedule next tasklet to continue
	*/
	int_status=ETH_REG32(MIS);
	if(int_status & (MIM_INIT))
	{
		/* we re-schedule a tasklet later */
		tasklet_hi_schedule(&tasklet);
	}
	else
#endif	
	ETH_REG32(MIM)&= ~MIM_INIT;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_flowctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_flowctrl(int onoff)
{
	if(pcta_eth->flowctrl_mode == 0)
		return;
	if(onoff)
	{
		/* send pause frame */
		ETH_REG32(MFC1) = 0x2;
		pcta_eth->flowctrl_state = 1;
		diag_printf("send pause!!%x\n", ETH_REG32(MFC1));
	}
	else if(pcta_eth->flowctrl_state)
	{
		/* send clear pause frame */
		ETH_REG32(MFC1) = 0x4;
		pcta_eth->flowctrl_state = 0;
		diag_printf("send clear pause!!\n");
	}
}

/*!-----------------------------------------------------------------------------
 *        eth_drv_recv1:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int eth_drv_recv1(struct net_device *dev, unsigned char *data, int len,
					char **nbufp, void *hw_nat, unsigned int w0, unsigned int w1)
{
	struct sk_buff *skb;
	unsigned int * buf = *((unsigned int**)nbufp);
	int rc;

	skb = (struct sk_buff*) buf[BUF_SW_OFS/4];
#if 1 //DEBUG
	if(0x08 != (((unsigned int)skb)>>28))
	{
		diag_printf("%s: skb is lost: %x\n",__func__, (unsigned int)skb);
		return -1;
	}
#endif

	struct sk_buff *skb2;
	skb->data = skb->tail = data;
	skb_put(skb,len);

//	printk("\n");

	if(DBG_RXMORE2 & DBG_MASK)
	{
		int j;
		int dlen = (skb->len > MAX_PKT_DUMP)? MAX_PKT_DUMP : skb->len;
		printk("<<< %s skb->data=%x , len=%x", __func__, (int)skb->data, skb->len);

		for(j=0;j<dlen;j++)
		{
				if(0==(j&0x1f))
					printk("\n%03x:", j);
				printk(" %02x",skb->data[j]&0xff);
		}
		printk("\n");
	}

#ifdef BRIDGE_FILTER
	if(cta_eth_filter(skb, data, 0))
	{
		return 0;
	}
#endif

	/* Use this for ether to WiFi */
#if 0
    if(arthur_sc)
    {
        if(NET_RX_DROP==arthur_forward_eth_skb(arthur_sc, skb))
            {
        dev_kfree_skb(skb);
        return 0;
        }
    }
#endif

#ifdef CONFIG_BRIDGE_
	rc = br_handle_recv_pkt(skb,dev);
#else
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);    /* set packet type */
#if !defined(REMOVE_HNAT)
	cta_hnat_set_semicb((unsigned long)SKB2SEMICB(skb), hw_nat, w0, w1);
#endif

#if defined(TRIAL_RX_CKSUM_OFFLOAD)
    skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif

	rc = netif_rx(skb);
#endif	
#if 0
//	skb2 = skb_copy(skb, GFP_ATOMIC);
	unsigned int vid;
	vid = (unsigned int)skb->data; //only lan vid can forward to wlan; need to modify
	cheetah_ethernet_forward_skb_to_wifi(skb2,dev,*(unsigned short *)vid);
#endif
	return rc;
}

/*!-----------------------------------------------------------------------------
 *        eth_drv_getbuf:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
char *eth_drv_getbuf()
{
	struct sk_buff *skb;
	
	skb = __dev_alloc_skb(1664, GFP_ATOMIC | GFP_DMA);
	
	if(NULL==skb)   return NULL;

	skb->data = (unsigned char*)(((unsigned int)(skb->data+0x3f)) & ~0x3f);
	*((unsigned int*) skb->data) = (unsigned int)skb ;
	DC_FLUSH_TX(skb->data, sizeof(unsigned int));

	return skb->data;
}

/*!-----------------------------------------------------------------------------
 *        eth_drv_freebuf:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void eth_drv_freebuf(char *buf)
{
	unsigned int *wbuf;
	struct sk_buff *skb;

	wbuf = (unsigned int*)(((unsigned int)buf) & ~0x3f);
	if(0x08 ==(wbuf[BUF_SW_OFS>>2]>>28))
	{
		skb = (struct sk_buff*)wbuf[BUF_SW_OFS>>2];
		kfree_skb(skb);
	}
	else
	{
		printk("%s , skb head missing:%x , buf=%x\n",
				__func__, wbuf[BUF_SW_OFS>>2], (unsigned int)buf);
	}
}

/*!-----------------------------------------------------------------------------
 *        an_cmd:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int an_cmd(short phyno, unsigned short val, unsigned short cap)
{
	unsigned short phy, ctl=0;

	if(cap & (PHY_CAP_AN|PHY_CAP_100F|PHY_CAP_10F|PHY_CAP_100F|PHY_CAP_100H))
	{
		phy = cta_eth_mdio_rd(phyno, 0);			// ctrl
		if(cap & PHY_CAP_AN)
			ctl |= PHYR_CTL_AN;
		if(cap & (PHY_CAP_10F))
			ctl |= PHYR_CTL_DUPLEX;
		if(cap & (PHY_CAP_100H))
			ctl |= PHYR_CTL_SPEED;
		if(val)
			phy |= ctl;
		else
			phy &= ~ctl;
		cta_eth_mdio_wr(phyno, 0, phy);
	}
	if(cap & (PHY_CAP_PAUSE|PHY_CAP_100F|PHY_CAP_10F|PHY_CAP_100F|PHY_CAP_100H))
	{
		phy = cta_eth_mdio_rd(phyno, 4);			// mine
		if(val)
			phy |= cap;
		else
			phy &= ~cap;
		cta_eth_mdio_wr(phyno, 4, phy);
	}

	{
#if defined(CONFIG_CHEETAH_FPGA)
		unsigned int id;
		if((id=cta_switch_phyid(phyno))==PHYID_MONT_EPHY)
			cta_eth_phy_reset(phyno, id);
		else
#endif
		{
			phy = cta_eth_mdio_rd(phyno, 0);
			cta_eth_mdio_wr(phyno, 0, phy | 0x200);	// AN
		}
	}
	return CLI_OK;
}

/*!-----------------------------------------------------------------------------
 *        phy_status:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
short phy_status(short phyno)
{
	short ret = 0;
	short phy,ctl,sts;

	sts = cta_eth_mdio_rd( phyno, 1);

	if(sts & PHYR_ST_AN)
		ret |= PHY_AN;

	if(sts & PHYR_ST_LINK)
		ret |= PHY_LINK;
	else
	{
#if 1
		/* To debug */
		printk(KERN_DEBUG "%s: phyno(%d) sts(%x) debug(%x)\n", __func__, phyno, sts, cta_eth_mdio_rd(phyno, 1));
#endif
		goto ret_;
	}
	/* AN Complete */
	if(sts & PHYR_ST_ANC)
	{
		ret |= PHY_ANC;
		phy = cta_eth_mdio_rd(phyno , 4);		// mine
		ctl = cta_eth_mdio_rd(phyno , 5);		// link parter
		if((cta_switch_phyid(phyno)==PHYID_MONT_EPHY) && (ctl==0))
		{
			sts = cta_eth_mdio_rd(phyno, 31);
			sts &= 0x7;
			/* Swap to page 0 */
			cta_eth_mdio_wr(phyno, 31, 0);
			ctl = cta_eth_mdio_rd(phyno , 19);	// result of parallel detection
			cta_eth_mdio_wr(phyno, 31, sts);
			if( ctl & 0x10)
				ret |= PHY_100M;
			if( ctl & 0x8)
				ret |= PHY_FDX;
		}
		else
		{
			phy = 0x3e0 & (phy & ctl);
			if(PHY_CAP_100F & phy)
				ret |= (PHY_100M|PHY_FDX);
			else
			if(PHY_CAP_100H & phy)
				ret |= (PHY_100M);
			else
			if(PHY_CAP_10F & phy)
				ret |= (PHY_FDX);
		}
	}
	else // force
	{
		ctl = cta_eth_mdio_rd( phyno, 0);
		if(ctl & PHYR_CTL_SPEED)
			ret |= PHY_100M;
		if(ctl & PHYR_CTL_DUPLEX)
			ret |= PHY_FDX;
	}
ret_:
	return ret;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_link_status:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
short cta_eth_link_status(int phy)
{
	short status = 0, link = 0;

	status = cta_eth_mdio_rd(phy, 1);
//	DBG_MSG(DEBUG_PHY, "port(%d) status=%x\n", phy, (unsigned int) status);
	link = status & (1 << 2);
	return (link?1 : 0);
}

/*!-----------------------------------------------------------------------------
 *        mdio_cmd:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int mdio_cmd(int argc, char *argv[])
{
	int set = 0;
	int adr = 0;
	int reg;
	int val;

	if(argc < 1)
		goto help;
	sscanf(argv[0], "%d", &adr);
	if(argc > 1)
	{
		if(1 != sscanf(argv[1], "%x", &reg))
			goto help;
	}
	else
	{
		printk("phy%d regs", adr);
		for(reg = 0; reg < 32; reg++)
		{
			val = cta_eth_mdio_rd(adr, reg) & 0xffff;
			if ((reg % 8) == 0)
				printk("\n%02X: ", reg);
			printk("%04x ", val);
		}
		printk("\n");
		return 0;
	}

	if(argc > 2)
	{
		if(1 != sscanf(argv[2], "%x", &val))
			goto help;
		cta_eth_mdio_wr(adr, reg, val);
		set++;
	}
	else
	{
		val = cta_eth_mdio_rd(adr, reg) & 0xffff;
	}
	printk("%sphy%d reg 0x%02x=%04x\n", set ? "SET " : "", adr, reg, val);
	return 0;
help:
	printk("mdio [adr] [reg] [val]\n\r");
	return -1;
}

/*!-----------------------------------------------------------------------------
 *        eth_cal_txrx:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int eth_cal_txrx(unsigned int idx, unsigned int txclk, unsigned int rxclk)
{
	cta_switch_txrx_ctrl(idx, txclk, rxclk);
	cta_switch_port_attach(idx, 0, 1);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_8021q_ctrl:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_eth_8021q_ctrl(unsigned int val)
{
	int vid, idx_8021q, sll, ofs, msk;

	/* Set vlan table of hnat */
	vid = (val & MASK_8021Q_VID) >> 16;
	idx_8021q = (val & MASK_BSSID_IDX) >> 8;
	if(val & FLG_8021Q_OUTB)
		vid |= (1<<12);

	//ETH_DHWV(idx_8021q) = (unsigned short)vid;					/* store in hw vlan table */
	sll = (idx_8021q%2)?16:0; 									/* shift or not */
	msk = (idx_8021q%2)?0xffff0000:0xffff; 						/* mask MSB or LSB */
	ofs = (idx_8021q/2)<<2;										/* offset */

	MREG((MVL01 + ofs)) &= ~msk;
	MREG((MVL01 + ofs)) |= (vid<<sll);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned int cta_eth_phy_init(unsigned char p)
{
	unsigned int id;

	id = (cta_eth_mdio_rd(p, 2) << 16) | cta_eth_mdio_rd(p, 3);
	if(id==PHYID_ICPLUS_IP101G)
	{
		/* Config IP101G to version 1.2 */
		cta_eth_mdio_wr(p, 20, 16);
		cta_eth_mdio_wr(p, 16, 0x1006);
#if defined(CONFIG_CHEETAH_FPGA)
		/* Config IP101G RXC driving current = 12.96mA */
		cta_eth_mdio_wr(p, 20, 4);
		cta_eth_mdio_wr(p, 22, 0xa000);
		/* Config IP101G TXC driving current = 12.96mA */
		cta_eth_mdio_wr(p, 20, 16);
		cta_eth_mdio_wr(p, 27, 0x0015);
		/* Config IP101G RXD driving current = 12.96mA */
		cta_eth_mdio_wr(p, 20, 16);
		cta_eth_mdio_wr(p, 26, 0x5b6d);
#endif
	}
	else if(id==PHYID_MONT_EPHY)
	{
#if defined(CONFIG_CHEETAH_FPGA)
		/* Swap to page 2 */
		cta_eth_mdio_wr(p, 31, 2);
		/* DSP initial val */
		cta_eth_mdio_wr(p, 18, 0x8975);
		cta_eth_mdio_wr(p, 19, 0xba60);
#else
		/* Swap to page 1 */
		cta_eth_mdio_wr(p, 31, 1);
		cta_eth_mdio_wr(p, 16, 0xb7e0);
		/* AFE tx control: enable termination impedance calibration */
		cta_eth_mdio_wr(p, 17, 0xa528);
#if defined(CONFIG_EPHY_GLITCH_PATCH)
		cta_eth_mdio_wr(p, 18, (cta_eth_mdio_rd(p, 18)&0x07ff));
		cta_eth_mdio_wr(p, 18, (cta_eth_mdio_rd(p, 18)|0x1800));
#endif
		cta_eth_mdio_wr(p, 19, 0xa4d8);
		/* AFE rx control */
		cta_eth_mdio_wr(p, 20, 0x3780);
		/* ADC VCM = 1.00 */
		cta_eth_mdio_wr(p, 22, (cta_eth_mdio_rd(p, 22)|0x6000));
#if defined(CONFIG_EPHY_GLITCH_PATCH)
		cta_eth_mdio_wr(p, 23, (cta_eth_mdio_rd(p, 23)&0xe01f));
		cta_eth_mdio_wr(p, 23, (cta_eth_mdio_rd(p, 23)|0x1500));
#endif
		/* Swap to page 2 */
		cta_eth_mdio_wr(p, 31, 2);
 		/* AGC thresholds, org=0x4030 */
 		cta_eth_mdio_wr(p, 17, 0x8059);
		/* DSP initial val */
		cta_eth_mdio_wr(p, 18, 0x8975);
		cta_eth_mdio_wr(p, 19, 0xba60);
 		/* Swap to page 4 */
 		cta_eth_mdio_wr(p, 31, 4);
 		/* 10T signal detection time control, org=0x5aa0 */
 		cta_eth_mdio_wr(p, 18, 0x5a40);
#endif
		/* Swap to page 0 */
		cta_eth_mdio_wr(p, 31, 0);
		/* RMII V1.2 */
		cta_eth_mdio_wr(p, 19, (cta_eth_mdio_rd(p, 19)|0x0040));
		/* Enable MDIX */
		cta_eth_mdio_wr(p, 20, (cta_eth_mdio_rd(p, 20)|0x3000));
	}
	else
	{
		printk("%s(%d): Unknow phy id:%x p:%d\n", __func__, __LINE__, id, p);
	}
	/* Restore control and capability */
	cta_eth_mdio_wr(p, 4, cta_switch_phybcap(p));
	cta_eth_mdio_wr(p, 0, cta_switch_phyctrl(p)|0x8000);
	return id;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_loopback:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_eth_phy_loopback(unsigned int p, unsigned int action)
{
	unsigned short val;

	val = cta_eth_mdio_rd(p, 0);
	if(action)
	{
		/* Enable loopback, disable AN, force 100M full mode */
		val |= 0x6100;
		val &= ~0x1000;
		cta_eth_mdio_wr(p, 0, val);
	}
	else
	{
		/* Disable loopback, restore capability and restart AN */
		val &= ~0x6100;
		val |= (0x200 | cta_switch_phyctrl(p));
		cta_eth_mdio_wr(p, 0, val);
	}
}

/*!-----------------------------------------------------------------------------
 *        eth_cmd:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int eth_cmd(int argc, char *argv[])
{
	char *val;

	if(argc < 1)
	{
		goto done;
	}
	
	if(!strcmp(argv[0], "mdio"))
	{
		mdio_cmd(argc-1, &argv[1]);
		goto done;
	}
	if(!strcmp(argv[0], "loopback"))
	{
		unsigned int p;
		if (argc < 3)
		{
			goto done;
		}
		sscanf(argv[1], "%d", &p);

		if(!strcmp(argv[2], "start"))
			cta_eth_phy_loopback(p,1);
		else
		if(!strcmp(argv[2], "stop"))
		{
			cta_eth_phy_loopback(p,0);
			cta_switch_port_detach(p);
		}

		goto done;
	}
	if(!strcmp(argv[0], "calibrate"))
	{
		unsigned int port;
		unsigned int txclk;
		unsigned int rxclk;
		if (argc < 4)
		{
			goto done;
		}
		sscanf(argv[1], "%d", &port);
		sscanf(argv[2], "%d", &txclk);
		sscanf(argv[3], "%d", &rxclk);
		eth_cal_txrx(port,txclk,rxclk);
		goto done;
	}
	if(!strcmp(argv[0], "sclk"))
	{
		cta_switch_sclk_ctrl();
		goto done;
	}
	if(!strcmp(argv[0], "an"))
	{
		cta_an();
		goto done;
	}
	if(!strcmp(argv[0], "8021q"))
	{
		unsigned int data;
		sscanf(argv[1], "%x", &data);
		cta_eth_8021q_ctrl(data);
		cta_switch_8021q_ctrl(data);
		goto done;
	}
	if(!strcmp(argv[0], "bssid"))
	{
		unsigned int data;
		sscanf(argv[1], "%x", &data);
		cta_switch_bssid_ctrl(data);
		goto done;
	}
	if(!strcmp(argv[0], "epvid"))
	{
		unsigned int data;
		sscanf(argv[1], "%x", &data);
		cta_switch_pvid_ctrl(data);
		goto done;
	}
	if(!strcmp(argv[0], "wmode"))
	{
		unsigned int data;
		sscanf(argv[1], "%d", &data);
		cta_switch_work_mode(data);
		goto done;
	}
	if(!strcmp(argv[0], "swctl"))
	{
		unsigned int data;
		sscanf(argv[1], "%d", &data);
		cta_switch_state_ctrl(data);
		goto done;
	}
	if(0==(val=strchr(argv[0],'=')))
	{
		printk("%s(%d) (%s)no = found\n",__func__,__LINE__,argv[0]);
		goto done;
	}
	*val++=0;
done:
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        debugfs_br_write_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int debugfs_br_write_state(void *priv, u64 val)
{
	/* write hw bridge old status */
	cta_switch_brg_write_state((u8)val);
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        debugfs_br_read_state:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int debugfs_br_read_state(void *priv, u64 *val)
{
	/* read hw bridge old status */
	*val = cta_switch_brg_read_state();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_br_state, debugfs_br_read_state, debugfs_br_write_state, "%llu\n");

/*!-----------------------------------------------------------------------------
 *        debugfs_br_get:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int debugfs_br_get(void *priv, u64 *val)
{
	/* hw bridge status */
	*val = cta_switch_brg_state();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_br, debugfs_br_get, NULL, "%llu\n");

/*!-----------------------------------------------------------------------------
 *        debugfs_pp_get:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int debugfs_pp_get(void *priv, u64 *val)
{
	/* polling phy status */
	*val = cta_switch_pp_state();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_pp, debugfs_pp_get, NULL, "%llu\n");

/*!-----------------------------------------------------------------------------
 *        proc_eth_write:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int proc_eth_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char buf[300];
	int rc;
	int argc ;
	char * argv[8] ;

	if(count > 0 && count < 299)
	{
		if(copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
		argc = get_args( (const char *)buf , argv );
		rc=eth_cmd(argc, argv);
	}
	return count;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_phy_map:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_phy_map(struct net_device *dev)
{
	int id;
	if_cta_eth_t *pifcta_eth;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
#else
	pifcta_eth = (if_cta_eth_t *)dev->priv;
#endif
	if((dev->priv_flags & IFF_802_1Q_VLAN) == 0)
	{
		id = pifcta_eth->phy_addr;
	}
	else
	{
		int unit = 0;
		if(dev == wdev)
			unit = 1;
		id = cta_switch_port_idx(unit);
	}
	return id;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_get_an:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static u32 cta_eth_get_an(int id, u16 addr)
{
	u32 result = 0;
	int advert;

	advert = cta_eth_mdio_rd(id, addr);
	if(advert & LPA_LPACK)
		result |= ADVERTISED_Autoneg;
	if(advert & ADVERTISE_10HALF)
		result |= ADVERTISED_10baseT_Half;
	if(advert & ADVERTISE_10FULL)
		result |= ADVERTISED_10baseT_Full;
	if(advert & ADVERTISE_100HALF)
		result |= ADVERTISED_100baseT_Half;
	if(advert & ADVERTISE_100FULL)
		result |= ADVERTISED_100baseT_Full;
	if(advert & ADVERTISE_PAUSE_CAP)
		result |= ADVERTISED_Pause;

	return result;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_get_settings:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	int id;
	u16 bmcr, bmsr;
	u32 nego;

	id = cta_eth_phy_map(dev);
	cmd->supported =
		(SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
		SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
		SUPPORTED_Autoneg | SUPPORTED_TP | SUPPORTED_MII |
		SUPPORTED_Pause);

	/* only supports twisted-pair */
	cmd->port = PORT_MII;

	/* only supports internal transceiver */
	cmd->transceiver = XCVR_INTERNAL;

	/* this isn't fully supported at higher layers */
	cmd->phy_address = id;
//	cmd->mdio_support = MDIO_SUPPORTS_C22;

	cmd->advertising = ADVERTISED_TP | ADVERTISED_MII;

	bmcr = cta_eth_mdio_rd(id, MII_BMCR);
	bmsr = cta_eth_mdio_rd(id, MII_BMSR);
	if(bmcr & BMCR_ANENABLE)
	{
		cmd->advertising |= ADVERTISED_Autoneg;
		cmd->autoneg = AUTONEG_ENABLE;

		cmd->advertising |= cta_eth_get_an(id, MII_ADVERTISE);

		if(bmsr & BMSR_ANEGCOMPLETE) {
			cmd->lp_advertising = cta_eth_get_an(id, MII_LPA);
		} else {
			cmd->lp_advertising = 0;
		}

		nego = cmd->advertising & cmd->lp_advertising;
		
		if(nego & (ADVERTISED_100baseT_Full |
			ADVERTISED_100baseT_Half))
		{
			cmd->speed = SPEED_100;
			cmd->duplex = !!(nego & ADVERTISED_100baseT_Full);
		} else {
			cmd->speed = SPEED_10;
			cmd->duplex = !!(nego & ADVERTISED_10baseT_Full);
		}
	}
	else
	{
		cmd->autoneg = AUTONEG_DISABLE;
		cmd->speed = ((bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10);
		cmd->duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	/* ignore maxtxpkt, maxrxpkt for now */

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_set_settings:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	int id;

	if(cmd->speed != SPEED_10 && cmd->speed != SPEED_100)
		return -EINVAL;
	if(cmd->duplex != DUPLEX_HALF && cmd->duplex != DUPLEX_FULL)
		return -EINVAL;
	if(cmd->port != PORT_MII)
		return -EINVAL;
	if(cmd->phy_address >= 2)
		return -EINVAL;
	if(cmd->autoneg != AUTONEG_DISABLE && cmd->autoneg != AUTONEG_ENABLE)
		return -EINVAL;
	id = cta_eth_phy_map(dev);

	/* ignore supported, maxtxpkt, maxrxpkt */
	if(cmd->autoneg == AUTONEG_ENABLE)
	{
		u32 bmcr, advert, tmp;

		if ((cmd->advertising & (ADVERTISED_10baseT_Half |
					  ADVERTISED_10baseT_Full |
					  ADVERTISED_100baseT_Half |
					  ADVERTISED_100baseT_Full)) == 0)
			return -EINVAL;
		
		/* advertise only what has been requested */
		advert = cta_eth_mdio_rd(id, MII_ADVERTISE);
		tmp = advert & ~(ADVERTISE_ALL | ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP);
		if (cmd->advertising & ADVERTISED_10baseT_Half)
			tmp |= ADVERTISE_10HALF;
		if (cmd->advertising & ADVERTISED_10baseT_Full)
			tmp |= ADVERTISE_10FULL;
		if (cmd->advertising & ADVERTISED_100baseT_Half)
			tmp |= ADVERTISE_100HALF;
		if (cmd->advertising & ADVERTISED_100baseT_Full)
			tmp |= ADVERTISE_100FULL;
		if (cmd->advertising & ADVERTISED_Pause)
			tmp |= ADVERTISE_PAUSE_CAP;
		if (advert != tmp) {
			cta_eth_mdio_wr(id, MII_ADVERTISE, tmp);
		}
		
		/* turn on autonegotiation, and force a renegotiate */
		bmcr = cta_eth_mdio_rd(id, MII_BMCR);
		bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
		cta_eth_mdio_wr(id, MII_BMCR, bmcr);
	}
	else
	{
		u32 bmcr, tmp;

		/* turn off auto negotiation, set speed and duplexity */
		bmcr = cta_eth_mdio_rd(id, MII_BMCR);
		tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 | BMCR_FULLDPLX);
		if(cmd->speed == SPEED_100)
			tmp |= BMCR_SPEED100;
		if(cmd->duplex == DUPLEX_FULL)
			tmp |= BMCR_FULLDPLX;
		if(bmcr != tmp)
			cta_eth_mdio_wr(id, MII_BMCR, tmp);
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_get_link:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static u32 cta_eth_get_link(struct net_device *dev)
{
	int phy;
	unsigned short cap, link;
	unsigned int id;

	/* Stop polling phy */
	if(!cta_switch_pp_state())
		return 1;
	phy = cta_eth_phy_map(dev);
	/* first, a dummy read, needed to latch some MII phys */
	cta_eth_mdio_rd(phy,MII_BMSR);
	cap = phy_status(phy);
	link = cap & PHY_LINK;
	id = cta_switch_phyid(phy);
	if(link)
	{
		if(cta_eth_phy_monitor(phy, id, cap))
		{
			link = 0;
			goto exit;
		}
	}
	if((link_status[phy] == 0) && link)
	{
		cta_switch_port_ctrl(phy, 1);
		cta_switch_led_action(phy, link);
		if(id==PHYID_MONT_EPHY)
		{
			pcta_eth->ephy.anc_fail=0;
			cta_eth_phy_isolate(phy, 0);
		}
	}
	else if((link_status[phy] & PHY_LINK) && (link==0))
	{
		cta_switch_port_ctrl(phy, 0);
		if(id==PHYID_MONT_EPHY)
		{
			/* Isolate EPHY is a workaround of AMD AM79C97 NIC */
			cta_eth_phy_isolate(phy, 1);
			cta_eth_phy_reset(phy, id);
		}
	}
	if(id==PHYID_MONT_EPHY)
	{
		if((cap & (PHY_ANC|PHY_LINK))==0)
		{
			/*
				Once EPHY enable MDIX, the time of link-up will
				increase if link partner disable AN
			*/
			if(pcta_eth->ephy.anc_fail==20)
			{
				/*
					EPHY may link fail once link partner's AN is disabled,
					so we reset EPHY while link is down
				*/
				DBGPRINTF(DBG_PHY, "%s: EPHY error: AN-complete =link status =0, reset EPHY(%d)\n",
									__func__, pcta_eth->ephy.anc_fail_rst++);
				cta_eth_phy_isolate(phy, 1);
				cta_eth_phy_reset(phy, id);
				pcta_eth->ephy.anc_fail=0;
			}
			else
				pcta_eth->ephy.anc_fail++;
		}
		if(cta_switch_crc_monitor(phy))
			cta_eth_phy_synchronization(phy);
#if defined(CONFIG_CHEETAH_FPGA)
		{
			/* Debug mdio */
			unsigned short val=0,tmp;
			if(pcta_eth->ephy.mdio_fail==0xffff)
				pcta_eth->ephy.mdio_fail=0;

			tmp = cta_eth_mdio_rd(phy, 31);
			tmp &= 0x7;
			/* Swap to page 1 */
			cta_eth_mdio_wr(phy, 31, 1);
			/* register read/write test */
			cta_eth_mdio_wr(phy, 23, pcta_eth->ephy.mdio_fail);
			val = cta_eth_mdio_rd(phy, 23);
			/* Swap to orig page */
			cta_eth_mdio_wr(phy, 31, tmp);

			if(pcta_eth->ephy.mdio_fail!= val)
			{
				DBGPRINTF(DBG_PHY, "%s: EPHY error: mdio  reg(%d), wr(%04x) rd(%04x)\n",
							__func__, 23, pcta_eth->ephy.mdio_fail, val);
			}
			pcta_eth->ephy.mdio_fail++;
		}
#endif
	}
exit:
	link_status[phy] = link;
	return link;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_nway_reset:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int cta_eth_nway_reset(struct net_device *dev)
{
	int id, ret = -EINVAL;
	short tmp;

	id = cta_eth_phy_map(dev);
	tmp = cta_eth_mdio_rd(id,MII_BMCR);
	if(tmp & BMCR_ANENABLE)
	{
		cta_eth_mdio_wr(id,MII_BMCR,tmp|BMCR_ANRESTART);
		ret = 0;
	}
	return ret;
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_sw_monitor:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_sw_monitor(unsigned long id)
{
	u8 status=0, rc;

	if(monitor_wifi_status_hook)
		status = monitor_wifi_status_hook();
	rc = cta_switch_monitor();
	if(rc)
	{
		if(set_wifi_recover_hook && status)
		{
			set_wifi_recover_hook(rc);
		}
		else
		{
			cta_switch_ethernet_forward_to_wifi(0);
			cta_switch_ethernet_forward_to_wifi(1);
		}
	}
	mod_timer(&pcta_eth->timer, jiffies + 2);
}

/*!-----------------------------------------------------------------------------
 *        cta_eth_get_drvinfo:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void cta_eth_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy(info->driver, "Cheetah Ether");
	strcpy(info->version, "0.1.2");
	strcpy(info->fw_version, "1.03");
	strcpy(info->bus_info, "RMII");
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31))
static struct net_device_ops eth_netdev_ops = {
	.ndo_open				= eth_start,
	.ndo_stop				= eth_stop,
	.ndo_start_xmit			= eth_send,
	.ndo_set_multicast_list	= eth_rx_mode,
	.ndo_do_ioctl			= eth_ioctl,
	.ndo_set_mac_address 	= cta_set_mac,
	.ndo_vlan_rx_register	= eth_vlan_rx_register,
	.ndo_vlan_rx_add_vid	= eth_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= eth_vlan_rx_kill_vid,
};
#endif
static const struct ethtool_ops cta_ethtool_ops = {
	.get_drvinfo		= cta_eth_get_drvinfo,
	.get_settings		= cta_eth_get_settings,
	.set_settings		= cta_eth_set_settings,
	.get_link			= cta_eth_get_link,
	.nway_reset			= cta_eth_nway_reset,
};

/*!-----------------------------------------------------------------------------
 *        mon_eth_drv_init:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int mon_eth_drv_init(int unit)
{
	if_cta_eth_t *pifcta_eth;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
#else
	char mac_addr[6] = { 0, 0 , 0x32, 0x10, 0x00, 0xaa };
#endif
	int rc;
	struct net_device *dev;

	/* dev and priv zeroed in alloc_etherdev */
	dev = alloc_etherdev(sizeof (if_cta_eth_t));
	if(dev == NULL)
	{
		printk("%s Unable to alloc new net device\n",__func__);
		return -ENOMEM;
	}
	sprintf(dev->name, "eth%d", unit);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	pifcta_eth = (if_cta_eth_t *) netdev_priv(dev);
#else
	pifcta_eth = (if_cta_eth_t *) dev->priv;
#endif
	dev->base_addr = MAC_BASE;
	dev->irq = IRQ_MAC;
	dev->ethtool_ops = &cta_ethtool_ops; //@ethtool
	pifcta_eth->unit=unit;

	DBGPRINTF(DBG_INFO, "Montage ethernet drv init: dev=%x, unit=%d\n",
				(unsigned int) dev, unit);

	/* need to notice !!! */
	if((pcta_eth == 0)  && cta_eth_init() != 0)
	{
		DBGPRINTF(DBG_ERR, "eth_drv_init: failed!\n");
		return -1;
	}
	pifcta_eth->sc = dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31))
	dev->netdev_ops = &eth_netdev_ops;
#endif
	rc = register_netdev (dev);
	if(rc)
		goto err_out1;

	/* need to notice !!! */
	if(cta_eth_add_if(unit, pifcta_eth) != 0)
		goto err_out2;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
	random_ether_addr(pifcta_eth->macaddr);
#else
	memcpy( pifcta_eth->macaddr, mac_addr, 6 );						/* Set MAC address */
#endif
	pifcta_eth->macaddr[5] += (char) unit ;
#if 0
	cta_eth_setmac(unit, pifcta_eth->macaddr);						/* Need to modify */
#endif
	memcpy( dev->dev_addr, pifcta_eth->macaddr, dev->addr_len );	/* Set MAC address */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31))
	dev->open				= &eth_start;
	dev->stop				= &eth_stop;
	dev->hard_start_xmit	= &eth_send;
	dev->set_multicast_list = &eth_rx_mode;
	dev->do_ioctl			= &eth_ioctl;
#endif
	dev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_FILTER | NETIF_F_HW_VLAN_CTAG_TX ;
	dev->priv_flags |= IFF_ETH_DEV;

#if defined(ENABLE_SGIO)
    dev->features |= NETIF_F_SG;
    dev->vlan_features |= NETIF_F_SG;
#endif

#if defined(ENABLE_TCPUDP_V4_CKSUM)
    dev->features |= NETIF_F_IP_CSUM;
    dev->vlan_features |= NETIF_F_IP_CSUM;
#endif

#if defined(ENABLE_TSO)
    dev->features |= NETIF_F_GSO;   //NETIF_F_TSO
    dev->vlan_features |= NETIF_F_GSO; //NETIF_F_TSO
#endif
	ldev = dev;

	return 0;
err_out2:
	cta_eth_del_if(unit);
err_out1:
	DBGPRINTF(DBG_ERR, "eth_drv_init: failed 2!\n");
	return -1;
}

/*!-----------------------------------------------------------------------------
 *        eth_init_module:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int __init eth_init_module(void)
{
	static int probed = 0;
	int rc;
	int i;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *res;

	if(!eth_debug_dir)
	{
		eth_debug_dir = debugfs_create_dir("eth", NULL);

		res = create_proc_entry("eth", S_IWUSR | S_IRUGO, NULL);
		if(!res)
			return -ENOMEM;

		res->read_proc = NULL;
		res->write_proc = proc_eth_write;

		debugfs_create_file("br_state", S_IRWXUGO, eth_debug_dir, NULL, &fops_br_state);
		debugfs_create_file("br", S_IRWXUGO, eth_debug_dir, NULL, &fops_br);
		debugfs_create_file("pp", S_IRWXUGO, eth_debug_dir, NULL, &fops_pp);
	}
#endif
	/* release hnat(switch port2) and switch module clk */
	GPREG(SWRST) &= ~(PAUSE_SW | PAUSE_HNAT);
	/* reset hnat and switch port2 */
	GPREG(SWRST) &= ~SWRST_HNAT;
	for (i=0; i < 200; i++);
	GPREG(SWRST) |= SWRST_HNAT;

	if(probed)
		return -ENODEV;
	probed = 1;

	printk(KERN_INFO "Cheetah Ether driver version %s\n", eth_ethdrv_version);

	for(i=0;i<ETH_DEV_INIT; i++)
	{
		rc = mon_eth_drv_init(i);
		if(rc)
			break;
	}

	return 0;
}

/*!-----------------------------------------------------------------------------
 *        eth_cleanup_module:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static void __exit eth_cleanup_module(void)
{
	int i;
	struct net_device * dev;

	printk(KERN_INFO "Cheetah Ether driver unloaded\n");
	for(i=0;i< ETH_DEV_INIT; i++)
	{
		if(if_cta_eth_g[i]==0)
			continue;
		if(0==(dev=if_cta_eth_g[i]->sc))
			continue;
		if(pcta_eth->sc_main==dev)
			free_irq(dev->irq, dev);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
		/* the priv data is part of allocated  dev */
#else
		if(dev->priv)
			kfree(dev->priv);
#endif
		unregister_netdev(dev);
		kfree(dev);
		kfree(if_cta_eth_g[i]);
		if_cta_eth_g[i]=0;
	}
	/* reset hnat and switch port2 */
	GPREG(SWRST) &= ~SWRST_HNAT;
	for (i=0; i < 200; i++);
	GPREG(SWRST) |= SWRST_HNAT;
	/* stop hnat(switch port2) and switch module clk */
	GPREG(SWRST) &= ~(PAUSE_SW | PAUSE_HNAT);
#ifdef CONFIG_PROC_FS
	if(eth_debug_dir)
	{
		remove_proc_entry("eth", NULL);
		debugfs_remove_recursive(eth_debug_dir);
		eth_debug_dir = NULL;
	}
#endif
}
module_init(eth_init_module);
module_exit(eth_cleanup_module);
