/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/

/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#ifdef __ECOS
#include <string.h>
#include <stdlib.h>
#include <config.h>
#include <sys/types.h>
#include <sys/param.h>

#include <sys/ioctl.h>
#include <cta_eth.h>
#include <cta_hnat.h>
#define DYN_NATDS

#else
// linux
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#if defined(CONFIG_NF_CONNTRACK)
#include <net/netfilter/nf_conntrack.h>
#endif
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/cta_eth.h>
#include <asm/mach-cheetah/cta_hnat.h>
#define printd				printk
#define x_malloc(s)			kmalloc(s, GFP_DMA)
#define free(p)				kfree(p)
#endif


/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define MAC_REG_BASE		MAC_BASE

#define NATD_HW_SZ			4096
#define NATD_SZ				NATD_HW_SZ
#define NATH_SZ				1024
#define MACD_SZ				1024
#define MACH_SZ				256

#define HNAT_MAGIC			0x4D54	/* MT */
#if 0
#define HNAT_PRINTF(fmt, ...) \
		do { \
			printd(fmt, ## __VA_ARGS__); \
		} while(0)

#else
#define HNAT_PRINTF(fmt, ...) \
		do { \
		} while(0)
#endif
// NAT HASH
#define NAT_HASHO(prot,sip,dip,sp,dp) \
		(((sip&3)<<8) | (sp&0xff))

#define NAT_HASHI(prot,sip,dip,sp,dp) \
		(((dip&3)<<8) | (dp&0xff))

// NAT TAG
#define NAT_TAGO(prot,sip,dip,sp,dp) \
		(((sip&0xff)<<8) | (sp>>8))

#define NAT_TAGI(prot,sip,dip,sp,dp) \
		(((dip&0xff)<<8) | (dp>>8))
#define MDBG(m)						MREG(MVL67)=m
#define NATDS_INIT
#define AUTO_TABLE
//#define M_CT_MIP_TOGGLE
#define CTA_HNAT_HASH_CRC10
//#define CTA_HNAT_BUCKET_SIZE		24

//#define HASH_REPLACE
#define HNAT_REVERT_TCP_RST_PKT

#define FIX_CHECKSUM(diff, cksum) { \
	diff += (cksum); \
	if (diff < 0) \
	{ \
		diff = -diff; \
		diff = (diff >> 16) + (diff & 0xffff); \
		diff += diff >> 16; \
		(cksum) = (u_short) ~diff; \
	} \
	else \
	{ \
		diff = (diff >> 16) + (diff & 0xffff); \
		diff += diff >> 16; \
		(cksum) = (u_short) diff; \
	} \
}

#ifdef CONFIG_HNAT_NONCACHED
#define hnat_addr(a)			nonca_addr(a)
#define DC_INVAL(a,l)
#define DC_STORE(a,l)
#else
#define hnat_addr(a)			ca_addr(a)
#ifdef __ECOS
#define DC_INVAL(a,l)			cta_eth_os_dcache_flush(a,l)
#define DC_STORE(a,l)			cta_eth_os_dcache_store(a,l)
#else
#define DC_INVAL(a,l)			HAL_DCACHE_FLUSH(a,l)
#define DC_STORE(a,l)			HAL_DCACHE_STORE(a,l)
#endif
#endif

typedef struct
{
	/* half-word 0 */
	unsigned short s:1;			/* static */
	unsigned short v:1;			/* valid */
	unsigned short idx:14;
	/* half-word 1 */
	unsigned short tag;
} nathashtab ;

typedef struct
{
	unsigned short s:1;			/* static */
	unsigned short v:1;			/* valid */
	unsigned short age:2;		/* age */
	unsigned short vl:1;		/* add vlan tag? */
	unsigned short vidx:3;		/* vlan id index */
	unsigned short stag:8;		/* specail tag */
	/* half-word 3,4,5 */
	unsigned char mac[6];
} macdes ;

typedef struct rxinfo_s{
	unsigned int dir:1;
	unsigned int res1:31;
	unsigned int hn_desc;
	unsigned short mac;
	unsigned short tcp_chksum;
}rxinfo_t;

#define IPA(a,b,c,d)			((a<<24)|(b<<16)|(c<<8)|(d))
struct iphdr
{
#if 1
  unsigned char version:4, ihl:4;
#else
  unsigned char ihl:4, version:4;
#endif
  char service;
  unsigned short len;
  unsigned short ident;
  unsigned short frags;
  char ttl;
  char protocol;
  unsigned short chksum;
  unsigned long src;
  unsigned long dst;
} __attribute__ ((aligned(2), packed));

struct tcphdr
{
  unsigned short src;
  unsigned short dst;
  unsigned int seqno;
  unsigned int ackno;
  unsigned char tcpoffset;
  unsigned char flags;
  unsigned short window;
  unsigned short chksum;
  unsigned short urgp;
} __attribute__ ((aligned(2), packed));

struct ppp_head
{
	short prot;
};

struct poe_head
{
	char ver;
	char code;
	short sid;
	short len;
	struct ppp_head ppp;
	char pad[2];
};
struct mip_cfg
{
	unsigned int mip;
	char poef;
};

/*=============================================================================+
| Variables                                                                    |
+=============================================================================*/
unsigned short hnat_max_num = NATD_HW_SZ;

/*static natdes NATDS[NATD_SZ] __attribute__ ((aligned (0x8)));*/
static macdes MACDS[MACD_SZ] __attribute__ ((aligned (0x8)));
static unsigned short MACH[MACH_SZ][4] __attribute__ ((aligned (0x8)));
static nathashtab NATHO[NATH_SZ][4] __attribute__ ((aligned (0x10)));
static nathashtab NATHI[NATH_SZ][4] __attribute__ ((aligned (0x10)));
#ifndef DYN_NATDS
static natdes NATDS[NATD_SZ] __attribute__ ((aligned (0x20)));
#endif

struct poe_head poe_info[CTA_HNAT_PUBLIC_PPPOE_MAX_NUM];

int hnat_mode_g = CTA_HNAT_NOHNAT;

struct mip_cfg mip_info[CTA_HNAT_PUBLIC_IP_MAX_NUM];

natdes *natds = 0;
nathashtab *natho = 0;
nathashtab *nathi = 0;
macdes *macds = 0;
unsigned short *mach = 0;
unsigned short free_macds_g = 0;

natdes *free_natds_g;
static short free_natds_num_g;
natdes *natds_ground = 0;
natdes *ml_head = 0;
#ifdef CONFIG_HNAT_PATCH
static nathashtab NATHSO[NATH_SZ][4] __attribute__ ((aligned (0x10)));
static nathashtab NATHSI[NATH_SZ][4] __attribute__ ((aligned (0x10)));
nathashtab *nathso = 0;
nathashtab *nathsi = 0;
#endif
int cta_hnat_rx_check(char *buf, char *pkt, unsigned int w0, unsigned int w1) __attribute__ ((section(".iram")));


unsigned long hnat_flags;
/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned long hmu_keygen(unsigned short ip, unsigned short port, unsigned short prot)
{
	unsigned long key;

	HNAT_LOCK;
	MREG(MHIP)=ip;
	MREG(MHPORT)=port;
	MREG(MHPROT)=prot;
	while (1)
	{
		if ((1<<31)&(key=MREG(MHKEY)))
			break;
	}
	HNAT_UNLOCK;

	return key;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int hmu_find_way(nathashtab *natht, int idx, int tag)
{
	short w,replace = 0;
	natdes *ds=&natds[idx];
	natdes *dw;
	for(w=0; w< 4; w++)
	{
		if (natht[w].v)
		{
			if (natht[w].tag!=tag)
				continue;
			dw=&natds[natht[w].idx];
			if (dw->prot!=ds->prot)
				continue;
			if (dw->siplsb!=ds->siplsb)
				continue;
			if (dw->sp!=ds->sp)
				continue;
			if (dw->dp!=ds->dp)
				continue;
			if (dw->dip!=ds->dip)
				continue;
			HNAT_PRINTF("entry%d, existed way=%d\n",idx, w);
			return w;
		}
		if (natht[w].s) // don't touch
			continue;
		return w;
	}
	HNAT_PRINTF("idx=%d, no way available\n", idx);
	for (w=0; w<4 ; w++)
	{
		HNAT_PRINTF("w=%d nat idx=%d tag=%x\n", w, natht[w].idx, (unsigned long)natht[w].tag);
		if (!natht[w].s) //!static
			replace=w;
	}
	return replace|0x4; //as a flag
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
nathashtab *hmu_find_match(nathashtab *natht, natdes *ds, int tag)
{
	short idx = ds - &natds[1];
	if(natht->v && (tag == natht->tag) && (idx == natht->idx))
		return natht;
	return 0;
}

#ifdef CONFIG_HNAT_PATCH
/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
natdes *hmu_find_natds(nathashtab *natht, int dir, int tag, unsigned int sip, unsigned int dip,
						unsigned char prot, unsigned short sdata, unsigned short ddata)
{
	short w;
	natdes *dw;
	unsigned long mip;

	for(w=0; w< 4; w++)
	{
		if (natht[w].v)
		{
			if (natht[w].tag!=tag)
				continue;
			dw=&natds[natht[w].idx+1];
			mip=mip_info[dw->mipidx].mip;
			if (dw->prot!=prot)
				continue;
			if(dir)
			{
#if 1
				if (dw->siplsb==0)
				{
					if (mip!=sip)
						continue;
				}
				else
#endif
				{
					if (dw->siplsb!=SIP2LSB(sip))
						continue;
				}
				if (dw->sp!=sdata)
					continue;
			}
			else
			{
				if (mip!=sip)
					continue;
				if (dw->mp!=sdata)
					continue;
			}
			if (dw->dp!=ddata)
				continue;
			if (dw->dip!=dip)
				continue;
			HNAT_PRINTF("entry%d, existed way=%d\n", sess_to_idx(dw), w);
			return dw;
		}
	}
	HNAT_PRINTF("tag=%x, no natds available\n", tag);
	return 0; //as a flag
}
#endif

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
short find_mipidx(unsigned long ip)
{
	short i;
	for (i=0; i<CTA_HNAT_PUBLIC_IP_MAX_NUM; i++)
	{
		if (ip==mip_info[i].mip)
		{
			return i;
		}
	}
	return -1;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int natds_insert(natdes *ds)
{
	unsigned short ih,itag,iway;
	unsigned short oh,otag,oway;
	unsigned long key,mip;
	short i;
	nathashtab *hx;

	i = (ds-&natds[1]);

	if (ds->dip==0 && ds->siplsb==0)
	{
		HNAT_PRINTF("dip & sip==0\n");
		goto err;
	}

	mip=mip_info[ds->mipidx].mip;
	/* outbound */
#if 1
	if (ds->siplsb==0) /* superDMZ */
		key = hmu_keygen(mip, ds->sp, ds->prot);
	else
#endif
		key = hmu_keygen(ds->siplsb, ds->sp, ds->prot);
	oh=(key<<6)>>(16+6);
	otag=key&0xffff;

	HNAT_PRINTF("oh=%x tag=%x\n", oh, (unsigned long)otag);
	oway = hmu_find_way(natho+4*oh, i, otag);
	/* found free or exist slot */
	if(oway <= 3)
	{
		hx=&natho[4*oh+oway];
		hx->tag=otag;
		*(unsigned short*)hx=i|(1<<14);
		DC_STORE((unsigned int)hx, sizeof(nathashtab));
	}
#ifdef CONFIG_HNAT_PATCH
	key = hmu_keygen(ds->siplsb^ds->dip, ds->sp, ds->prot);
	oh=(key<<6)>>(16+6);
	otag=key&0xffff;

	HNAT_PRINTF("oh=%x tag=%x\n", oh, (unsigned long)otag);
	oway = hmu_find_way(nathso+4*oh, i, otag);
	/* found free or exist slot */
	if(oway <= 3)
	{
		hx=&nathso[4*oh+oway];
		hx->tag=otag;
		*(unsigned short*)hx=i|(1<<14);
		DC_STORE((unsigned int)hx, sizeof(nathashtab));
	}
#endif

	/* inbound */
	if (mip==0)	/* can't setup */
	{
		HNAT_PRINTF("MIP can't be zero\n");
		goto err;
	}

#ifndef INBOND_HASH_DIP_XOR_SIP
	key = hmu_keygen(mip, ds->mp, ds->prot);
#else
	key = hmu_keygen(0xffff&(ds->dip^mip), ds->mp, ds->prot);
#endif
	ih=(key<<6)>>(16+6);
	itag=key&0xffff;
	HNAT_PRINTF("ih=%x tag=%x\n", ih, (unsigned long)itag);
	iway = hmu_find_way(nathi+ih*4, i, itag);
	/* found free or exist slot */
	if(iway <= 3)
	{
		hx=&nathi[4*ih+iway];
		hx->tag=itag;
		*(unsigned short*)hx=i|(1<<14);
		DC_STORE((unsigned int)hx, sizeof(nathashtab));
	}
#ifdef CONFIG_HNAT_PATCH
	key = hmu_keygen(ds->dip^mip, ds->mp, ds->prot);
	ih=(key<<6)>>(16+6);
	itag=key&0xffff;
	HNAT_PRINTF("ih=%x tag=%x\n", ih, (unsigned long)itag);
	iway = hmu_find_way(nathsi+ih*4, i, itag);
	/* found free or exist slot */
	if(iway <= 3)
	{
		hx=&nathsi[4*ih+iway];
		hx->tag=itag;
		*(unsigned short*)hx=i|(1<<14);
		DC_STORE((unsigned int)hx, sizeof(nathashtab));
	}
#endif

	ds->hnat_acc = HNAT_ACC_ATTACHED;
	DC_STORE((unsigned int)ds, sizeof(natdes));
	HNAT_PRINTF("natds_insert successful!\n");
	return HNRC_OK;

err:
		return HNRC_ERR;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned short mach_keygen(char *mac)
{
	return (unsigned char)(mac[4] ^ mac[5] ^ 0x5a);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int mach_attach(int idx, char *mac, char tag, char inout, int reset)
{
	if(reset)
	{
		memset(&macds[idx], 0, sizeof(macdes));
		DC_STORE((unsigned int)&macds[idx], sizeof(macdes));
		return idx;
	}
	memcpy(&macds[idx].mac[0], &mac[0], 6);
	macds[idx].v=1;
	macds[idx].vl=1;
	macds[idx].vidx=(inout&7);
	macds[idx].stag=(tag&7);
	DC_STORE((unsigned int )&macds[idx], sizeof(macdes));
	return idx;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int macds_alloc(char *mac, char tag, char inout)
{
	unsigned short key, i, *machptr, idx, replace=0xffff;

	if(free_macds_g >= MACD_SZ)
	{
		free_macds_g = 1;
	}
	/* find MAC hash address */
	key = mach_keygen(mac);
	machptr = &mach[4*key];

	for(i=0; i<4; i++)
	{
		key = machptr[i];
		/* never use? */
		if(!key)
			break;
		idx = key & (~0x8000);
		/* found the same mac already exists? */
		if(!memcmp(mac, macds[idx].mac, 6))
			return mach_attach(idx, mac, tag, inout, 0);
		if(key & 0x8000)
			replace = i;
	}
	mach_attach(free_macds_g, mac, tag, inout, 0);
	if(i >= 4)
	{
		idx = free_macds_g | 0x8000;
		if(replace == 0xffff)
			machptr[0] = idx;
		else
		{
			machptr[replace] &= ~0x8000;
			if(replace == 3)
				machptr[0] = idx;
			else
				machptr[replace+1] = idx;
		}
	}
	else
		machptr[i] = free_macds_g;
	return free_macds_g++;
#if 0
	short i;
	/* zero was reserved */
	for (i=1; i < MACD_SZ; i++)
	{
		if (!macds[i].v)
			break;
	
		// found the same mac already exists?
		if (!memcmp(mac, macds[i].mac, 6) && macds[i].vidx==(inout&7))
		{
			/* replace old special tag */
			macds[i].stag = tag;
			return i;
		}
	}
	if (i >= MACD_SZ)
	{
		return -1;
	}
#if 0
	*(unsigned short*)&macds[i].mac[0] = *(unsigned short*)&mac[0];
	*(unsigned short*)&macds[i].mac[2] = *(unsigned short*)&mac[2];
	*(unsigned short*)&macds[i].mac[4] = *(unsigned short*)&mac[4];
#else
	memcpy(&macds[i].mac[0], &mac[0], 6);
#endif
	macds[i].v=1;
	macds[i].vl=1;
	macds[i].vidx=inout&7;
	macds[i].stag=tag;
	DC_STORE((unsigned int)&macds[i], sizeof(macdes));
	return i;
#endif
}

#if 0
int macds_free(short i)
{
	if (!macds[i].v)
		return -1;

	macds[i].v=0;
	macds[i].stag=0;
	return 0;
}
#endif

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int setpppoe(int idx, unsigned char* poeh)
{
	int i;
	unsigned long cmd;

	cmd=(1<<30)|(idx<<20);
	for (i=0; i<5; i++)
	{
		MREG(MCT) = cmd|(i<<16)|poeh[i*2]<<8|poeh[i*2+1];
	}
	MREG(MCT)= (1<<31)|(idx<<20);
	return 0;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int setmip(int idx, struct mip_cfg *mipc)
{
	unsigned long cmd;
	unsigned char* ip  = (unsigned char*) &mipc->mip;
	unsigned char poef = mipc->poef;

	cmd=(1<<30)|(idx<<20)|(1<<25);

	MREG(MCT) = cmd|(0<<16)|(ip[0]<<8)|(ip[1]);
	MREG(MCT) = cmd|(1<<16)|(ip[2]<<8)|(ip[3]);
	MREG(MCT) = cmd|(2<<16)|(poef<<8);

	MREG(MCT)= (1<<31)|(idx<<20)|(1<<25);
	return 0;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_init(void)
{
	memset((char *)poe_info, 0, sizeof(struct poe_head)*CTA_HNAT_PUBLIC_PPPOE_MAX_NUM);
	memset(mip_info, 0, sizeof(struct mip_cfg)*CTA_HNAT_PUBLIC_IP_MAX_NUM);

	macds = (macdes *) hnat_addr(&MACDS[0]);
	mach = (unsigned short *) (&MACH[0][0]);
	natho = (nathashtab *) hnat_addr(&NATHO[0][0]);
	nathi = (nathashtab *) hnat_addr(&NATHI[0][0]);
#ifdef CONFIG_HNAT_PATCH
	nathso = (nathashtab *) hnat_addr(&NATHSO[0][0]);
	nathsi = (nathashtab *) hnat_addr(&NATHSI[0][0]);
#endif
	
	MREG(MARPB) = virtophy(macds);
	MREG(MHTOBB) = virtophy(natho);
	MREG(MHTIBB) = virtophy(nathi);

	// disable TCP sync meter b16~23
	MREG(MTSR) = 0x000003ff;

	cta_hnat_reset();

	return 0;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_reset(void)
{
	int i;
	natdes *ds = NULL;

	cta_hnat_cfgmode(CTA_HNAT_NOHNAT);

	// clear NAT descriptor
	memset(natho, 0, sizeof(nathashtab)*NATH_SZ*4);
	memset(nathi, 0, sizeof(nathashtab)*NATH_SZ*4);
#ifdef CONFIG_HNAT_PATCH
	memset(nathso, 0, sizeof(nathashtab)*NATH_SZ*4);
	memset(nathsi, 0, sizeof(nathashtab)*NATH_SZ*4);
#endif
	memset(macds, 0, sizeof(macdes)*MACD_SZ);
	memset(mach, 0, sizeof(unsigned short)*MACH_SZ*4);

	DC_STORE((unsigned int)&macds[0], sizeof(macdes)*MACD_SZ);
	DC_STORE((unsigned int)&natho[0], sizeof(nathashtab)*NATH_SZ*4);
	DC_STORE((unsigned int)&nathi[0], sizeof(nathashtab)*NATH_SZ*4);
#ifdef CONFIG_HNAT_PATCH
	DC_STORE((unsigned int)&nathso[0], sizeof(nathashtab)*NATH_SZ*4);
	DC_STORE((unsigned int)&nathsi[0], sizeof(nathashtab)*NATH_SZ*4);
#endif

	memset((char *)poe_info, 0, sizeof(struct poe_head)*CTA_HNAT_PUBLIC_PPPOE_MAX_NUM);
	memset(mip_info, 0, sizeof(struct mip_cfg)*CTA_HNAT_PUBLIC_IP_MAX_NUM);

	if(natds == 0)
	{
#ifdef DYN_NATDS
		if((natds = (natdes*)x_malloc(sizeof(natdes)*(NATD_SZ))) == 0)
		{
			printd("malloc for napt seesion fail!\n");
			return ;
		}
#else
		natds = &NATDS[0];
#endif
		natds = (natdes*)hnat_addr(natds);
		natds_ground = natds;

		MREG(MTB) = virtophy(&natds[1]);
	}

	memset(natds_ground, 0, sizeof(natdes)*NATD_SZ);

	for(i=1; i < NATD_SZ; i++)
	{
		ds = &natds[i];
		ds->ihnext = i+1;
	}
	ds->ihnext = 0; /* pointer to ground */

	DC_STORE((unsigned int)&natds[0], sizeof(natdes)*(NATD_SZ+1));
	free_natds_g = &natds[1];
	free_natds_num_g = NATD_SZ-1;
	ml_head = natds_ground;
	free_macds_g = 1;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_unload(void)
{
	free_natds_g = 0;
	free_natds_num_g = 0;
	ml_head = 0;
	free_macds_g = 1;
	// clear NAT descriptor
	memset(natho, 0, sizeof(nathashtab)*NATH_SZ*4);
	memset(nathi, 0, sizeof(nathashtab)*NATH_SZ*4);
#ifdef CONFIG_HNAT_PATCH
	memset(nathso, 0, sizeof(nathashtab)*NATH_SZ*4);
	memset(nathsi, 0, sizeof(nathashtab)*NATH_SZ*4);
#endif
	//memset(natds, 0, sizeof(natdes)*NATD_SZ);
	memset(macds, 0, sizeof(macdes)*MACD_SZ);
	memset(mach, 0, sizeof(unsigned short)*MACH_SZ*4);
	if(natds_ground)
	{
#ifdef DYN_NATDS
		free(natds_ground);
#endif
		natds = 0;
		natds_ground = 0;
	}
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_cfgmode(int mode)
{
	if(mode == CTA_HNAT_FULLHNAT)
	{
		MREG(MCR) &= ~(MCR_BYBASS_LOOKUP|MCR_TOCPU|MCR_HMU_TESTMODE);
		MREG(MCR) |= (MCR_HNAT_EN|MCR_HMU_EN|MCR_CACHE_EN|MCR_PREFETCH_DIS);
		MREG(MTSR) |= 0x03ff;
	}
	else if(mode == CTA_HNAT_SEMIHNAT)
	{
		MREG(MCR) &= ~(MCR_BYBASS_LOOKUP|MCR_HMU_TESTMODE);
		MREG(MCR) |= (MCR_HNAT_EN|MCR_HMU_EN|MCR_CACHE_EN|MCR_TOCPU|MCR_PREFETCH_DIS);
		MREG(MTSR) |= 0x03ff;
	}
#if 1
	else if(mode == CTA_HNAT_NOHNAT)
	{
		// MCR_HNAT_EN : should not turn off to avoid h/w bug
		MREG(MCR) |= MCR_BYBASS_LOOKUP;
		MREG(MCR) &= ~(MCR_HMU_EN|MCR_CACHE_EN|MCR_TOCPU);
	}
	// protocol support should not disable to keep RXDES status
	MREG(MPROT) &= ~0xf;
#else
	else if(mode == CTA_HNAT_NOHNAT)
	{
		/* turn on test mode to bypass zero checksum */
		MREG(MCR) &= ~MCR_BYBASS_LOOKUP;
		MREG(MCR) |= (MCR_HNAT_EN|MCR_HMU_EN|MCR_HMU_TESTMODE|MCR_TOCPU);
		MREG(MTSR) &= 0xffff0000;
	}
#endif
#ifdef CTA_HNAT_BUCKET_SIZE_24
	MREG(MCR) &= ~MCR_NATBUCKET_SIZE;
#else
	MREG(MCR) |= MCR_NATBUCKET_SIZE;
#endif
#ifdef CTA_HNAT_HASH_CRC10
	MREG(MCR) &= ~MCR_HASH_FUNC;
#else
	MREG(MCR) |= MCR_HASH_FUNC;
#endif
#ifdef INBOND_HASH_DIP_XOR_SIP
	MREG(NHM) = 1;
#else
	MREG(NHM) = 0;
#endif
	MREG(MPOET) = PPPOE_PACKET_TYPE;

	HNAT_PRINTF("MCR=0x%x\n", MREG(MCR));
	hnat_mode_g = mode;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_cfgip(int which, int ip, int is_pppoe)
{
	int mip;
	if(which == CTA_HNAT_PRIVATE_IF)
	{
		MREG(MSIP) = ip;
		return HNRC_OK;
	}
	else if(which == CTA_HNAT_PUBLIC_IF)
	{
		for(mip=0; mip<CTA_HNAT_PUBLIC_IP_MAX_NUM; mip++)
		{
			if(mip_info[mip].mip!=0)
				continue;
			mip_info[mip].poef = 0;
			if(is_pppoe)
			{
				int ppp;
				for(ppp=0; ppp<CTA_HNAT_PUBLIC_PPPOE_MAX_NUM; ppp++)
				{
					if(*(unsigned short*)&poe_info[ppp].pad[0]!=0)
						continue;
					*(unsigned short*)&poe_info[ppp].pad[0] = HNAT_MAGIC;
					mip_info[mip].poef = ((is_pppoe<<3)|(0x7&ppp));
					goto done;
				}
				goto err;
			}
			goto done;
		}
	}
err:
	return HNRC_ERR;
done:
	mip_info[mip].mip = ip;
	setmip(mip, &mip_info[mip]);
	return mip;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
short sess_to_idx(natdes *ds)
{
	return (ds - &natds[0]);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
natdes *idx_to_sess(short s)
{
	return (natdes *)&natds[s];
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned int idx_to_mip(int mipidx)
{
	return (mip_info[mipidx].mip);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_manual_cfgmac(natdes *ds, char *mac, char tag, int dir)
{
	if(ds)
		ds->smac = macds_alloc(mac, tag, dir);
	else
		macds_alloc(mac, tag, dir);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_get_statistic(void)
{
	return free_natds_num_g;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
static int ml_del(natdes *ds)
{
	natdes *ds1, *prev_ds;

	ds1 = ml_head;
	prev_ds = 0;
	while(ds1->used)
	{
		if(ds == ds1)
		{
			if(prev_ds)
				prev_ds->ml_next = ds1->ml_next;
			else
				ml_head =  &natds[ds1->ml_next];
			ds->ml_next = 0;
			DC_STORE((unsigned int)ds, sizeof(natdes));
			return 1;
		}
		
		prev_ds = ds1;
		ds1 = &natds[ds1->ml_next];
	}
	return 0;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
natdes *cta_hnat_alloc(void)
{
	natdes *ds;

	if(free_natds_num_g <= 0)
		return 0;
	/* get a session from free pool */
	ds = free_natds_g;
	/* no available ds */
	if(ds == &natds[0])
		return 0;
	HNAT_LOCK;
	free_natds_g = &natds[ds->ihnext];
	free_natds_num_g--;
	ds->ihnext = 0;
	DC_STORE((unsigned int)ds, sizeof(natdes));
	HNAT_UNLOCK;

	return ds;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_free(natdes *ds)
{
	//printd("cta_hnat_free:%d, ds->flg0=%x\n", s-1, ds->flg0);
	cta_hnat_detach(ds);

	HNAT_LOCK;
	//memset(ds, 0, sizeof(natdes));
	ds->ihnext = free_natds_g - &natds[0];
	free_natds_g = ds;

	free_natds_num_g++;
	DC_STORE((unsigned int)ds, sizeof(natdes));
	HNAT_UNLOCK;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_attach(natdes *ds)
{
	DC_STORE((unsigned int)ds, sizeof(natdes));
	if(hnat_mode_g == CTA_HNAT_NOHNAT)
		return HNRC_OK;
	if(ds->used == 0)
		return HNRC_OK;
	if(sess_to_idx(ds) >= NATD_HW_SZ)
		return HNRC_OK;

//printd("#####%s(%d), ds=%x\n", __func__, __LINE__, ds);
	HNAT_LOCK;
	/* if it has been in list, delete it */
	ml_del(ds);

	ds->ml_next = ml_head - &natds[0];
	ml_head = ds;
	ds->hnat_acc = HNAT_ACC_LEARNMAC;
	DC_STORE((unsigned int)ds, sizeof(natdes));
	HNAT_UNLOCK;
	return HNRC_LEARNMAC ;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_detach(natdes *ds)
{
	unsigned short ih,itag;
	unsigned short oh,otag;
	unsigned long key,mip;
	nathashtab *hx;
	char i;

	ds->hnat_acc = HNAT_ACC_NONE;
	DC_STORE((unsigned int)ds, sizeof(natdes));

	if(!ds->smac || !ds->dmac)
	{
		HNAT_LOCK;
		ml_del(ds);
		HNAT_UNLOCK;
		return;
	}

	mip=mip_info[ds->mipidx].mip;
	/* outbound */
#if 1
	if (ds->siplsb==0) /* superDMZ */
		key = hmu_keygen(mip, ds->sp, ds->prot);
	else
#endif
		key = hmu_keygen(ds->siplsb, ds->sp, ds->prot);
	oh=(key<<6)>>(16+6);
	otag=key&0xffff;
	/* find all match hashed table and release it */
	for(i=0; i< 4 ; i++)
	{
		if ((hx = hmu_find_match(natho+4*oh+i, ds, otag)))
		{
		    *(unsigned int*)hx=0;
			DC_STORE((unsigned int)hx, sizeof(nathashtab));
		}
	}
#ifdef CONFIG_HNAT_PATCH
	key = hmu_keygen(ds->siplsb^ds->dip, ds->sp, ds->prot);
	oh=(key<<6)>>(16+6);
	otag=key&0xffff;
	/* find all match hashed table and release it */
	for(i=0; i< 4 ; i++)
	{
		if ((hx = hmu_find_match(nathso+4*oh+i, ds, otag)))
		{
		    *(unsigned int*)hx=0;
			DC_STORE((unsigned int)hx, sizeof(nathashtab));
		}
	}
#endif
	/* inbound */
	if (mip==0)	/* can't setup */
	{
		HNAT_PRINTF("MIP can't be zero\n");
		return;
	}

#ifndef INBOND_HASH_DIP_XOR_SIP
	key = hmu_keygen(mip, ds->mp, ds->prot);
#else
	key = hmu_keygen(0xffff&(ds->dip^mip), ds->mp, ds->prot);
#endif
	ih=(key<<6)>>(16+6);
	itag=key&0xffff;
	/* find all match hashed table and release it */
	for (i=0 ; i<4 ; i++)
	{
		if ((hx = hmu_find_match(nathi+4*ih+i, ds, itag)))
		{
			*(unsigned int*)hx=0;
			DC_STORE((unsigned int)hx, sizeof(nathashtab));
		}
	}
#ifdef CONFIG_HNAT_PATCH
	key = hmu_keygen(ds->dip^mip, ds->mp, ds->prot);
	ih=(key<<6)>>(16+6);
	itag=key&0xffff;
	/* find all match hashed table and release it */
	for(i=0; (i<4); i++)
	{
		if ((hx = hmu_find_match(nathsi+4*ih+i, ds, itag)))
		{
			*(unsigned int*)hx=0;
			DC_STORE((unsigned int)hx, sizeof(nathashtab));
		}
	}
#endif
	return;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_find_mac(char *mac)
{
	unsigned short key, i, *machptr, idx;

	key = mach_keygen(mac);
	machptr = &mach[4*key];

	for(i=0; i<4; i++)
	{
		if(!machptr[i])
			continue;
		idx = machptr[i] & (~0x8000);
		/* found the same mac already exists? */
		if(macds[idx].v == 0)
			continue;
		if(macds[idx].vl == 0)
			continue;
		if(memcmp(mac, macds[idx].mac, 6))
			continue;
		return macds[idx].vidx;
	}
	return -1;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void hnat_learn_mac(char *hwadrp, char dir, char vid)
{
	struct iphdr *ip;
	natdes *ds, *ds1;
	int pppidx;
	struct poe_head *pppoe = 0;
	unsigned short *ptype;
	char tag = *(char *)(hwadrp + 12 + 1);
//printd("ml_head=%x\n", ml_head);
//printd("ml_head->used=%x\n", ml_head->used);
#ifdef CONFIG_SPECIAL_TAG
	if(!tag)
		return;
	tag = 1 << ((tag & 0x7) - 1);
#endif

	ptype=(unsigned short *)(hwadrp+12);
again:
	switch (*ptype)
	{
		case PPPOE_PACKET_TYPE:
			ip = (struct iphdr *)((unsigned int)(ptype+1) + 8); /* pppoe hdr + ppp hdr*/
			pppoe = (struct poe_head *)(ptype+1);
			break;
		case IP_PACKET_TYPE:
			ip = (struct iphdr *)(ptype+1);
			break;
		case VLAN_PACKET_TYPE:
			ptype+=2;	// next 4 bytes
			goto again;
		default:
			return;
	}
	/* packet incoming*/
	if(dir == 0)
	{
		HNAT_PRINTF("RECV> ip->src:%x, ip->dst:%x\n", ip->src, ip->dst);
#if 0 /* sniffer */
		if((unsigned int)(ip->protocol) == 0x6)
		{
			struct tcphdr *tcp = (struct tcphdr *)(((int)ip)+20);
			if(tcp->th_dport == 0x50)
			{
			printd("http packet\n");
			}
		}
#endif
	}
	else
	/*packet outgoing*/
	{
		HNAT_PRINTF("SEND> ip->src:%x, ip->dst:%x\n", ip->src, ip->dst);
	}

	HNAT_LOCK;
	ds = ml_head;
	while(ds->used)
	{
		/*outbound*/
		if(!ds->dmac &&
			(dir == 1)&&
			(ip->dst == ds->dip))
		{
			ds->dmac = macds_alloc(hwadrp, tag, vid);
		}
		else if(!ds->smac &&
			(dir == 0)&&
			(ip->src == LSB2SIP(ds->siplsb, ds->sipmsb)))
		{
			ds->smac = macds_alloc(hwadrp+6, tag, vid);
		}
		/*inbound*/
		else if(!ds->smac &&
			(dir == 1)&&
			(ip->dst == LSB2SIP(ds->siplsb, ds->sipmsb)))
		{
			ds->smac = macds_alloc(hwadrp, tag, vid);
		}
		else if(!ds->dmac &&
			(dir == 0)&&
			(ip->src == ds->dip))
		{
			ds->dmac = macds_alloc(hwadrp+6, tag, vid);
		}

		ds1 = &natds[ds->ml_next];

		if(ds->dmac && ds->smac)
		{
			/* It is a pppoe session, fill the session ID*/
			if(mip_info[ds->mipidx].poef & 0x8)
			{
				/* we do not learn session ID by broadcast/multicast address */
				if(!pppoe || (hwadrp[0] & 1) || (hwadrp[6] & 1))
				{
					HNAT_UNLOCK;
					return;
				}

				pppidx = (int)mip_info[ds->mipidx].poef & 0x7;
				if(((poe_info[pppidx].ver != 0x11) || (poe_info[pppidx].sid != pppoe->sid)))
				{
					poe_info[pppidx].ver = 0x11;
					poe_info[pppidx].code = 0;
					poe_info[pppidx].sid = pppoe->sid;
					poe_info[pppidx].len = 0x0001;
					poe_info[pppidx].ppp.prot = 0x0021;
					setpppoe(pppidx, (char *)&(poe_info[pppidx]));
				}
			}

			ml_del(ds);
			if (0 > natds_insert(ds))
			{
				HNAT_UNLOCK;
				HNAT_PRINTF("????fail to natds_insert()\n");
				return;
			}
		}
		ds = ds1;
	}
	HNAT_UNLOCK;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_learn_mac(char *hwadrp, char dir, char vid)
{
	if((ml_head == 0) || (ml_head->used == 0))
		return;

	hnat_learn_mac(hwadrp,dir,vid);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned short ip_cksum(unsigned short *p, unsigned int len)
{
	unsigned int sum = 0;
	int nwords = len >> 1;

	while (nwords-- != 0)
		sum += *p++;

	if (len & 1)
		sum += *(u_char *)p;

	/* end-around-carry */
	sum = (sum >> 16) + (sum & 0xffff);
	sum += (sum >> 16);
	return (~sum);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
int cta_hnat_rx_check(char *buf, char *pkt, unsigned int w0, unsigned int w1)
{
	rxinfo_t *rxinfo;
	natdes *ds;

	/* no support fragment packet */
	if((hnat_mode_g == CTA_HNAT_NOHNAT) || (w0 & DS0_IPF))
		return HNRC_OK;

	cta_hnat_learn_mac(pkt, 0, RX_VIDX(w0));

	if(w0 & DS0_NATH)
	{
		rxinfo = (rxinfo_t *)(buf+BUF_HW_OFS);

		if(w0 & DS0_INB)
			rxinfo->dir = 1;
		else
			rxinfo->dir = 0;

		ds = (natdes *)(hnat_addr((((rxinfo->hn_desc) << 8) >> 8)<<2));
		if(((unsigned int)ds<(unsigned int)&natds[0])||((unsigned int)ds>(unsigned int)&natds[NATD_SZ-1]))
		{
			printd("HNAT BUG: ds(%x) not in the valid addr range(%x-%x)\n",(unsigned int)ds,(unsigned int)&natds[0],(unsigned int)&natds[NATD_SZ-1]);
#ifdef CONFIG_HNAT_PATCH
			{
				unsigned short ih,itag;
				unsigned short oh,otag;
				unsigned long key;
				int hlen;
				struct iphdr *ip;
				struct tcphdr *tcp;

				ip = (struct iphdr *)(buf + BUF_IPOFF + BUF_HW_OFS);
				hlen = ip->ihl << 2;
				tcp = (struct tcphdr *)((char *)ip + hlen);

				{
					if(rxinfo->dir)
					{
						key = hmu_keygen(ip->dst^ip->src, tcp->dst, ip->protocol);
						oh=(key<<6)>>(16+6);
						otag=key&0xffff;
						HNAT_PRINTF("oh=%x tag=%x\n", oh, (unsigned long)otag);

						ds = hmu_find_natds(nathso+4*oh, rxinfo->dir, otag, ip->dst, ip->src,
											ip->protocol, tcp->dst, tcp->src);
					}
					else
					{
						key = hmu_keygen(ip->dst^ip->src, tcp->src, ip->protocol);
						ih=(key<<6)>>(16+6);
						itag=key&0xffff;
						HNAT_PRINTF("ih=%x tag=%x\n", ih, (unsigned long)itag);

						ds = hmu_find_natds(nathsi+4*ih, rxinfo->dir, itag, ip->src, ip->dst,
											ip->protocol, tcp->src, tcp->dst);
					}
					if(!ds)
						return -1;
				}
			}
#else
			return -1;
#endif
		}

//printd("%s(%d), DS0_INB=%x, TH_RST=%x, ds=%x,\n", __func__, __LINE__, (w0&DS0_INB), (w0&DS0_TRST), ds, rxinfo->mac);
#ifdef HNAT_REVERT_TCP_RST_PKT
		if(w0 & DS0_TRSF)
		{
			int diff, hlen;
			struct iphdr *ip;
			struct tcphdr *tcp;
			
			ip = (struct iphdr *)(buf + BUF_IPOFF + BUF_HW_OFS);
			hlen = ip->ihl << 2;
			tcp = (struct tcphdr *)((char *)ip + hlen);
			if(w0 & DS0_INB)
			{
				unsigned int mip = mip_info[ds->mipidx].mip;

				diff = (ip->dst >> 16);
				diff += (ip->dst & 0x0ffff);
				ip->dst = mip;
				diff -= (mip>>16);
				diff -= (mip & 0x0ffff);
			
				diff += tcp->dst;
				tcp->dst = ds->mp;
				diff -= tcp->dst;
			}
			else
			{
				diff = (ip->src >> 16);
#if 1
				if(ds->siplsb!=0)	/* not superDMZ */
#endif
				{
					diff += (ip->src & 0x0ffff);
					ip->src = LSB2SIP(ds->siplsb, ds->sipmsb);
					diff -= ds->siplsb;
				}
				diff -= ds->sipmsb;
				
				diff += tcp->src;
				tcp->src = ds->sp;
				diff -= ds->sp;
			}
			FIX_CHECKSUM(diff, tcp->chksum);

			ip->ttl += 1;
			ip->chksum = 0;
			ip->chksum = ip_cksum((unsigned short *)ip, hlen);
			return 0;
		}
#endif
		return (int)ds;
	}
	return 0;
}

#ifdef __ECOS
/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_copy_cbinfo(char *iphdr, int len, natdes *ds, cb_t *cb)
{
	char poe;
	short mac_index;

	if(cb->dir || !ds || cb->force)
	{
		poe = 0;
		if(cb->force)
		{
			cb->force = ~cb->force;
			mac_index = macds_alloc((char *)ds, 0, 0);
		}
		else if(ds)
			mac_index = ds->smac;
		else
		{
			cb->force = ~cb->force;
			mac_index = 0;
		}
	}
	else
	{
		poe = mip_info[ds->mipidx].poef;
		mac_index = ds->dmac;
	}
	cb->poe = poe;
	cb->pkt_len = len;
	cb->mac_index = mac_index;
	cb->pkt = (unsigned int)iphdr;
}
#else
extern cta_eth_t *pcta_eth;
/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_fast_send(char *iphdr, int len, unsigned long key, semi_cb_t *semi_cb)
{
	char poe;
	short mac_index;
	natdes *ds = (natdes *)semi_cb->ds;
	int dir = (semi_cb->w0 & DS0_INB) ? 1 : 0;

	if(dir == 0)
	{
		poe = mip_info[ds->mipidx].poef;
		mac_index = ds->dmac;
	}
	else
	{
		poe = 0;
		mac_index = ds->smac;
	}

	semi_cb->w1 = (poe << DS1_PSI_S) | (0x00ffffff & (int)iphdr);
	semi_cb->w0 = (len << DS0_LEN_S) | DS0_TX_OWNBIT | mac_index;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31))
	pcta_eth->sc_main->netdev_ops->ndo_start_xmit((void *)key, pcta_eth->sc_main);
#else
	pcta_eth->sc_main->hard_start_xmit((void *)key, pcta_eth->sc_main);
#endif

	return;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_set_semicb(unsigned long buf_cb, void *hw_nat, unsigned int w0, unsigned int w1)
{
	semi_cb_t *semi_cb = (semi_cb_t *)buf_cb;

	if(hw_nat)
	{
		/* put rx descriptor to socket buffer control block*/
		semi_cb->flags |= M_HWMOD;
		semi_cb->ds = hw_nat;
		semi_cb->w0 = w0;
		semi_cb->w1 = w1;
		semi_cb->magic = SEMI_CB_MAGIC;
	}
	else
	{
		semi_cb->flags &= ~M_HWMOD;
	}
}
#endif

#ifdef CONFIG_SPECIAL_TAG
/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
unsigned short cta_hnat_get_tpid(natdes *ds, cb_t *cb)
{
	int idx;
	if(cb->force)
		return cb->cb_stag;
	if(cb->dir)
		idx = ds->smac;
	else
		idx = ds->dmac;
	/* output port mask add priority assignment */
	return (macds[idx].stag | cb->cb_stag);
}
#endif

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void sess_dc_inval(natdes *ds)
{
	DC_INVAL((unsigned int)ds, sizeof(natdes));
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void sess_dc_store(natdes *ds)
{
	DC_STORE((unsigned int)ds, sizeof(natdes));
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *      \brief
 *      \param
 *      \return
 +----------------------------------------------------------------------------*/
void cta_hnat_dump(short cmd, void *parm)
{
	int i,j;
	natdes *ds;
	nathashtab *hx;

	if (cmd & HNAT_DBG_NATDS)
	{
		for(i=0; i<CTA_HNAT_PUBLIC_IP_MAX_NUM ; i++)
		{
			if(mip_info[i].mip)
				printd("[%d] MIP=%x, poe=%d, poe_sid=%x\n", i, mip_info[i].mip, mip_info[i].poef, poe_info[i].sid);
		}
		printd("----------natds------------\n");
		for(i=1; i<NATD_SZ; i++)
		{
			ds=&natds[i];
			if(ds->used == 0)
				continue;
			printd("[%d]: c=%d, sp:%x, sv=%d, smac=", i-1, ds->counter, ds->sp, macds[ds->smac].vidx);
			for(j=0;j<6;j++)
				printd("%02x",  macds[ds->smac].mac[j]);
			printd(", dv=%d, dmac=", macds[ds->dmac].vidx);
			for(j=0;j<6;j++)
				printd("%02x",  macds[ds->dmac].mac[j]);
			printd("\n");
			printd("      p:%x, mp:%x, siplsb:%x, dip:%x, sp:%x, dp:%x, midx=%d\n", ds->prot, ds->mp, ds->siplsb,
					ds->dip, ds->sp, ds->dp, ds->mipidx);
		}
		printd("\n");
	}

	if (cmd & HNAT_DBG_HASHO)
	{
		printd("----------ohash------------\n");
		for(i=0; i<NATH_SZ; i++)
		{
			for(j=0; j<4; j++)
			{
				hx = &natho[4*i+j];
				if(hx->v)
					printd("{%d|%d}: idx=%d, tag=%x, s=%d\n", i, j, hx->idx, hx->tag, hx->s);
			}
		}
		printd("\n");
	}

	if (cmd & HNAT_DBG_HASHI)
	{
		printd("----------ihash------------\n");
		for(i=0; i<NATH_SZ; i++)
		{
			for(j=0; j<4; j++)
			{
				hx = &nathi[4*i+j];
				if(hx->v)
					printd("{%d|%d}: idx=%d, tag=%x, s=%d\n", i, j, hx->idx, hx->tag, hx->s);
			}
		}
		printd("\n");
	}

	printd("NATDS:%x Free:%d\n", (unsigned int)natds, free_natds_num_g);
	printd("NATHO:%x NATHI:%x\n", (unsigned int)natho, (unsigned int)nathi);
	printd("MACDS:%x\n", (unsigned int)macds);
	printd("MACH:%x\n", (unsigned int)mach);
	printd("NATDS:%x\n", (unsigned int)natds);
	printd("NATE_ML:%x\n", (unsigned int)ml_head);
}

#ifndef __ECOS
EXPORT_SYMBOL(cta_hnat_cfgmode);
EXPORT_SYMBOL(cta_hnat_cfgip);
EXPORT_SYMBOL(cta_hnat_get_statistic);
EXPORT_SYMBOL(cta_hnat_alloc);
EXPORT_SYMBOL(cta_hnat_attach);
EXPORT_SYMBOL(cta_hnat_detach);
EXPORT_SYMBOL(cta_hnat_learn_mac);
EXPORT_SYMBOL(cta_hnat_rx_check);
EXPORT_SYMBOL(cta_hnat_fast_send);
EXPORT_SYMBOL(cta_hnat_set_semicb);
EXPORT_SYMBOL(hnat_mode_g);
#endif
