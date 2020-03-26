/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*
*   \file 
*   \brief
*   \author Montage
*/
#ifndef _OS_COMPAT_H_
#define _OS_COMPAT_H_


/*=============================================================================+
| Header File                                                                  |
+=============================================================================*/
#ifdef CONFIG_LINUX_WLA
/* LINUX */
#include <linux/kernel.h>
#include <asm/mach-cheetah/common.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>
//#include "../../../../../net/mac80211/ieee80211_i.h"

#if defined(CONFIG_CHEETAH_FPGA)
#ifndef CONFIG_FPGA
#define CONFIG_FPGA
#endif
#endif

#else
/* ECOS */
#include <sys/types.h>
#include <mt_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include <cyg/infra/cyg_type.h>
//#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_cache.h>
//#include <cyg/infra/diag.h>
//#include <cyg/hal/drv_api.h>
//#include <netdev.h>
#include <machine/param.h>
//#include <ether.h>
#include <time.h>
#include <os_api.h>
#include <delay.h>
#include <str_util.h>
//#include <mbuf_debug.h>

#include <stdlib.h>

#include <cdb_api.h>
#include <cdb_info.h>
#include <common.h>

#endif
#include <mac_ctrl.h>
#include <wbuf.h>
#include <mac_common.h>

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define WLA_MAC_INFO	wla_mac_info()
int wla_forward_to_ethernet(struct wbuf *wb, ehdr *ef);

#define BHDR_LOCK()		wla_lock()
#define BHDR_UNLOCK()	wla_unlock()

#define WLA_DATA_FRAME_ETH_FORWARD(a,b) wla_forward_to_ethernet(a,b)


#define WBUF_ALLOC				wbuf_alloc	
#define WBUF_FREE				wbuf_free
#define WBUF_CLONE				wbuf_clone
#define WBUF_COPY				wbuf_copy
#define WBUF_COPY_TXBUF			wbuf_txbuf
#define WBUF_COPY_DATAP			wbuf_datap



//MEM Flag
#define DMA_MALLOC_CACHED       0x0
#define DMA_MALLOC_UNCACHED     0x1
#define DMA_MALLOC_BZERO        0x2		/* bzero the allocated memory */
#define DMA_MALLOC_ALIGN        0x4		/* allocate more reserved space to align */
#define DMA_MALLOC_ASSERTION    0x8		/* assert when the malloc is failed */
#define DMA_MALLOC_ATOMIC       0x10	/* use atomic memory pool   for linux  */


//os time
#define WLA_CURRENT_TIME		wla_current_time()

#if defined(CONFIG_FPGA)
/* adjust the pre-tbtt to be 1/3 of beacon interval */
#define PRE_TBTT_TIME(beacon_interval)  (((beacon_interval) * 1) / 3)
#else
/* minimize pretbtt time since ASIC is much faster */
#define PRE_TBTT_TIME(beacon_interval)  (((beacon_interval) * 1) / 8)
#endif

#ifdef CONFIG_LINUX_WLA

#define DBG(...)				printk(KERN_DEBUG __VA_ARGS__)

#define WLA_DUMP_BUF(a,l)		wla_dump_buf(a,l,8)
#define WLA_DUMP_BUF_16BIT(a,l)	wla_dump_buf(a,l,16)
#define WLA_DUMP_BUF_32BIT(a,l)	wla_dump_buf(a,l,32)

#define WLA_GETS		wla_gets

#define WLA_CDB_GET		wla_cdb_get_int

#define $parm_hw_mode               0
#define $parm_wl_power_backoff      1
#define $parm_rfchip                2
#define $parm_ex_ldo                3
#define $wl_forward_mode			4

#define WLA_MFREE(a)				mac_free(a)
#define WLA_MALLOC(size, flags)		mac_malloc(size, flags)

//Cache
#define DCACHE_LINE_SIZE            16
#define DCACHE_INVALIDATE(a,l)      dma_map_single(NULL, (void *)a, l, DMA_FROM_DEVICE)
#define DCACHE_FLUSH(a,l)           dma_map_single(NULL, (void *)a, l, DMA_BIDIRECTIONAL)
#define DCACHE_STORE(a,l)           dma_map_single(NULL, (void *)a, l, DMA_TO_DEVICE)


//endian
#define htows cpu_to_le16
#define htowl cpu_to_le32
#define wtohs cpu_to_be16
#define wtohl cpu_to_be32


#else

#define DBG(...)				diag_printf(__VA_ARGS__)

#define WLA_DUMP_BUF			diag_dump_buf
#define WLA_DUMP_BUF_16BIT		diag_dump_buf_16bit
#define WLA_DUMP_BUF_32BIT		diag_dump_buf_32bit

#define WLA_GETS			gets

#define WLA_MFREE(a)				dma_mfree(a)
#define WLA_MALLOC(size, flags)		dma_malloc(size, flags)

#define DCACHE_LINE_SIZE			HAL_DCACHE_LINE_SIZE
#define DCACHE_INVALIDATE(a,l)		HAL_DCACHE_INVALIDATE(a,l)
#define DCACHE_FLUSH(a,l)			HAL_DCACHE_FLUSH(a,l)
#define DCACHE_STORE(a,l)			HAL_DCACHE_STORE(a,l)

#define UMAC_REG_BASE    0xaf003000UL
#define ANA_BASE    (0xaf005800)
#define ANAREG(reg) ((volatile unsigned int*)(ANA_BASE))[reg]

#define	GPREG(reg) ((volatile unsigned int*)(GPIO_BASE))[reg]

#define WLA_CDB_GET		cdb_get_int

int fls(int x);

#endif

/*=============================================================================+
| Function                                                                     |
+=============================================================================*/
MAC_INFO *wla_mac_info(void);
int wla_forward_to_ethernet(struct wbuf *wb, ehdr *ef);

void wla_gets(char *buf);
void *wla_zalloc(size_t size);

void wla_dump_buf(void* buf, u32 len, u8 offset);

extern int wla_cdb_get_int(unsigned short index, int def_val); 

void mac_free(void *ptr);
void *mac_malloc(unsigned int size, unsigned char flags);

struct wbuf *wbuf_alloc(u32 size);
void wbuf_free(struct wbuf *wb);
struct wbuf *wbuf_clone(struct wbuf *wb);
struct wbuf *wbuf_copy(struct wbuf *wb_old, struct wbuf *wb_new);
struct wbuf *wbuf_txbuf(struct wbuf *wb_old, struct wbuf *wb_new);
int wbuf_datap(struct wbuf *wb_old, struct wbuf *wb_new);

void wm_ba_event(u8 cmd, u8 tid, u32 sta);

unsigned int wla_current_time(void);
char *wmaddr_ntoa (char *mac);
char *wmaddr_aton (char *mac);

void wla_lock(void);
void wla_unlock(void);

void wla_sleep(unsigned int s_tick);

void wla_add_timer(unsigned int interval);
void wla_del_timer(void);
void wla_txfail_timer(unsigned long data);
void wla_txfail_chk(void);

#ifdef CONFIG_CHEETAH_SMART_CONFIG
int smart_state(char *ptr, unsigned int len);
int smartcfg_process(struct rx_q *rxq);
extern u32 __smartcfg_enable;
#endif

#endif

