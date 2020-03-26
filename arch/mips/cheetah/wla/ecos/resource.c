/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file resource.c
*   \brief 
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <stdlib.h>
#include <sys/mbuf.h>
#include <wbuf.h>
#include <mbuf_debug.h>
#include <wla_debug.h>
#include <os_api.h>
#include <ecos_wla.h>
#include <eth_drv.h>

#undef malloc
#undef free

unsigned int wbuf_mc_used = 0;
unsigned int wbuf_mem_used = 0;

/*=============================================================================+
| Functions
+=============================================================================*/
MAC_INFO *wla_mac_info()
{
	return &(my_wla->mac_info);
}

unsigned int wla_current_time(void)
{
	return os_current_time();
}

void wbuf_free(struct wbuf *wb)
{
	unsigned int ra;

	ra = (unsigned int) __builtin_return_address(0);

	if(!wb)
		return;
#ifdef CONFIG_CLUSTER_TRACE
	if(addr_is_mcluster((char *)wb))
		mcl_trace_tag((char *)wb, 31);
#endif

#ifdef CONFIG_WBUF_CLONE
	if((wb->flags & (WBUF_CHAIN | WBUF_EXT)) && wb->wb_next)
		wbuf_free(wb->wb_next);

	if(wb->flags & WBUF_EXT)
		return;
#else
	if((wb->flags & WBUF_CHAIN) && wb->wb_next)
		wbuf_free(wb->wb_next);
#endif

	if(wb->flags & WBUF_FIXED_SIZE)
	{
		if(!addr_is_mcluster((char *)wb))
			WLA_DBG(WLAERROR, "%s(%d), !!!!!!!!!free cluster but fail!!!!!!!!\n", __func__, __LINE__);
		/* put ra for debug */
		((int *)wb)[1] = ra;

		MCLFREE(wb);
		wbuf_mc_used--;
	}
	else
	{
#ifdef CONFIG_ZOMBIE_FREE		
extern char __rfc_start[];
#else		
extern char __heap1[];
#endif
extern char __heap1_end[];


		if(addr_is_mcluster((char *)wb))
		{
			WLA_DBG(WLAERROR, "%s(%d), !!!!!!!!!free mem but fail!!!!!!!!\n", __func__, __LINE__);
			panic("");
		}
#ifdef CONFIG_ZOMBIE_FREE		
		if(((unsigned int)PHYSICAL_ADDR(wb) < (unsigned int)PHYSICAL_ADDR(__rfc_start)) || ((unsigned int)PHYSICAL_ADDR(wb) > ((unsigned int)PHYSICAL_ADDR(__heap1_end))))
#else		
		if(((unsigned int)PHYSICAL_ADDR(wb) < (unsigned int)PHYSICAL_ADDR(__heap1)) || ((unsigned int)PHYSICAL_ADDR(wb) > ((unsigned int)PHYSICAL_ADDR(__heap1_end))))
#endif		
		{
			WLA_DBG(WLAERROR, "%s(%d), !!!!!!!!!free mem but fail!!!!!!!!\n", __func__, __LINE__);
			panic("");
		}
		free((void *)CACHED_ADDR(wb));
		wbuf_mem_used--;
	}
}

struct wbuf *wbuf_alloc(const u32 size)
{
	struct wbuf *wb;
	unsigned int ra=0;

	if(size)
		wb = (struct wbuf *)malloc(size);
	else
	{
		MCLALLOC(wb, M_DONTWAIT, 0);
		if(wb)
		{
			ra = ((int *)wb)[1]; /* for debug */ 
		}
	}

	if(wb)
	{
#ifdef CONFIG_CLUSTER_TRACE
		mcl_trace_tag((char *)wb, 0);
#endif
		memset(wb, 0, sizeof(struct wbuf));
#if 0
		if(!size) /* debug */
		{
			memset((char *)wb+sizeof(struct wbuf), 0x5a, 64);
		}
#endif
		if(size == 0)
		{
			wb->flags = WBUF_FIXED_SIZE;
			wbuf_mc_used++;
#if !defined(CONFIG_NET_NONCACHED_CLUSTER)
			DCACHE_FLUSH((unsigned int)wb, sizeof(struct wbuf));
#endif
		}
		else
			wbuf_mem_used++;
	}	
	return wb;
}

int wbuf_check_header(struct wbuf *wb)
{
	if(0 == (wb->flags & WBUF_FIXED_SIZE))
		return 1;

	if(wb->pkt_len + wb->data_off > MCLBYTES)
		panic("wbuf_clone() too large data\n");

	return 0;
}

void wbuf_flush_header(struct wbuf *wb_old, struct wbuf *wb_new, int copy)
{
	char *dst, *src;
	char off, new_off;
	int len, new_len, flush_len;

	off = wb_old->data_off;
	len = wb_old->pkt_len + off;
#if defined(WBUF_IP_HEADER_ALIGN)
	/* Maybe wbuf from ether driver data offset is align to 4 bytes in cheetah platform.
	   In worst, IP header would not align to 4 bytes and occur exception.
	   Shift wbuf data offset to do IP header alignment */
	if(((int)((char *)wb_old + off) & 3) == 0)
	{
		new_off = off + 2;
		new_len = len + 2;
	}
	else
#endif
	{
		new_off = off;
		new_len = len;
	}
	src = (char *)CACHED_ADDR(wb_old);
	dst = (char *)CACHED_ADDR(wb_new);
	if(copy)
	{
		DCACHE_INVALIDATE((unsigned int)src, DEF_SW_PATH_BUF_SIZE);
		memcpy(dst, src, off);
		memcpy((char *)dst + new_off, (char *)src + off, wb_old->pkt_len);
		flush_len = new_len;
	}
	else
	{
		/* Only copy wbuf length, then use WBUF_EXT and wb_next to indicate original wbuf to send */
		memcpy(dst, src, sizeof(struct wbuf));
		wb_new->flags |= WBUF_EXT;
		wb_new->wb_next = wb_old;
		mclrefcnt(wb_old)++;
		flush_len = sizeof(struct wbuf);
	}
	/* flush wbuf header to ensure get right information by non-cached wb_new */
	DCACHE_FLUSH((unsigned int)dst, flush_len);

	wb_new->data_off = new_off;
}

#ifdef CONFIG_WBUF_CLONE
static struct wbuf *_wbuf_clone(struct wbuf *wb_old, struct wbuf *wb_new)
{
	if(wbuf_check_header(wb_old))
		return 0;
	if(!wb_new)
		return 0;

	memset(wb_new, 0, sizeof(struct wbuf));
	wbuf_flush_header(wb_old, wb_new, 0);

	return wb_new;
}
#endif

static struct wbuf *_wbuf_copy(struct wbuf *wb_old)
{
	struct wbuf *wb_new;

	if(wbuf_check_header(wb_old))
		return 0;
	if(!(wb_new = wbuf_alloc(0)))
		return 0;

	wbuf_flush_header(wb_old, wb_new, 1);
	
	return wb_new;
}

struct wbuf *wbuf_copy(struct wbuf *wb_old, struct wbuf *wb_new)
{
#ifdef CONFIG_WBUF_CLONE
	if(wb_old->flags & WBUF_TX_REQ_STATUS)
		return _wbuf_copy(wb_old);
	else
		return _wbuf_clone(wb_old, wb_new);
#else
	return _wbuf_copy(wb_old);
#endif
}

struct wbuf *wbuf_txbuf(struct wbuf *wb_old, struct wbuf *wb_new)
{
	if(wb_new->flags & WBUF_EXT)
		return wb_old;
	else
		return wb_new;
}

int wbuf_datap(struct wbuf *wb_old, struct wbuf *wb_new)
{
	if(wb_new->flags & WBUF_EXT)
		return (int)wb_old + wb_new->data_off;
	else
		return (int)wb_new + wb_new->data_off;
}

void wla_lock(void)
{
	if(os_thread_current() != my_wla->wla_lock_thread) 
	{
		while(os_mutex_lock( my_wla->mutex ) != OS_SUCCESS)
			continue;
		my_wla->wla_lock_thread = os_thread_current();
	}
		
	my_wla->wla_lock_count++;
}

void wla_unlock(void)
{
	if(my_wla->wla_lock_thread) 
	{
		if(--my_wla->wla_lock_count == 0)
		{
			my_wla->wla_lock_thread = 0; 
			os_mutex_unlock(my_wla->mutex);
		}
	}
}

void wla_sleep(unsigned int sleep_tick)
{
	os_thread_sleep(sleep_tick);
}
 
char *wmaddr_ntoa (char *mac)
{
	return ether_ntoa((struct ether_addr *)mac);
}

int wla_forward_to_ethernet(struct wbuf *wb, ehdr *ef)
{
	struct eth_drv_sg sg_list[2];
	struct eth_drv_sc *sc;
	MAC_INFO *info = WLA_MAC_INFO;

	//WLA_DBG(WLADEBUG, "RX my data frame, wb->flags = 0x%x\n", wb->flags);
	//diag_dump_buf(ef, wb->pkt_len);

	if(!info->mac_is_ready)
	{
		WLA_DBG(WLAWARN, "%s(), mac is not ready!\n", __func__);
		wbuf_free(wb);
		return 0;
	}
	sg_list[0].buf = (int)ef;
	sg_list[0].len = wb->pkt_len;
	sg_list[1].buf = 0;

	sc = my_wla->vifs[info->mbss[wb->bss_desc].vif_id].sc;
	if(eth_drv_recv(sc, &sg_list[0], 1, (void *)wb))
	{
		wbuf_free(wb);
		return 0;	
	}

	return 1;
}


void wla_add_timer(unsigned int interval)
{
	timeout_lo((OS_FUNCPTR)wla_hw_sync, 0, interval);
}

void wla_del_timer(void)
{
	untimeout_lo((OS_FUNCPTR)wla_hw_sync, 0);
}

#ifdef CONFIG_MIPS16
inline void CODE_NOMIPS16 dcache_store(unsigned int a, int l)
{
    HAL_DCACHE_STORE(a,l);
}

inline void CODE_NOMIPS16 dcache_flush(unsigned int a, int l)
{
    HAL_DCACHE_FLUSH(a,l);
}

inline void CODE_NOMIPS16 dcache_invalidate(unsigned int a, int l)
{
    HAL_DCACHE_INVALIDATE(a,l);
}
#endif
