#include "gdma.h"

static volatile int dma_descr_windex;
static volatile int dma_descr_rindex;
static dma_descriptor dma_descr[GDMA_DESCR_COUNT];

static spinlock_t gdma_lock;
struct tasklet_struct gdma_tasklet;

#define __GDMA_LOCK()       spin_lock(&gdma_lock)
#define __GDMA_UNLOCK()     spin_unlock(&gdma_lock)

dma_descriptor *pdma_descr;

static inline void gdma_interrupt_mask(void)
{
    GDMAREG_WRITE32(DMA_INTR_MASK, (DMA_INTR_MASK_COUNT | DMA_INTR_MASK_TIME));
}

static inline void gdma_interrupt_unmask(void)
{
    GDMAREG_WRITE32(DMA_INTR_MASK, DMA_INTR_MASK_TIME);
}

static inline int get_dma_hw_descr_rindex(void)
{
    return ((GDMAREG_READ32(DMA_READ_DESCR_ADDR) - PHYSICAL_ADDR((u32)pdma_descr)) / sizeof(dma_descriptor));
}

/* control variable to insert NULL descr. for ISR to sync. dma_descr_rindex */
static volatile int num_after_sync_descr;

static void gdma_irq_tasklet(void *data)
{
    int dma_index;
    dma_callback cb;

    GDMAREG_WRITE32(DMA_INTR_STATUS, GDMAREG_READ32(DMA_INTR_STATUS));

    dma_index = get_dma_hw_descr_rindex();
    while (dma_descr_rindex != dma_index)
    {
        if (pdma_descr[dma_descr_rindex].ctrl == GDMA_CTRL_DONE_STOP)
        {
            if (pdma_descr[dma_descr_rindex].sw_callback)
            {
                //DMADBG("	ISR	callback	%d\n",	dma_descr_rindex);

                cb = (dma_callback)pdma_descr[dma_descr_rindex].sw_callback;
                cb((void  *)&pdma_descr[dma_descr_rindex]);

                pdma_descr[dma_descr_rindex].sw_callback = 0;
                pdma_descr[dma_descr_rindex].sw_priv = 0;
                pdma_descr[dma_descr_rindex].sw_priv2 = 0;
            }

            pdma_descr[dma_descr_rindex].sw_inuse = 0;
        } 
        else if (pdma_descr[dma_descr_rindex].ctrl == GDMA_CTRL_DONE_SKIP)
        {
            /* do nothing intentionally */
        }
        else
        {
            DMAPANIC("GDMA bug: descr. %d not done\n", dma_descr_rindex);
            break;
        }

        dma_descr_rindex = (dma_descr_rindex + 1)% GDMA_DESCR_COUNT;
    }

    gdma_interrupt_unmask();
}

static irqreturn_t gdma_isr(int irq, void *data)
{
    gdma_interrupt_mask();

    tasklet_schedule(&gdma_tasklet);

    return IRQ_HANDLED;
}

/* internal function; should be called with GDMA_LOCK() */
static inline short ___get_next_stopped_descr_idx(short start_idx)
{
    int ret_idx = -1;
    int idx = (start_idx + 1) % GDMA_DESCR_COUNT;
    int dma_index;

    dma_index = get_dma_hw_descr_rindex();
    while(1)
    {
        if (idx == dma_index) 
            break;

        if (pdma_descr[idx].ctrl == GDMA_CTRL_DONE_STOP)
        {
            if (pdma_descr[idx].sw_inuse == 0)
                ret_idx = idx;

            break;
        }
        else if (pdma_descr[idx].ctrl == GDMA_CTRL_DONE_SKIP)
        {
            if (pdma_descr[idx].sw_inuse == 0)
            {
                pdma_descr[idx].ctrl = GDMA_CTRL_DONE_STOP;
                ret_idx = idx;
                break;
            }
        }
        else
        {
            break;
        }

        idx = (idx + 1) % GDMA_DESCR_COUNT;
    }

    return ret_idx;
}

/* the returned TX descr will be guaranteed to be consecutive */
/* return negative value on allocation failure */
int gdma_get_free_descr(short *descr_idx, int count)
{
    int ret = 0;
    short idx, next_idx;
    int i;
#if defined(DO_RETRY_FOR_CONSECUTIVE_ALLOCATION)
    int j;
    int skip = 0;
#endif

    if(NULL==pdma_descr)
        return -1;

    __GDMA_LOCK();

    if((count + num_after_sync_descr) >= INSERT_SYNC_DESCR_THRESHOLD)
    {
        idx = dma_descr_windex;
        next_idx =  ___get_next_stopped_descr_idx(idx);
        if (next_idx < 0)
        {
            ret = -1;
            goto EXIT;
        }

        dma_descr_windex = next_idx;
        num_after_sync_descr = 0;

#if defined(USE_GDMA_CW_MACRO)
        pdma_descr[idx].src_addr = 0;
        pdma_descr[idx].cksum_length = 0;
        pdma_descr[idx].operation_length = 0;
        pdma_descr[idx].ctrl_word0 = GDMA_INTR_CW0(idx, GDMA_OP_CSUM, 0);
#else
        pdma_descr[idx].operation = GDMA_OP_CSUM;
        pdma_descr[idx].src_addr = 0;
        pdma_descr[idx].cksum_length = 0;
        pdma_descr[idx].operation_length = 0;
        pdma_descr[idx].sw_inuse = 1;

        pdma_descr[idx].ctrl = GDMA_CTRL_GO;
#endif

        gdma_kick();
    }

    idx = dma_descr_windex;
    i = 0;
    while (i < count)
    {
        next_idx =  ___get_next_stopped_descr_idx(idx);
        if (next_idx < 0)
        {
            /* at least one descr shall be used as STOP descriptor after the allocation, otherwise the hardware will not stop */
            ret = -1;
            break;
        }

#if defined(DO_RETRY_FOR_CONSECUTIVE_ALLOCATION)
        if (((i + 1) < count) && (next_idx != ((idx + 1) % GDMA_DESCR_COUNT)))
        {
            //DMADBG(" ## %d %d ## \n", idx, next_idx);

            /* none consecutive allocation found */
            dma_descr_windex = next_idx;

            for (j = 0; j < i; j++) 
            {
                pdma_descr[descr_idx[j]].ctrl = GDMA_CTRL_DONE_SKIP;
                skip++;
            }
            pdma_descr[idx].ctrl = GDMA_CTRL_DONE_SKIP;
            skip++;

            if (skip > (GDMA_DESCR_COUNT / 4))
            {
                /* to many skips, give up */
                ret = -2;
                break;
            }
            else
            {
                idx = next_idx;
                i = 0;
                continue;
            }
        }
#endif

        descr_idx[i] = idx;
        idx = next_idx;
        i++;
    }

    if (ret == 0)
    {
        for (i = 0; i < count; i++)
        {
            pdma_descr[descr_idx[i]].sw_inuse = 1;
        }
        dma_descr_windex = idx;

        num_after_sync_descr += count;
    }

EXIT:
    __GDMA_UNLOCK();

    return ret;
}

static int __init gdma_init(void)
{
    int irq = GDMA_SW_INTR_NUM;
    volatile u32 *ic= (u32 *) IC_BASE;

    /* mask all GDMA interrupt */
    gdma_interrupt_mask();

    GDMAREG_WRITE32(DMA_ENABLE, 0);

    spin_lock_init(&gdma_lock);

    memset(&dma_descr[0], 0, sizeof(dma_descriptor) * GDMA_DESCR_COUNT);
    dma_cache_wback_inv((u32 ) &dma_descr[0], sizeof(dma_descriptor) * GDMA_DESCR_COUNT);

    pdma_descr = (dma_descriptor *) UNCACHED_ADDR((u32) &dma_descr[0]);
    pdma_descr[GDMA_DESCR_COUNT - 1].eor = 1;

    dma_descr_windex = 0;
    dma_descr_rindex = 0;
    num_after_sync_descr = 0;

    GDMAREG_WRITE32(DMA_INTR_STATUS, (DMA_INTR_MASK_COUNT | DMA_INTR_MASK_TIME));
    GDMAREG_WRITE32(DMA_DESCR_BADDR, PHYSICAL_ADDR((u32)pdma_descr));

    ic[ISR1] &= 0xfffffff0UL;
    ic[ISR1] |= GDMA_INTR_NUM;

    tasklet_init(&gdma_tasklet, (void (*)(unsigned long)) gdma_irq_tasklet, (unsigned long) 0);

    if(0 != request_irq(irq, gdma_isr, 0, "GDMA", 0))
        panic("gdma_init(): request IRQ failed.\n");

    GDMAREG_WRITE32(DMA_INTR_THRESHOLD, 0);

    gdma_interrupt_unmask();

    GDMAREG_WRITE32(DMA_ENABLE, 1);
    GDMAREG_WRITE32(DMA_KICK, 1);

    printk("gdma: Registered IRQ %d, %d descriptors\n", irq, GDMA_DESCR_COUNT);

    return 0;
}

static __exit void gdma_exit(void)
{
    int irq = GDMA_SW_INTR_NUM;

    GDMAREG_WRITE32(DMA_ENABLE, 0);

    GDMAREG_WRITE32(DMA_INTR_MASK, (DMA_INTR_MASK_COUNT | DMA_INTR_MASK_TIME));
    GDMAREG_WRITE32(DMA_INTR_STATUS, (DMA_INTR_MASK_COUNT | DMA_INTR_MASK_TIME));

    /* XXX: TODO - cleanup for pending opertions */
    free_irq(irq, 0);

    tasklet_kill(&gdma_tasklet);

    pdma_descr = NULL;
}

module_init(gdma_init)
module_exit(gdma_exit)

