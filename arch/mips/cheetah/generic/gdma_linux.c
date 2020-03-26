#include "gdma.h"

#include <linux/string.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/checksum.h>

#include "gdma_test_common.h"

/* include only one of following header file to select the test cases */
#include "gdma_test.h"	    /*	first, original 192 test cases */
//#include "gdma_test2.h"	/* copy length from 60~70 bytes, 16 x 11 test cases */

/* gdma_basic_test_cmd() can only do POLLING mode verification with Linux GDMA driver enabled */
//#define INTERRUPT_VERIFICATION
//#define TIMER_INTERRUPT_VERIFICATION
    #define TEST_TIMER_THRESHOLD    10    // in ms  

#define CKSUM_VERIFICATION

#define DMA_DBG1 (DMA_REGBASE + 0x20)
#define DMA_DBG2 (DMA_REGBASE + 0x24)

unsigned char* mem_compare(unsigned char *p1, unsigned char *p2, int length)
{
#if 1
    return (unsigned char *) memcmp((void *)p1, (void *)p2, length);
#else
    int i;
    
    for (i = 0; i < length; i++)
    {
        GDMAREG_WRITE32(DMA_DBG1, p1[i]);
        GDMAREG_WRITE32(DMA_DBG2, p2[i]);

        if (p1[i] != p2[i])
            return p1;
    }
    
    return 0;
#endif
}

/* unaligned copy test */
unsigned char dma_test_dest[MAX_DMA_TEST_LENGTH+4];

int gdma_basic_test_cmd(void *data)
{
    int ret;
    short descr_idx[MAX_OPS];
    int i;
    int j;
    int op_count;
    int idx;
    volatile unsigned char *psrc, *pdst;

    unsigned char *p1, *p2;
    int iteration = 0;

#if defined(INTERRUPT_VERIFICATION)
    volatile unsigned long *ic = (volatile unsigned long *) IC_BASE;
#endif

    psrc = (unsigned char *)UNCACHED_ADDR(dma_test_pattern);
    pdst = (unsigned char *)UNCACHED_ADDR(&dma_test_dest[4]);

#if defined(TIMER_INTERRUPT_VERIFICATION)
    GDMAREG_WRITE32(DMA_INTR_THRESHOLD, (((TEST_TIMER_THRESHOLD-1)<<16)|3));    //	0: 1ms
    GDMAREG_WRITE32(DMA_INTR_MASK, 0);
#elif defined(INTERRUPT_VERIFICATION)
    GDMAREG_WRITE32(DMA_INTR_THRESHOLD, 0);	//	0:	raise interrupt	per	descriptor done,	1: raise interrupt on 2	descriptors	done
    GDMAREG_WRITE32(DMA_INTR_MASK, DMA_INTR_MASK_TIME);
#endif

    while (1)
    {
        iteration++;
        DMADBG("\nIteration %d", iteration);

        for (i = 0; i < (sizeof(testparm) / sizeof(struct test_params)); i++)
        {
            op_count = testparm[i].op_count;
            if (op_count == 0)
                continue;

            ret = gdma_get_free_descr(descr_idx, op_count);
            if (0 > ret)
            {
                DMADBG("EXIT\n");
                return 0;
               // DMAPANIC("No free descr.");
            }

            //DMADBG("Test item %d", i);
            /* fill in descr */
            for (j = 0; j < op_count; j++)
            {
                idx = descr_idx[j];
                if (j == 0)
                {
                    pdma_descr[idx].cksum_init = 1;
                    pdma_descr[idx].cksum_initval = testparm[i].cksum_initval;
                    pdma_descr[idx].cksum_offset = testparm[i].cksum_offset_1;
                    pdma_descr[idx].cksum_length = testparm[i].cksum_length_1;
                }
                else if (j == 1)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;
                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length;
                }
                else if (j == 2)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;
                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length - (testparm[i].test_id % 3);
                }
                else
                {
                    DMAPANIC("Not engineered op_count");
                }

                //pdma_descr[idx].hw_dbg = testparm[i].test_id;
                pdma_descr[idx].operation = testparm[i].operation_type;

                pdma_descr[idx].dest_addr = PHYSICAL_ADDR((u32)pdst + testparm[i].olp[j].dst_offset);
                pdma_descr[idx].src_addr = PHYSICAL_ADDR((u32)psrc + testparm[i].olp[j].src_offset);
                pdma_descr[idx].operation_length = testparm[i].olp[j].length;
            }

            pdst[testparm[i].olp[0].dst_offset + testparm[i].total_len] = 0xff;
            pdst[testparm[i].olp[0].dst_offset - 1] = 0xff;

#if defined(TIMER_INTERRUPT_VERIFICATION)
            GDMAREG_WRITE32(DMA_INTR_THRESHOLD, (((TEST_TIMER_THRESHOLD-1)<<16) | op_count));    //	0: 1ms
#elif defined(INTERRUPT_VERIFICATION)
            GDMAREG_WRITE32(DMA_INTR_THRESHOLD, op_count - 1);	//	0:	raise interrupt	per	descriptordone,	1: raise interrupt on 2	descriptors	done
#endif

            /* shoot */
            for (j = (op_count - 1); j >= 0; j--)
            {
#if defined(TIMER_INTERRUPT_VERIFICATION) || defined(INTERRUPT_VERIFICATION)
                pdma_descr[descr_idx[j]].ctrl = GDMA_CTRL_GO;
#else
                pdma_descr[descr_idx[j]].ctrl = GDMA_CTRL_GO_POLL;
#endif
                //if (descr_idx[j]==0)
                //    DMADBG("(K %d, %x, %d)\n", descr_idx[j], &pdma_descr[descr_idx[j]], pdma_descr[descr_idx[j]].ctrl);
            }

            gdma_kick();


#if defined(TIMER_INTERRUPT_VERIFICATION)
            DMADBG("*");

            while(0 == (ic[ISTS] & (0x01UL << GDMA_INTR_NUM)))
                ;

            while(0 == (DMA_INTR_STATUS_TIME & GDMAREG_READ32(DMA_INTR_STATUS)))
                ;

            if(DMA_INTR_STATUS_COUNT & GDMAREG_READ32(DMA_INTR_STATUS))
                DMAPANIC("Shall not raise count intr");

            GDMAREG_WRITE32(DMA_INTR_STATUS, GDMAREG_READ32(DMA_INTR_STATUS));

            ic[ISTS] = (0x01UL << GDMA_INTR_NUM);

            for (j = (op_count - 1); j >= 0; j--)
            {
                if(pdma_descr[descr_idx[j]].ctrl != GDMA_CTRL_DONE_STOP)
                    DMAPANIC("Wrong ctrl state machine");
            }
            //DMADBG("\n");
#elif defined(INTERRUPT_VERIFICATION)
            DMADBG("*");

            while(0 == (ic[ISTS] & (0x01UL << GDMA_INTR_NUM)))
                ;

            while(0 == (DMA_INTR_STATUS_COUNT & GDMAREG_READ32(DMA_INTR_STATUS)))
                ;

            GDMAREG_WRITE32(DMA_INTR_STATUS, GDMAREG_READ32(DMA_INTR_STATUS));

            ic[ISTS] = (0x01UL << GDMA_INTR_NUM);

            for (j = (op_count - 1); j >= 0; j--)
            {
                if(pdma_descr[descr_idx[j]].ctrl != GDMA_CTRL_DONE_STOP)
                    DMAPANIC("Wrong ctrl state machine");
            }
#else
            DMADBG("*");

            /* wait done */
            while (pdma_descr[descr_idx[op_count - 1]].ctrl != GDMA_CTRL_DONE_SKIP)
                DMADBG(".");
#endif

#if 0
            DMADBG("\n");
            for (j = 0; j < op_count; j++)
            {
                //DMADBG("<%d %x %d>", descr_idx[j], &pdma_descr[descr_idx[j]], pdma_descr[descr_idx[j]].ctrl);
                DMADBG("<%d %x>", descr_idx[j], &pdma_descr[descr_idx[j]]);
            }
            DMADBG("\n");
#endif
            /* check result */
            idx = descr_idx[op_count - 1];
            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_CSUM))
            {
#if defined(CKSUM_VERIFICATION)
                /* csum check */
                if (pdma_descr[idx].cksum_result != testparm[i].cksum_result)
                    DMAPANIC("Wrong checksum result %d %x %x", i, testparm[i].cksum_result, pdma_descr[idx].cksum_result);
#endif
            }

            /* dma copy boundary check */
            if (pdst[testparm[i].olp[0].dst_offset + testparm[i].total_len] != 0xff)
                DMAPANIC("Tail memory corruption");

            if (pdst[testparm[i].olp[0].dst_offset - 1] !=	0xff)
                DMAPANIC("Head memory corruption");

            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_COPY))
            {
                /* memory copy result check */
                p1 = (unsigned char *)&pdst[testparm[i].olp[0].dst_offset];
                p2 = (unsigned char *)&psrc[testparm[i].olp[0].src_offset];
                if (mem_compare(p1, p2, testparm[i].total_len))
                    DMAPANIC("DMA copy result check failed");
            }
            //DMADBG(" Done\n");

            /* release descr */
            for (j = 0; j < op_count; j++)
            {
                //DMADBG("< %d %d >", descr_idx[j], pdma_descr[descr_idx[j]].ctrl);
                //if (descr_idx[j]==0)
                    //DMADBG("<D %d %x %d>\n", descr_idx[j], &pdma_descr[descr_idx[j]], pdma_descr[descr_idx[j]].ctrl);
                    //DMADBG("<D %d %x>\n", descr_idx[j], &pdma_descr[descr_idx[j]]);
                pdma_descr[descr_idx[j]].sw_inuse = 0;
            }
        }
    }

    return 0;
}

/* end of unaligned copy test */

/* 1~64K copy test test */
#if 1  // CSUM function
static u32 ___cksum;
static u16 ___odd;

void cksum_init(u16 initval)
{
    ___cksum = initval;
    ___odd = 0;
}
u16 cksum_feed(u8 val)
{
    if (___odd)
    {
        ___cksum += val;
    }
    else
    {
        ___cksum += (val << 8);
    }

    ___odd = (___odd	+ 1) %	2;

    while (___cksum &	0xffff0000UL)
    {
        ___cksum = (___cksum & 0xffffUL) + (___cksum >> 16);
    }

    return  ___cksum;
}
#endif

#define MAX_PADDING 65535 // 8192
#define MAX_ECOS_DMA_TEST_LENGTH    ( MAX_DMA_TEST_LENGTH + MAX_PADDING )

unsigned char dma_ecos_test_pattern[MAX_ECOS_DMA_TEST_LENGTH];
unsigned char dma_ecos_random_dest[MAX_ECOS_DMA_TEST_LENGTH+4];
int gdma_64k_test_cmd(void *data)
{
    int ret;
    short descr_idx[MAX_OPS];
    int i;
    int j;
    int op_count;
    int idx;
    volatile unsigned char *psrc, *pdst;

    unsigned char *p1, *p2;
    int iteration = 0;

    int target_d3_extend_length, d3_extend_length;

    int csum_total_len, total_len;

    psrc = (unsigned char *)UNCACHED_ADDR(dma_ecos_test_pattern);
    pdst = (unsigned char *)UNCACHED_ADDR(&dma_ecos_random_dest[4]);

    for(j=0;j<MAX_ECOS_DMA_TEST_LENGTH;j++)
    {
        psrc[j] = 0xff - (j % 255);
    }

    while (1)
    {
        target_d3_extend_length = (iteration * 64) % MAX_PADDING;
        DMADBG("\nIteration %d, target_d3_extend_length %d ", iteration, target_d3_extend_length);

        for (i = 0; i < (sizeof(testparm) / sizeof(struct test_params)); i++)
        {
            op_count = testparm[i].op_count;
            if (op_count != 3)
                continue;

            d3_extend_length = target_d3_extend_length;
            csum_total_len = 0;
            total_len = 0;

            ret = gdma_get_free_descr(descr_idx, op_count);
            if (0 > ret)
            {
                DMAPANIC("No free descr.");
            }

            //DMADBG("Test item %d", i);
            /* fill in descr */
            for (j = 0; j < op_count; j++)
            {
                idx = descr_idx[j];
                if (j == 0)
                {
                    pdma_descr[idx].cksum_init = 1;
                    pdma_descr[idx].cksum_initval = testparm[i].cksum_initval;
                    pdma_descr[idx].cksum_offset = testparm[i].cksum_offset_1;
                    pdma_descr[idx].cksum_length = testparm[i].cksum_length_1;
                    total_len = testparm[i].olp[j].length;
                    csum_total_len = testparm[i].cksum_length_1;
                }
                else if (j == 1)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;
                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length;
                    total_len += testparm[i].olp[j].length;
                    csum_total_len += testparm[i].olp[j].length;
                }
                else if (j == 2)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;

                    total_len += testparm[i].olp[j].length;
                    csum_total_len += testparm[i].olp[j].length;
                            //DMADBG(" ==> %d %d", d3_extend_length, total_len);
                    if(d3_extend_length > (65535 - total_len))
                    {
                        //DMADBG(" ===> %d %d", d3_extend_length, total_len);
                        d3_extend_length = 65535 - total_len;
                    }
                    if(d3_extend_length > (65535 - csum_total_len + (testparm[i].test_id % 3)))
                    {
                        //DMADBG(" ====> %d %d", d3_extend_length, csum_total_len);
                        d3_extend_length = 65535 - csum_total_len + (testparm[i].test_id % 3);
                    }

                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length + d3_extend_length - (testparm[i].test_id % 3);
                    csum_total_len += (d3_extend_length - (testparm[i].test_id % 3));
                }
                else
                {
                    DMAPANIC("Not engineered op_count");
                }

                //pdma_descr[idx].hw_dbg = testparm[i].test_id;
                pdma_descr[idx].operation = testparm[i].operation_type;

                pdma_descr[idx].dest_addr = PHYSICAL_ADDR((u32)pdst + testparm[i].olp[j].dst_offset);
                pdma_descr[idx].src_addr = PHYSICAL_ADDR((u32)psrc + testparm[i].olp[j].src_offset);

                if (j==2)
                    pdma_descr[idx].operation_length = testparm[i].olp[j].length + d3_extend_length;
                else
                    pdma_descr[idx].operation_length = testparm[i].olp[j].length;
            }

            pdst[testparm[i].olp[0].dst_offset + d3_extend_length + testparm[i].total_len] = 0xff;
            pdst[testparm[i].olp[0].dst_offset - 1] = 0xff;

            /* shoot */
            for (j = (op_count - 1); j >= 0; j--)
            {
                pdma_descr[descr_idx[j]].ctrl = GDMA_CTRL_GO_POLL;
            }

            gdma_kick();

            yield();

            /* wait done */
            while (pdma_descr[descr_idx[op_count - 1]].ctrl != GDMA_CTRL_DONE_SKIP)
                ; //DMADBG(".");

            /* check result */


            /* dma copy boundary check */
            if (pdst[testparm[i].olp[0].dst_offset + d3_extend_length + testparm[i].total_len] != 0xff)
                DMAPANIC("Tail memory corruption");

            if (pdst[testparm[i].olp[0].dst_offset - 1] !=	0xff)
                DMAPANIC("Head memory corruption");

            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_COPY))
            {
                /* memory copy result check */
                p1 = (unsigned char *)&pdst[testparm[i].olp[0].dst_offset];
                p2 = (unsigned char *)&psrc[testparm[i].olp[0].src_offset];
                if (mem_compare(p1, p2, testparm[i].total_len + d3_extend_length))
                    DMAPANIC("DMA copy result check failed");
            }

            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_CSUM))
            {
#if defined(CKSUM_VERIFICATION)
                cksum_init(testparm[i].cksum_initval);
                ___cksum = testparm[i].cksum_initval;

                p2 = (unsigned char *)&psrc[testparm[i].olp[0].src_offset + testparm[i].cksum_offset_1];

                for (j = 0; j < csum_total_len; j++)
                    cksum_feed(p2[j]);

                idx = descr_idx[op_count - 1];
            //DMADBG("\n[%d] %d %d %x", i, testparm[i].total_len + d3_extend_length,csum_total_len, ___cksum);
                /* csum check */
                if (pdma_descr[idx].cksum_result != ___cksum)
                    DMAPANIC("Wrong checksum result %d %x %x", i, testparm[i].cksum_result, ___cksum);
#endif
            }

            DMADBG("\n[%d] %d %d %x", i, testparm[i].total_len + d3_extend_length,csum_total_len, ___cksum);
            //DMADBG(" Done\n");

            /* release descr */
            for (j = 0; j < op_count; j++)
            {
                pdma_descr[descr_idx[j]].sw_inuse = 0;
            }
        }

        iteration++;
    }

    return 0;
}
/* end of ~64K copy test test */

/* 9 threads stress test */
#define TEST_THREADS 4
#define GDMA_TEST_PRIORITY  15
#define GDMA_TEST_STACKSIZE 8192

int heartbeat_polltest[TEST_THREADS];
int heartbeat_intrtest[TEST_THREADS];

#define POLL_THREAD_MEMCPY_LEN  1500
int gdma_poll_test_thread(void *data)
{
    int threadid = (int) data;
    int get_descr_count = (threadid % TEST_THREADS) + 1;
    unsigned char dst[POLL_THREAD_MEMCPY_LEN];
    unsigned char src[POLL_THREAD_MEMCPY_LEN];
    int ret;
    short descr_idx[GDMA_DESCR_COUNT];
    int i;

    printk("Poll test thread(%d) start\n", (u32) threadid);

    while (1)
    {
        memset(descr_idx, 0, sizeof(descr_idx));
        ret = gdma_get_free_descr(descr_idx, get_descr_count);

        if (ret == 0)
        {
            //sprintf(msg, "P(%d) GET %d, Ret %d, %d %d %d %d\n", threadid, get_descr_count, ret, descr_idx[0], descr_idx[1], descr_idx[2], descr_idx[3]);
            //printf("%s", msg);

            for(i=0; i<get_descr_count; i++)
            {
                pdma_descr[descr_idx[i]].operation = GDMA_OP_COPY_CSUM;
                pdma_descr[descr_idx[i]].cksum_init = 1;
                //pdma_descr[descr_idx[i]].hw_dbg = 0;
                pdma_descr[descr_idx[i]].dest_addr = PHYSICAL_ADDR(&dst[threadid]);
                pdma_descr[descr_idx[i]].src_addr = PHYSICAL_ADDR(&src[threadid]);
                pdma_descr[descr_idx[i]].cksum_offset = threadid;
                pdma_descr[descr_idx[i]].cksum_length = POLL_THREAD_MEMCPY_LEN - threadid;
                pdma_descr[descr_idx[i]].operation_length = POLL_THREAD_MEMCPY_LEN + threadid;
            }

            for(i=(get_descr_count-1);i >= 0; i--)
            {
#if 0
if(pdma_descr[descr_idx[i]].ctrl!=GDMA_CTRL_DONE_STOP)
    DMAPANIC("PANICP: %d %d\n", descr_idx[i], pdma_descr[descr_idx[i]]);
pdma_descr[descr_idx[i]].cksum_result = 0xffff;
DMADBG(" %dP", descr_idx[i]);
#endif
                pdma_descr[descr_idx[i]].ctrl = GDMA_CTRL_GO_POLL;
            }

            gdma_kick();

            yield();

            while (pdma_descr[descr_idx[get_descr_count - 1]].ctrl != GDMA_CTRL_DONE_SKIP)
                ;

            //sprintf(msg, "	Poll(%d)	Release	%d\n",	threadid,	descr_idx[get_descr_count - 1]);
            //printf("%s", msg);

            heartbeat_polltest[threadid]++;

            msleep(((threadid + 1)* 5));
            yield();

            for (i = 0; i < get_descr_count; i++)
            {
                pdma_descr[descr_idx[i]].sw_inuse = 0;
                yield();
                msleep((threadid + 1));
            }
        }
        else
        {
            yield();
        }
    }

    printk("Poll test thread(%d) exit\n", threadid);

    return 0;
}

int intr_test_callback(void *descr)
{
    dma_descriptor* dma_descr = (dma_descriptor *) descr;
    int threadid;

    threadid = dma_descr->sw_priv2;

    *(u32 *) (dma_descr->sw_priv2) = 0xffffffff;

    return 0;
}

#define INTR_THREAD_MEMCPY_LEN  1500
int gdma_intr_test_thread(void *data)
{
    int threadid = (int) data;
    int get_descr_count = (threadid % TEST_THREADS) + 1;
    unsigned char dst[INTR_THREAD_MEMCPY_LEN];
    unsigned char src[INTR_THREAD_MEMCPY_LEN];
    int ret;
    short descr_idx[GDMA_DESCR_COUNT];
    int i;

    volatile u32 done;

    printk("Intr test thread(%d) start\n", (u32) threadid);

    while (1)
    {
        memset(descr_idx, 0, sizeof(descr_idx));
        ret = gdma_get_free_descr(descr_idx, get_descr_count);

        if (ret == 0)
        {
            //sprintf(msg, "P(%d) GET %d, Ret %d, %d %d %d %d\n", threadid, get_descr_count, ret, descr_idx[0], descr_idx[1], descr_idx[2], descr_idx[3]);
            //printf("%s", msg);

            for(i=0; i<get_descr_count; i++)
            {
                pdma_descr[descr_idx[i]].operation = GDMA_OP_COPY_CSUM;
                pdma_descr[descr_idx[i]].cksum_init = 1;
                //pdma_descr[descr_idx[i]].hw_dbg = 0;
                pdma_descr[descr_idx[i]].dest_addr = PHYSICAL_ADDR(&dst[threadid]);
                pdma_descr[descr_idx[i]].src_addr = PHYSICAL_ADDR(&src[threadid]);
                pdma_descr[descr_idx[i]].cksum_offset = threadid;
                pdma_descr[descr_idx[i]].cksum_length = INTR_THREAD_MEMCPY_LEN - threadid;
                pdma_descr[descr_idx[i]].operation_length = INTR_THREAD_MEMCPY_LEN + threadid;
            }

            done = 0;
            pdma_descr[descr_idx[get_descr_count - 1]].sw_callback = (u32) intr_test_callback;
            pdma_descr[descr_idx[get_descr_count - 1]].sw_priv2 = (u32) &done;

            for(i=(get_descr_count-1);i >= 0; i--)
            {
#if 0
if(pdma_descr[descr_idx[i]].ctrl!=GDMA_CTRL_DONE_STOP)
    DMAPANIC("PANICI: %d %d\n", descr_idx[i], pdma_descr[descr_idx[i]]);
pdma_descr[descr_idx[i]].cksum_result = 0xffff;
DMADBG(" %dI", descr_idx[i]);
#endif
                pdma_descr[descr_idx[i]].ctrl = GDMA_CTRL_GO;
            }

            gdma_kick();

            //yield();
            msleep(5 * (threadid + 1));

            while(done != 0xffffffff)
            {
                #if 0
                DMADBG(" (%d %d)\n", threadid, done);
                for (i = 0; i < get_descr_count; i++)
                {
                    DMADBG("  < %d:  %x %x %d >\n", descr_idx[i], pdma_descr[descr_idx[i]].sw_callback, pdma_descr[descr_idx[i]].sw_priv2,
                           pdma_descr[descr_idx[i]].ctrl);
                }
                #endif
                msleep((threadid + 1));
                //msleep(1);
                 //gdma_kick();
            }

            //while (pdma_descr[descr_idx[get_descr_count - 1]].ctrl != GDMA_CTRL_DONE_STOP)
                //;

            //sprintf(msg, "	Poll(%d)	Release	%d\n",	threadid,	descr_idx[get_descr_count - 1]);
            //printf("%s", msg);

            heartbeat_intrtest[threadid]++;

            yield();

#if 0
            for (i = 0; i < get_descr_count; i++)
            {
                pdma_descr[descr_idx[i]].sw_inuse = 0;
                yield();
            }
#endif
        }
        else
        {
            yield();
        }

    }

    printk("Intr test thread(%d) exit\n", threadid);

    return 0;
}

int gdma_64k_test_thread(void *data)
{
    int ret;
    short descr_idx[MAX_OPS];
    int i;
    int j;
    int op_count;
    int idx;
    volatile unsigned char *psrc, *pdst;

    unsigned char *p1, *p2;
    int iteration = 0;

    int target_d3_extend_length, d3_extend_length;

    int csum_total_len, total_len;


    psrc = (unsigned char *)UNCACHED_ADDR(dma_ecos_test_pattern);
    pdst = (unsigned char *)UNCACHED_ADDR(&dma_ecos_random_dest[4]);

    for(j=0;j<MAX_ECOS_DMA_TEST_LENGTH;j++)
    {
        psrc[j] = 0xff - (j % 255);
    }

    while (1)
    {
        target_d3_extend_length = (iteration * 64) % MAX_PADDING;
        DMADBG("\nIteration %d, target_d3_extend_length %d ", iteration, target_d3_extend_length);

        for (i = 0; i < (sizeof(testparm) / sizeof(struct test_params)); i++)
        {
            op_count = testparm[i].op_count;
            if (op_count != 3)
                continue;

            d3_extend_length = target_d3_extend_length;
            csum_total_len = 0;
            total_len = 0;

            ret = gdma_get_free_descr(descr_idx, op_count);
            if (0 > ret)
            {
                //DMAPANIC("No free descr.");
                DMADBG("No free descr\n");
                yield();
                continue;
            }

            //DMADBG("Test item %d", i);
            /* fill in descr */
            for (j = 0; j < op_count; j++)
            {
                idx = descr_idx[j];
                if (j == 0)
                {
                    pdma_descr[idx].cksum_init = 1;
                    pdma_descr[idx].cksum_initval = testparm[i].cksum_initval;
                    pdma_descr[idx].cksum_offset = testparm[i].cksum_offset_1;
                    pdma_descr[idx].cksum_length = testparm[i].cksum_length_1;
                    total_len = testparm[i].olp[j].length;
                    csum_total_len = testparm[i].cksum_length_1;
                }
                else if (j == 1)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;
                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length;
                    total_len += testparm[i].olp[j].length;
                    csum_total_len += testparm[i].olp[j].length;
                }
                else if (j == 2)
                {
                    pdma_descr[idx].cksum_init = 0;
                    pdma_descr[idx].cksum_offset = 0;

                    total_len += testparm[i].olp[j].length;
                    csum_total_len += testparm[i].olp[j].length;
                            //DMADBG(" ==> %d %d", d3_extend_length, total_len);
                    if(d3_extend_length > (65535 - total_len))
                    {
                        //DMADBG(" ===> %d %d", d3_extend_length, total_len);
                        d3_extend_length = 65535 - total_len;
                    }
                    if(d3_extend_length > (65535 - csum_total_len + (testparm[i].test_id % 3)))
                    {
                        //DMADBG(" ====> %d %d", d3_extend_length, csum_total_len);
                        d3_extend_length = 65535 - csum_total_len + (testparm[i].test_id % 3);
                    }

                    pdma_descr[idx].cksum_length = testparm[i].olp[j].length + d3_extend_length - (testparm[i].test_id % 3);
                    csum_total_len += (d3_extend_length - (testparm[i].test_id % 3));
                }
                else
                {
                    DMAPANIC("Not engineered op_count");
                }

                //pdma_descr[idx].hw_dbg = testparm[i].test_id;
                pdma_descr[idx].operation = testparm[i].operation_type;

                pdma_descr[idx].dest_addr = PHYSICAL_ADDR((u32)pdst + testparm[i].olp[j].dst_offset);
                pdma_descr[idx].src_addr = PHYSICAL_ADDR((u32)psrc + testparm[i].olp[j].src_offset);

                if (j==2)
                    pdma_descr[idx].operation_length = testparm[i].olp[j].length + d3_extend_length;
                else
                    pdma_descr[idx].operation_length = testparm[i].olp[j].length;
            }

            pdst[testparm[i].olp[0].dst_offset + d3_extend_length + testparm[i].total_len] = 0xff;
            pdst[testparm[i].olp[0].dst_offset - 1] = 0xff;

            /* shoot */
            for (j = (op_count - 1); j >= 0; j--)
            {
#if 0
if(pdma_descr[descr_idx[j]].ctrl!=GDMA_CTRL_DONE_STOP)
    DMAPANIC("PANICp: %d %d\n", descr_idx[j], pdma_descr[descr_idx[j]]);
pdma_descr[descr_idx[j]].cksum_result = 0xffff;
DMADBG(" %dp", descr_idx[j]);
#endif
                pdma_descr[descr_idx[j]].ctrl = GDMA_CTRL_GO_POLL;
            }

            gdma_kick();

            //msleep(1);

            /* wait done */
            while (pdma_descr[descr_idx[op_count - 1]].ctrl != GDMA_CTRL_DONE_SKIP)
                ; //DMADBG(".");

            /* check result */
#if 1

            /* dma copy boundary check */
            if (pdst[testparm[i].olp[0].dst_offset + d3_extend_length + testparm[i].total_len] != 0xff)
                DMAPANIC("Tail memory corruption");

            if (pdst[testparm[i].olp[0].dst_offset - 1] !=	0xff)
                DMAPANIC("Head memory corruption");

            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_COPY))
            {
                /* memory copy result check */
                p1 = (unsigned char *)&pdst[testparm[i].olp[0].dst_offset];
                p2 = (unsigned char *)&psrc[testparm[i].olp[0].src_offset];
                if (mem_compare(p1, p2, testparm[i].total_len + d3_extend_length))
                    DMAPANIC("DMA copy result check failed");
            }

            if ((testparm[i].operation_type == GDMA_OP_COPY_CSUM) || (testparm[i].operation_type == GDMA_OP_CSUM))
            {
#if defined(CKSUM_VERIFICATION)
                cksum_init(testparm[i].cksum_initval);
                ___cksum = testparm[i].cksum_initval;

                p2 = (unsigned char *)&psrc[testparm[i].olp[0].src_offset + testparm[i].cksum_offset_1];

                for (j = 0; j < csum_total_len; j++)
                    cksum_feed(p2[j]);

                idx = descr_idx[op_count - 1];
            //DMADBG("\n[%d] %d %d %x", i, testparm[i].total_len + d3_extend_length,csum_total_len, ___cksum);
                /* csum check */
                if (pdma_descr[idx].cksum_result != ___cksum)
                    DMAPANIC("Wrong checksum result %d %x %x", i, testparm[i].cksum_result, ___cksum);
#endif
            }

            //DMADBG("\n[%d] %d %d %x", i, testparm[i].total_len + d3_extend_length,csum_total_len, ___cksum);
            //DMADBG(" Done\n");
#endif
            /* release descr */
            for (j = 0; j < op_count; j++)
            {
                pdma_descr[descr_idx[j]].sw_inuse = 0;
            }
        }

        iteration++;
    }

    return 0;
}


int gdma_64k_test_and_4_polling_threads(void *data)
{
    int i;

    kthread_run(gdma_64k_test_cmd, NULL, "gdma_64k_test_cmd");

    msleep(500);

    for (i=0;i<TEST_THREADS;i++)
    {
        kthread_run(gdma_poll_test_thread, (void *) i, "gdma_poll_test_thread");
    }

    while (1)
    {
        printk("### %d %d %d %d ###\n", heartbeat_polltest[0],heartbeat_polltest[1],heartbeat_polltest[2],heartbeat_polltest[3]);
        msleep(5);
    }

    return 0;
}

int gdma_64k_test_and_4_polling_and_4_intr_threads(void *data)
{
    int i;

    for (i=0;i<TEST_THREADS;i++)
    {
        kthread_run(gdma_poll_test_thread, (void *) i, "gdma_poll_test_thread");
    }

    for (i=0;i<TEST_THREADS;i++)
    {
        kthread_run(gdma_intr_test_thread, (void *) i, "gdma_intr_test_thread");
    }


    kthread_run(gdma_64k_test_thread, (void *) 0, "gdma_64k_test_thread");

    while (1)
    {
        printk("### %d %d %d %d %d %d %d %d ###\n", 
                    heartbeat_polltest[0],heartbeat_polltest[1],heartbeat_polltest[2],heartbeat_polltest[3],
                    heartbeat_intrtest[0],heartbeat_intrtest[1],heartbeat_intrtest[2],heartbeat_intrtest[3]);
        msleep(5);
    }

    return 0;
}

/* end of 9 threads stress test */

/* basic test suite */

/* check compiler alignment problem for src/dst stack variable test */
//#define USE_STACK_VAR
#define UNIT_TEST_MEMCPY_LEN    1632

int heartbeat_poll_unit_test[5];
int heartbeat_intr_unit_test[5];

#if !defined(USE_STACK_VAR)
unsigned char ___dst_poll[5][UNIT_TEST_MEMCPY_LEN];
unsigned char ___src_poll[5][UNIT_TEST_MEMCPY_LEN];
unsigned char ___dst_intr[5][UNIT_TEST_MEMCPY_LEN];
unsigned char ___src_intr[5][UNIT_TEST_MEMCPY_LEN];
#endif

int gdma_poll_unit_test_thread(void *data)
{
    int threadid = (int) data;
    int ret;
    short descr_idx;
    int i;

#if defined(USE_STACK_VAR)
    unsigned char __dst[UNIT_TEST_MEMCPY_LEN] __attribute__((aligned(16)));
    unsigned char __src[UNIT_TEST_MEMCPY_LEN] __attribute__((aligned(16)));
    unsigned char *dst = (unsigned char *) UNCACHED_ADDR((u32) __dst);
    unsigned char *src = (unsigned char *) UNCACHED_ADDR((u32) __src);

    dma_cache_wback_inv((u32) __dst, UNIT_TEST_MEMCPY_LEN);
    dma_cache_wback_inv((u32) __src, UNIT_TEST_MEMCPY_LEN);
#else
    unsigned char *dst = (unsigned char *) UNCACHED_ADDR((u32) ___dst_poll[threadid]);
    unsigned char *src = (unsigned char *) UNCACHED_ADDR((u32) ___src_poll[threadid]);
#endif

    printk("Poll test thread(%d) start %p %p\n", (u32) threadid, dst, src);
    memset(src, 0x00, UNIT_TEST_MEMCPY_LEN);
    memset(dst, 0xff, UNIT_TEST_MEMCPY_LEN);

    while (1)
    {
        ret = gdma_get_free_descr(&descr_idx, 1);
        if (ret<0)
        {
            msleep(10);
            continue;
        }

        memset(dst, 0xFF, UNIT_TEST_MEMCPY_LEN);

        pdma_descr[descr_idx].dest_addr = PHYSICAL_ADDR(dst);
        pdma_descr[descr_idx].src_addr = PHYSICAL_ADDR(src);
        pdma_descr[descr_idx].cksum_init = 1;
        pdma_descr[descr_idx].cksum_initval = 0x1234;
        pdma_descr[descr_idx].cksum_result = 0xAAAA;

        if (threadid==0)
        {
            pdma_descr[descr_idx].operation = GDMA_OP_CSUM;
            pdma_descr[descr_idx].cksum_offset = 0;
            pdma_descr[descr_idx].cksum_length = UNIT_TEST_MEMCPY_LEN;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }
        else if (threadid==1)
        {
            pdma_descr[descr_idx].operation = GDMA_OP_COPY;
            pdma_descr[descr_idx].cksum_offset = 0xffff;
            pdma_descr[descr_idx].cksum_length = 0xffff;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }
        else
        {
            pdma_descr[descr_idx].operation = GDMA_OP_COPY_CSUM;
            pdma_descr[descr_idx].cksum_offset = 0;
            pdma_descr[descr_idx].cksum_length = UNIT_TEST_MEMCPY_LEN;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }


        pdma_descr[descr_idx].ctrl = GDMA_CTRL_GO_POLL;

        gdma_kick();

        msleep(1);

        while (pdma_descr[descr_idx].ctrl != GDMA_CTRL_DONE_SKIP)
            ;


        if (threadid==0)
        {
            if (pdma_descr[descr_idx].cksum_result != 0x1234)
            {
                printk("P%d: csum error\n", threadid);
            }

            if (!memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("P%d: shall not memcpy\n", threadid);
            }
        }
        else if (threadid==1)
        {
            if (memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("P%d: memcpy error\n", threadid);
            }

            if (pdma_descr[descr_idx].cksum_result != 0xAAAA)
            {
                printk("P%d: shall not have csum result\n", threadid);
            }
        }
        else
        {
            if (memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("P%d: memcpy error\n", threadid);
                for (i=0;i<UNIT_TEST_MEMCPY_LEN;i++)
                {
                    printk("%d %x %x\n", i, dst[i], src[i]);
                }
            }

#if 0
printk("P%d: (%x %x %d %d) (%d %d) CSUM(%d %x %x)\n", threadid, 
            pdma_descr[descr_idx].dest_addr, pdma_descr[descr_idx].src_addr,
            pdma_descr[descr_idx].operation, pdma_descr[descr_idx].operation_length,
            pdma_descr[descr_idx].cksum_offset, pdma_descr[descr_idx].cksum_length,
            pdma_descr[descr_idx].cksum_init, pdma_descr[descr_idx].cksum_initval, pdma_descr[descr_idx].cksum_result);
#endif

            if (pdma_descr[descr_idx].cksum_result != 0x1234)
            {
                printk("P%d: cksum error %x\n", threadid, pdma_descr[descr_idx].cksum_result);
                for (i=0;i<UNIT_TEST_MEMCPY_LEN;i++)
                {
                    printk("%d %x %x\n", i, dst[i], src[i]);
                }
            }
        }

        yield();

        heartbeat_poll_unit_test[threadid]++;

        pdma_descr[descr_idx].sw_inuse = 0;
    }

    printk("Poll test thread(%d) exit\n", threadid);

    return 0;
}

int intr_unit_test_callback(void *descr)
{
    dma_descriptor* dma_descr = (dma_descriptor *) descr;
    u32 *done;
    int threadid;

    done = (u32 *) dma_descr->sw_priv2;
    threadid = *done;

    if (threadid==0)
    {
        if (dma_descr->cksum_result != 0x1234)
        {
            printk("I%d: csum error\n", threadid);
        }

    }
    else if (threadid==1)
    {
        if (dma_descr->cksum_result != 0xAAAA)
        {
            printk("I%d: shall not have csum result\n", threadid);
        }
    }
    else
    {
        if (dma_descr->cksum_result != 0x1234)
        {
            printk("I%d: csum error\n", threadid);
        }
    }

    *done = 0xffffffff;

    return 0;
}

int gdma_intr_unit_test_thread(void *data)
{
    int threadid = (int) data;
    int ret;
    short descr_idx;

    volatile u32 done;

#if defined(USE_STACK_VAR)
    unsigned char __dst[UNIT_TEST_MEMCPY_LEN] __attribute__((aligned(16)));
    unsigned char __src[UNIT_TEST_MEMCPY_LEN] __attribute__((aligned(16)));
    unsigned char *dst = (unsigned char *) UNCACHED_ADDR((u32) __dst);
    unsigned char *src = (unsigned char *) UNCACHED_ADDR((u32) __src);

    dma_cache_wback_inv((u32) __dst, UNIT_TEST_MEMCPY_LEN);
    dma_cache_wback_inv((u32) __src, UNIT_TEST_MEMCPY_LEN);
#else
    unsigned char *dst = (unsigned char *) UNCACHED_ADDR((u32) ___dst_intr[threadid]);
    unsigned char *src = (unsigned char *) UNCACHED_ADDR((u32) ___src_intr[threadid]);
#endif

    printk("Intr test thread(%d) start %p %p\n", (u32) threadid, dst, src);
    memset(src, 0x00, UNIT_TEST_MEMCPY_LEN);
    memset(dst, 0xff, UNIT_TEST_MEMCPY_LEN);

    while (1)
    {
        ret = gdma_get_free_descr(&descr_idx, 1);
        if(ret<0)
        {
            msleep(10);
            continue;
        }

        memset(dst, 0xFF, UNIT_TEST_MEMCPY_LEN);

        pdma_descr[descr_idx].dest_addr = PHYSICAL_ADDR(dst);
        pdma_descr[descr_idx].src_addr = PHYSICAL_ADDR(src);
        pdma_descr[descr_idx].cksum_init = 1;
        pdma_descr[descr_idx].cksum_initval = 0x1234;
        pdma_descr[descr_idx].cksum_result = 0xAAAA;

        if (threadid==0)
        {
            pdma_descr[descr_idx].operation = GDMA_OP_CSUM;
            pdma_descr[descr_idx].cksum_offset = 0;
            pdma_descr[descr_idx].cksum_length = UNIT_TEST_MEMCPY_LEN;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }
        else if (threadid==1)
        {
            pdma_descr[descr_idx].operation = GDMA_OP_COPY;
            pdma_descr[descr_idx].cksum_offset = 0xffff;
            pdma_descr[descr_idx].cksum_length = 0xffff;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }
        else
        {
            pdma_descr[descr_idx].operation = GDMA_OP_COPY_CSUM;
            pdma_descr[descr_idx].cksum_offset = 0;
            pdma_descr[descr_idx].cksum_length = UNIT_TEST_MEMCPY_LEN;
            pdma_descr[descr_idx].operation_length = UNIT_TEST_MEMCPY_LEN;
        }

        done = threadid;
        pdma_descr[descr_idx].sw_callback = (u32) intr_unit_test_callback;
        pdma_descr[descr_idx].sw_priv2 = (u32) &done;

        pdma_descr[descr_idx].ctrl = GDMA_CTRL_GO;

        gdma_kick();

        msleep(1);

        while(done != 0xffffffff)
            ;


        if (threadid==0)
        {
            if (!memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("I%d: shall not memcpy\n", threadid);
            }
        }
        else if (threadid==1)
        {
            if (memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("I%d: memcpy error\n", threadid);
            }
        }
        else
        {
            if (memcmp(dst,src,UNIT_TEST_MEMCPY_LEN))
            {
                printk("I%d: memcpy error\n", threadid);
            }
        }

        heartbeat_intr_unit_test[threadid]++;

        yield();
    }

    printk("Intr test thread(%d) exit\n", threadid);

    return 0;
}

int gdma_operation_basic_test(void *data)
{
    int i;

    for (i=0;i<5;i++)
    {
        kthread_run(gdma_poll_unit_test_thread, (void *) i, "gdma_poll_unit_test_thread");
    }

    for (i=0;i<5;i++)
    {
        kthread_run(gdma_intr_unit_test_thread, (void *) i, "gdma_intr_unit_test_thread");
    }

    //os_thread_create((OS_FUNCPTR) gdma_64k_test_thread, (void *) 0, "GDMA", GDMA_TEST_PRIORITY, GDMA_TEST_STACKSIZE);

    while (1)
    {
        printk("### %d %d %d %d %d %d %d %d %d %d ###\n", 
                    heartbeat_poll_unit_test[0],heartbeat_poll_unit_test[1],heartbeat_poll_unit_test[2],heartbeat_poll_unit_test[3],heartbeat_poll_unit_test[4],
                    heartbeat_intr_unit_test[0],heartbeat_intr_unit_test[1],heartbeat_intr_unit_test[2],heartbeat_intr_unit_test[3],heartbeat_intr_unit_test[4]);
        msleep(1000);
    }

    return 0;
}

/* end of basic test suite */

/* speed test */
#define SPEED_TEST_LEN      65520
#define DESCR_NO_PER_LOOP   (GDMA_DESCR_COUNT - 2)
u8 speed_test_dst[SPEED_TEST_LEN];
u8 speed_test_src[SPEED_TEST_LEN];
int gdma_speed_test_thread(void *data)
{
    int threadid = (int) data;
    int ret;
    short descr_idx[GDMA_DESCR_COUNT];
    int i;
    int j;
    int jiffies_prev;
    int n;

    printk("GDMA speed test thread start, %d descriptors per test\n", DESCR_NO_PER_LOOP);

    while (1)
    {
        ret = gdma_get_free_descr(descr_idx, DESCR_NO_PER_LOOP);
        if (ret<0)
        {
            printk("gdma_get_free_descr failed, %d\n", ret);
            return 0;
        }

        for(i=0;i<DESCR_NO_PER_LOOP;i++)
        {
            pdma_descr[descr_idx[i]].dest_addr = PHYSICAL_ADDR(speed_test_dst);
            pdma_descr[descr_idx[i]].src_addr = PHYSICAL_ADDR(speed_test_src);
            pdma_descr[descr_idx[i]].cksum_init = 1;
            pdma_descr[descr_idx[i]].cksum_initval = 0xFFFF;
            pdma_descr[descr_idx[i]].operation = GDMA_OP_COPY_CSUM;
            pdma_descr[descr_idx[i]].cksum_offset = 0;
            pdma_descr[descr_idx[i]].cksum_length = SPEED_TEST_LEN;
            pdma_descr[descr_idx[i]].operation_length = SPEED_TEST_LEN;
        }

        for(i=(DESCR_NO_PER_LOOP-1);i>=0;i--)
        {
            pdma_descr[descr_idx[i]].ctrl = GDMA_CTRL_GO_POLL;
        }

        jiffies_prev = jiffies;

        gdma_kick();

        while (pdma_descr[descr_idx[DESCR_NO_PER_LOOP-1]].ctrl != GDMA_CTRL_DONE_SKIP)
            for(n=0;n<1000;n++)
                __asm__ volatile("nop;nop;nop");

        printk("GDMA copy+csum %d Bytes in %ld ticks\n", SPEED_TEST_LEN * DESCR_NO_PER_LOOP,  jiffies - jiffies_prev);
        for(i=0;i<DESCR_NO_PER_LOOP;i++)
        {
            pdma_descr[descr_idx[i]].sw_inuse = 0;
        }


        ret = gdma_get_free_descr(descr_idx, DESCR_NO_PER_LOOP);
        if (ret<0)
        {
            printk("gdma_get_free_descr failed, %d\n", ret);
            return 0;
        }

        for(i=0;i<DESCR_NO_PER_LOOP;i++)
        {
            pdma_descr[descr_idx[i]].dest_addr = PHYSICAL_ADDR(speed_test_dst);
            pdma_descr[descr_idx[i]].src_addr = PHYSICAL_ADDR(speed_test_src);
            pdma_descr[descr_idx[i]].cksum_init = 1;
            pdma_descr[descr_idx[i]].cksum_initval = 0xFFFF;
            pdma_descr[descr_idx[i]].operation = GDMA_OP_CSUM;
            pdma_descr[descr_idx[i]].cksum_offset = 0;
            pdma_descr[descr_idx[i]].cksum_length = SPEED_TEST_LEN;
            pdma_descr[descr_idx[i]].operation_length = SPEED_TEST_LEN;
        }

        for(i=(DESCR_NO_PER_LOOP-1);i>=0;i--)
        {
            pdma_descr[descr_idx[i]].ctrl = GDMA_CTRL_GO_POLL;
        }

        jiffies_prev = jiffies;

        gdma_kick();

        while (pdma_descr[descr_idx[DESCR_NO_PER_LOOP-1]].ctrl != GDMA_CTRL_DONE_SKIP)
            for(n=0;n<1000;n++)
                __asm__ volatile("nop;nop;nop");

        printk("GDMA csum %d Bytes in %ld ticks\n", SPEED_TEST_LEN * DESCR_NO_PER_LOOP,  jiffies - jiffies_prev);
        for(i=0;i<DESCR_NO_PER_LOOP;i++)
        {
            pdma_descr[descr_idx[i]].sw_inuse = 0;
        }

        jiffies_prev = jiffies;
        for(j=0;j<DESCR_NO_PER_LOOP;j++)
        {
            memcpy(speed_test_dst, speed_test_src, SPEED_TEST_LEN);
            dma_cache_wback_inv((u32) speed_test_dst, SPEED_TEST_LEN);
        }
        printk("Memory copy %d Bytes in %ld ticks\n", SPEED_TEST_LEN * DESCR_NO_PER_LOOP,  jiffies - jiffies_prev);

        jiffies_prev = jiffies;
        for(j=0;j<DESCR_NO_PER_LOOP;j++)
        {
            csum_partial(speed_test_src, SPEED_TEST_LEN, 0);
        }
        printk("Checksum %d Bytes in %ld ticks\n", SPEED_TEST_LEN * DESCR_NO_PER_LOOP,  jiffies - jiffies_prev);
    }

    printk("Speed test thread(%d) exit\n", threadid);

    return 0;
}

/* end of speed test */

/* gdma unit test , for checking some killer pattern */
int gdma_unit_test_thread(void *data)
{
    int threadid = (int) data;
    int ret;
    int i;
    int j = 0;
    short idx[16];
    dma_descriptor *pdma;

    printk("GDMA unit test thread start\n");

    while (1)
    {
        ret = gdma_get_free_descr(idx, 3);
        if (ret<0)
        {
            printk("gdma_get_free_descr failed, %d\n", ret);
            return 0;
        }

	    pdma = &pdma_descr[idx[0]];

	    pdma->cksum_init = 1;
	    pdma->cksum_initval = 0;
	    pdma->cksum_offset = 0;

	    pdma->src_addr = 22003996;
	    pdma->operation_length = pdma->cksum_length = 20;

	    pdma->operation = GDMA_OP_CSUM;
	    pdma->ctrl = GDMA_CTRL_GO_POLL;

        gdma_kick();

		pdma = &pdma_descr[idx[1]];
	
		pdma->cksum_init = 0;
		pdma->cksum_offset = 0;

		pdma->src_addr = 24103721;
		pdma->operation_length = pdma->cksum_length = 1239;

		pdma->operation = GDMA_OP_CSUM;
		pdma->ctrl = GDMA_CTRL_GO_POLL;


		pdma = &pdma_descr[idx[2]];
	
		pdma->cksum_init = 0;
		pdma->cksum_offset = 0;

		pdma->src_addr = 24104960;
		pdma->operation_length = pdma->cksum_length = 221;

		pdma->operation = GDMA_OP_CSUM;
		pdma->ctrl = GDMA_CTRL_GO_POLL;

        gdma_kick();

        while(pdma_descr[idx[2]].ctrl != GDMA_CTRL_DONE_SKIP)
        {
            //for(n=0;n<1000;n++) __asm__ volatile("nop;nop;nop");
            printk("*");
        }

        for(i=0;i<3;i++)
        {
            pdma_descr[idx[i]].sw_inuse = 0;
        }

        j++;
        printk("(%d)\n", j);
    }

    printk("Unit test thread(%d) exit\n", threadid);

    return 0;
}

/* end of gdma unit test */

#define TEST_SUITE_NO   6 //4

int gdma_test_thread(void *data)
{
    msleep(8000);

    switch(TEST_SUITE_NO)
    {
        case 0:
            kthread_run(gdma_basic_test_cmd, (void *) 0, "basic unaligned copy test");
            break;
        case 1:
            kthread_run(gdma_64k_test_and_4_polling_threads, (void *) 0, "up to 64k copy test");
            break;
        case 2:
            kthread_run(gdma_operation_basic_test, (void *) 0, "3 operations test");
            break;
        case 3:
            kthread_run(gdma_64k_test_and_4_polling_and_4_intr_threads, (void *) 0, "9 threads stress test");
            break;
        case 4:
            kthread_run(gdma_speed_test_thread, (void *) 0, "gdma speed test");
            break;
        case 5:
            kthread_run(gdma_unit_test_thread, (void *) 0, "gdma unit test");
            break;
        default:
            break;
    }

    return 0;
}

static int __init gdma_linux_init_module(void)
{   
    printk("Enable GDMA test module, Compiled with test suite %d (TEST_SUITE_NO) enabled\n", TEST_SUITE_NO);

    kthread_run(gdma_test_thread, (void *) 0, "GDMA root test thread");

    return 0;
}

module_init(gdma_linux_init_module);

