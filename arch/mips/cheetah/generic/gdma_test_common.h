#ifndef __GDMA_TEST_COMMON_H__

#if 1
#define MAX_DMA_TEST_LENGTH ( ( 3 * 64 ) + 64 + 8192 )
#else
#define MAX_DMA_TEST_LENGTH ( ( 3 * 64 ) + 64 )
#endif

#define MAX_TEST_ITEMS 4096

#define MAX_OPS 3

struct offset_length_pair
{
    u16 dst_offset;
    u16 src_offset;
    u16 length;
};

struct test_params
{
    u16 test_id;
    u16 op_count;
    u16 operation_type;
    struct offset_length_pair olp[MAX_OPS];
    u16 cksum_initval;
    u16 cksum_result;
    u16 cksum_offset_1;
    u16 cksum_length_1;
    u16 total_len;
};
#endif // __GDMA_TEST_COMMON_H__
