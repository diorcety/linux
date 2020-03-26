#ifndef __SMARTCFG_H__
#define __SMARTCFG_H__

// test USER SPACE CODE
// cd /media/user/exdisk/van/kernel/linux-2.6.32.11.as/arch/mips/cheetah/wla/linux
// gcc -o test.exe -DUSR_SPACE_TEST -DCONFIG_CHEETAH_SMART_CONFIG smartcfg2.c smartcfg.h

#define SC_LOCK_SPEC_CHAR               0x14
#define SC_LOCK_INTERVAL_LEN            0x10
#define SC_LOCK_LENGTH_BASE             400 // SMART_CONFIG_LENGTH_BASE - 100
#define SC_FRAME_TIME_INTERVAL          18      // unit: ms

#define SC_LOCK_PHASE_DATA_NUMS         256
#define SC_LOCK_PHASE_BIT_MAP_NUMS      SC_LOCK_PHASE_DATA_NUMS/32

struct lock_phase_ta {
    unsigned char addr[6];
    unsigned int bit_map[SC_LOCK_PHASE_BIT_MAP_NUMS];

    struct lock_phase_ta *next;
};

struct try_lock_data {
    unsigned int frames[SC_LOCK_PHASE_DATA_NUMS];
    unsigned int timestamp[SC_LOCK_PHASE_DATA_NUMS];
    unsigned int start_pos;
    unsigned int next_pos;

    struct lock_phase_ta *ta_list;
};

//unsigned int* lock_stream_gen(unsigned int *stream, unsigned int base, unsigned int spec_char, unsigned int interval);
int try_lock_channel(unsigned int data, unsigned char *addr, struct try_lock_data *lock_data, int *ret_state);
int free_lock_data(struct try_lock_data *lock_data);

#define SMART_CONFIG_LENGTH_BASE    500
#define SMART_CONFIG_SHIFT          2
#define SMART_CONFIG_BLOCK_BYTES    8
#define SMART_CONFIG_BLOCK_FRAMES   SMART_CONFIG_BLOCK_BYTES*2 // one frame 4 bits
#define SMART_CONFIG_BLOCK_NUMS     13
#define SMART_CONFIG_LENGTH_MASK    0xff03    // SC info is only in bit[7:2]

#define SC_CONTROL_DATA             0x10 // BIT(4)
#define SC_EVEN_ODD                 0x20 // BIT(5)

#define SC_CTL_SSID_BLOCK_0         0x00
#define SC_CTL_SSID_BLOCK_1         0x01
#define SC_CTL_SSID_BLOCK_2         0x02
#define SC_CTL_SSID_BLOCK_3         0x03
#define SC_CTL_SSID_BLOCK_4         0x04
#define SC_CTL_SSID_BLOCK_5         0x05
#define SC_CTL_SSID_BLOCK_6         0x06
#define SC_CTL_SSID_BLOCK_7         0x07
#define SC_CTL_SSID_BLOCK_8         0x08
#define SC_CTL_SSID_BLOCK_9         0x09
#define SC_CTL_SSID_BLOCK_10        0x0A
#define SC_CTL_SSID_BLOCK_11        0x0B
#define SC_CTL_SSID_BLOCK_12        0x0C


struct sc_block_entry {
    unsigned char current_rx_count;
    unsigned char value[SMART_CONFIG_BLOCK_FRAMES];
};

struct sc_rx_block {
    unsigned int block_map;
    unsigned char block_counter[SMART_CONFIG_BLOCK_NUMS];
    struct sc_block_entry block[SMART_CONFIG_BLOCK_NUMS];

    unsigned char rx_even[32];
    unsigned char rx_even_count;
    int current_rx_block;
    unsigned char pending_rx_data;
};

unsigned char* sc_decoder(unsigned int data, struct sc_rx_block *blocks);

#if defined(USR_SPACE_TEST)
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>

unsigned int* lock_stream_gen(unsigned int *stream, unsigned int base, unsigned int spec_char, unsigned int interval);
unsigned int* sc_encoder(unsigned char *data, int data_len, int *encoded_stream_len);
#define SC_PRINT     printf
#define SC_MALLOC(x) malloc((x))
#define SC_FREE      free

enum {
    encode_arg_prefer_header_length = 1,
    encode_arg_mode,
    encode_arg_user,
    encode_arg_pw,
    encode_arg_frameloss_ratio,
    encode_arg_rx_loop
};

// "prog 0 <wisp(3) or station(9)> <user> <password> <frame lost ratio> <rx_loop>\n"
struct opt_test_encode {
    int prefer_header_length;
    unsigned char mode; // WISP or station
    char * user;
    char * password;
    int frame_lost_ratio;
    int rx_loop;
};

#define MAX_AIR_PACKET 10000

char *fgetln(FILE *, size_t *);

#if 0
unsigned int current_timestamp(void) {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    unsigned int milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    return milliseconds;
}
#else
unsigned int current_timestamp(void) {
    static unsigned int milliseconds = 400;
    
    milliseconds++;

    return milliseconds;
}
#endif

#else
#include <mac_ctrl.h>
#include <wlan_def.h>
#include <linux/jiffies.h>
// channel lock
static struct try_lock_data *lock_data=NULL;
//int len_low = SC_LOCK_LENGTH_BASE + SC_LOCK_SPEC_CHAR + 24; // 24: normal wifi header
//int len_high = len_low + 40 + SC_LOCK_INTERVAL_LEN*3; // 40 is a safe value
static int header_len_diff;

// decode
static unsigned char lock_mac_addr[6]; //={0xfful,0xfful,0xfful,0xfful,0xfful,0xfful};
static struct sc_rx_block *blocks=NULL;
#define SC_PRINT     printk
#define SC_MALLOC(x) kmalloc((x), GFP_ATOMIC)
#define SC_FREE      kfree

#define LE_WLAN_BA_TID      0x00f0ul
#define LE_WLAN_BA_TID_S    4
#define LE_WLAN_BA_SSN_L    0xf000ul
#define LE_WLAN_BA_SSN_L_S  12
#define LE_WLAN_BA_SSN      0x00fful
#define LE_WLAN_BA_SSN_S    4
#define WLAN_BA_SSN_FIELD   0x0ffful
struct wlan_ba
{
    struct wlan_ctrl_hdr    hdr;
    u16                     bactl;
    u16                     baseqctl;
} __attribute__ ((packed));

unsigned int current_timestamp(void)
{
    unsigned int milliseconds;

    milliseconds=(unsigned int)(jiffies + 30000);

    return milliseconds;
}
#endif // USR_SPACE_TEST

#endif // __SMARTCFG_H__
