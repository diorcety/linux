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
#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/sort.h>
#include <linux/skbuff.h>
#include <linux/ctype.h>

#include <asm/mach-cheetah/cheetah.h>

#include <os_compat.h>
#include <rf.h>
#include <wla_bb.h>
#include <linux_wla.h>
#include <linux/mm.h>

#define WLA_PROCFS_NAME	"wla"

#define SEQ_FILE_BUFSIZE    PAGE_SIZE
#define MAX_CMD_STRING_LENGTH   256

#define MAX_WEP_KEYLEN 13

static char ___procfs_cmd[MAX_CMD_STRING_LENGTH];
static struct wla_softc *sc;

struct wla_config
{
    char *name;
    void *data;
    int array_size;
    int (*config_set)(void *, void *, int);
    int (*config_get)(void *, struct seq_file *, int);
    int (*update)(void *, struct seq_file *, int);
    char *hint;
};

int config_set_wds_key(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    int key_idx = (int) working_cfg->data;
    struct wds_key *working_key;
    u8 *key;
    int max_key_length;
    int i, keylen;
    char *input_str = (char *) input;
    char temp_str[3];
    
	if(index<working_cfg->array_size)
    {
        if(key_idx!=0)
            max_key_length = MAX_WEP_KEYLEN;
        else
            max_key_length = MAX_WAPI_KEYLEN;

        working_key = &__wds_key[index];
        working_key->key[key_idx] = &working_key->key_data[key_idx * MAX_WEP_KEYLEN];

        key = working_key->key[key_idx];
        keylen = 0;

		if((__wds_key[index].cipher_type != AUTH_CIPHER_WEP40) && (__wds_key[index].cipher_type != AUTH_CIPHER_WEP104))
		{	
			if((strlen(input_str)>2) && (input_str[2]==' '))
        	{
        	    for(i=0;i<strlen(input_str);i+=3)
        	    {
        	        if(keylen>=max_key_length)
        	            break;
    
        	        *key = (u8) simple_strtoul(&input_str[i], NULL, 16);
        	        key++;
        	        keylen++;
        	    }
        	}
        	else
        	{
        	    memset(temp_str, 0, sizeof(temp_str));
        	    for(i=0;i<strlen(input_str);i+=2)
        	    {
        	        if(keylen>=max_key_length)
        	            break;

        	        temp_str[0] = input_str[i];
        	        temp_str[1] = input_str[i + 1];
        	        *key = (u8) simple_strtoul(temp_str, NULL, 16);
        	        key++;
        	        keylen++;
        	    }
        	}
		}
		else
		{
			for(i=0;i<strlen(input_str);i+=1)
			{
				if(keylen>=max_key_length)
					break;
				temp_str[0] = input_str[i];
				*key=toascii(temp_str[0]);
				key++;
				keylen++;
			}
		}	

        working_key->keylen[key_idx] = keylen;

        printk(KERN_DEBUG "wla: set %s.%d=%s\n", working_cfg->name, index, (char *) input);    
    }

    return 0;
}

int config_get_wds_key(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    int key_idx = (int) working_cfg->data;
    struct wds_key *working_key;
    int i;

    if(index<working_cfg->array_size)
    {
        working_key = &__wds_key[index];

        if(working_key->key[key_idx] && working_key->keylen[key_idx])
        {
            for(i=0;i<working_key->keylen[key_idx];i++)
                seq_printf(s, "%02x ", working_key->key[key_idx][i]);
        }

        seq_printf(s, "\n");
    }

    return 0;
}

int config_set_wds_keyid(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;

    if(index<working_cfg->array_size)
    {
        __wds_key[index].txkeyid = simple_strtoul(input, NULL, 10);

        printk(KERN_DEBUG "wla: set %s.%d=%d\n", working_cfg->name, index, __wds_key[index].txkeyid);
    }

    return 0;
}

int config_get_wds_keyid(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;

    if(index<working_cfg->array_size)
        seq_printf(s, "%d\n", __wds_key[index].txkeyid);
    else
        seq_printf(s, "\n");

    return 0;
}

int config_set_wds_cipher_type(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;

    if(index<working_cfg->array_size)
    {
        __wds_key[index].cipher_type = simple_strtoul(input, NULL, 10);

        printk(KERN_DEBUG "wla: set %s.%d=%d\n", working_cfg->name, index, __wds_key[index].cipher_type);
    }

    return 0;
}

int config_get_wds_cipher_type(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;

    if(index<working_cfg->array_size)
        seq_printf(s, "%d\n", __wds_key[index].cipher_type);
    else
        seq_printf(s, "\n");

    return 0;
}

int config_set_int(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    int *working_data = (int *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
        {
            working_data[index] = simple_strtoul(input, NULL, 10);
        
            printk(KERN_DEBUG "wla: set %s.%d=%d\n", working_cfg->name, index, working_data[index]);    
        }
    }
    else
    {
        *working_data = simple_strtoul(input, NULL, 10);
    
        printk(KERN_DEBUG "wla: set %s=%d\n", working_cfg->name, *working_data);
    }

    return 0;
}

int config_get_int(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    int *working_data = (int *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
            seq_printf(s, "%d\n", working_data[index]);
        else
            seq_printf(s, "\n");
    }
    else
    {
        seq_printf(s, "%d\n", *working_data);
		printk("wla: %s=%d\n",working_cfg->name, *working_data);
    }

    return 0;
}

int config_set_u64(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    u64 *working_data = (u64 *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
        {
            working_data[index] = simple_strtoull(input, NULL, 16);
        
            printk(KERN_DEBUG "wla: set %s.%d=%llx\n", working_cfg->name, index, working_data[index]);
        }
    }
    else
    {
        *((u32 *) working_cfg->data) = simple_strtoull(input, NULL, 16);
    
        printk(KERN_DEBUG "wla: set %s=%llx\n", working_cfg->name, *((u64 *) working_cfg->data));
    }

    return 0;
}

int config_get_u64(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    u64 *working_data = (u64 *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
            seq_printf(s, "%llx\n", working_data[index]);
        else
            seq_printf(s, "\n");
    }
    else
    {
        seq_printf(s, "%llx\n", *working_data);
    }

    return 0;
}

int config_set_u32(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    u32 *working_data = (u32 *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
        {
            working_data[index] = simple_strtoul(input, NULL, 16);
        
            printk(KERN_DEBUG "wla: set %s.%d=%x\n", working_cfg->name, index, working_data[index]);
        }
    }
    else
    {
        *((u32 *) working_cfg->data) = simple_strtoul(input, NULL, 16);
    
        printk(KERN_DEBUG "wla: set %s=%x\n", working_cfg->name, *((u32 *) working_cfg->data));
    }

    return 0;
}

int config_get_u32(void *cfg, struct seq_file *s, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    u32 *working_data = (u32 *) working_cfg->data;

    if(working_cfg->array_size)
    {
        if(index<working_cfg->array_size)
            seq_printf(s, "%x\n", working_data[index]);
        else
            seq_printf(s, "\n");
    }
    else
    {
        seq_printf(s, "%x\n", *working_data);
    }

    return 0;
}

int config_set_rfc(void *cfg, void *input, int index)
{
    struct wla_config *working_cfg = (struct wla_config *) cfg;
    char *input_str = (char *) input;
    char temp_str[3];
    int i;
    int input_bytes = 0;
    int max_input_bytes = MAX_RFC_REG_NUM * 2;
    u8* rfc_data = (u8 *) &rfc_tbl[0];

    char *rf_setting;
    u32  reg, val;

    memset(&rfc_tbl, 0, sizeof(struct bb_regs) * MAX_RFC_REG_NUM);
    memset(&rf_tbl, 0, sizeof(struct rf_regs) *  MAX_RF_REG_NUM);

    rf_setting = input_str;
    i = 0;
    while((rf_setting = strchr(rf_setting, ',')))
    {
        *rf_setting = '\0';
        rf_setting++;

        reg = simple_strtoul(rf_setting, NULL, 16);

        if((rf_setting = strchr(rf_setting, '=')))
        {
            *rf_setting = '\0';
            rf_setting++;
    
            val = simple_strtoul(rf_setting, NULL, 16);

            if(i<MAX_RF_REG_NUM)
            {
                //printk("RF %d %x %x\n", i, reg, val);

                rf_tbl[i].num = reg;
                rf_tbl[i].val = val;
                i++;
            }
        }
    }

    //printk("RFC %s\n", input_str);

    if((strlen(input_str)>2) && (input_str[2]==' '))
    {
        for(i=0;i<strlen(input_str);i+=3)
        {
            if(input_bytes>=max_input_bytes)
                break;

            *rfc_data = (u8) simple_strtoul(&input_str[i], NULL, 16);
            rfc_data++;
            input_bytes++;
        }
    }
    else
    {
        memset(temp_str, 0, sizeof(temp_str));
        for(i=0;i<strlen(input_str);i+=2)
        {
            if(input_bytes>=max_input_bytes)
                break;

            temp_str[0] = input_str[i];
            temp_str[1] = input_str[i + 1];
            *rfc_data = (u8) simple_strtoul(temp_str, NULL, 16);
            rfc_data++;
            input_bytes++;
        }
    }

    printk(KERN_DEBUG "wla: set %s=%s\n", working_cfg->name, (char *) input);    

    return 0;
}

int config_get_rfc(void *cfg, struct seq_file *s, int index)
{
    int i;

    for(i=0;i<MAX_RFC_REG_NUM;i++)
    {
        if(rfc_tbl[i].num==0)
            break;

        seq_printf(s, "%02x%02x", rfc_tbl[i].num, rfc_tbl[i].val);
    }

    seq_printf(s, "\n");

    return 0;
}

#if 0
/*Currently we did not have sta info*/
int config_get_sta(void *cfg, struct seq_file *s, int index)
{
    int i;
    if(NULL==sc)
    {
        seq_printf(s, "\n");
        return -1;
    }

    //mutex_lock(&sc->mutex);

    for(i=0;i<MAC_ADDR_LOOKUP_STA_TABLE_ENTRIES;i++)
    {
        if(sc->stainfo[i].valid)
        {
            seq_printf(s, "mac=%02x:%02x:%02x:%02x:%02x:%02x&mode=%s&rate=%d&signal=%02x\n"
                        ,sc->stainfo[i].macaddr[0], sc->stainfo[i].macaddr[1], sc->stainfo[i].macaddr[2]
                        ,sc->stainfo[i].macaddr[3], sc->stainfo[i].macaddr[4], sc->stainfo[i].macaddr[5]
                        ,sc->stainfo[i].ht_supported ? "11n":"11b/g", sc->stainfo[i].tx_rates[0].idx, sc->mac_info->sta_cap_tbls[i].rssi);
        }
    }

    seq_printf(s, "\n");
    //mutex_unlock(&sc->mutex);

    return 0;
}
#endif

int config_set_channel(void *cfg, void *input, int index)
{
    u32 data;
    int channel;
    int mode;

    data = simple_strtoul(input, NULL, 10);

    mode = (data / 100);
    channel = (data % 100);

    wla_lock();
    wla_cfg(SET_CHANNEL, channel, mode); 
    wla_unlock();

    printk(KERN_DEBUG "wla: set channel %d, mode %d\n", channel, mode);

    return 0;
}

static struct wla_config config[] =
{
    /* CAUTION: always put these variable in alphanumeric order, as we are accessing them via binary sort */
    { "config.hw_mode", &__hw_mode, 0, config_set_u32, config_get_u32, NULL,"(ecos param)", },
    { "config.wl_tx_pwr", &__tx_power, 0, config_set_int, config_get_int, NULL ,"tx power level(dbm)", },
    { "config.wl_power_backoff", &__wl_power_backoff, 0, config_set_u32, config_get_u32, NULL, "(ecos param)", },
    { "config.rfchip", &__rfchip, 0, config_set_u32, config_get_u32, NULL, "0x70:IP301-7,0xb0:IP301-B", },
    { "config.ex_ldo", &__ex_ldo, 0, config_set_u32, config_get_u32, NULL, "0:internal LDO,1:external LDO", },
    { "config.wl_forward_mode", &__sw_forward, 0, config_set_int, config_get_int, NULL ,"(0:HW PATH; 1:SW PATH)", },
    { "config.wl_ap_isolated", &__ap_isolated, 0, config_set_int, config_get_int, NULL ,"(0:ap not isolated; 1:ap isolated)", },
    { "config.wl_bss_isolated", &__bss_isolated, 0, config_set_int, config_get_int, NULL ,"(0:bss not isolated; 1:bss isolated)", },
    { "config.wds_mode", &__wds_mode, 0, config_set_int, config_get_int, NULL ,"0:disable,1:bridge mode,2:repeater mode", },
    { "config.sta_mat", &__sta_mat, 0, config_set_int, config_get_int, NULL ,"0:disable,1:enable", },
    { "config.pw_save", &__pw_save, 0, config_set_int, config_get_int, NULL ,"0:disable,others:enable", },
    { "config.no_ampdu", &__no_ampdu, 0, config_set_int, config_get_int, NULL ,"0:accept rx ampdu, 1:reject rx ampdu", },
    { "config.ampdu_tx_mask", &__ampdu_tx_mask, 0, config_set_int, config_get_int, NULL ,"mask by bit map", },
    { "config.apid", &__apid, 0, config_set_int, config_get_int, NULL ,"AP unit id", },
    { "config.staid", &__staid, 0, config_set_int, config_get_int, NULL ,"STA unit id", },
    { "config.fix_txrate", &__fix_txrate, 0, config_set_int, config_get_int, NULL ,"Fix tx rate", },
    { "config.rate_flags", &__rate_flags, 0, config_set_int, config_get_int, NULL ,"Rate flags", },
    { "config.recovery", &__recovery, 0, config_set_int, config_get_int, NULL ,"0:disable,1:enable", },
    { "config.wds_peer_addr", __wds_peer_addr, WDS_TABLE_ENTRIES, config_set_u64, config_get_u64, NULL ,"WDS peer-AP MAC address", },
	{ "config.wds_cipher_type", (void *) 0, WDS_TABLE_ENTRIES, config_set_wds_cipher_type, config_get_wds_cipher_type, NULL ,"0:None,1:WEP40,2:WEP104,3:TKIP,4:CCMP,5:WAPI", },
	{ "config.wds_keyid", __wds_key, WDS_TABLE_ENTRIES, config_set_wds_keyid, config_get_wds_keyid, NULL ,"TX keyid for WEP", },
    { "config.wds_key0", (void *) 0, WDS_TABLE_ENTRIES, config_set_wds_key, config_get_wds_key, NULL ,"KEY0 for WEP,KEY+MICKEY for TKIP/CCMP/WAPI", },
    { "config.wds_key1", (void *) 1, WDS_TABLE_ENTRIES, config_set_wds_key, config_get_wds_key, NULL ,"KEY1 for WEP", },
    { "config.wds_key2", (void *) 2, WDS_TABLE_ENTRIES, config_set_wds_key, config_get_wds_key, NULL ,"KEY2 for WEP", },
    { "config.wds_key3", (void *) 3, WDS_TABLE_ENTRIES, config_set_wds_key, config_get_wds_key, NULL ,"KEY3 for WEP", },
    { "config.uapsd", &__uapsd_enable, 0, config_set_int, config_get_int, NULL ,"UAPSD switch", },
    { "config.channel", (void *) 0, 0, config_set_channel, NULL, NULL, "channel number and bandwidth", },
#ifdef CONFIG_CHEETAH_SMART_CONFIG
	{ "config.smart_config", &__smartcfg_enable, 0, config_set_int, config_get_int, NULL ,"0:disable,1:enable", },
	{ "config.smart_config_debug", &__smartcfg_dbgmask, 0, config_set_int, config_get_int, NULL ,"debug mask", },
#endif
#if 0	
    { "config.bg_protection", &___bg_protection, 0, config_set_int, config_get_int, "1:enable B/G protection", },
    { "config.hw_buffer_hdr_count", &___hw_buffer_hdr_count, 0, config_set_int, config_get_int, NULL, },
    { "config.sta_cap_tbl_count", &___sta_cap_tbl_count, 0, config_set_int, config_get_int, NULL, },
    { "config.sw_buffer_hdr_count", &___sw_buffer_hdr_count, 0, config_set_int, config_get_int, NULL, },
    { "config.sw_rx_buffer_hdr_count", &___sw_rx_buffer_hdr_count, 0, config_set_int, config_get_int, NULL, },
    { "config.tx_descr_count", &___tx_descr_count, 0, config_set_int, config_get_int, NULL, },
    { "config.rate_tbl_count", &___rate_tbl_count, 0, config_set_int, config_get_int, NULL, },
    { "config.rfc", NULL, 0, config_set_rfc, config_get_rfc, "RFC parameters", },
    { "config.rx_descr_count", &___rx_descr_count, 0, config_set_int, config_get_int, NULL, },
    { "config.wds_bss_idx", __wds_bss_idx, WDS_TABLE_ENTRIES, config_set_int, config_get_int, "BSS index", },
    { "config.wireless_separation", &___wireless_separation, 0, config_set_int, config_get_int, "1:enable wireless separation", },
    { "sta", NULL, 0, NULL, config_get_sta, "Display associated station information", },
#endif	
};

#define NUM_OF_CONFIGS  (sizeof(config)/sizeof(struct wla_config))
#undef isdigit
#define isdigit(c)	('0' <= (c) && (c) <= '9')

static int wla_procfs_exec_set_command(char* cmd)
{
    char *p;
    int i;
    int k;
    int index = 0;

    if(0==strlen(cmd))
        return 0;

    p = strchr(cmd, '=');
    if(p==NULL)
        return 0;

    *p = '\0';
    p++;

    k = strlen(cmd) - 1;
    if(k>=0 && isdigit(cmd[k]))
    {
        index = cmd[k] - '0';
        k--;

        if(k>=0 && isdigit(cmd[k]))
        {
            index += (cmd[k] - '0') * 10;
            k--;
        }

        if((k>=0)&&(cmd[k]=='.'))
            cmd[k] = '\0';
    }

    for(i=0;i<NUM_OF_CONFIGS;i++)
    {
        if(!strcmp(config[i].name, cmd))
        {
            if(config[i].config_set)
                config[i].config_set(&config[i], p, index);

            break;
        }
    }

    return 1;
}

void wla_procfs_show_usage(struct seq_file *s)
{
    int i;

    seq_printf(s, "Configurable attributes list\n");

    for(i=0;i<NUM_OF_CONFIGS;i++)
    {
        if(config[i].array_size)
        {
            seq_printf(s, "   %s.[0~%d]", config[i].name, config[i].array_size - 1);
        }
        else
        {
            seq_printf(s, "   %s", config[i].name);
        }

        if(config[i].hint)
            seq_printf(s, "\t\t%s", config[i].hint);

        seq_printf(s, "\n");
    }

    seq_printf(s, "\nExample:\n");
    seq_printf(s, "   echo config.rfchip=0x70 > /proc/wla\n");
    //seq_printf(s, "   echo config.bg_protection=1 > /proc/wla\n");
    //seq_printf(s, "   echo config.bg_protection > /proc/wla; cat /proc/wla\n");
    //seq_printf(s, "   echo config.wds_cipher_type.0=4 > /proc/wla\n");
    //seq_printf(s, "   echo config.wds_peer_addr.0=0x1234567890 > /proc/wla\n");
    //seq_printf(s, "   echo config.wds_key0.0=00112233445566778899aabbccddeeff > /proc/wla\n");
    //seq_printf(s, "   echo config.wds_mode.0=2 > /proc/wla\n");
    //seq_printf(s, "   echo config.rfc=2001217f22f2 > /proc/wla\n");
    //eq_printf(s, "   echo sta > /proc/wla; cat /proc/wla\n");  
}

static int wla_procfs_exec_get_command(char* cmd, struct seq_file *s)
{
    int i;
    int k;
    int index = 0;

    //printk(KERN_DEBUG "wla: get %s\n", cmd);

    if(0==strlen(cmd))
    {   
        wla_procfs_show_usage(s);
        return 0;
    }

    k = strlen(cmd) - 1;
    if(k>=0 && isdigit(cmd[k]))
    {
        index = cmd[k] - '0';
        k--;

        if(k>=0 && isdigit(cmd[k]))
        {
            index += (cmd[k] - '0') * 10;
            k--;
        }

        if((k>=0)&&(cmd[k]=='.'))
            cmd[k] = '\0';
    }

    for(i=0;i<NUM_OF_CONFIGS;i++)
    {
        if(!strcmp(config[i].name, cmd))
        {
            if(config[i].config_get)
                config[i].config_get(&config[i], s, 0);

            break;
        }
    }

    return 0;
}

static int wla_procfs_show(struct seq_file *s, void *priv)
{
    char user_cmd[MAX_CMD_STRING_LENGTH];

    mutex_lock(&sc->mutex);
    strcpy(user_cmd, ___procfs_cmd);
    ___procfs_cmd[0] = '\0';
    mutex_unlock(&sc->mutex);

    wla_procfs_exec_get_command(user_cmd, s);
    return 0;
}

static int wla_procfs_open(struct inode *inode, struct file *file)
{
    int ret;
    struct seq_file *p;

    ret = single_open(file, wla_procfs_show, NULL);
    if(ret==0)
    {
        p = (struct seq_file *) file->private_data;
    
        if(p->buf==NULL)
        {
            p->buf = kmalloc(SEQ_FILE_BUFSIZE, GFP_KERNEL);

            if(p->buf)
                p->size = SEQ_FILE_BUFSIZE;
        }
    }

    return ret;
}

static ssize_t wla_procfs_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
    int copy_count;
    char user_cmd[MAX_CMD_STRING_LENGTH];

    copy_count = (count > (MAX_CMD_STRING_LENGTH - 1)) ? (MAX_CMD_STRING_LENGTH - 1):count;

    if(0==copy_from_user(user_cmd, buffer, copy_count))
    {
        if(copy_count>0)
        {
            user_cmd[copy_count] = '\0';
            if(user_cmd[copy_count-1]=='\n')
                user_cmd[copy_count-1] = '\0';
        }
        else
        {
            user_cmd[0] = '\0';
        }

        if(!wla_procfs_exec_set_command(user_cmd))
        {
            mutex_lock(&sc->mutex);
            strcpy(___procfs_cmd, user_cmd);
            mutex_unlock(&sc->mutex);
        }

        return copy_count;
    }
    else
    {
        mutex_lock(&sc->mutex); 
        ___procfs_cmd[0] = '\0';
        mutex_unlock(&sc->mutex);

        return -EIO;
    }
}

/* keep track of how many times it is mmapped */
void mmap_open(struct vm_area_struct *vma)
{
		;
}

void mmap_close(struct vm_area_struct *vma)
{
		;
}

/* 
	nopage is called the first time a memory area is accessed which is not in memory,
	it does the actual mapping between kernel and user space memory
*/
static int mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct page *page;
	MAC_INFO *info;
	int ret = 0;

	/* is the address valid? */
	if ((unsigned long)vmf->virtual_address > vma->vm_end) {
		printk("invalid address\n");
		return VM_FAULT_SIGBUS;
	}
	/* the data is in vma->vm_private_data */
	info = (MAC_INFO *)vma->vm_private_data;

	if (!info) {
		printk("no data\n");
		return VM_FAULT_SIGBUS;
	}

	/* get the page */
	page = virt_to_page(info);

	/* increment the reference count of this page */
	get_page(page);

	vmf->page = page;

	return ret | VM_FAULT_LOCKED;;
}

struct vm_operations_struct mmap_vm_ops = {
	.open	= mmap_open,
	.close	= mmap_close,
	.fault	= mmap_fault,
};

static const struct file_operations wla_procfs_fops = {
    .open       = wla_procfs_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
    .write      = wla_procfs_write,
};

int debug_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;
	
	(void)size;
	if (offset & ~PAGE_MASK)
	{
		printk("offset not aligned: %ld\n", offset);
		return -ENXIO;
	}
		
	vma->vm_ops = &mmap_vm_ops;
	vma->vm_flags |= VM_LOCKED;
	/* assign the file private data to the vm private data */
	vma->vm_private_data = filp->private_data;
	mmap_open(vma);
	return 0;
}

int debug_close(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

int debug_open(struct inode *inode, struct file *filp)
{
	MAC_INFO *info = WLA_MAC_INFO;

	filp->private_data = (char *)info;

	return 0;
}

static const struct file_operations wla_debug_fops = {
	.open		= debug_open,
	.release	= debug_close,
	.mmap		= debug_mmap,
};


int wla_procfs_init(struct wla_softc *wla_sc)
{
    if(NULL==proc_create(WLA_PROCFS_NAME, S_IWUSR, NULL, &wla_procfs_fops))
        return -EIO;
    
	if(NULL==proc_create("wla_debug", S_IWUSR, NULL, &wla_debug_fops))
        return -EIO;

    sc = wla_sc;

    return 0;
}

void wla_procfs_exit(void)
{
    remove_proc_entry(WLA_PROCFS_NAME, NULL);

    sc = NULL;
}
