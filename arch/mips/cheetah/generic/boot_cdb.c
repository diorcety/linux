/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/mtd/mtd.h>
#include <linux/sched.h>

#if !defined(CONFIG_BOOT_PARM_OFS)
#define CONFIG_BOOT_PARM_OFS 0x2000
#endif
#if !defined(CONFIG_BOOT_PARM_SZ)
#define CONFIG_BOOT_PARM_SZ  0x4000
#endif
#define BOOTP_ERASE_RETRY_COUNT 3
#define BOOTP_ERASE_RETRY_INTERVAL 10 //tick
#define BOOT_SIZE 0x10000

struct cdb_id
{
    unsigned short mod:5;
    unsigned short type:3;
    unsigned short idx:7;
    unsigned short array:1;
};

#define CDB_ID_(type, index, array)		(((31 & 0x1f) << 11) | ((type & 0x07) << 8) | ((index & 0x7f) << 1) | (array & 0x01))

#define CDB_ID_MOD(id)                  ((id >> 11) & 0x1f)
#define CDB_ID_TYPE(id)                 ((id >> 8) & 0x07)
#define CDB_ID_IDX(id)                  ((id >> 1) & 0x7f)
#define CDB_ID_SUB(id)                  (id & 0x01)

#define CDB_DATA_SZ             16384
#define CDB_EACH_SZ             256

enum
{
    CDB_INT     =1,
    CDB_STR     =2,
    CDB_IP      =3,
    CDB_MAC     =4,
    CDB_IPV6    =5,
    CDB_VER     =3,
    CDB_LEN_INT     = 4,
    CDB_LEN_IP      = 4,
    CDB_LEN_MAC     = 6,
    CDB_LEN_IPV6    = 16,

    CDB_BOOT_HVER   = CDB_ID_(CDB_VER, 0, 0),
    CDB_BOOT_ID     = CDB_ID_(CDB_INT, 1, 0),
    CDB_BOOT_FILE   = CDB_ID_(CDB_STR, 2, 0),
    CDB_BOOT_MAC    = CDB_ID_(CDB_MAC, 3, 1),
    CDB_BOOT_MODE   = CDB_ID_(CDB_INT, 4, 0),
    CDB_BOOT_VER    = CDB_ID_(CDB_VER, 5, 0),
    CDB_BOOT_BUF    = CDB_ID_(CDB_INT, 6, 0),
    CDB_BOOT_SZ     = CDB_ID_(CDB_INT, 7, 0),
    CDB_BOOT_SRC    = CDB_ID_(CDB_INT, 8, 0),
    CDB_BOOT_IP     = CDB_ID_(CDB_IP,  9, 0),
    CDB_BOOT_MSK    = CDB_ID_(CDB_IP, 10, 0),
    CDB_BOOT_GW     = CDB_ID_(CDB_IP, 11, 0),
    CDB_BOOT_SVR    = CDB_ID_(CDB_IP, 12, 0),
    CDB_BOOT_SRC2   = CDB_ID_(CDB_INT, 13, 0),
    CDB_BOOT_LOG_SRC= CDB_ID_(CDB_INT, 14, 0),
    CDB_BOOT_LOG_SZ = CDB_ID_(CDB_INT, 15, 0),
    CDB_BOOT_CDB_LOC= CDB_ID_(CDB_INT, 16, 0),
    CDB_BOOT_CDB_SZ = CDB_ID_(CDB_INT, 17, 0),
    CDB_BOOT_CVER   = CDB_ID_(CDB_STR, 18, 0),
    CDB_BOOT_RFC    = CDB_ID_(CDB_STR, 19, 0),
    CDB_BOOT_PLL    = CDB_ID_(CDB_INT, 20, 0),
	CDB_BOOT_TXVGA	= CDB_ID_(CDB_STR, 21, 0),
	CDB_BOOT_RXVGA	= CDB_ID_(CDB_STR, 22, 0),
	CDB_BOOT_SERIAL	= CDB_ID_(CDB_INT, 23, 0),
	CDB_BOOT_PIN	= CDB_ID_(CDB_STR, 24, 0),
	CDB_BOOT_FREQ_OFS = CDB_ID_(CDB_INT, 25, 0),
	CDB_BOOT_MADC_VAL = CDB_ID_(CDB_STR, 26, 1),

    CDB_ID_SZ = sizeof(struct cdb_id),
    CDB_AID_SZ = CDB_ID_SZ + sizeof(int) ,  /* array idx , ie 4*/
    CDB_LEN_SZ = sizeof(short),
};

#define cdb_log(...)
//#define cdb_log	printk

//-------------------------------------------------------------
struct cdbobj
{
    union 
    {
        unsigned short v; 
        struct cdb_id f;
    } id ;
    unsigned short len;
    int idx;        // skip if not arrary
};

struct parmd
{
    void *val;
    union 
    {
        unsigned short v; 
        struct cdb_id f;
    } id ;
    char *name;
    char *desc;
};

#define CDB_FFST(n) (n+sizeof(unsigned int))
#define CDB_DLEN(p) ((p->len+3)&0xfffc)
#define CDB_AIDX(p) (p->idx)
#define CDB_AIDX_SZ sizeof(int)
#define CDB_ID_LEN_SZ sizeof(short)
#define CDBV(id) 	(*((unsigned short*)&id))

#define	CDBE(v, i, n, d)	{ val: v, id: { i, }, name: n, desc: d}

struct parmd parmds[] =
{
    CDBE(NULL, CDB_BOOT_HVER, "hver", NULL),
    CDBE(NULL, CDB_BOOT_CVER, "cver", NULL),
    CDBE(NULL, CDB_BOOT_ID,   "id", NULL),
    CDBE(NULL, CDB_BOOT_FILE, "file", NULL),
    CDBE(NULL, CDB_BOOT_MAC,  "mac0", NULL),
    CDBE(NULL, CDB_BOOT_MAC,  "mac1", NULL),
    CDBE(NULL, CDB_BOOT_MAC,  "mac2", NULL),
    CDBE(NULL, CDB_BOOT_MODE, "mode", NULL),
    CDBE(NULL, CDB_BOOT_VER,  "ver", NULL),
    CDBE(NULL, CDB_BOOT_BUF,  "buf", NULL),
    CDBE(NULL, CDB_BOOT_SZ,   "size", NULL),
    CDBE(NULL, CDB_BOOT_SRC,  "addr", NULL),
    CDBE(NULL, CDB_BOOT_IP,   "ip", NULL),
    CDBE(NULL, CDB_BOOT_MSK,  "msk", NULL),
    CDBE(NULL, CDB_BOOT_GW,   "gw", NULL),
    CDBE(NULL, CDB_BOOT_SVR,  "server", NULL),
    CDBE(NULL, CDB_BOOT_SRC2, "backup_addr", NULL),
    CDBE(NULL, CDB_BOOT_LOG_SRC, "log_addr", NULL),
    CDBE(NULL, CDB_BOOT_LOG_SZ, "log_size", NULL),
    CDBE(NULL, CDB_BOOT_RFC, "rfc", NULL),
    CDBE(NULL, CDB_BOOT_PLL, "pll", NULL),
    CDBE(NULL, CDB_BOOT_TXVGA, "txvga", NULL),
    CDBE(NULL, CDB_BOOT_RXVGA, "rxvga", NULL),
	CDBE(NULL, CDB_BOOT_SERIAL,"serial", NULL),
	CDBE(NULL, CDB_BOOT_PIN, "pin", "WPS PIN number"), 
	CDBE(NULL, CDB_BOOT_FREQ_OFS, "freq_ofs", "frequnecy offset calib value"),
	CDBE(NULL, CDB_BOOT_MADC_VAL, "madc_val0", "NULL"),
	CDBE(NULL, CDB_BOOT_MADC_VAL, "madc_val1", "NULL"),
};

static struct mtd_info *mtd_info = NULL;
char mac0_buf[18];
char mac1_buf[18];
char mac2_buf[18];

//-------------------------------------------------------------
#define CDB_ID_NUM			(sizeof(parmds)/sizeof(struct parmd))
#define CDB_ID_END			0xffff

/*!-----------------------------------------------------------------------------
 * function: id_to_idx
 *
 * 	\brief 	
 * 	\param 	id
 * 	\return 0: ok
 +----------------------------------------------------------------------------*/
static int id_to_idx(unsigned short id)
{
    int i;
    for(i = 0; i < CDB_ID_NUM; i++)
    {
        if(id == parmds[i].id.v)
            return i;
    }
    cdb_log ("Invalid id: %08x\n", id);
    return -1;
}

/*!-----------------------------------------------------------------------------
 * function: name_to_idx
 *
 * 	\brief 	
 * 	\param 	name
 * 	\return 
 +----------------------------------------------------------------------------*/
static int name_to_idx(const char *name)
{
    int i;
    for(i = 0; i < CDB_ID_NUM; i++)
    {
        if(!strcmp(parmds[i].name, name))
            return i;
    }
    cdb_log ("Invalid name: %s\n", name);
    return -1;
}

#define isdigit(c)          ((c) >= '0' && (c) <= '9')

static int atoi(const char *cp)
{
    char *enddp;
    return simple_strtoul(cp, &enddp, 10); 
}

/*!-----------------------------------------------------------------------------
 * function: name_to_aidx
 *
 *  \brief  convert array index
 *  \param  name
 *  \return 0: ok
 +----------------------------------------------------------------------------*/
unsigned short name_to_aidx(char *name)
{
    unsigned short array_num = 0;
    int i, num, len = strlen(name);

    if(len == 0)
        return 0;

    for(num = 0, i = 1; i <= 3; i++, num++)
        if(!isdigit(name[len - i]))
            break;

    if(num)
        array_num = (unsigned short)atoi(name + len - num);
    return array_num;
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *  \brief
 *  \param offset
 *  \param to
 *  \param len
 *  \return
 +----------------------------------------------------------------------------*/
int flash_read(unsigned int offset, unsigned int to, int len)
{
    return mtd_read(mtd_info, offset, len, NULL, (u_char*)to);
}

/*!-----------------------------------------------------------------------------
 * function:
 *
 *  \brief
 *  \param offset
 *  \param from
 *  \param len
 *  \return
 +----------------------------------------------------------------------------*/
int flash_write(unsigned int offset, unsigned int from, int len)
{
    return mtd_write(mtd_info, offset, len, NULL, (u_char*)from);
}

/*!-----------------------------------------------------------------------------
 * function: flash_erase
 *
 *  \brief
 *  \param offset
 *  \param len
 *  \return
 +----------------------------------------------------------------------------*/
int flash_erase(unsigned int offset, int len)
{
    struct erase_info instr;
    instr.mtd = mtd_info;
    instr.addr = offset;
    instr.len = len;
    instr.callback = NULL;
    instr.fail_addr = MTD_FAIL_ADDR_UNKNOWN;
    return mtd_erase(mtd_info, &instr);
}

#define MAX_ATTR_VALUE_SIZE	256

static int boot_cdb_show(struct seq_file *s, void *priv)
{
    unsigned char buf[MAX_ATTR_VALUE_SIZE+1];
    struct parmd *pd;
    struct cdbobj obj;
    struct cdbobj *p = &obj;
    unsigned short array_num = 0;
    int idx, dl, num = 0;
    unsigned int offset ,dp;

    for(offset = CONFIG_BOOT_PARM_OFS;
       offset < (CONFIG_BOOT_PARM_OFS + CONFIG_BOOT_PARM_SZ);
       offset = (CDB_FFST(offset) + CDB_DLEN(p)))
    {
        flash_read(offset, (unsigned int)p, sizeof(struct cdbobj));
        //seq_printf(s, "MOD %d, TYPE %d, IDX %d, SUB %d\n", CDB_ID_MOD(p->id.v), CDB_ID_TYPE(p->id.v), CDB_ID_IDX(p->id.v), CDB_ID_SUB(p->id.v));                 

        if(p->id.v == CDB_ID_END)
            break;

        if((idx = id_to_idx(p->id.v)) <0)
            continue;
        array_num = 0;

        if(p->id.f.array)
            array_num = CDB_AIDX(p);
        pd = &parmds[idx + array_num];

        dp = CDB_FFST(offset);
        dl = CDB_DLEN(p);

        if(dl>=MAX_ATTR_VALUE_SIZE)
            dl = MAX_ATTR_VALUE_SIZE;

        if(p->id.f.array)
        {
            dp += 4;    /* 4 bytes index */
            dl -= 4;
        }
        flash_read(dp, (unsigned int)buf, dl);
        if(CDB_STR == p->id.f.type)
            buf[dl] = 0;

        seq_printf(s, "%s=", pd->name);
        switch(pd->id.f.type)
        {
            case CDB_INT:
                seq_printf(s, "%08x\n", *(unsigned int*) buf);
                break;
            case CDB_IP:
                seq_printf(s, "%d.%d.%d.%d\n", buf[0], buf[1], buf[2], buf[3]);
                break;
            case CDB_MAC:
                seq_printf(s, "%02x:%02x:%02x:%02x:%02x:%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                if(!strcmp(pd->name,"mac0"))
                   sprintf(mac0_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                else if(!strcmp(pd->name,"mac1"))
                   sprintf(mac1_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                else if(!strcmp(pd->name,"mac2"))
                   sprintf(mac2_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                break;
            case CDB_STR:
                seq_printf(s, "%s\n", buf);
                break;
        }

        num++;
    }

    return 0;
}


static int boot_cdb_open(struct inode *inode, struct file *file)
{
	struct mtd_info *rc;
	if (!mtd_info) {
		rc = get_mtd_device(NULL, 0);
		if(IS_ERR(rc))
			return PTR_ERR(rc);
		else
			mtd_info = rc;
	}
	return single_open(file, boot_cdb_show, NULL);
}

#define MAX_ARGV 3

static int get_args(const char *string, char *argvs[])
{
	char *p;
	int n;

	argvs[0] = 0;
	n = 0;

	p = (char*) string;
	while (*p == ' ')
		p++;
	while (*p)
	{
		argvs[n] = p;
		while (*p != ' ' && *p)
			p++;
		if (*p == 0)
			goto out;
		*p++ = '\0';
		while (*p == ' ' && *p)
			p++;
out:
		n++;
		if (n == MAX_ARGV)
			break;
	}
	return n;
}

/*!-----------------------------------------------------------------------------
 * function:
 *      
 *  \brief
 *  \param buf
 *  \param mac
 *  \return
 +----------------------------------------------------------------------------*/
    
static int ether_aton(char *buf, unsigned char *mac)
{
    short i;
    unsigned int m[6];
    if (6 != (sscanf
         (buf, "%x:%x:%x:%x:%x:%x", m, m + 1, m + 2, m + 3, m + 4, m + 5)) )
        return -1;
    for (i = 0; i < 6; i++)
        mac[i] = m[i];
    return 0;
}  

/*!-----------------------------------------------------------------------------
 * function:
 *      
 *  \brief
 *  \param str
 *  \param dp
 *  \return
 +----------------------------------------------------------------------------*/

static int inet_aton(char *str, void *dp)
{
    short i;
    int d[6];
    char *ip = (char *) dp;
    if (4 != (sscanf(str, "%d.%d.%d.%d", d, d + 1, d + 2, d + 3)) )
        return -1;
    for (i = 0; i < 4; i++)
        ip[i] = 0xff&d[i] ;
    return 0;
}

/*!-----------------------------------------------------------------------------
 * function: cdb_check_data_len
 *  \brief  check cdb length is valid
 *  \param 
 *  \return 
 +----------------------------------------------------------------------------*/
#define ALIGN4(a) ((a+3)&~3) //align to 4 bytes
unsigned short cdb_check_data_len(unsigned short type, void *data)
{
    unsigned short len;
    switch(type)
    {
        case CDB_IP:
        case CDB_INT:
            len = CDB_LEN_INT;
            break;
        case CDB_MAC:
            len = CDB_LEN_MAC;
            break;
        case CDB_IPV6:
            len = CDB_LEN_IPV6;
            break;
        case CDB_STR:
        default:
            len = strlen(data);
            if(len > CDB_EACH_SZ)
                len = CDB_EACH_SZ;
            break;
    }
    len = ALIGN4(len);
    return len;
}

/*!-----------------------------------------------------------------------------
 * function: cdb_move_bootp
 *  \brief  move bootp to dram
 *  \param dst(DST address)
 *  \return 
 +----------------------------------------------------------------------------*/
void cdb_move_bootp(unsigned int dst)
{
	short len;
	struct cdbobj obj;
	struct cdbobj *p = &obj;
	unsigned short cid;
	int p_end=0;
	unsigned int offset;

	for (offset = CONFIG_BOOT_PARM_OFS;
		offset < (CONFIG_BOOT_PARM_OFS + CONFIG_BOOT_PARM_SZ);
		offset = (CDB_FFST(offset) + CDB_DLEN(p)))
	{
		flash_read(offset, (unsigned int)p, sizeof(struct cdbobj));
		cid = p->id.v;
		if (CDB_ID_END == cid) //finish case
			break;
		len = CDB_AIDX_SZ + p->len;
		if(!(id_to_idx(cid)<0)) {
			cdb_log("[CDB] bootp: offset=%06x id=%x len=%03d is_array=%d aidx=%02d\n", offset, cid, len, CDB_ID_SUB(cid), CDB_ID_SUB(cid)?CDB_AIDX(p):-1);
			flash_read(offset, dst+p_end, len);
			p_end += len;
		}
	}
	cdb_log("[CDB] bootp: move to @ %x~%x\n", dst, dst+p_end);
}

void cdb_write_flash(unsigned short array_num, short len, short is_array, int cid, void *val)
{
    struct cdbobj obj;
    struct cdbobj *p=&obj, wp;	
    unsigned int offset, roffset=0, wh, fp;
    void *pp;
    int j=0;

update_cdb:    
        for(offset = CONFIG_BOOT_PARM_OFS;
           (offset + len + 8) < (CONFIG_BOOT_PARM_OFS + CONFIG_BOOT_PARM_SZ);
           offset = (CDB_FFST(offset) + CDB_DLEN(p)))
        {
            flash_read(offset, (unsigned int)p, sizeof(struct cdbobj));
            if(p->id.v == cid)                        
            {
                if (is_array && (CDB_AIDX(p) != array_num))       
                    continue;                         
                roffset = offset;                              
                continue;                         
            }
            else if ( CDB_ID_END != p->id.v )         
                    continue;
            //cdb_log("[CDB] bootp: id=%x, aidx=%d, @ %p, old=%p\n", id, array_num, p, roffset);
            
            wp.id.v = cid;
            wp.len = len;
            wp.idx = array_num;
            wh = CDB_ID_SZ + CDB_LEN_SZ ;
            fp = (unsigned int)CDB_FFST(offset);
            if (is_array)
            {
                wp.len += CDB_AIDX_SZ ; // +4
                wh += CDB_AIDX_SZ;
                fp += CDB_AIDX_SZ;
            }

            //cdb_log("[CDB] going to write %u, size %d\n", offset, wh);
            flash_write(offset, (unsigned int)&wp, wh);
            //cdb_log("[CDB] going to write %u, size %d, val %s\n", fp, len, argv[1]);
            flash_write(fp, (unsigned int)val, len);
            if (roffset)
            {
                unsigned short zero=0; 
                //cdb_log("[CDB] going to write %u, size %d all zero\n", roffset), CDB_ID_SZ);
                flash_write(roffset, (unsigned int)&zero, CDB_ID_SZ);
            }
            break;
        }

        //handle boot parameter area full case
        if (((offset + len + 8) >= (CONFIG_BOOT_PARM_OFS + CONFIG_BOOT_PARM_SZ)) && j++<BOOTP_ERASE_RETRY_COUNT) {
            printk("[BOOTVAR] boot parameter area full case\n");
            if ((pp = kmalloc(BOOT_SIZE, GFP_KERNEL))) {
                // 1. read bootldr to DRAM
                flash_read(0, (unsigned int)pp, BOOT_SIZE);
                // 2. update boot parameters
                memset((void *)((unsigned int)pp + CONFIG_BOOT_PARM_OFS), 0xff, CONFIG_BOOT_PARM_SZ);
                cdb_move_bootp((unsigned int)pp + CONFIG_BOOT_PARM_OFS);
                // 3. erase flash
                flash_erase(0, BOOT_SIZE);
                // 4. write back to flash
                flash_write(0, (unsigned int)pp, BOOT_SIZE);
                kfree(pp);
            }
            else {
                printk("[BOOTVAR] non-MEM to backup bootldr; retry:%d\n", j);
                schedule();
            }
            goto update_cdb;
        }
}

void rewrite_mac_buf(int idx,unsigned char *mac)
{
    mac[6] = 0;
    mac[7] = 0;
    if(idx == 4)
       sprintf(mac0_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    else if(idx == 5)
       sprintf(mac1_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    else if(idx == 6)
       sprintf(mac2_buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static ssize_t boot_cdb_write(struct file *filp, const char *buff, size_t count, loff_t *off)
{
	char buf[MAX_ATTR_VALUE_SIZE+1] = {0};
	int argc = 0;
	char *argv[MAX_ARGV];
    int id, cid;
    short is_array, len;
    unsigned short array_num = 0;
    void *val;
    int i, ip;
    char mac[8];
    
    if (copy_from_user(buf, buff, count))
        return -EFAULT;

    buf[count-1] = '\0';
	argc = get_args((const char *)buf, argv);

    //cdb_log ("argc %d\n", argc);
    
    if(argc == 2)
    {
        //if(argv[0]
        if((id = name_to_idx(argv[0])) < 0)
            return count;

        cid = parmds[id].id.v;
            
        switch(CDB_ID_TYPE(cid))
        {
            case CDB_INT:
                if(sscanf(argv[1], "%x", &i) != 1)
                {
                    cdb_log("[CDB] bootp: cdb data type error!\n");
                    return count;
                }
                val = &i;
                break;
            case CDB_MAC:
                if(ether_aton(argv[1], mac))
                {
                    cdb_log("[CDB] bootp: cdb data type error!\n");
                    return count;
                }
                rewrite_mac_buf(id,mac);
                val = mac;
                break;
            case CDB_IP:
                if(inet_aton(argv[1], &ip))
                {
                    cdb_log("[CDB] bootp: cdb data type error!\n");
                    return count;
                }
                val = &ip;
                break;
            case CDB_STR:
            default:
                val = argv[1];
                break;
        }
        
        cdb_log("MOD %d, TYPE %d, IDX %d, SUB %d\n", CDB_ID_MOD(cid), CDB_ID_TYPE(cid), CDB_ID_IDX(cid), CDB_ID_SUB(cid));                 

        array_num = name_to_aidx(argv[0]);

        len = cdb_check_data_len(CDB_ID_TYPE(cid), argv[1]);
        is_array = (CDB_ID_SUB(cid));

        if(CDB_ID_TYPE(cid) == CDB_MAC)
        {
           ether_aton(mac0_buf,mac);
           cdb_write_flash(name_to_aidx("mac0"),len,is_array,parmds[4].id.v,mac);
           ether_aton(mac1_buf,mac);
           cdb_write_flash(name_to_aidx("mac1"),len,is_array,parmds[5].id.v,mac);
           ether_aton(mac2_buf,mac);
           cdb_write_flash(name_to_aidx("mac2"),len,is_array,parmds[6].id.v,mac);
        }
        else
           cdb_write_flash(array_num,len,is_array,cid,val);
    }
	
    return count;
}

static const struct file_operations boot_cdb_fops = {
    .open       = boot_cdb_open,
    .read       = seq_read,
    .write      = boot_cdb_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int __init boot_cdb_init(void)
{
    if(NULL==proc_create("bootvars", S_IRUSR, NULL, &boot_cdb_fops))
        return -EIO;

    return 0;
}

device_initcall(boot_cdb_init);


