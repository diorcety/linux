
#ifndef _CTA_I2S_H
#define _CTA_I2S_H

#define BIT0    0x0001
#define BIT1    0x0002
#define BIT2    0x0004
#define BIT3    0x0008
#define BIT4    0x0010
#define BIT5    0x0020
#define BIT6    0x0040
#define BIT7    0x0080
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000


/* could adjust it from fixed address while ANA_BASE appear */
#define REG_UPDATE32(x,val,mask) do {                     \
        u32 newval;                                           \
        newval = *(volatile u32*) ((x));                      \
        newval = (( newval & ~(mask) ) | ( (val) & (mask) )); \
        *(volatile u32*)((x)) = newval;                       \
} while(0)
#define I2S_CLKUPDATE(x, val)       REG_UPDATE32( (x), \
                     I2S_CLK_NOGATE | (val), \
                     I2S_CLK_UPDATE | I2S_CLK_NOGATE | I2S_CLK_POSTDIV | I2S_CLK_PREDIV)
#define PCM_CLKUPDATE(x, val)       REG_UPDATE32( (x), \
                     PCM_CLK_NOGATE | (val), \
                     PCM_CLK_UPDATE | PCM_CLK_NOGATE | PCM_CLK_POSTDIV | PCM_CLK_PREDIV)


/* I2S clock */
#define CTA_I2S_SYSCLK      0

#define AIREG(reg)  ((volatile unsigned int*)(AI_BASE))[reg]
#define CFG         (0x00/4)
#define CH0_TXBASE  (0x04/4)
#define CH0_RXBASE  (0x08/4)
#define CH1_TXBASE  (0x0c/4)
#define CH1_RXBASE  (0x10/4)
#define R05         (0x14/4)
#define CH0_CTRL    (0x18/4)
#define CH1_CTRL    (0x1c/4)
#define DUMMY       (0x20/4)
#define NOTIFY      (0x24/4)

#define DES_OWN     (1<<31)      //Owner bit 1:SW 0:HW
#define DES_EOR     (1<<30)      //End of Ring
#define DES_SKP     (1<<29)      //Skip compensated data
#define DES_HWD     (1<<28)      //Hw haneld
#define CNT_MASK    (0x000001ff) //Mask of Counter

#define NOTIFY_TX_BIT(ch) (ch * 16)     //0*16 or 1*16
#define NOTIFY_RX_BIT(ch) (ch * 16 + 1) //0*16+1 or 1*16+1

#define CTA_CH0 0
#define CTA_CH1 1
#define NUM_CH  2
#define TX      0
#define RX      1
#define NUM_DIR 2

#define IF_PCM 0  
#define IF_I2S 1

#define CTA_ONE_REQ 16
/* CTA_IDX_BUFSIZE could be 0, 1, 2. meaning 80, 160, 320 bytes */
#define CTA_IDX_BUFSIZE 2
#define CTA_MAX_BUFSIZE ((80 << CTA_IDX_BUFSIZE) * CTA_ONE_REQ)
#define CTA_DESC_NUM (CTA_ONE_REQ * 2)
#define CTA_BOTH_DESC_NUM (CTA_DESC_NUM * 2)
#define CTA_DESC_BUFSIZE (CTA_BOTH_DESC_NUM * sizeof(struct cta_desc))

/* possible sample rate in cta_i2s.c
  8000, *11025, 16000, 22050, *24000, 32000, 44100, 48000, *88200, 96000, 128000
  "*" means not supported for wm8750
  24000 & 88200 is not used but reserved
*/
#define SAMPLE_RATE_SUPPORT (11) 

struct cta_desc {
    volatile unsigned int info;
    volatile void * buf;
};

struct cta_device {
    int if_mode; //0:PCM 1:I2S
/* common register */
    int dma_mode;
    int freq;
    int buf_size;
    int rol;    //0:Master 1:Slave
/* PCM register */
    int fs_pol;
    int fs_rate;
    int lfs_en;
    int lfs_interval;
    int pcm_mclk;
    int rx_latch;
/* I2S register */
    int swapch;
    int timing;
    int i2s_rat;
    int rat_updated;
    spinlock_t rat_lock;  /* protects update rat */
};

struct cta_master {
    struct resource *res;
    struct resource *ioarea;
    void __iomem *ioaddr;       /* Mapped address */
    struct platform_device *pdev;

    struct cta_device device;
    int master;
    u32 fmt;
};

struct cta_rat_reg {
    int val_1;
    int val_2;
};

extern spinlock_t i2c_lock; //spinlock for share pinmux access

#endif
