/*
 * ALSA Soc Audio Layer
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/atomic.h>

#include "cta-pcm.h"
#include "cta-i2s.h"

//#define CTA_DEBUG
#ifdef CTA_DEBUG
#undef pr_debug
#define pr_debug(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)

#define dp(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#endif

static int sample_rate_table[SAMPLE_RATE_SUPPORT] = {
    8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000, 128000
};

static int pcm_rate_table[SAMPLE_RATE_SUPPORT] = {
#ifdef CONFIG_CHEETAH_FPGA
    0, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1 
#else
    0, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1
#endif
};

static int i2s_rate_table[SAMPLE_RATE_SUPPORT] = {
#ifdef CONFIG_CHEETAH_FPGA
    5, 4, 5, 4, -1, 3, 4, 5, -1, 6, -1
#else
    0, -1, 1, 2, -1, 3, 4, 5, -1, 6, 7
#endif
};

static struct cta_rat_reg pcm_rate_reg_table[SAMPLE_RATE_SUPPORT] = {
    {0x6,0xfa0}, {-1,-1}, {0x6,0xfa0}, {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1}
};

static struct cta_rat_reg i2s_rate_reg_table[SAMPLE_RATE_SUPPORT] = {
    {0x78,0xfa0}, {-1,-1}, {0x3c,0xfa0}, {0x40,0xaa1}, {-1,-1}, {0x1e,0xfa0}, {0x20,0xaa1}, {0x14,0xfa0}, {-1,-1}, {0x2,0xfa0}, {0x2,0xbb8}
};

static void cta_device_init(struct cta_device *dev, struct snd_soc_dai *cpu_dai);

static atomic_t i2s_clk_ref = {.counter = 0};
//static atomic_t pcm_clk_ref = {.counter = 0};

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/

#ifdef AIREG
#undef AIREG
#define AIREG(reg) ((volatile unsigned int*)(master->ioaddr))[reg]
//#define AIREG(reg)  ((volatile unsigned int*)(AI_BASE))[reg]
#endif
#define CFGREG(val, mask) tmp = ((tmp&~mask)|val)

#ifdef CONFIG_CHEETAH_FPGA
static unsigned int cta_i2s_rate(int rat)
{
    switch (rat)
    {
        case 0:
            return 8;
        case 1:
            return 16;
        case 2:
            return 22;
        case 3:
            return 32;
        case 4:
            return 44;
        case 5:
            return 48;
        case 6:
            return 96;
        case 7:
        default:
            return 128;
    }
}
#endif

static int cta_rate2idx(int rat)
{
    int i;
    for (i=0; i<SAMPLE_RATE_SUPPORT; i++) {
        if (rat == sample_rate_table[i]) return i;
    }
    return -1;
}

static void cta_device_init(struct cta_device *dev, struct snd_soc_dai *cpu_dai)
{
    int tmp = 0;    //variable for CFGREG macro
    struct cta_master * master;
    master = snd_soc_dai_get_drvdata(cpu_dai);
 
    /* common configuration */
    {
        CFGREG((dev->dma_mode     << 14), BIT14);  //should always be 0
        CFGREG((CTA_IDX_BUFSIZE     << 9) , (BIT10|BIT9));
        CFGREG((dev->rol       << 1) , BIT1);
    }
    /* PCM configuration */
    if (dev->if_mode == IF_PCM)
    {
        CFGREG((dev->pcm_mclk     << 19), (BIT21|BIT20|BIT19));
        CFGREG((dev->rx_latch     << 15), BIT15);
        CFGREG((dev->freq         << 11), (BIT13|BIT12|BIT11));
        CFGREG((7                 << 5) , (BIT8|BIT7|BIT6|BIT5)); //dev->lfs_interval
        CFGREG((dev->fs_rate      << 4) , BIT4);
        CFGREG((0                 << 3) , BIT3);   //dev->lfs_en
        CFGREG((1                 << 2) , BIT2);
        CFGREG((1                 << 0) , BIT0); //enable module
    }
    /* I2S configuration */
    else
    {
#ifdef CONFIG_CHEETAH_FPGA

        /* lack PLL provide multi-clock in FPGA, using GPIO temporarily */
        switch(dev->i2s_rat)
        {
            case 0:
            case 1:
            case 2:
            case 7:
                printk(KERN_EMERG "FPGA not support this kind of I2S clock for sample rate %dKHz\n", cta_i2s_rate(dev->i2s_rat));
                break;
        }
    //TODO: move set clock into cta_i2s_set_dai_sysclk 
        GPREG(TMIICLK) &= ~(7<<28);
        GPREG(TMIICLK) |= (dev->i2s_rat<<28);
#endif
        CFGREG((dev->timing       << 18), BIT18);
        CFGREG((dev->swapch       << 17), BIT17);
        CFGREG((1                 << 16), BIT16); //enable module
    }
    pr_debug("dump PCM register :AIBASE %p, CFG %x, CH0TX %x, CH1TX %x, CH0ctl %x, CH1ctl %x\n",
        master->ioaddr, AIREG(CFG), AIREG(CH0_TXBASE), AIREG(CH1_TXBASE), AIREG(CH0_CTRL), AIREG(CH1_CTRL));
    AIREG(CFG) = tmp;
    pr_debug("<Enable Audio Interface Module>");
}

static void cta_reset_module(void)
{
    int i;
    pr_debug("<Reset Audio Interface Module>");
    /* reset audio interface module */
#ifdef CONFIG_CHEETAH_SND_PCM0
    GPREG(SWRST) &= ~(1<<14);//set audio interface reset
    for (i=0; i < 200; i++);
    GPREG(SWRST) |= (1<<14);//clr audio interface reset
#endif
#ifdef CONFIG_CHEETAH_SND_PCM1
    GPREG(SWRST) &= ~(1<<18);
    for (i=0; i < 200; i++);
    GPREG(SWRST) |= (1<<18);
#endif
}

static void cta_init(void)
{
    /* audio interface pin selection */
    GPREG(GDEBUG) |= (1<<20);

    if (GPREG(MISC1) & BIT11)
    {
        /* enable RMII mode if package is BGA196 */
        GPREG(TMIICLK) |= (1<<6);
    }
    else if (GPREG(MISC1) & (BIT7 | BIT8))
    {
        /* disable ASIC debug port(31 pins), bit[4] = 1'b0
           if package is LQFP128-RMII and LQFP128-MII */
        GPREG(GDEBUG) &= ~(1<<4);
    }

    /* release audio interface module clk */
#ifdef CONFIG_CHEETAH_SND_PCM0
    GPREG(SWRST) &= ~(1<<15);
#endif

#ifdef CONFIG_CHEETAH_SND_PCM1
    GPREG(SWRST) &= ~(1<<19);
#endif

    /* reset audio interface module */
    cta_reset_module();
}

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/
static int cta_i2s_startup(struct snd_pcm_substream *substream,
                  struct snd_soc_dai *cpu_dai)
{
    pr_debug("%s:%d\n", __func__, __LINE__);
    snd_soc_dai_get_drvdata(cpu_dai);

    atomic_inc(&i2s_clk_ref);
    return 0;
}

/* wait for I2S controller to be ready */
static inline int cta_i2s_wait(void)
{
    return 0;
}

static int cta_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
        unsigned int fmt)
{
    struct cta_master * master;

    pr_debug("%s:%d name=%s fmt=%d\n", __func__, __LINE__, cpu_dai->name, fmt);

    master = snd_soc_dai_get_drvdata(cpu_dai);

    /* interface format */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S: /* I2S mode */
        master->fmt = SND_SOC_DAIFMT_I2S;
        break;
    case SND_SOC_DAIFMT_RIGHT_J: /* Right Justified mode */
        break;
    case SND_SOC_DAIFMT_LEFT_J: /* Left Justified mode */
        break;
    case SND_SOC_DAIFMT_DSP_A: /* L data MSB after FRM LRC */
        master->fmt = SND_SOC_DAIFMT_DSP_A;
        break;
    case SND_SOC_DAIFMT_DSP_B: /* L data MSB during FRM LRC */
        break;
    case SND_SOC_DAIFMT_AC97: /* AC97 */
        break;
    }

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBS_CFS:
        master->master = 1;
        break;
    case SND_SOC_DAIFMT_CBM_CFS:
        master->master = 0;
        break;
    default:
        break;
    }

    return 0;
}

static int cta_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
        int clk_id, unsigned int freq, int dir)
{
    pr_debug("%s:%d name=%s clk_id=%d freq=%d dir=%d\n", __func__, __LINE__, cpu_dai->name, clk_id, freq, dir);

    if (clk_id != CTA_I2S_SYSCLK)
        return -ENODEV;

    return 0;
}

static int cta_i2s_hw_params(struct snd_pcm_substream *substream,
                struct snd_pcm_hw_params *params,
                struct snd_soc_dai *cpu_dai)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    int ch0_config = 0, ch1_config = 0;
    struct cta_master *master;
    struct cta_device *device;
    unsigned int reg_tmp;
    int i2s_rat;
    int fs_rate;
    int idx=0;
    struct cta_rat_reg reg={0,0};
    unsigned long flags;

    master = snd_soc_dai_get_drvdata(cpu_dai);
    if (!master) {
        pr_debug("%s:%d no master!!\n", __func__, __LINE__);
        return -1;
    }

    device = &master->device;

    idx = cta_rate2idx(params_rate(params));
    pr_debug("%s:%d rate:%d, idx:%d\n", __func__, __LINE__, params_rate(params), idx);
    if (idx < 0) {
        printk(KERN_ERR "Not support sample rate\n");
    }

    spin_lock_irqsave(&device->rat_lock, flags);
    if (!device->rat_updated) {
        if (idx >= 0) {
            device->i2s_rat = i2s_rate_table[idx];        
            device->fs_rate = pcm_rate_table[idx];        
            if (master->fmt == SND_SOC_DAIFMT_I2S) {
#ifndef CONFIG_CHEETAH_FPGA
                reg.val_1 = i2s_rate_reg_table[idx].val_1;
                reg.val_2 = i2s_rate_reg_table[idx].val_2;
#endif
            }
            else {
                switch (device->fs_rate) {
                    case 0:
                    case 1:
                        device->freq = 2; //can't be 0
                        device->pcm_mclk = 5;
                        pr_debug("%s:%d freq:%d, pcm_mclk:%d in PCM\n", __func__, __LINE__, device->freq, device->pcm_mclk);
                        break;
                    default:
                        printk(KERN_ERR "Abnormal sample rate:%d in PCM(0)\n", device->fs_rate);
                        break;
                }
#ifndef CONFIG_CHEETAH_FPGA
                reg.val_1 = pcm_rate_reg_table[idx].val_1;
                reg.val_2 = pcm_rate_reg_table[idx].val_2;
#endif
            }
#ifdef CONFIG_CHEETAH_FPGA
            device->rat_updated = 1;
            pr_debug("%s:%d updated=%d\n", __func__, __LINE__, device->rat_updated);
#else
            if ((reg.val_1 < 0) || (reg.val_2 < 0)) {
                printk(KERN_ERR "Abnormal reg value:(%d,%d) in mode:%d\n", reg.val_1, reg.val_2, master->fmt);
            } else {
                pr_debug("%s:%d reg value:(0x%x,0x%x) in mode %d\n", __func__, __LINE__, reg.val_1, reg.val_2, master->fmt);
                I2S_CLKUPDATE(I2S_FREQUENCY_SETTING, (reg.val_1) | ((reg.val_2) << 8));
                I2S_CLKUPDATE(I2S_FREQUENCY_SETTING, I2S_CLK_UPDATE | (reg.val_1) | ((reg.val_2) << 8));
                I2S_CLKUPDATE(I2S_FREQUENCY_SETTING, (reg.val_1) | ((reg.val_2) << 8));
                device->rat_updated = 1;
                pr_debug("%s:%d updated=%d\n", __func__, __LINE__, device->rat_updated);
            }
#endif
        } else {
            printk(KERN_ERR "Abnormal idx:%d\n", idx);
        }
    } else {
        pr_debug("%s:%d No need to update sample rate\n", __func__, __LINE__);
    }
    spin_unlock_irqrestore(&device->rat_lock, flags);

    if (atomic_read(&i2s_clk_ref) > 1) {
        i2s_rat = (idx<0) ? (-1) : i2s_rate_table[idx];
        fs_rate = (idx<0) ? (-1) : pcm_rate_table[idx];
        pr_debug("%s:%d i2s_rate:%d, fs_rate:%d\n", __func__, __LINE__, i2s_rat, fs_rate);
        if (master->fmt == SND_SOC_DAIFMT_I2S) {
            if (device->i2s_rat != i2s_rat) {
                pr_debug("%s:%d fail to set, i2s_rat=[%d:%d]\n", __func__, __LINE__, device->i2s_rat, i2s_rat);
                return -1;
            }
        }
        else {
            if (device->fs_rate != fs_rate) {
                pr_debug("%s:%d fail to set, fs_rate=[%d:%d]\n", __func__, __LINE__, device->fs_rate, fs_rate);
                return -1;
            }
        }
    }

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
        AIREG(CH0_TXBASE) = prtd->dma_desc_array_phys;
        AIREG(CH1_TXBASE) = prtd->dma_desc_array_phys2;
    }
    else {
        AIREG(CH0_RXBASE) = prtd->dma_desc_array_phys;
        AIREG(CH1_RXBASE) = prtd->dma_desc_array_phys2;
    }

    if (master->fmt == SND_SOC_DAIFMT_I2S)
    {
        device->if_mode = IF_I2S;
//        if (atomic_read(&i2s_clk_ref) == 0)
//            GPREG(0x81c/4) |= (1 << 6);  //i2s clk enable
//        atomic_inc(&i2s_clk_ref);
    }
    else
    {
        device->if_mode = IF_PCM;
        device->rx_latch = 1;    /* this statement influences PCM capture */

//        if (atomic_read(&pcm_clk_ref) == 0)
//            GPREG(0x81c/4) |= (1 << 7);
//        atomic_inc(&pcm_clk_ref);
    }
    device->timing = 0;
    device->swapch = 0;

    if (master->master)
        device->rol = 0;
    else
        device->rol = 1;

    pr_debug("%s:%d format=%d\n", __func__, __LINE__, params_format(params));
    /* set bit order to MSB */
    ch0_config |= BIT10;
    switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S8:
        case SNDRV_PCM_FORMAT_U8:
            ch0_config &= ~(BIT12 | BIT13);
            //notice that 8 bit only support in PCM mode in spec
            break;
        case SNDRV_PCM_FORMAT_S16_LE:
        case SNDRV_PCM_FORMAT_S16_BE:
        case SNDRV_PCM_FORMAT_U16_LE:
        case SNDRV_PCM_FORMAT_U16_BE:
            ch0_config &= ~(BIT12 | BIT13);
            ch0_config |= (BIT12);
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
        case SNDRV_PCM_FORMAT_S24_BE:
        case SNDRV_PCM_FORMAT_U24_LE:
        case SNDRV_PCM_FORMAT_U24_BE:
        case SNDRV_PCM_FORMAT_S32_LE:
        case SNDRV_PCM_FORMAT_S32_BE:
        case SNDRV_PCM_FORMAT_U32_LE:
        case SNDRV_PCM_FORMAT_U32_BE:
            ch0_config |= (BIT12| BIT13);
            //notice that 24,32 bit only support in I2S mode in spec
            //Attention: 24, 32 bit is not functional at this time
            break;
    }


    ch0_config |= BIT0;
    ch1_config = ch0_config;
    if(master->fmt != SND_SOC_DAIFMT_I2S){
        ch1_config |= BIT4; /* set ch1 PCM slot id to 2(divide 2 while 16bit), ch0 slot id should always be 0 */
    }
    reg_tmp = AIREG(CH0_CTRL);
    reg_tmp = ((reg_tmp & ~(BIT0 | BIT10 | BIT12 | BIT13)) | ch0_config);
    AIREG(CH0_CTRL) = reg_tmp;
    reg_tmp = AIREG(CH1_CTRL);
    reg_tmp = ((reg_tmp & ~(BIT0 | BIT10 | BIT12 | BIT13)) | ch1_config);
    AIREG(CH1_CTRL) = reg_tmp;
    cta_device_init(device, cpu_dai);
    return 0;
}

static int cta_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
                  struct snd_soc_dai *cpu_dai)
{
    int ret = 0;
    struct cta_master * master; //used in AIREG macro
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8750) || defined(CONFIG_CHEETAH_SND_PCM1_MP320)
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct cta_runtime_data *prtd = runtime->private_data;
#endif

    pr_debug("%s:%d name=%s cmd=%d id=%d\n", __func__, __LINE__, cpu_dai->name, cmd, cpu_dai->id);

    master =  snd_soc_dai_get_drvdata(cpu_dai);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
            AIREG(CH0_CTRL) |= BIT1;
            AIREG(CH1_CTRL) |= BIT1;
            AIREG(NOTIFY) |= 1<<(NOTIFY_TX_BIT(0));    // notify channel 1 is ready
            AIREG(NOTIFY) |= 1<<(NOTIFY_TX_BIT(1));    // notify channel 2 also prepare to play
        }
        else{    //CAPTURE
            /*--- following if statement is due to PCM1's codec CJC8988 would delay ADC signal while capture,
                  therefore, we have to wait for that.
              --- This statement need to adapt when eather I2S name configure or PCM1's codec changes ---*/
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8750) || defined(CONFIG_CHEETAH_SND_PCM1_MP320)
            if((!strcmp(rtd->cpu_dai->name, "cta-i2s.1"))&&(prtd->codec_init == 0)){ //delay only once prevent multiple delay cause by overrun
                prtd->codec_init = 1;
                switch(runtime->rate){
                    case 8000:
                    case 16000:
                    case 32000:
                        mdelay(1000);
                        break;
                    case 44100:
                    case 48000:
                        mdelay(700);
                        break;
                    case 96000:
                        mdelay(300);
                        break;
                    default:
                        mdelay(1000);
                        break;
                }
            }
#endif
            AIREG(CH0_CTRL) |= BIT2;
            AIREG(CH1_CTRL) |= BIT2;
            AIREG(NOTIFY) |= 1<<(NOTIFY_RX_BIT(0));
            AIREG(NOTIFY) |= 1<<(NOTIFY_RX_BIT(1));
        }
        break; 
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
            AIREG(CH0_CTRL) &= ~(BIT1);
            AIREG(CH1_CTRL) &= ~(BIT1);
        }
        else{    //CAPTURE
            AIREG(CH0_CTRL) &= ~(BIT2);
            AIREG(CH1_CTRL) &= ~(BIT2);
        }
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static void cta_i2s_shutdown(struct snd_pcm_substream *substream,
                struct snd_soc_dai *dai)
{

/* TODO: evaluate disable i2s/pcm clk would cause problem */

    struct cta_master *master;
    unsigned long flags;
    pr_debug("%s:%d name=%s\n", __func__, __LINE__, dai->name);
    master = snd_soc_dai_get_drvdata(dai);
    if(master->fmt == SND_SOC_DAIFMT_I2S){
//        atomic_dec(&i2s_clk_ref);
/*        if(atomic_read(&i2s_clk_ref) == 0){
              GPREG(0x81c/4) &= ~(1 << 6);
              pr_debug("close I2S clk\n");
          }*/
    }
    else{ //SND_SOC_DAIFMT_DSP_A, PCM in fact
//        atomic_dec(&pcm_clk_ref);
/*        if(atomic_read(&pcm_clk_ref) == 0){
              GPREG(0x81c/4) &= ~(1 << 7);
              pr_debug("close PCM clk\n");
          }
*/
    }
    atomic_dec(&i2s_clk_ref);

    spin_lock_irqsave(&(master->device.rat_lock), flags);
    if (atomic_read(&i2s_clk_ref) == 0) {
        master->device.rat_updated = 0;
        pr_debug("%s:%d updated=%d\n", __func__, __LINE__, master->device.rat_updated);
    }
    spin_unlock_irqrestore(&(master->device.rat_lock), flags);

    cta_i2s_wait();
}

#ifdef CONFIG_PM
static int cta_i2s_suspend(struct snd_soc_dai *dai)
{
    /* store registers */

    /* deactivate link */
    cta_i2s_wait();
    return 0;
}

static int cta_i2s_resume(struct snd_soc_dai *dai)
{
    cta_i2s_wait();

    return 0;
}

#else
#define cta_i2s_suspend NULL
#define cta_i2s_resume  NULL
#endif

#define CTA_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)
#undef CTA_I2S_RATES
#define CTA_I2S_RATES (SNDRV_PCM_RATE_8000 | \
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_32000| SNDRV_PCM_RATE_22050 |\
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

static const struct snd_soc_dai_ops cta_i2s_dai_ops = {
    .startup    = cta_i2s_startup,
    .shutdown   = cta_i2s_shutdown,
    .trigger    = cta_i2s_trigger,
    .hw_params  = cta_i2s_hw_params,
    .set_fmt    = cta_i2s_set_dai_fmt,
    .set_sysclk = cta_i2s_set_dai_sysclk,
};

static struct snd_soc_dai_driver cta_i2s_dai = {
    .suspend = cta_i2s_suspend,
    .resume = cta_i2s_resume,
    .playback = {
        .channels_min = 1,       //set channels_min 1 is due to mono wav and stereo wav need different machanism
        .channels_max = 2,
        .rates = CTA_I2S_RATES,
        .formats = (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE),},   
        /*fix .formats to SNDRV_PCM_FMTBIT_S16_BE from SNDRV_PCM_FMBIT_S16_LE, ALSA would deal the endian issue,
            notice that codec format also need to change*/
    .capture = {
        .channels_min = 1,
        .channels_max = 2,
        .rates = CTA_I2S_RATES,
        .formats = (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE),},
    .ops = &cta_i2s_dai_ops,
};

EXPORT_SYMBOL_GPL(cta_i2s_dai);

static int cta_i2s_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;
    struct resource *res;
    struct resource *ioarea;
    void __iomem *ioaddr;

    struct cta_master *master; 

    master = kzalloc(sizeof(struct cta_master), GFP_KERNEL);

    if (unlikely(!master)) {
        dev_err(dev, "Could not allocate master\n");
        ret = -ENOMEM;
        goto err_irq;
    }
    master->device.rat_updated = 0;
    spin_lock_init(&(master->device.rat_lock));

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(res < 0)) {
        dev_err(dev, "no memory specified\n");
        ret = -ENOENT;
        goto err_irq;
    }
    ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
    if (unlikely(!ioarea)) {
        dev_err(dev, "failed to reserve register area\n");
        ret = -ENXIO;
        goto err_irq;
    }
    ioaddr = ioremap_nocache(res->start, resource_size(res));
    pr_debug("ioarea %p ioaddr %p\n", ioarea, ioaddr);
    if (unlikely(!ioaddr)) {
        dev_err(dev, "failed to map registers\n");
        ret = -ENXIO;
        goto err_region;
    }

    platform_set_drvdata(pdev, &cta_i2s_dai);
    ret = snd_soc_register_dai(&pdev->dev, &cta_i2s_dai);

    if (unlikely(ret != 0)) {
        dev_err(dev, "failed to register dai\n");
        goto err_map;
    }

    master->res = res;
    master->ioarea = ioarea;
    master->ioaddr = ioaddr;
    master->pdev = pdev;

    dev_set_drvdata(dev, (void *)master);

    /* To enable PCM1's codec, need PINMUX switched, provide TMIICLK then enable PCM1 module */
    cta_init();
    /* disable ETH1 is due to PINMUX conflict */
    lock_kernel();
    GPREG(PINMUX) &= ~EN_ETH1_FNC;
#ifdef CONFIG_CHEETAH_SND_PCM0
    /* enable PCM/I2S AI0 function */
    GPREG(PINMUX) |= EN_PCM0_FNC;
#endif
#ifdef CONFIG_CHEETAH_SND_PCM1
    /* enable PCM/I2S AI1 function, and disable ETH0/MDIO_AUX */
    GPREG(PINMUX) &= ~(EN_ETH0_FNC | EN_MDIO_AUX_FNC);
    GPREG(PINMUX) |= EN_PCM1_FNC;
#endif
/* there is an problem that codec isn't operational unless we give MCLK in external module */
/* codec NAU8822 could initialize without giving MCLK, codec wm8988/wm8750 do not */
#ifdef CONFIG_CHEETAH_FPGA
        GPREG(TMIICLK) &= ~(7<<28);
        GPREG(TMIICLK) |= (4<<28);
#else
// GPREG(0x81c/4) |= (1 << 6); //enable I2S clk, enable PCM work well, too
// I2S_CLKUPDATE(I2S_FREQUENCY_SETTING, (0x20) | ((0xaa1) << 8));
#endif
    unlock_kernel();

// AIREG(CFG) = BIT16; //enable I2S module, not sure it's nessesary or not


    return ret;

err_map:
    iounmap(ioaddr);

err_region:
    release_mem_region(res->start, resource_size(res));

err_irq:
    if (master) {
        kfree(master);
        master = NULL;
    }
    return ret;
}

static int __devexit cta_i2s_remove(struct platform_device *pdev)
{
    struct device *dev;
    struct cta_master* master;

    dev=&pdev->dev;
    master = dev_get_drvdata(dev);
    if (likely(master)) {
        iounmap(master->ioaddr);
        release_mem_region(master->res->start, resource_size(master->res));
        kfree(master);
        master = NULL;
    }
    snd_soc_unregister_dai(&pdev->dev);
    return 0;
}

static struct platform_driver cta_i2s_driver = {
    .probe = cta_i2s_probe,
    .remove = __devexit_p(cta_i2s_remove),

    .driver = {
        .name = "cta-i2s",
        .owner = THIS_MODULE,
    },
};

static int __init cta_i2s_init(void)
{
    return platform_driver_register(&cta_i2s_driver);
}

static void __exit cta_i2s_exit(void)
{
    platform_driver_unregister(&cta_i2s_driver);
}

module_init(cta_i2s_init);
module_exit(cta_i2s_exit);

/* Module information */
MODULE_AUTHOR("Nady Chen");
MODULE_DESCRIPTION("I2S SoC Interface");
MODULE_LICENSE("GPL");
