/*
 * SoC audio for Cheetah
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>
#include "codec/wm8750.h"
#include "codec/wm8988.h"
#include "codec/nau8822.h"
#include "codec/mp320.h"
#include "codec/wm8737.h"
#include "codec/ad83586.h"
#include "codec/es8316.h"
#include "cta-pcm.h"
#include "cta-i2s.h"

//#define CTA_DEBUG
#ifdef CTA_DEBUG
#undef pr_debug
#define pr_debug(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#if defined(CONFIG_CHEETAH_SND_PCM0_WM8750) || defined(CONFIG_CHEETAH_SND_PCM1_WM8750)
#define CODEC_SYSCLK WM8750_SYSCLK
#define WM8750_I2C_ADDR_0 0x1b
#define WM8750_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_WM8988) || defined(CONFIG_CHEETAH_SND_PCM1_WM8988)
#define CODEC_SYSCLK WM8988_SYSCLK
#define WM8988_I2C_ADDR_0 0x1b
#define WM8988_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_NAU8822) || defined(CONFIG_CHEETAH_SND_PCM1_NAU8822)
#define CODEC_SYSCLK NAU8822_MCLK
#define NAU8822_I2C_ADDR_0 0x1b
#define NAU8822_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_TAS5711) || defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
#define CODEC_SYSCLK 0
#define TAS5711_I2C_ADDR_0 0x1b
#define TAS5711_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_ES8316) || defined(CONFIG_CHEETAH_SND_PCM1_ES8316)
#define CODEC_SYSCLK 0
#define ES8316_I2C_ADDR_0 0x10
#define ES8316_I2C_ADDR_1 0x11
#elif defined(CONFIG_CHEETAH_SND_PCM0_ES9023) || defined(CONFIG_CHEETAH_SND_PCM1_ES9023)
#define CODEC_SYSCLK 0
#define ES9023_I2C_ADDR_0 0x1b
#define ES9023_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_MP320) || defined(CONFIG_CHEETAH_SND_PCM1_MP320)
#define CODEC_SYSCLK MP320_SYSCLK
#define MP320_I2C_ADDR_0 0x1b
#define MP320_I2C_ADDR_1 0x1a
#elif defined(CONFIG_CHEETAH_SND_PCM0_WM8737) || defined(CONFIG_CHEETAH_SND_PCM1_WM8737)
#define CODEC_SYSCLK 0
#define WM8737_I2C_ADDR_0 0x1a
#define WM8737_I2C_ADDR_1 0x1b
#elif defined(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586)
#define CODEC_SYSCLK 0
#define WM8737_I2C_ADDR_0 0x1a
#define AD83586_I2C_ADDR_0 0x30
#else
#error "Error: Forget To Assign Codec!!"
#endif

spinlock_t i2c_lock __attribute__((weak));      /* use week symbol to solve multiple definition of i2c_lock among different machine drivers */

#if defined(CONFIG_CHEETAH_SND_PCM0)
static struct i2c_board_info cheetah_i2c_devs_0[] = {
    {
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8750)
        I2C_BOARD_INFO("wm8750", WM8750_I2C_ADDR_0),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8988)
        I2C_BOARD_INFO("wm8988", WM8988_I2C_ADDR_0),
#endif
/* NAU8822 only accept 0x1a as its I2C addr */
#if defined(CONFIG_CHEETAH_SND_PCM0_NAU8822)
        I2C_BOARD_INFO("nau8822", NAU8822_I2C_ADDR_1),
#endif
/* TAS5711 only accept 0x1a as its I2C addr */
#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711)
        I2C_BOARD_INFO("tas5711", TAS5711_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES8316)
        I2C_BOARD_INFO("es8316", ES8316_I2C_ADDR_0),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES9023)
        I2C_BOARD_INFO("es9023", ES9023_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_MP320)
        I2C_BOARD_INFO("mp320", MP320_I2C_ADDR_0),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737)
        I2C_BOARD_INFO("wm8737", WM8737_I2C_ADDR_0),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586)
        I2C_BOARD_INFO("wm8737", WM8737_I2C_ADDR_0),
    },
    {
        I2C_BOARD_INFO("ad83586", AD83586_I2C_ADDR_0),
#endif
    },
};
#endif

#if defined(CONFIG_CHEETAH_SND_PCM1)
static struct i2c_board_info cheetah_i2c_devs_1[] = {
    {
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8750)
        I2C_BOARD_INFO("wm8750", WM8750_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8988)
        I2C_BOARD_INFO("wm8988", WM8988_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_NAU8822)
        I2C_BOARD_INFO("nau8822", NAU8822_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
        I2C_BOARD_INFO("tas5711", TAS5711_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES8316)
        I2C_BOARD_INFO("es8316", ES8316_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES9023)
        I2C_BOARD_INFO("es9023", ES9023_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_MP320)
        I2C_BOARD_INFO("mp320", MP320_I2C_ADDR_1),
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8737)
        I2C_BOARD_INFO("wm8737", WM8737_I2C_ADDR_1),
#endif
    },
};
#endif

static int cta_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    unsigned int clk = 0;
    int ret = 0;

    switch (params_rate(params)) {
    case 11025:
    case 22050:
    case 44100:
        clk = 11289600;
        break;
    case 8000:
    case 16000:
    case 32000:
    case 48000:
    case 96000:
        clk = 12288000;
        break;
    }

    /* set codec DAI configuration */
    switch (params_rate(params)){
        case 8000:
        case 16000:
        /* following define decide sample rate 8000&16000 would use I2S either PCM */
#if 1
            ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
            if (ret < 0)
                return ret;
            ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
            if (ret < 0)
                return ret;
            dev_info(rtd->dev, "%s PCM %d KHz\n", (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "PLAYBACK" : "CAPTURE", params_rate(params)/1000);
            break;
#endif
        case 11025:
        case 22050:
        case 32000:
        case 44100:
        case 48000:
        case 96000:
            ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
            if (ret < 0)
                return ret;
            ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
            if (ret < 0)
                return ret;
            dev_info(rtd->dev, "%s I2S %d KHz\n", (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "PLAYBACK" : "CAPTURE", params_rate(params)/1000);
            break;
    }

    /* set the codec system clock for DAC and ADC */
    ret = snd_soc_dai_set_sysclk(codec_dai, CODEC_SYSCLK, clk,
        SND_SOC_CLOCK_IN);
    if (ret < 0)
        return ret;

    /* set the I2S system clock as input */
    ret = snd_soc_dai_set_sysclk(cpu_dai, CTA_I2S_SYSCLK, 0,
        SND_SOC_CLOCK_IN);
    if (ret < 0)
        return ret;

    return 0;
}

static struct snd_soc_ops cta_ops = {
    .hw_params = cta_hw_params,
};

#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711) || defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
#define MCLKFS_RATIO 256
static int cta_tas5711_init(struct snd_soc_dapm_context *dapm)
{
    struct snd_soc_codec *codec = dapm->codec;
    snd_soc_codec_set_sysclk(codec, 1, 0, 48000 * MCLKFS_RATIO, 0);
    return 0; 
}   
#endif

#ifndef CONFIG_CHEETAH_SND_DEDICATED_I2C
static hw_write_t cta_i2c_write = NULL;

static struct i2c_core mi2c;

static void mi2c_init(void)
{
    if (!(GPREG(PINMUX) & EN_SIP_FNC))
    {
        // SDR mode
        mi2c.sda = CONFIG_SND_I2C_SDA_SDR;
        mi2c.scl = CONFIG_SND_I2C_SCL_SDR;
    }
    else
    {
        // DDR mode
        mi2c.sda = CONFIG_SND_I2C_SDA_DDR;
        mi2c.scl = CONFIG_SND_I2C_SCL_DDR;
    }
}

static int i2c_grant_write(struct i2c_client *client,const char *buf ,int count)
{
    static int init_mi2c = 0;
    static int mi2c_ref = 0;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_algo_bit_data *bit_data = adap->algo_data;
    struct i2c_gpio_platform_data *pdata = bit_data->data;
    int ret;

    if (unlikely(!init_mi2c))
    {
        mi2c_init();
        // free default PIN assignment
        gpio_free(pdata->scl_pin);
        gpio_free(pdata->sda_pin);
        // replace correct PIN assignment
        pdata->scl_pin = mi2c.scl;
        pdata->sda_pin = mi2c.sda;
        // enhance driving for mi2c
        GPREG(DRIVER1) |= 0x03;
        
        ret = gpio_request(pdata->sda_pin, "sda");
        if (ret) {
            gpio_free(pdata->sda_pin);
            return -1;
        }
        ret = gpio_request(pdata->scl_pin, "scl");
        if (ret) {
            gpio_free(pdata->scl_pin);
            return -1;
        }
        gpio_direction_output(pdata->scl_pin, 1);
        gpio_direction_output(pdata->sda_pin, 1);
        init_mi2c = 1;
    }
    /*--- return this function value 2 for cheating ALSA we do transfer the buffer ---*/
//  return 2;

	//printk("[%s:%d] reg:%x val:0x%03x ,count:%x\n",__func__,__LINE__,buf[0]>>1, (buf[0]&0x1)<<8 | (0xff&buf[1]), count);
    spin_lock(&i2c_lock);
    mi2c_ref++;
    spin_unlock(&i2c_lock);
    ret = cta_i2c_write(client, buf, count);
    spin_lock(&i2c_lock);
    mi2c_ref--;
    if (mi2c_ref == 0) {
        if (!(GPREG(PINMUX) & EN_SIP_FNC)) {
            // SDR mode
            GPREG(PINMUX) &= ~EN_SWI2C_AUX_FNC;
            GPREG(PINMUX) |= EN_UART_AUX_FNC;
        }
        else {
            // DDR mode
            GPREG(PINMUX) &= ~EN_SWI2C_FNC;
            GPREG(PINMUX) |= EN_UART_FNC;
        }
    }
    spin_unlock(&i2c_lock);

    return ret;
}

static int cta_card_i2c_patch(struct i2c_client *client,const char *buf ,int count)
{
    struct snd_soc_codec *codec = (struct snd_soc_codec *)client;
    printk("%s:%d Replace I2C Write Function For Audio Codec!!\n", __func__, __LINE__);
    cta_i2c_write = (hw_write_t)buf;
    codec->hw_write = (hw_write_t)i2c_grant_write;
    return 0;
}
#endif

static int cta_card_probe(struct snd_soc_card *card)
{
    struct snd_soc_codec *codec;
    int i;

    for (i=0; i < card->num_links; i++)
    {
        if ((codec = snd_soc_get_codec(card, i)) == NULL) {
            printk("%s:%d error!! no match codec!!\n", __func__, __LINE__);
            return -1;
        }
#ifdef CONFIG_CHEETAH_SND_DEDICATED_I2C
    codec->hw_write = 0;
#else
    codec->hw_write = (hw_write_t)cta_card_i2c_patch;
#endif
    }

#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711) || defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
    //PDN   --- GPIO7
    //RESET --- GPIO8
    ret = gpio_request(7, "PDN");
    if (ret) {
        gpio_free(7);
        return -1;
    }
    ret = gpio_request(8, "RESET");
    if (ret) {
        gpio_free(8);
        return -1;
    }
    GPREG(PINMUX) &= ~(EN_ADC_OUT_FNC | EN_LED_FNC);
    gpio_direction_output(7, 1);
    gpio_direction_output(8, 1);
    gpio_set_value(7, 0);
    gpio_set_value(7, 1);
    udelay(1000);
    gpio_set_value(8, 0);
    gpio_set_value(8, 1);
    udelay(1000);
#endif

    return 0;
}

/*
 * Logic for a codec as connected on a Cheetah Device
 */

/* cheetah digital audio interface glue - connects codec <--> CPU */
#if defined(CONFIG_CHEETAH_SND_PCM0)
static struct snd_soc_dai_link cheetah_dai_0 []= {
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8750)
    {
    .name = "wm8750",
    .stream_name = "WM8750",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "wm8750-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8750.0-001b",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8988)
    {
    .name = "wm8988",
    .stream_name = "WM8988",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "wm8988-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8988.0-001b",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_NAU8822)
    {
    .name = "nau8822",
    .stream_name = "NAU8822",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "nau8822-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "nau8822.0-001a",  
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711)
    {
    .name = "tas5711",
    .stream_name = "TAS5711",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "tas5711",
    .platform_name = "cta-pcm-audio",
    .codec_name = "tas5711.0-001a",  
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES8316)
    {
    .name = "es8316",
    .stream_name = "ES8316",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "es8316-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "es8316.0-0010",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES9023)
    {
    .name = "es9023",
    .stream_name = "Hifi DAC ES9023",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "es9023",
    .platform_name = "cta-pcm-audio",
    .codec_name = "es9023.0-001a",  
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_MP320)
    {
    .name = "mp320",
    .stream_name = "MP320",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "mp320-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "mp320.0-001b",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737)
    {
    .name = "wm8737",
    .stream_name = "WM8737",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "wm8737",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8737.0-001a",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586)
    {
    .name = "wm8737",
    .stream_name = "WM8737",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "wm8737",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8737.0-001a",
    .ops = &cta_ops,
    },
    {
    .name = "ad83586",
    .stream_name = "AD83586",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "ad83586-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "ad83586.0-0030",  
    .ops = &cta_ops,
    },
#endif
};
#endif

#if defined(CONFIG_CHEETAH_SND_PCM1)
static struct snd_soc_dai_link cheetah_dai_1 []= {
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8750)
    {
    .name = "cjc8988",
    .stream_name = "CJC8988",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "wm8750-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8750.0-001a",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8988)
    {
    .name = "wm8988",
    .stream_name = "WM8988",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "wm8988-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8988.0-001a",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_NAU8822)
    {
    .name = "nau8822",
    .stream_name = "NAU8822",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "nau8822-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "nau8822.0-001a",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
    {
    .name = "tas5711",
    .stream_name = "TAS5711",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "tas5711",
    .platform_name = "cta-pcm-audio",
    .codec_name = "tas5711.0-001a",  
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES8316)
    {
    .name = "es8316",
    .stream_name = "ES8316",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "es8316-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "es8316.0-0011",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES9023)
    {
    .name = "es9023",
    .stream_name = "Hifi DAC ES9023",
    .cpu_dai_name = "cta-i2s.0",
    .codec_dai_name = "es9023",
    .platform_name = "cta-pcm-audio",
    .codec_name = "es9023.0-001a",  
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_MP320)
    {
    .name = "mp320",
    .stream_name = "MP320",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "mp320-hifi",
    .platform_name = "cta-pcm-audio",
    .codec_name = "mp320.0-001a",
    .ops = &cta_ops,
    },
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8737)
    {
    .name = "wm8737",
    .stream_name = "WM8737",
    .cpu_dai_name = "cta-i2s.1",
    .codec_dai_name = "wm8737",
    .platform_name = "cta-pcm-audio",
    .codec_name = "wm8737.0-001b",
    .ops = &cta_ops,
    },
#endif
};
#endif

struct snd_soc_aux_dev cheetah_aux_dev[] = {
#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711) || defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
    {
        .name = "tas5711",
        .codec_name = "tas5711.0-001a",
        .init = cta_tas5711_init,
    },
#endif
};

struct snd_soc_codec_conf cheetah_codec_conf[] = {
#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711) || defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
    {
        .dev_name = "tas5711.0-001a",
        .name_prefix = "AMP",
    },
#endif
};

/* cheetah audio machine driver */
#if defined(CONFIG_CHEETAH_SND_PCM0)
static struct snd_soc_card snd_soc_cheetah_0 = {
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8750)
    .name = "cta-pcm0-wm8750",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8988)
    .name = "cta-pcm0-wm8988",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_NAU8822)
    .name = "cta-pcm0-nau8822",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_TAS5711)
    .name = "cta-pcm0-tas5711",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
//  .aux_dev = cheetah_aux_dev,
//  .num_aux_devs = ARRAY_SIZE(cheetah_aux_dev),
//  .codec_conf = cheetah_codec_conf,
//  .num_configs = ARRAY_SIZE(cheetah_codec_conf),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES8316)
    .name = "cta-pcm0-es8316",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_ES9023)
    .name = "cta-pcm0-es9023",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_MP320)
    .name = "cta-pcm0-mp320",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737)
    .name = "cta-pcm0-wm8737",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586)
    .name = "cta-pcm0-wm8737-ad83586",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_0,
    .num_links = ARRAY_SIZE(cheetah_dai_0),
    .probe = cta_card_probe,
#endif
};

static struct platform_device *cheetah_snd_device_0;
#endif

#if defined(CONFIG_CHEETAH_SND_PCM1)
static struct snd_soc_card snd_soc_cheetah_1 = {
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8750)
    .name = "cta-pcm1-wm8750",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8988)
    .name = "cta-pcm1-wm8988",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_NAU8822)
    .name = "cta-pcm1-nau8822",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_TAS5711)
    .name = "cta-pcm1-tas5711",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
//  .aux_dev = cheetah_aux_dev,
//  .num_aux_devs = ARRAY_SIZE(cheetah_aux_dev),
//  .codec_conf = cheetah_codec_conf,
//  .num_configs = ARRAY_SIZE(cheetah_codec_conf),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES8316)
    .name = "cta-pcm1-es8316",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_ES9023)
    .name = "cta-pcm1-es9023",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_MP320)
    .name = "cta-pcm1-mp320",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1_WM8737)
    .name = "cta-pcm1-wm8737",
    .owner = THIS_MODULE,
    .dai_link = cheetah_dai_1,
    .num_links = ARRAY_SIZE(cheetah_dai_1),
    .probe = cta_card_probe,
#endif
};

static struct platform_device *cheetah_snd_device_1;
#endif

static int __init cheetah_snd_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_client *client;
    int ret = 0;

    (void) cta_ops;	

    spin_lock_init(&i2c_lock);
    adapter = i2c_get_adapter(0);
    if (!adapter)
        return -ENODEV;

#ifdef CONFIG_CHEETAH_SND_PCM0
    client = i2c_new_device(adapter, cheetah_i2c_devs_0);
    i2c_put_adapter(adapter);
    if (!client)
        return -ENODEV;

#if defined(CONFIG_CHEETAH_SND_PCM0_WM8737_AD83586)
    client = i2c_new_device(adapter, &cheetah_i2c_devs_0[1]);
    i2c_put_adapter(adapter);
    if (!client)
        return -ENODEV;
#endif

    cheetah_snd_device_0 = platform_device_alloc("soc-audio", 0);
    if (!cheetah_snd_device_0)
        return -ENOMEM;

    platform_set_drvdata(cheetah_snd_device_0, &snd_soc_cheetah_0);
    ret = platform_device_add(cheetah_snd_device_0);
    if(ret)
        platform_device_put(cheetah_snd_device_0);
#endif
#ifdef CONFIG_CHEETAH_SND_PCM1
    client = i2c_new_device(adapter, cheetah_i2c_devs_1);
    i2c_put_adapter(adapter);
    if (!client)
        return -ENODEV;

    cheetah_snd_device_1 = platform_device_alloc("soc-audio", 1);
    if (!cheetah_snd_device_1)
        return -ENOMEM;

    platform_set_drvdata(cheetah_snd_device_1, &snd_soc_cheetah_1);
    ret = platform_device_add(cheetah_snd_device_1);
    if (ret)
        platform_device_put(cheetah_snd_device_1);
#endif
    return ret;
}

static void __exit cheetah_snd_exit(void)
{
#ifdef CONFIG_CHEETAH_SND_PCM0
    platform_device_unregister(cheetah_snd_device_0);
#endif
#ifdef CONFIG_CHEETAH_SND_PCM1
    platform_device_unregister(cheetah_snd_device_1);
#endif
}

module_init(cheetah_snd_init);
module_exit(cheetah_snd_exit);

MODULE_AUTHOR("Nady Chen");
MODULE_DESCRIPTION("ALSA SoC Cheetah");
MODULE_LICENSE("GPL");
