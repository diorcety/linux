#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "ad83586.h"

#define CODEC_DEBUG printk

#define AD83586_PLL_BYPASSED

#define ad83586_RATES (SNDRV_PCM_RATE_32000 | \
		      SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000| \
		      SNDRV_PCM_RATE_64000| \
		      SNDRV_PCM_RATE_88200| \
		      SNDRV_PCM_RATE_96000| \
		      SNDRV_PCM_RATE_176400| \
		      SNDRV_PCM_RATE_192000)

#define ad83586_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
	 SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

static const u8 ad83586_reg[] = {
	0x00, 0x04, 0x00, 0x14, 0x14, 0x14, 0x14, 0x10,//0x00
	0x10, 0x02, 0x90, 0x02, 0x02, 0x00, 0x6a, 0x00,//0x08
	0x00, 0x22, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00,//0x10
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//0x18
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//0x20
	0x00, 0x00, 0x6d, 0x3f, 0x00, 0x20, 		   //0x28-0x2d
};

/* codec private data */
struct ad83586_priv {
	struct snd_soc_codec *codec;
	struct ad83586_platform_data *pdata;

	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned mclk;
};

static const DECLARE_TLV_DB_SCALE(mvol_tlv, -10300, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -10300, 50, 1);

struct _coeff_div {
	u32 mclk;
	u32 fs;					// Sampleling Frequency, FS
	u16 ratio;				// MCLK/Fs
	u8 reg_bclk_sel:1;
	u8 reg_fs:2;
	u8 reg_pmf:4;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {

	/* 32k */
	{12288000, 32000,  384, 0x1, 0x0, 0x5},

	/* 44.1k */
	{11289600, 44100,  256, 0x0, 0x0, 0x4},

	/* 48k */
	{12288000, 48000,  256, 0x0, 0x0, 0x4},

	/* 96k */
	{12288000, 96000,  128, 0x1, 0x1, 0x2},

};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].fs == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

static const struct snd_kcontrol_new ad83586_snd_controls[] = {
	// default MPD ALSA mixer volume control name "PCM"
	SOC_SINGLE_TLV("PCM Volume", AD83586_MASTER_VOL, 0, 0xe6, 1, mvol_tlv),

	SOC_SINGLE_TLV("Ch1 Volume", AD83586_CHANNEL1_VOL, 0, 0xe6, 1, chvol_tlv),
	SOC_SINGLE_TLV("Ch2 Volume", AD83586_CHANNEL2_VOL, 0, 0xe6, 1, chvol_tlv),
	SOC_SINGLE_TLV("Ch3 Volume", AD83586_CHANNEL3_VOL, 0, 0xe6, 1, chvol_tlv),
};

static int ad83586_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ad83586_priv *wm83586 = snd_soc_codec_get_drvdata(codec);

	wm83586->mclk = freq;

	return 0;
}

static int ad83586_set_dai_fmt(struct snd_soc_dai *codec_dai,
				  unsigned int fmt)
{
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
	default:
		return -EINVAL;
	}

/*
	CODEC_DEBUG(
			"interface format %x\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
*/

	return 0;
}

static int ad83586_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ad83586_priv *ad83586 = snd_soc_codec_get_drvdata(codec);
	int coeff;

	coeff = get_coeff(ad83586->mclk, params_rate(params));
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), ad83586->mclk);
		return coeff;
	}

#ifdef AD83586_PLL_BYPASSED
	if (coeff_div[coeff].ratio == 1024 ||
		coeff_div[coeff].ratio == 512 ||
		coeff_div[coeff].ratio == 256)
		;
	else {
		dev_err(codec->dev,
			"Unable to configure ratio %dx\n" \
			"When PLL is bypassed, AD83586B only supports 1024x, 512x and 256x MCLK/Fs ratio\n",
			coeff_div[coeff].ratio);
		return -EINVAL;
	}
#endif

	if (coeff >= 0)
		snd_soc_update_bits(codec, AD83586_STATE_CTRL2, 0x007f, 
			(coeff_div[coeff].reg_bclk_sel << 6) | 
			(coeff_div[coeff].reg_fs << 4) | 
			(coeff_div[coeff].reg_pmf));

/*
	CODEC_DEBUG(
			"............... configure sample rate %dHz with %dHz MCLK -- coeff %x\n",
			params_rate(params), ad83586->mclk, coeff);
*/

	return 0;
}

static const struct snd_soc_dai_ops ad83586_dai_ops = {
	.hw_params	= ad83586_hw_params,
	.set_sysclk	= ad83586_set_dai_sysclk,
	.set_fmt	= ad83586_set_dai_fmt,
};

static struct snd_soc_dai_driver ad83586_dai = {
	.name = "ad83586-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ad83586_RATES,
		.formats = ad83586_FORMATS,
	},
	.ops = &ad83586_dai_ops,
};

static int ad83586_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	codec->dapm.bias_level = level;
	return 0;
}

static int ad83586_suspend(struct snd_soc_codec *codec)
{
	ad83586_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int ad83586_resume(struct snd_soc_codec *codec)
{
	ad83586_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int ad83586_init(struct snd_soc_codec *codec)
{
//	snd_soc_write(codec, 0x02, 0x0f);//mute

	snd_soc_write(codec, 0x03, 0x14);//Master Vol default=2dB

	snd_soc_write(codec, 0x04, 0x14);//CH1 Vol default=2dB

	snd_soc_write(codec, 0x05, 0x14);//CH2 Vol default=2dB

	snd_soc_write(codec, 0x06, 0x14);//CH3 Vol default=2dB

#if 0

	snd_soc_write(codec, 0x07, 0x10);//Bass_tone_control 12dB~-12dB 360Hz 0dB

	snd_soc_write(codec, 0x08, 0x15);//Treble_tone_control 12dB~-12dB 7KHz -5dB

	snd_soc_write(codec, 0x09, 0x02);//Bass_management_crossover_frequency 0x01=120Hz

	snd_soc_write(codec, 0x0a, 0x98);//State_Control_4(surround_off+Bass_treble_off+EQ_CH1toCH2)

	snd_soc_write(codec, 0x0b, 0x02);//CH1 DRC & Power clipping enable RMS mode+HPF-off

	snd_soc_write(codec, 0x0c, 0x02);//CH2 DRC & Power clipping enable RMS mode+HPF-off

	snd_soc_write(codec, 0x0d, 0x02);//CH3 DRC on & Power clipping enable RMS mode+LPF-off

	snd_soc_write(codec, 0x11, 0x20);//State_Control_4(reset_off+MCLK_on+power_saving_off+PWM_Q_mode)

	snd_soc_write(codec, 0x12, 0x81);//PVDD_UVP default=0x81=off+9.7V	0x01=on+9.7V

	snd_soc_write(codec, 0x02, 0x00);//unmute

#endif
	return 0;
}

static int ad83586_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct ad83586_priv *ad83586 = snd_soc_codec_get_drvdata(codec);

    codec->control_type = ad83586->control_type;

    ret = snd_soc_codec_set_cache_io(codec, 8, 8, ad83586->control_type);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
        return ret;
    }

	snd_soc_write(codec, AD83586_RESET, 0x00);	//RESET
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		return ret;
	}

	ad83586_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	msleep(200);

	ad83586_init(codec);

	return 0;
}

static int ad83586_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static const struct snd_soc_codec_driver ad83586_codec = {
	.probe =			ad83586_probe,
	.remove =			ad83586_remove,
	.suspend = 			ad83586_suspend,
	.resume = 			ad83586_resume,
	.set_bias_level = 	ad83586_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ad83586_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ad83586_reg,

	.controls =			ad83586_snd_controls,
	.num_controls =		ARRAY_SIZE(ad83586_snd_controls),
//	.dapm_widgets =		ad83586_dapm_widgets,
//	.num_dapm_widgets = ARRAY_SIZE(ad83586_dapm_widgets),
};

static __devinit int ad83586_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct ad83586_priv *ad83586;
	int ret;
	
	ad83586 = kzalloc(sizeof(struct ad83586_priv), GFP_KERNEL);
	if (!ad83586)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ad83586);
	ad83586->control_type = SND_SOC_I2C;
	//ad83586->control_data = i2c;

	ret = snd_soc_register_codec(&i2c->dev, &ad83586_codec, &ad83586_dai, 1);
	if (ret != 0)
		kfree(ad83586);

	return ret;
}

static __devexit int ad83586_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	devm_kfree(&client->dev, i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ad83586_i2c_id[] = {
	{ "ad83586", 0 },
	{ }
};

static struct i2c_driver ad83586_i2c_driver = {
	.driver = {
		.name = "ad83586",
		.owner = THIS_MODULE,
	},
	.probe =    ad83586_i2c_probe,
	.remove =   __devexit_p(ad83586_i2c_remove),
	.id_table = ad83586_i2c_id,
};

static int __init AD83586_init(void)
{
	return i2c_add_driver(&ad83586_i2c_driver);
}

static void __exit AD83586_exit(void)
{
	i2c_del_driver(&ad83586_i2c_driver);
}


module_init(AD83586_init);
module_exit(AD83586_exit);
