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

#define CODEC_DEBUG

#define es9023_RATES ( SNDRV_PCM_RATE_32000 | \
		      SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000 | \
			  SNDRV_PCM_RATE_96000)

#define es9023_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

/* codec private data */
struct es9023_priv {
	struct snd_soc_codec *codec;
	struct es9023_platform_data *pdata;

	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned mclk;
};

static const DECLARE_TLV_DB_SCALE(mvol_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -10300, 50, 1);

static int es9023_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	
	CODEC_DEBUG("~~~~%s\n", __func__);

	return 0;
}

static int es9023_set_dai_fmt(struct snd_soc_dai *codec_dai,
				  unsigned int fmt)
{
	CODEC_DEBUG("~~~~%s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int es9023_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	unsigned int rate;
	
	CODEC_DEBUG("~~~~%s\n", __func__);

	rate = params_rate(params);
	pr_debug("rate: %u\n", rate);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
		pr_debug("24bit\n");
		/* fall through */
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		pr_debug("20bit\n");

		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		pr_debug("16bit\n");

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int es9023_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	pr_debug("%s,level = %d\n",__func__,level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* Full power on */

		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
		}

		/* Power up to mute */
		/* FIXME */
		break;

	case SND_SOC_BIAS_OFF:
		/* The chip runs through the power down sequence for us. */
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static const struct snd_soc_dai_ops es9023_dai_ops = {
	.hw_params	= es9023_hw_params,
	.set_sysclk	= es9023_set_dai_sysclk,
	.set_fmt	= es9023_set_dai_fmt,
};

static struct snd_soc_dai_driver es9023_dai_driver = {
	.name = "es9023",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = es9023_RATES,
		.formats = es9023_FORMATS,
	},
	.ops = &es9023_dai_ops,
};

static int es9023_probe(struct snd_soc_codec *codec)
{
	CODEC_DEBUG("Codec initialize\n");
	return 0;
}

static int es9023_remove(struct snd_soc_codec *codec)
{
	CODEC_DEBUG("~~~~~~~~~~~~%s", __func__);
	return 0;
}

static const struct snd_soc_codec_driver soc_codec_dev_es9023 = {
	.probe =		es9023_probe,
	.remove =		es9023_remove
};

static __devinit int es9023_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct es9023_priv *es9023;
	int ret;
	
	es9023 = devm_kzalloc(&i2c->dev, sizeof(struct es9023_priv),
			      GFP_KERNEL);
	if (!es9023)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es9023);
	es9023->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_es9023, 
			&es9023_dai_driver, 1);
	if (ret != 0){
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);
	}
	return ret;
}

static __devexit int es9023_i2c_remove(struct i2c_client *client)
{
	//snd_soc_unregister_codec(&client->dev);
	devm_kfree(&client->dev, i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id es9023_i2c_id[] = {
	{ "es9023", 0 },
	{ }
};

static struct i2c_driver es9023_i2c_driver = {
	.driver = {
		.name = "es9023",
		.owner = THIS_MODULE,
	},
	.probe =    es9023_i2c_probe,
	.remove =   __devexit_p(es9023_i2c_remove),
	.id_table = es9023_i2c_id,
};

static int __init ES9023_init(void)
{
	return i2c_add_driver(&es9023_i2c_driver);
}

static void __exit ES9023_exit(void)
{
	i2c_del_driver(&es9023_i2c_driver);
}
module_init(ES9023_init);
module_exit(ES9023_exit);
