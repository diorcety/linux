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

#include "tas5711.h"

#define CODEC_DEBUG printk

#define tas5711_RATES (SNDRV_PCM_RATE_8000 | \
		      SNDRV_PCM_RATE_11025 | \
		      SNDRV_PCM_RATE_16000 | \
		      SNDRV_PCM_RATE_22050 | \
		      SNDRV_PCM_RATE_32000 | \
		      SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000)

#define tas5711_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

/* Power-up register defaults */
static const u8 tas5711_regs[DDX_NUM_BYTE_REG] = {
	0x6c, 0x70, 0x00, 0xA0, 0x05, 0x40, 0x00, 0xFF,
	0x30, 0x30,	0xFF, 0x00, 0x00, 0x00, 0x91, 0x00,//0x0F
	0x02, 0xAC, 0x54, 0xAC,	0x54, 0x00, 0x00, 0x00,//0x17
	0x00, 0x30, 0x0F, 0x82, 0x02,
};

static const u8 TAS5711_subwoofer_table[2][21]={
	//0x5A   150Hz-lowpass
	{0x5A,
	 0x00,0x00,0x03,0x1D,
	 0x00,0x00,0x06,0x3A,
	 0x00,0x00,0x03,0x1D,
	 0x00,0xFC,0x72,0x05,
	 0x0F,0x83,0x81,0x85},
	//0x5B   150HZ-10dB
	{0x5B,
	 0x00,0x81,0x50,0x89,
	 0x0F,0x03,0x68,0x8C,
	 0x00,0x7B,0x6A,0x2F,
	 0x00,0xFC,0xA3,0x83,
	 0x0F,0x83,0x51,0x56}
};

/* codec private data */
struct tas5711_priv {
	struct snd_soc_codec *codec;
	struct tas5711_platform_data *pdata;

	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned mclk;
};

static const DECLARE_TLV_DB_SCALE(mvol_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -10300, 50, 1);


static const struct snd_kcontrol_new tas5711_snd_controls[] = {
	SOC_SINGLE_TLV("Master Volume", DDX_MASTER_VOLUME, 0, 0xff, 1, mvol_tlv),
	SOC_SINGLE_TLV("Ch1 Volume", DDX_CHANNEL1_VOL, 0, 0xff, 1, chvol_tlv),
	SOC_SINGLE_TLV("Ch2 Volume", DDX_CHANNEL2_VOL, 0, 0xff, 1, chvol_tlv),
	SOC_SINGLE_TLV("Ch3 Volume", DDX_CHANNEL3_VOL, 0, 0xff, 1, chvol_tlv),
	SOC_SINGLE("Ch1 Switch", DDX_SOFT_MUTE, 0, 1, 1),
	SOC_SINGLE("Ch2 Switch", DDX_SOFT_MUTE, 1, 1, 1),
	SOC_SINGLE("Ch3 Switch", DDX_SOFT_MUTE, 2, 1, 1),
};

static int tas5711_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5711_priv *tas5711 = snd_soc_codec_get_drvdata(codec);
	CODEC_DEBUG("~~~~%s\n", __func__);
	tas5711->mclk = freq;
	if(freq == 512* 48000)
		snd_soc_write(codec, DDX_CLOCK_CTL, 0x74);//0x74 = 512fs; 0x6c = 256fs
	else
		snd_soc_write(codec, DDX_CLOCK_CTL, 0x6c);//0x74 = 512fs; 0x6c = 256fs		
	return 0;
}

static int tas5711_set_dai_fmt(struct snd_soc_dai *codec_dai,
				  unsigned int fmt)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
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

static int tas5711_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_codec *codec = rtd->codec;
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

static int tas5711_set_bias_level(struct snd_soc_codec *codec,
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

static const struct snd_soc_dai_ops tas5711_dai_ops = {
	.hw_params	= tas5711_hw_params,
	.set_sysclk	= tas5711_set_dai_sysclk,
	.set_fmt	= tas5711_set_dai_fmt,
};

static struct snd_soc_dai_driver tas5711_dai = {
	.name = "tas5711",
	.playback = {
		.stream_name = "HIFI Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = tas5711_RATES,
		.formats = tas5711_FORMATS,
	},
	.ops = &tas5711_dai_ops,
};

static int tas5711_init(struct snd_soc_codec *codec)
{
	unsigned char burst_data[][5]= {
		{DDX_INPUT_MUX,0x00,0x01,0x77,0x72},
		{DDX_CH4_SOURCE_SELECT,0x00,0x00,0x42,0x03},
		{DDX_PWM_MUX,0x01,0x01,0x32,0x45},
	};


//	unsigned char ddx_0xF8[5] = {DDX_CH_DEV_ADDR_ENABLE,0xA5,0xA5,0xA5,0xA5};
//	unsigned char ddx_0xC9[5] = {0xc9,0x00,0x01,0x00,0xAB};
	unsigned char ddx_0xCA[9] = {0xca,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00};
//	unsigned char ddx_0xC9_1[5] = {0xc9,0x00,0x01,0x00,0xAc};
//	unsigned char ddx_0xCA_1[9] = {0xca,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char ddx_0x20[5] = {DDX_INPUT_MUX,0x00,0x01,0x77,0x72};
	unsigned char ddx_0x25[5] = {DDX_PWM_MUX,0x01,0x02,0x13,0x45};
	unsigned char ddx_0x29[21] = {DDX_CH1_BQ_0,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char ddx_0x5A[21] = {DDX_SUBCHANNEL_BQ_0,0x00,0x00,0x05,0x83,0x00,0x00,0x0B,0x06,0x00,0x00,0x05,0x83,0x00,0xFB,0x42,0xC1,0x0F,0x84,0xA7,0x33};
	unsigned char ddx_0x3A[9] = {DDX_DRC1_AE,0x00,0x7F,0xFF,0xB4,0x00,0x00,0x00,0x4B};
	unsigned char ddx_0x3D[9] = {DDX_DRC2_AE,0x00,0x7F,0xFF,0xB4,0x00,0x00,0x00,0x08};

	unsigned char ddx_buff[21] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	unsigned char ddx_0x40[5] = {DDX_DRC1_T,0xFC,0x83,0x10,0xD4};
	unsigned char ddx_0x41[5] = {DDX_DRC1_K,0x0F,0x83,0x33,0x34};
	unsigned char ddx_0x42[5] = {DDX_DRC1_O,0x00,0x08,0x42,0x10};

	unsigned char ddx_0x43[5] = {DDX_DRC2_T,0xFC,0x83,0x10,0xD4};
	unsigned char ddx_0x44[5] = {DDX_DRC2_K,0x0F,0x83,0x33,0x34};
	unsigned char ddx_0x45[5] = {DDX_DRC2_O,0x00,0x08,0x42,0x10};
	
	unsigned char ddx_0x52[13] = {DDX_CH_2_OUTPUT_MIXER,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char ddx_0x53[17] = {DDX_CH_1_INPUT_MIXER,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00};
	unsigned char ddx_0x56[5] = {DDX_OUTPUT_POST_SCALE,0x00,0x80,0x00,0x00};
	unsigned char ddx_0x57[5] = {DDX_OUTPUT_PRE_SCALE,0x00,0x20,0x00,0x00};
	printk("~~~~tas5711_init~~~~1\n");	
	#if 1//

	snd_soc_bulk_write_raw(codec, DDX_CH_DEV_ADDR_ENABLE, burst_data[0], 5);
//printk("~~~~tas5711_init~~~~11\n");	
	snd_soc_write(codec, DDX_CLOCK_CTL, 0x6c);//0x74 = 512fs; 0x6c = 256fs
//	printk("~~~~tas5711_init~~~~1_2\n");	

	snd_soc_write(codec, DDX_SYS_CTL_1, 0xa0);
//	printk("~~~~tas5711_init~~~~1_3\n");	
	snd_soc_write(codec, DDX_SERIAL_DATA_INTERFACE, 0x05);
//	printk("~~~~tas5711_init~~~~1_4\n");	

	snd_soc_write(codec, DDX_BKND_ERR, 0x02);

/*	snd_soc_bulk_write_raw(codec, DDX_CH_DEV_ADDR_ENABLE, ddx_0xF8[0], 5);
	snd_soc_bulk_write_raw(codec, 0xC9, ddx_0xC9[0], 5);
	snd_soc_bulk_write_raw(codec, 0xCA, ddx_0xCA[0], 9);
	snd_soc_bulk_write_raw(codec, 0xC9, ddx_0xC9_1[0], 5);
	snd_soc_bulk_write_raw(codec, 0xCA, ddx_0xCA_1[0], 9);*/

 //	snd_soc_write(codec, DDX_OSC_TRIM, 0x00);
 	snd_soc_write(codec, DDX_SOFT_MUTE, 0x3F);
	//printk("~~~~tas5711_init~~~~1_6\n");	
 	snd_soc_write(codec, DDX_CHANNEL3_VOL, 0x30);
	//printk("~~~~tas5711_init~~~~1_7\n");	
	
 	snd_soc_write(codec, DDX_CHANNEL2_VOL, 0x30);
	//printk("~~~~tas5711_init~~~~1_8\n");	
	
 	snd_soc_write(codec, DDX_CHANNEL1_VOL, 0x30);
	//printk("~~~~tas5711_init~~~~1_9\n");	
 /*	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_4, 0x54);
 	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_3, 0xAC);
 	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_2, 0x54);
 	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_1, 0xAC);
*/
	//printk("~~~~tas5711_init~~~~1_10\n");	

	snd_soc_bulk_write_raw(codec, DDX_INPUT_MUX, ddx_0x20, 5);
	//printk("~~~~tas5711_init~~~~2\n");	
	
 	snd_soc_write(codec, DDX_MODULATION_LIMIT, 0x02);
	
 	snd_soc_write(codec, DDX_RESV_0X0B, 0x00);
 	snd_soc_write(codec, DDX_MODULATION_LIMIT, 0x02);
	snd_soc_write(codec, DDX_BKND_ERR, 0x02);
	snd_soc_write(codec, DDX_PWM_SHUTDOWN_GROUP, 0x30);

	snd_soc_bulk_write_raw(codec, DDX_PWM_MUX, ddx_0x25, 5);
	ddx_buff[0] = DDX_BANKSWITCH_AND_EQCTL;
	snd_soc_bulk_write_raw(codec, DDX_BANKSWITCH_AND_EQCTL, ddx_buff, 5);
	ddx_buff[0] = 0;
//printk("~~~~tas5711_init~~~~3\n");	
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_0, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_1;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_1, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_2;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_2, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_3;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_3, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_4;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_4, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_5;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_5, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_6;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_6, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_7;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_7, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_8;
	snd_soc_bulk_write_raw(codec, DDX_CH1_BQ_8, ddx_0x29, 21);
//printk("~~~~tas5711_init~~~~4\n");	
	ddx_0x29[0]= DDX_CH2_BQ_0;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_0, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_1;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_1, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_2;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_2, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_3;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_3, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_4;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_4, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_5;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_5, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_6;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_6, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_7;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_7, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH2_BQ_8;
	snd_soc_bulk_write_raw(codec, DDX_CH2_BQ_8, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_0;
//	printk("~~~~tas5711_init~~~~5\n");	
	snd_soc_bulk_write_raw(codec, DDX_SUBCHANNEL_BQ_0, ddx_0x5A, 21);
	ddx_0x29[0]= DDX_CH1_BQ_0;
	snd_soc_bulk_write_raw(codec, DDX_SUBCHANNEL_BQ_1, ddx_0x29, 21);
	ddx_0x29[0]= DDX_CH1_BQ_0;
//	printk("~~~~tas5711_init~~~~5_1\n");	
	 	snd_soc_write(codec, DDX_VOLUME_CONFIG, 0x91);

//	printk("~~~~tas5711_init~~~~6\n");	
	snd_soc_bulk_write_raw(codec, DDX_DRC1_AE, ddx_0x3A, 9);
	ddx_0x3A[0]= DDX_DRC1_AA;
	snd_soc_bulk_write_raw(codec, DDX_DRC1_AA, ddx_0x3A, 9);
	ddx_0x3A[0]= DDX_DRC1_AD;
	snd_soc_bulk_write_raw(codec, DDX_DRC1_AD, ddx_0x3A, 9);
	ddx_0x3A[0]= DDX_DRC1_AE;
//		printk("~~~~tas5711_init~~~~7\n");	
	snd_soc_bulk_write_raw(codec, DDX_DRC1_T, ddx_0x40, 5);
	snd_soc_bulk_write_raw(codec, DDX_DRC1_K, ddx_0x41, 5);
	snd_soc_bulk_write_raw(codec, DDX_DRC1_O, ddx_0x42, 5);
//		printk("~~~~tas5711_init~~~~8\n");	
	ddx_buff[0] = DDX_DRC_CTL;
	snd_soc_bulk_write_raw(codec, DDX_DRC_CTL, ddx_buff, 5);
//			printk("~~~~tas5711_init~~~~9\n");	
	ddx_buff[0] = 0x39;
	snd_soc_bulk_write_raw(codec, 0x39, ddx_buff, 9);
	ddx_buff[0] = 0x00;
	snd_soc_bulk_write_raw(codec, DDX_DRC2_AE, ddx_0x3D, 9);
	ddx_0x3D[0] = DDX_DRC2_AA;
	snd_soc_bulk_write_raw(codec, DDX_DRC2_AA, ddx_0x3D, 9);
	ddx_0x3D[0] = DDX_DRC2_AD;
	snd_soc_bulk_write_raw(codec, DDX_DRC2_AD, ddx_0x3D, 9);
	ddx_0x3D[0] = DDX_DRC2_AE;
//			printk("~~~~tas5711_init~~~~10\n");	
	snd_soc_bulk_write_raw(codec, DDX_DRC2_T, ddx_0x43, 5);
	snd_soc_bulk_write_raw(codec, DDX_DRC2_T, ddx_0x44, 5);
	snd_soc_bulk_write_raw(codec, DDX_DRC2_T, ddx_0x45, 5);
//				printk("~~~~tas5711_init~~~~11\n");	
	ddx_buff[0] = DDX_DRC_CTL;
	snd_soc_bulk_write_raw(codec, DDX_DRC_CTL, ddx_buff, 5);
	ddx_buff[0] = 0;
//printk("~~~~tas5711_init~~~~12\n");	
	snd_soc_bulk_write_raw(codec, DDX_CH_2_OUTPUT_MIXER, ddx_0x52, 13);
	ddx_0xCA[0] = 0x60;
	snd_soc_bulk_write_raw(codec, DDX_CH_4_OUTPUT_MIXER, ddx_0xCA, 9);
	ddx_0xCA[0] = 0xca;
//printk("~~~~tas5711_init~~~~13\n");	
	
	snd_soc_bulk_write_raw(codec, DDX_CH_1_INPUT_MIXER, ddx_0x53, 17);
	ddx_0x53[0] = DDX_CH_2_INPUT_MIXER;
	snd_soc_bulk_write_raw(codec, DDX_CH_2_INPUT_MIXER, ddx_0x53, 17);
	ddx_0x53[0] = DDX_CH_1_INPUT_MIXER;
//printk("~~~~tas5711_init~~~~14\n");	

	snd_soc_bulk_write_raw(codec, DDX_OUTPUT_POST_SCALE, ddx_0x56, 5);
	snd_soc_bulk_write_raw(codec, DDX_OUTPUT_PRE_SCALE, ddx_0x57, 5);
//printk("~~~~tas5711_init~~~~15\n");	
	ddx_0x52[0] = DDX_CH_1_OUTPUT_MIXER;
	snd_soc_bulk_write_raw(codec, DDX_CH_1_OUTPUT_MIXER, ddx_0x52, 13);
	ddx_0x52[0] = DDX_CH_1_OUTPUT_MIXER;
	snd_soc_bulk_write_raw(codec, DDX_CH_3_INPUT_MIXER, ddx_0x52, 13);
	ddx_0x52[0] = DDX_CH_2_OUTPUT_MIXER;
	snd_soc_bulk_write_raw(codec, DDX_CH_2_OUTPUT_MIXER, ddx_0x52, 13);
//printk("~~~~tas5711_init~~~~16\n");	
 	snd_soc_write(codec, DDX_SYS_CTL_2, 0x00);
 	snd_soc_write(codec, DDX_SOFT_MUTE, 0x00);
 	snd_soc_write(codec, DDX_MASTER_VOLUME, 0x30);
	
	#else//org
	snd_soc_bulk_write_raw(codec, DDX_CH_DEV_ADDR_ENABLE, burst_data[0], 5);
	printk("~~~~tas5711_init~~~~17\n");	
	snd_soc_write(codec, DDX_CLOCK_CTL, 0x6c);//0x74 = 512fs; 0x6c = 256fs
	snd_soc_write(codec, DDX_SYS_CTL_1, 0xa0);
	snd_soc_write(codec, DDX_SERIAL_DATA_INTERFACE, 0x05);

/*	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_1, 0xac);
	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_2, 0x54);
	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_3, 0xac);
	snd_soc_write(codec, DDX_IC_DELAY_CHANNEL_4, 0x54);
*/	

	snd_soc_write(codec, DDX_BKND_ERR, 0x02);

	snd_soc_bulk_write_raw(codec, DDX_INPUT_MUX, burst_data[0], 5);
	snd_soc_bulk_write_raw(codec, DDX_CH4_SOURCE_SELECT, burst_data[1], 5);
	snd_soc_bulk_write_raw(codec, DDX_PWM_MUX, burst_data[2], 5);
	printk("~~~~tas5711_init~~~~18\n");	
	//subwoofer
	snd_soc_bulk_write_raw(codec, DDX_SUBCHANNEL_BQ_0, TAS5711_subwoofer_table[0], 21);
	snd_soc_bulk_write_raw(codec, DDX_SUBCHANNEL_BQ_1, TAS5711_subwoofer_table[1], 21);
	
	snd_soc_write(codec, DDX_VOLUME_CONFIG, 0xD1);
	snd_soc_write(codec, DDX_SYS_CTL_2, 0x84);
	snd_soc_write(codec, DDX_START_STOP_PERIOD, 0x95);
	snd_soc_write(codec, DDX_PWM_SHUTDOWN_GROUP, 0x30);
	snd_soc_write(codec, DDX_MODULATION_LIMIT, 0x02);
	//normal operation
	snd_soc_write(codec, DDX_MASTER_VOLUME, 0x00);
	snd_soc_write(codec, DDX_CHANNEL1_VOL, 0x30);
	snd_soc_write(codec, DDX_CHANNEL2_VOL, 0x30);
	snd_soc_write(codec, DDX_CHANNEL3_VOL, 0x30);
	snd_soc_write(codec, DDX_SOFT_MUTE, 0x00);
       #endif
	return 0;
}
static int tas5711_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct tas5711_priv *tas5711 = snd_soc_codec_get_drvdata(codec);

	printk("~~~~~~~~~~~~%s\n", __func__);
	//codec->control_data = tas5711->control_data;
	codec->control_type = tas5711->control_type;
    ret = snd_soc_codec_set_cache_io(codec, 8, 8, tas5711->control_type);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
        return ret;
    }
	
	snd_soc_write(codec, DDX_OSC_TRIM, 0x00);
	msleep(50);
	//TODO: set the DAP
	tas5711_init(codec);

	return 0;
}

static int tas5711_remove(struct snd_soc_codec *codec)
{
	printk("~~~~~~~~~~~~%s", __func__);
	return 0;
}

#ifdef CONFIG_PM
static int tas5711_suspend(struct snd_soc_codec *codec)
{
	tas5711_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int tas5711_resume(struct snd_soc_codec *codec)
{
	tas5711_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define tas5711_suspend NULL
#define tas5711_resume NULL
#endif

static const struct snd_soc_codec_driver tas5711_codec = {
	.probe =		tas5711_probe,
	.remove =		tas5711_remove,
	.suspend =		tas5711_suspend,
	.resume =		tas5711_resume,
	.reg_cache_size = DDX_NUM_BYTE_REG,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = tas5711_regs,
	//.volatile_register =	tas5711_reg_is_volatile,
	.set_bias_level = tas5711_set_bias_level,
	.controls =		tas5711_snd_controls,
	.num_controls =		ARRAY_SIZE(tas5711_snd_controls),
/*	.dapm_widgets =		tas5711_dapm_widgets,
	.num_dapm_widgets =	ARRAY_SIZE(tas5711_dapm_widgets),
	.dapm_routes =		tas5711_dapm_routes,
	.num_dapm_routes =	ARRAY_SIZE(tas5711_dapm_routes),
*/};

static __devinit int tas5711_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct tas5711_priv *tas5711;
	int ret;
	
	printk("~~~~~~tas5711_i2c_probe~~~~~~~~~\n");
	
	tas5711 = devm_kzalloc(&i2c->dev, sizeof(struct tas5711_priv),
			      GFP_KERNEL);
	if (!tas5711)
		return -ENOMEM;

	i2c_set_clientdata(i2c, tas5711);
	tas5711->control_type = SND_SOC_I2C;
	//tas5711->control_data = i2c;

	ret = snd_soc_register_codec(&i2c->dev, &tas5711_codec, 
			&tas5711_dai, 1);
	if (ret != 0){
		dev_err(&i2c->dev, "Failed to register codec (%d)\n", ret);
		//devm_kfree(&i2c->dev, STA381xx);
	}
	return ret;
}

static __devexit int tas5711_i2c_remove(struct i2c_client *client)
{
	//snd_soc_unregister_codec(&client->dev);
	devm_kfree(&client->dev, i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id tas5711_i2c_id[] = {
	{ "tas5711", 0 },
	{ }
};

static struct i2c_driver tas5711_i2c_driver = {
	.driver = {
		.name = "tas5711",
		.owner = THIS_MODULE,
	},
	.probe =    tas5711_i2c_probe,
	.remove =   __devexit_p(tas5711_i2c_remove),
	.id_table = tas5711_i2c_id,
};

static int __init TAS5711_init(void)
{
	return i2c_add_driver(&tas5711_i2c_driver);
}

static void __exit TAS5711_exit(void)
{
	i2c_del_driver(&tas5711_i2c_driver);
}
module_init(TAS5711_init);
module_exit(TAS5711_exit);
