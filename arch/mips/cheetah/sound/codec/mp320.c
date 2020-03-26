/*
 * mp320.c -- MP320 ALSA SoC audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 *
 * Based on WM8750.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
//#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

#include "mp320.h"

//#define mp_debug
#ifdef mp_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#endif


/*
 * mp320 register cache
 * We can't read the MP320 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 mp320_reg[] = {
	0x0097, 0x0097, 0x0079, 0x0079,  /*  0 */
	0x0000, 0x0008, 0x0000, 0x000a,  /*  4 */
	0x0000, 0x0000, 0x00ff, 0x00ff,  /*  8 */
	0x000f, 0x000f, 0x0000, 0x0000,  /* 12 */
	0x0000, 0x007b, 0x0000, 0x0032,  /* 16 */
	0x0000, 0x00c3, 0x00c3, 0x00c0,  /* 20 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 24 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 28 */
	0x0000, 0x0000, 0x0050, 0x0050,  /* 32 */
	0x0050, 0x0050, 0x0050, 0x0050,  /* 36 */
	0x0079, 0x0079, 0x0079,          /* 40 */
};

/* codec private data */
struct mp320_priv {
	unsigned int sysclk;
	enum snd_soc_control_type control_type;
};

#define mp320_reset(c)	snd_soc_write(c, MP320_RESET, 0)

/*
 * MP320 Controls
 */
static const char *mp320_bass[] = {"Linear Control", "Adaptive Boost"};
static const char *mp320_bass_filter[] = { "130Hz @ 48kHz", "200Hz @ 48kHz" };
static const char *mp320_treble[] = {"8kHz", "4kHz"};
static const char *mp320_3d_lc[] = {"200Hz", "500Hz"};
static const char *mp320_3d_uc[] = {"2.2kHz", "1.5kHz"};
static const char *mp320_3d_func[] = {"Capture", "Playback"};
static const char *mp320_alc_func[] = {"Off", "Right", "Left", "Stereo"};
static const char *mp320_ng_type[] = {"Constant PGA Gain",
	"Mute ADC Output"};
static const char *mp320_line_mux[] = {"Line 1", "Line 2", "Line 3", "PGA",
	"Differential"};
static const char *mp320_pga_sel[] = {"Line 1", "Line 2", "Line 3",
	"Differential"};
static const char *mp320_out3[] = {"VREF", "ROUT1 + Vol", "MonoOut",
	"ROUT1"};
static const char *mp320_diff_sel[] = {"Line 1", "Line 2"};
static const char *mp320_adcpol[] = {"Normal", "L Invert", "R Invert",
	"L + R Invert"};
static const char *mp320_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *mp320_mono_mux[] = {"Stereo", "Mono (Left)",
	"Mono (Right)", "Digital Mono"};

static const struct soc_enum mp320_enum[] = {
SOC_ENUM_SINGLE(MP320_BASS, 7, 2, mp320_bass),
SOC_ENUM_SINGLE(MP320_BASS, 6, 2, mp320_bass_filter),
SOC_ENUM_SINGLE(MP320_TREBLE, 6, 2, mp320_treble),
SOC_ENUM_SINGLE(MP320_3D, 5, 2, mp320_3d_lc),
SOC_ENUM_SINGLE(MP320_3D, 6, 2, mp320_3d_uc),
SOC_ENUM_SINGLE(MP320_3D, 7, 2, mp320_3d_func),
SOC_ENUM_SINGLE(MP320_ALC1, 7, 4, mp320_alc_func),
SOC_ENUM_SINGLE(MP320_NGATE, 1, 2, mp320_ng_type),
SOC_ENUM_SINGLE(MP320_LOUTM1, 0, 5, mp320_line_mux),
SOC_ENUM_SINGLE(MP320_ROUTM1, 0, 5, mp320_line_mux),
SOC_ENUM_SINGLE(MP320_LADCIN, 6, 4, mp320_pga_sel), /* 10 */
SOC_ENUM_SINGLE(MP320_RADCIN, 6, 4, mp320_pga_sel),
SOC_ENUM_SINGLE(MP320_ADCTL2, 7, 4, mp320_out3),
SOC_ENUM_SINGLE(MP320_ADCIN, 8, 2, mp320_diff_sel),
SOC_ENUM_SINGLE(MP320_ADCDAC, 5, 4, mp320_adcpol),
SOC_ENUM_SINGLE(MP320_ADCDAC, 1, 4, mp320_deemph),
SOC_ENUM_SINGLE(MP320_ADCIN, 6, 4, mp320_mono_mux), /* 16 */

};

static const struct snd_kcontrol_new mp320_snd_controls[] = {

SOC_DOUBLE_R("Capture Volume", MP320_LINVOL, MP320_RINVOL, 0, 63, 0),
SOC_DOUBLE_R("Capture ZC Switch", MP320_LINVOL, MP320_RINVOL, 6, 1, 0),
SOC_DOUBLE_R("Capture Switch", MP320_LINVOL, MP320_RINVOL, 7, 1, 1),

SOC_DOUBLE_R("Headphone Playback ZC Switch", MP320_LOUT1V,
	MP320_ROUT1V, 7, 1, 0),
SOC_DOUBLE_R("Speaker Playback ZC Switch", MP320_LOUT2V,
	MP320_ROUT2V, 7, 1, 0),

SOC_ENUM("Playback De-emphasis", mp320_enum[15]),

SOC_ENUM("Capture Polarity", mp320_enum[14]),
SOC_SINGLE("Playback 6dB Attenuate", MP320_ADCDAC, 7, 1, 0),
SOC_SINGLE("Capture 6dB Attenuate", MP320_ADCDAC, 8, 1, 0),

SOC_DOUBLE_R("PCM Volume", MP320_LDAC, MP320_RDAC, 0, 255, 0),

SOC_ENUM("Bass Boost", mp320_enum[0]),
SOC_ENUM("Bass Filter", mp320_enum[1]),
SOC_SINGLE("Bass Volume", MP320_BASS, 0, 15, 1),

SOC_SINGLE("Treble Volume", MP320_TREBLE, 0, 15, 1),
SOC_ENUM("Treble Cut-off", mp320_enum[2]),

SOC_SINGLE("3D Switch", MP320_3D, 0, 1, 0),
SOC_SINGLE("3D Volume", MP320_3D, 1, 15, 0),
SOC_ENUM("3D Lower Cut-off", mp320_enum[3]),
SOC_ENUM("3D Upper Cut-off", mp320_enum[4]),
SOC_ENUM("3D Mode", mp320_enum[5]),

SOC_SINGLE("ALC Capture Target Volume", MP320_ALC1, 0, 7, 0),
SOC_SINGLE("ALC Capture Max Volume", MP320_ALC1, 4, 7, 0),
SOC_ENUM("ALC Capture Function", mp320_enum[6]),
SOC_SINGLE("ALC Capture ZC Switch", MP320_ALC2, 7, 1, 0),
SOC_SINGLE("ALC Capture Hold Time", MP320_ALC2, 0, 15, 0),
SOC_SINGLE("ALC Capture Decay Time", MP320_ALC3, 4, 15, 0),
SOC_SINGLE("ALC Capture Attack Time", MP320_ALC3, 0, 15, 0),
SOC_SINGLE("ALC Capture NG Threshold", MP320_NGATE, 3, 31, 0),
SOC_ENUM("ALC Capture NG Type", mp320_enum[7]),
SOC_SINGLE("ALC Capture NG Switch", MP320_NGATE, 0, 1, 0),

SOC_SINGLE("Left ADC Capture Volume", MP320_LADC, 0, 255, 0),
SOC_SINGLE("Right ADC Capture Volume", MP320_RADC, 0, 255, 0),

SOC_SINGLE("ZC Timeout Switch", MP320_ADCTL1, 0, 1, 0),
SOC_SINGLE("Playback Invert Switch", MP320_ADCTL1, 1, 1, 0),

SOC_SINGLE("Right Speaker Playback Invert Switch", MP320_ADCTL2, 4, 1, 0),

/* Unimplemented */
/* ADCDAC Bit 0 - ADCHPD */
/* ADCDAC Bit 4 - HPOR */
/* ADCTL1 Bit 2,3 - DATSEL */
/* ADCTL1 Bit 4,5 - DMONOMIX */
/* ADCTL1 Bit 6,7 - VSEL */
/* ADCTL2 Bit 2 - LRCM */
/* ADCTL2 Bit 3 - TRI */
/* ADCTL3 Bit 5 - HPFLREN */
/* ADCTL3 Bit 6 - VROI */
/* ADCTL3 Bit 7,8 - ADCLRM */
/* ADCIN Bit 4 - LDCM */
/* ADCIN Bit 5 - RDCM */

SOC_DOUBLE_R("Mic Boost", MP320_LADCIN, MP320_RADCIN, 4, 3, 0),

SOC_DOUBLE_R("Bypass Left Playback Volume", MP320_LOUTM1,
	MP320_LOUTM2, 4, 7, 1),
SOC_DOUBLE_R("Bypass Right Playback Volume", MP320_ROUTM1,
	MP320_ROUTM2, 4, 7, 1),
SOC_DOUBLE_R("Bypass Mono Playback Volume", MP320_MOUTM1,
	MP320_MOUTM2, 4, 7, 1),

SOC_SINGLE("Mono Playback ZC Switch", MP320_MOUTV, 7, 1, 0),

SOC_DOUBLE_R("Headphone Playback Volume", MP320_LOUT1V, MP320_ROUT1V,
	0, 127, 0),
SOC_DOUBLE_R("Speaker Playback Volume", MP320_LOUT2V, MP320_ROUT2V,
	0, 127, 0),

SOC_SINGLE("Mono Playback Volume", MP320_MOUTV, 0, 127, 0),

};

/*
 * DAPM Controls
 */

/* Left Mixer */
static const struct snd_kcontrol_new mp320_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Playback Switch", MP320_LOUTM1, 8, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", MP320_LOUTM1, 7, 1, 0),
SOC_DAPM_SINGLE("Right Playback Switch", MP320_LOUTM2, 8, 1, 0),
SOC_DAPM_SINGLE("Right Bypass Switch", MP320_LOUTM2, 7, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new mp320_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left Playback Switch", MP320_ROUTM1, 8, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", MP320_ROUTM1, 7, 1, 0),
SOC_DAPM_SINGLE("Playback Switch", MP320_ROUTM2, 8, 1, 0),
SOC_DAPM_SINGLE("Right Bypass Switch", MP320_ROUTM2, 7, 1, 0),
};

/* Mono Mixer */
static const struct snd_kcontrol_new mp320_mono_mixer_controls[] = {
SOC_DAPM_SINGLE("Left Playback Switch", MP320_MOUTM1, 8, 1, 0),
SOC_DAPM_SINGLE("Left Bypass Switch", MP320_MOUTM1, 7, 1, 0),
SOC_DAPM_SINGLE("Right Playback Switch", MP320_MOUTM2, 8, 1, 0),
SOC_DAPM_SINGLE("Right Bypass Switch", MP320_MOUTM2, 7, 1, 0),
};

/* Left Line Mux */
static const struct snd_kcontrol_new mp320_left_line_controls =
SOC_DAPM_ENUM("Route", mp320_enum[8]);

/* Right Line Mux */
static const struct snd_kcontrol_new mp320_right_line_controls =
SOC_DAPM_ENUM("Route", mp320_enum[9]);

/* Left PGA Mux */
static const struct snd_kcontrol_new mp320_left_pga_controls =
SOC_DAPM_ENUM("Route", mp320_enum[10]);

/* Right PGA Mux */
static const struct snd_kcontrol_new mp320_right_pga_controls =
SOC_DAPM_ENUM("Route", mp320_enum[11]);

/* Out 3 Mux */
static const struct snd_kcontrol_new mp320_out3_controls =
SOC_DAPM_ENUM("Route", mp320_enum[12]);

/* Differential Mux */
static const struct snd_kcontrol_new mp320_diffmux_controls =
SOC_DAPM_ENUM("Route", mp320_enum[13]);

/* Mono ADC Mux */
static const struct snd_kcontrol_new mp320_monomux_controls =
SOC_DAPM_ENUM("Route", mp320_enum[16]);

static const struct snd_soc_dapm_widget mp320_dapm_widgets[] = {
	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
		&mp320_left_mixer_controls[0],
		ARRAY_SIZE(mp320_left_mixer_controls)),
		
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
		&mp320_right_mixer_controls[0],
		ARRAY_SIZE(mp320_right_mixer_controls)),

	SND_SOC_DAPM_MIXER("Mono Mixer", MP320_PWR2, 2, 0,
		&mp320_mono_mixer_controls[0],
		ARRAY_SIZE(mp320_mono_mixer_controls)),

	SND_SOC_DAPM_PGA("Right Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias", MP320_PWR1, 1, 0),
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", MP320_PWR1, 2, 0),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", MP320_PWR1, 3, 0),

	SND_SOC_DAPM_MUX("Left PGA Mux", MP320_PWR1, 5, 0,
		&mp320_left_pga_controls),
	SND_SOC_DAPM_MUX("Right PGA Mux", MP320_PWR1, 4, 0,
		&mp320_right_pga_controls),
	SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
		&mp320_left_line_controls),
	SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
		&mp320_right_line_controls),

	SND_SOC_DAPM_MUX("Out3 Mux", SND_SOC_NOPM, 0, 0, &mp320_out3_controls),
	SND_SOC_DAPM_PGA("Out 3", SND_SOC_NOPM, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Mono Out 1", SND_SOC_NOPM, 2, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
		&mp320_diffmux_controls),
	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
		&mp320_monomux_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
		&mp320_monomux_controls),
	
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	
	SND_SOC_DAPM_OUTPUT("MONO1"),
	SND_SOC_DAPM_OUTPUT("OUT3"),
	SND_SOC_DAPM_OUTPUT("VREF"),

	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("LINPUT2"),
	SND_SOC_DAPM_INPUT("LINPUT3"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT2"),
	SND_SOC_DAPM_INPUT("RINPUT3"),
};

static const struct snd_soc_dapm_route mp320_dapm_routes[] = {
	/* left mixer */
	{"Left Mixer", "Playback Switch", "Left DAC"},
	{"Left Mixer", "Left Bypass Switch", "Left Line Mux"},
	{"Left Mixer", "Right Playback Switch", "Right DAC"},
	{"Left Mixer", "Right Bypass Switch", "Right Line Mux"},

	/* right mixer */
	{"Right Mixer", "Left Playback Switch", "Left DAC"},
	{"Right Mixer", "Left Bypass Switch", "Left Line Mux"},
	{"Right Mixer", "Playback Switch", "Right DAC"},
	{"Right Mixer", "Right Bypass Switch", "Right Line Mux"},

	/* left out 1 */
	{"Left Out 1", NULL, "Left Mixer"},
	{"LOUT1", NULL, "Left Out 1"},

	/* left out 2 */
	{"Left Out 2", NULL, "Left Mixer"},
	{"LOUT2", NULL, "Left Out 2"},

	/* right out 1 */
	{"Right Out 1", NULL, "Right Mixer"},
	{"ROUT1", NULL, "Right Out 1"},

	/* right out 2 */
	{"Right Out 2", NULL, "Right Mixer"},
	{"ROUT2", NULL, "Right Out 2"},

	/* mono mixer */
	{"Mono Mixer", "Left Playback Switch", "Left DAC"},
	{"Mono Mixer", "Left Bypass Switch", "Left Line Mux"},
	{"Mono Mixer", "Right Playback Switch", "Right DAC"},
	{"Mono Mixer", "Right Bypass Switch", "Right Line Mux"},

	/* mono out */
	{"Mono Out 1", NULL, "Mono Mixer"},
	{"MONO1", NULL, "Mono Out 1"},

	/* out 3 */
	{"Out3 Mux", "VREF", "VREF"},
	{"Out3 Mux", "ROUT1 + Vol", "ROUT1"},
	{"Out3 Mux", "ROUT1", "Right Mixer"},
	{"Out3 Mux", "MonoOut", "MONO1"},
	{"Out 3", NULL, "Out3 Mux"},
	{"OUT3", NULL, "Out 3"},

	/* Left Line Mux */
	{"Left Line Mux", "Line 1", "LINPUT1"},
	{"Left Line Mux", "Line 2", "LINPUT2"},
	{"Left Line Mux", "Line 3", "LINPUT3"},
	{"Left Line Mux", "PGA", "Left PGA Mux"},
	{"Left Line Mux", "Differential", "Differential Mux"},

	/* Right Line Mux */
	{"Right Line Mux", "Line 1", "RINPUT1"},
	{"Right Line Mux", "Line 2", "RINPUT2"},
	{"Right Line Mux", "Line 3", "RINPUT3"},
	{"Right Line Mux", "PGA", "Right PGA Mux"},
	{"Right Line Mux", "Differential", "Differential Mux"},

	/* Left PGA Mux */
	{"Left PGA Mux", "Line 1", "LINPUT1"},
	{"Left PGA Mux", "Line 2", "LINPUT2"},
	{"Left PGA Mux", "Line 3", "LINPUT3"},
	{"Left PGA Mux", "Differential", "Differential Mux"},

	/* Right PGA Mux */
	{"Right PGA Mux", "Line 1", "RINPUT1"},
	{"Right PGA Mux", "Line 2", "RINPUT2"},
	{"Right PGA Mux", "Line 3", "RINPUT3"},
	{"Right PGA Mux", "Differential", "Differential Mux"},

	/* Differential Mux */
	{"Differential Mux", "Line 1", "LINPUT1"},
	{"Differential Mux", "Line 1", "RINPUT1"},
	{"Differential Mux", "Line 2", "LINPUT2"},
	{"Differential Mux", "Line 2", "RINPUT2"},

	/* Left ADC Mux */
	{"Left ADC Mux", "Stereo", "Left PGA Mux"},
	{"Left ADC Mux", "Mono (Left)", "Left PGA Mux"},
	{"Left ADC Mux", "Digital Mono", "Left PGA Mux"},

	/* Right ADC Mux */
	{"Right ADC Mux", "Stereo", "Right PGA Mux"},
	{"Right ADC Mux", "Mono (Right)", "Right PGA Mux"},
	{"Right ADC Mux", "Digital Mono", "Right PGA Mux"},

	/* ADC */
	{"Left ADC", NULL, "Left ADC Mux"},
	{"Right ADC", NULL, "Right ADC Mux"},
};

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k,16k,32k */           // addition from cheetah
//	{12288000, 8000, 1024, 0x8, 0x0},
//	{12288000, 16000, 512, 0x1c, 0x0},
//	{12288000, 32000, 256, 0x0, 0x0},

	/* 8k */
//	{12288000, 8000, 1536, 0x6, 0x0},
	{12288000, 8000, 1024, 0x8, 0x0}, //replace by our setting
	{11289600, 8000, 1408, 0x16, 0x0},
	{18432000, 8000, 2304, 0x7, 0x0},
	{16934400, 8000, 2112, 0x17, 0x0},
	{12000000, 8000, 1500, 0x6, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0},
	{16934400, 11025, 1536, 0x19, 0x0},
	{12000000, 11025, 1088, 0x19, 0x1},

	/* 16k */
//	{12288000, 16000, 768, 0xa, 0x0},
	{12288000, 16000, 512, 0x1c, 0x0}, //replace by our setting
	{18432000, 16000, 1152, 0xb, 0x0},
	{12000000, 16000, 750, 0xa, 0x1},

	/* 22.05k */
//	{11289600, 22050, 512, 0x1a, 0x0},
	{11289600, 22050, 256, 0x10, 0x0}, //replace by our setting
	{16934400, 22050, 768, 0x1b, 0x0},
	{12000000, 22050, 544, 0x1b, 0x1},

	/* 32k */
//	{12288000, 32000, 384, 0xc, 0x0},
	{12288000, 32000, 256, 0x0, 0x0}, //replace by our setting
	{18432000, 32000, 576, 0xd, 0x0},
	{12000000, 32000, 375, 0xa, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0},
	{16934400, 44100, 384, 0x11, 0x0},
	{12000000, 44100, 272, 0x11, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0},
	{18432000, 48000, 384, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0},
	{16934400, 88200, 192, 0x1f, 0x0},
	{12000000, 88200, 136, 0x1f, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0},
	{18432000, 96000, 192, 0xf, 0x0},
	{12000000, 96000, 125, 0xe, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	printk(KERN_ERR "mp320: could not get coeff for mclk %d @ rate %d\n",
		mclk, rate);
	return -EINVAL;
}

static int mp320_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct mp320_priv *mp320 = snd_soc_codec_get_drvdata(codec);
	pr_debug("in function %s\n", __func__); 
	
	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		mp320->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int mp320_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;
	pr_debug("in function %s\n", __func__); 
	
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, MP320_IFACE, iface);
	return 0;
}

static int mp320_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mp320_priv *mp320 = snd_soc_codec_get_drvdata(codec);
	u16 iface = snd_soc_read(codec, MP320_IFACE) & 0x1f3;
	u16 srate = snd_soc_read(codec, MP320_SRATE) & 0x1c0;
	int coeff = get_coeff(mp320->sysclk, params_rate(params));

	pr_debug("in function %s\n", __func__);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x000c;
		break;
	}
	
//	printk(KERN_EMERG "codec srate %d, coeff_div[coeff].sr: %d\n", srate , coeff_div[coeff].sr);

	/* set iface & srate */
	snd_soc_write(codec, MP320_IFACE, iface);
	if (coeff >= 0)
		snd_soc_write(codec, MP320_SRATE, srate |
			(coeff_div[coeff].sr << 1) | coeff_div[coeff].usb);
	return 0;
}

static int mp320_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = snd_soc_read(codec, MP320_ADCDAC) & 0xfff7;
	pr_debug("in function %s\n", __func__); 
	
	if (mute)
		snd_soc_write(codec, MP320_ADCDAC, mute_reg | 0x8);
	else
		snd_soc_write(codec, MP320_ADCDAC, mute_reg);
	return 0;
}

static int mp320_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 pwr_reg = snd_soc_read(codec, MP320_PWR1) & 0xfe3e;
	pr_debug("in function %s, level: %d\n", __func__, level);
	pr_debug("origin %u\n", pwr_reg); 
	printk("origin %u level:%d\n", pwr_reg, level); 
	switch (level) {
	case SND_SOC_BIAS_ON:
		/* set vmid to 50k and unmute dac */
//		snd_soc_write(codec, MP320_PWR1, pwr_reg | 0x00c0);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			snd_soc_cache_sync(codec);

			/* Set VMID to 5k For fast start-up */
//			snd_soc_write(codec, MP320_PWR1, pwr_reg | 0x01c1);

			/* ...and ramp */
			msleep(1000);
		}

		/* mute dac and set vmid to 500k, enable VREF */
//		snd_soc_write(codec, MP320_PWR1, pwr_reg | 0x0141);
		break;
	case SND_SOC_BIAS_OFF:
//		snd_soc_write(codec, MP320_PWR1, 0x0001);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define MP320_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_32000 |\
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define MP320_FORMATS (SNDRV_PCM_FMTBIT_S16_BE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)
//change MP320_FORMAT
static struct snd_soc_dai_ops mp320_dai_ops = {
	.hw_params	= mp320_pcm_hw_params,
	.digital_mute	= mp320_mute,
	.set_fmt	= mp320_set_dai_fmt,
	.set_sysclk	= mp320_set_dai_sysclk,
};

static struct snd_soc_dai_driver mp320_dai = {
	.name = "mp320-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MP320_RATES,
		.formats = MP320_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MP320_RATES,
		.formats = MP320_FORMATS,},
	.ops = &mp320_dai_ops,
};

static int mp320_suspend(struct snd_soc_codec *codec)
{
	mp320_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int mp320_resume(struct snd_soc_codec *codec)
{
	mp320_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int mp320_probe(struct snd_soc_codec *codec)
{
	struct mp320_priv *mp320 = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = snd_soc_codec_set_cache_io(codec, 7, 9, mp320->control_type);
	if (ret < 0) {
		printk(KERN_ERR "mp320: failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = mp320_reset(codec);
	if (ret < 0) {
		printk(KERN_ERR "mp320: failed to reset: %d\n", ret);
		return ret;
	}

	/* charge output caps */
	mp320_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* set the update bits */
	snd_soc_update_bits(codec, MP320_LDAC, 0x0100, 0x0100);
	snd_soc_update_bits(codec, MP320_RDAC, 0x0100, 0x0100);

	snd_soc_update_bits(codec, MP320_LOUT1V, 0x0180, 0x0180);
	snd_soc_update_bits(codec, MP320_ROUT1V, 0x0180, 0x0180);
	snd_soc_update_bits(codec, MP320_LOUT2V, 0x0180, 0x0180); 
	snd_soc_update_bits(codec, MP320_ROUT2V, 0x0180, 0x0180); 
	snd_soc_update_bits(codec, MP320_LINVOL, 0x0100, 0x0100);
	snd_soc_update_bits(codec, MP320_RINVOL, 0x0100, 0x0100);
	
	snd_soc_update_bits(codec, MP320_LOUTM1, 0x0100, 0x0100);
	snd_soc_update_bits(codec, MP320_ROUTM2, 0x0100, 0x0100);
	
	snd_soc_update_bits(codec, MP320_PWR1, 0x00c2, 0x00c2);
	snd_soc_update_bits(codec, MP320_PWR2, 0x01f8, 0x01f8);

	return ret;
}

static int mp320_remove(struct snd_soc_codec *codec)
{
	mp320_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_mp320 = {
	.probe =	mp320_probe,
	.remove =	mp320_remove,
	.suspend =	mp320_suspend,
	.resume =	mp320_resume,
	.set_bias_level = mp320_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(mp320_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = mp320_reg,

	.controls = mp320_snd_controls,
	.num_controls = ARRAY_SIZE(mp320_snd_controls),
	.dapm_widgets = mp320_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mp320_dapm_widgets),
	.dapm_routes = mp320_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(mp320_dapm_routes),
};

static const struct of_device_id mp320_of_match[] = {
	{ .compatible = "wlf,mp320", },
	{ }
};
MODULE_DEVICE_TABLE(of, mp320_of_match);

#if defined(CONFIG_SPI_MASTER)
static int __devinit mp320_spi_probe(struct spi_device *spi)
{
	struct mp320_priv *mp320;
	int ret;

	mp320 = devm_kzalloc(&spi->dev, sizeof(struct mp320_priv),
			      GFP_KERNEL);
	if (mp320 == NULL)
		return -ENOMEM;

	mp320->control_type = SND_SOC_SPI;
	spi_set_drvdata(spi, mp320);

	ret = snd_soc_register_codec(&spi->dev,
			&soc_codec_dev_mp320, &mp320_dai, 1);
	return ret;
}

static int __devexit mp320_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	return 0;
}

static const struct spi_device_id mp320_spi_ids[] = {
	{ "mp320", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mp320_spi_ids);

static struct spi_driver mp320_spi_driver = {
	.driver = {
		.name	= "mp320",
		.owner	= THIS_MODULE,
//		.of_match_table = mp320_of_match,
	},
	.id_table	= mp320_spi_ids,
	.probe		= mp320_spi_probe,
	.remove		= __devexit_p(mp320_spi_remove),
};
#endif /* CONFIG_SPI_MASTER */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static __devinit int mp320_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct mp320_priv *mp320;
	int ret;

	mp320 = devm_kzalloc(&i2c->dev, sizeof(struct mp320_priv),
			      GFP_KERNEL);
	if (mp320 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, mp320);
	mp320->control_type = SND_SOC_I2C;

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_mp320, &mp320_dai, 1);
	return ret;
}

static __devexit int mp320_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id mp320_i2c_id[] = {
	{ "mp320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mp320_i2c_id);

static struct i2c_driver mp320_i2c_driver = {
	.driver = {
		.name = "mp320",
		.owner = THIS_MODULE,
//		.of_match_table = mp320_of_match,
	},
	.probe =    mp320_i2c_probe,
	.remove =   __devexit_p(mp320_i2c_remove),
	.id_table = mp320_i2c_id,
};
#endif

static int __init mp320_modinit(void)
{
	int ret = 0;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&mp320_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register mp320 I2C driver: %d\n",
		       ret);
	}
#endif
#if defined(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&mp320_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register mp320 SPI driver: %d\n",
		       ret);
	}
#endif
	return ret;
}
module_init(mp320_modinit);

static void __exit mp320_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&mp320_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&mp320_spi_driver);
#endif
}
module_exit(mp320_exit);

MODULE_DESCRIPTION("ASoC MP320 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
