/*
 * nau8822.h		--  codec driver for NAU8822
 *
 * Copyright 2009 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NAU8822_H__
#define __NAU8822_H__


// Configuration Register Defines.
#define CFG_SOFTWARE_RESET          (0x000)
#define CFG_POWER_MANAGEMENT1       (0x001)
#define CFG_POWER_MANAGEMENT2  	    (0x002)
#define CFG_POWER_MANAGEMENT3       (0x003)
#define CFG_AUDIO_INTERFACE	        (0x004)
#define CFG_COMPANDING_CTRL         (0x005)
#define CFG_CLK_GEN_CTRL	        (0x006)
#define CFG_ADDITIONAL_CTRL	        (0x007)
#define CFG_GPIO 			        (0x008)
#define CFG_JACK_DETECT_CTRL1       (0x009)
#define CFG_DAC_CTRL		        (0x00A)
#define CFG_LEFT_DAC_DIGITAL_VOL    (0x00B)
#define CFG_RIGHT_DAC_DIGITAL_VOL   (0x00C)
#define CFG_JACK_DETECT_CTRL2  	    (0x00D)
#define CFG_ADC_CTRL		  	    (0x00E)
#define CFG_LEFT_ADC_DIGITAL_VOL    (0x00F)
#define CFG_RIGHT_ADC_DIGITAL_VOL   (0x010)
#define CFG_EQ1_SHELF_LOW   	    (0x012)
#define CFG_EQ2_PEAK1 			    (0x013)
#define CFG_EQ3_PEAK2 			    (0x014)
#define CFG_EQ4_PEAK3			    (0x015)
#define CFG_EQ5_HIGH_SHELF		    (0x016)
#define CFG_DAC_LIMITER1		    (0x018)
#define CFG_DAC_LIMITER2 		    (0x019)
#define CFG_NOTCH_FILTER1		    (0x01B)
#define CFG_NOTCH_FILTER2 		    (0x01C)
#define CFG_NOTCH_FILTER3 		    (0x01D)
#define CFG_NOTCH_FILTER4           (0x01E)
#define CFG_ALC_CTRL1			    (0x020)
#define CFG_ALC_CTRL2 			    (0x021)
#define CFG_ALC_CTRL3 			    (0x022)
#define CFG_NOISE_GATE 	            (0x023)
#define CFG_PLLN		            (0x024)
#define CFG_PLL_K1	                (0x025)
#define CFG_PLL_K2 		            (0x026)
#define CFG_PLL_K3 		            (0x027)
#define CFG_ATTENUATION_CTRL	    (0x028)
#define CFG_3D_CONTROL              (0x029)
#define CFG_BEEP_CONTROL            (0x02B)
#define CFG_INPUT_CTRL              (0x02C)
#define CFG_LEFT_INP_PGA_GAIN_CTRL  (0x02D)
#define CFG_RIGHT_INP_PGA_GAIN_CTRL (0x02E)
#define CFG_LEFT_ADC_BOOST_CTRL     (0x02F)
#define CFG_RIGHT_ADC_BOOST_CTRL 	(0x030)
#define CFG_OUTPUT_CTRL			    (0x031)
#define CFG_LEFT_MIXER_CTRL			(0x032)
#define CFG_RIGHT_MIXER_CTRL 		(0x033)
//LOUT1 --> HP-,ROUT1 --> HP+,LOUT2 --> SPKOUT-,ROUT2 --> SPKOUT+,OUT3 --> AUXOUT2,OUT4 --> AUXOUT1
#define CFG_LOUT1_HP_VOLUME_CTRL	(0x034)
#define CFG_ROUT1_HP_VOLUME_CTRL 	(0x035)
#define CFG_LOUT2_SPKR_VOLUME_CTRL	(0x036)
#define CFG_ROUT2_SPKR_VOLUME_CTRL 	(0x037)
#define CFG_OUT3_MIXER_CTRL         (0x038)
#define CFG_OUT4_MIXER_CTRL         (0x039)

/*
 * Register values.
 */
#define NAU8822_RESET				0x00
#define NAU8822_POWER_MANAGEMENT_1		0x01
#define NAU8822_POWER_MANAGEMENT_2		0x02
#define NAU8822_POWER_MANAGEMENT_3		0x03
#define NAU8822_AUDIO_INTERFACE			0x04
#define NAU8822_COMPANDING_CONTROL		0x05
#define NAU8822_CLOCKING				0x06
#define NAU8822_ADDITIONAL_CONTROL		0x07
#define NAU8822_GPIO_CONTROL			0x08
#define NAU8822_JACK_DETECT_CONTROL_1		0x09
#define NAU8822_DAC_CONTROL			0x0A
#define NAU8822_LEFT_DAC_DIGITAL_VOLUME		0x0B
#define NAU8822_RIGHT_DAC_DIGITAL_VOLUME		0x0C
#define NAU8822_JACK_DETECT_CONTROL_2		0x0D
#define NAU8822_ADC_CONTROL			0x0E
#define NAU8822_LEFT_ADC_DIGITAL_VOLUME		0x0F
#define NAU8822_RIGHT_ADC_DIGITAL_VOLUME		0x10
#define NAU8822_EQ1				0x12
#define NAU8822_EQ2				0x13
#define NAU8822_EQ3				0x14
#define NAU8822_EQ4				0x15
#define NAU8822_EQ5				0x16
#define NAU8822_DAC_LIMITER_1			0x18
#define NAU8822_DAC_LIMITER_2			0x19
#define NAU8822_NOTCH_FILTER_1			0x1b
#define NAU8822_NOTCH_FILTER_2			0x1c
#define NAU8822_NOTCH_FILTER_3			0x1d
#define NAU8822_NOTCH_FILTER_4			0x1e
#define NAU8822_ALC_CONTROL_1			0x20
#define NAU8822_ALC_CONTROL_2			0x21
#define NAU8822_ALC_CONTROL_3			0x22
#define NAU8822_NOISE_GATE			0x23
#define NAU8822_PLL_N				0x24
#define NAU8822_PLL_K1				0x25
#define NAU8822_PLL_K2				0x26
#define NAU8822_PLL_K3				0x27
#define NAU8822_3D_CONTROL			0x29
#define NAU8822_BEEP_CONTROL			0x2b
#define NAU8822_INPUT_CONTROL			0x2c
#define NAU8822_LEFT_INP_PGA_CONTROL		0x2d
#define NAU8822_RIGHT_INP_PGA_CONTROL		0x2e
#define NAU8822_LEFT_ADC_BOOST_CONTROL		0x2f
#define NAU8822_RIGHT_ADC_BOOST_CONTROL		0x30
#define NAU8822_OUTPUT_CONTROL			0x31
#define NAU8822_LEFT_MIXER_CONTROL		0x32
#define NAU8822_RIGHT_MIXER_CONTROL		0x33
#define NAU8822_LOUT1_HP_CONTROL			0x34
#define NAU8822_ROUT1_HP_CONTROL			0x35
#define NAU8822_LOUT2_SPK_CONTROL		0x36
#define NAU8822_ROUT2_SPK_CONTROL		0x37
#define NAU8822_OUT3_MIXER_CONTROL		0x38
#define NAU8822_OUT4_MIXER_CONTROL		0x39

#define NAU8822_CACHEREGNUM			58

/* Clock divider Id's */
enum nau8822_clk_id {
	NAU8822_OPCLKRATE,
	NAU8822_BCLKDIV,
};

enum nau8822_sysclk_src {
	NAU8822_PLL,
	NAU8822_MCLK
};

#endif	/* __NAU8822_H__ */
