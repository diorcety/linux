/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file ip301.h
*   \brief 
*   \author Montage
*/



void ip301_set_20mhz(void);
void ip301_set_40mhz(void);
void ip301_set_channel(int channel_freq, int channel_num, int level);
void ip301_set_40mhz_channel(int channel_num, int mode, int level);

unsigned int ip301_set_tx_power(unsigned int level);

void ip301_set_iqcal(u8 loop_type);
void ip301_set_iqcal_vga(u8 loop_type, u32 rxvga, u32 txvga);

int ip301_b_init(void);
int ip301_b_recover(void);
int mt301_init(void);
int mt301_recover(void);


#define W_DAC_ADC_DIV		(0x18/4)
	#define W_DAC_40M			0x00000001
	#define W_ADC_240M			0x00000010
	#define PLL_CPVR_SEL		0x00000300
	#define PLL_VCO_SEL			0x00003000
	#define PLL_CAL_DIV			0x000f0000
#define W_ADC				(0x2C/4)
	#define W_ADC_PD			0x00000001
	#define W_ADC_SLEEP			0x00000100
#define W_ADC_SW_REG		(0x30/4)
#define W_DAC				(0x38/4)
	#define W_DAC_PD			0x00000001
	#define W_DAC_CLK_POLARITY	0x00000010
	#define W_DAC_VC			0x00030000
	#define W_DAC_CI_I			0x00007000
	#define W_DAC_CI_Q			0x00000700
	#define W_DAC_SLEEP			0x80000000

/* record the rf ctrl interface info */
enum {
	RF_CTRL_UNKNOW,
	RF_CTRL_W6,
	RF_CTRL_HWSPI,
};

struct rf_ctrl_iface {
	u32 type;
	u32 reg01;
};

extern struct rf_ctrl_iface *rf_if;
extern void (*ip301_spi_init)(void);
extern void (*ip301_spi_write)(u8 reg, u32 data);
extern u32 (*ip301_spi_read)(u8 reg);
int rf_interface_check(void);
