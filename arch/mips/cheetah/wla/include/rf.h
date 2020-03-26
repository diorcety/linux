#ifndef __RF_H__
#define __RF_H__
#include <mac_ctrl.h>

unsigned int rf_read(char reg);
void rf_write(char reg, int val);
void rf_set_40mhz_channel(int primary_ch_freq, int channel_type, int level);
void rf_set_channel(int channel_freq, int channel_num, int level);
int rf_init(void);
unsigned int rf_set_tx_gain(unsigned long gain);


enum {
	RFCHIP_MT301_A1 = 1,	
	RFCHIP_IP301_B = 11,
	RFCHIP_IP301_E = 14,
	RFCHIP_IP301_G = 16,
	RFCHIP_MT301_A0 = 19,
	RFCHIP_IP301_L = 21,
	RFCHIP_IP301_GX2 = (16+0x0100),

	RFCHIP_NO_RF = 254,
};

#define IP301_NORMAL_MODE		0x3F954
#define MAX_RF_REG_NUM		10

struct rf_regs {
	u8 num;
	u32 val;
};

extern struct rf_regs rf_tbl[MAX_RF_REG_NUM];

#define MAX_RF_K_CH_NUM		16
extern u32 rf_reg7_txvag[MAX_RF_K_CH_NUM];
extern u32 rf_reg11_freq_ofs;

struct rf_dev{
	unsigned short chip_ver; /* RF chip version */
	unsigned char internal_ldo;
	unsigned char spi_idx;
	unsigned char spi_clk;
	int (*init)(void);
	void (*set_channel)(int channel_freq, int channel_num, int level);
	void (*set_40mhz_channel)(int primary_ch_freq, int channel_type, int level);
	unsigned int (*read)(unsigned char reg);
	void (*write)(unsigned char reg, unsigned int val);
	unsigned int (*set_tx_gain)(unsigned int gain);
};

struct rf_init_tbl{
	int (*init)(void);
};

int no_rf_init(void);

extern struct rf_dev *my_rf_dev;

#endif // __RF_H__

