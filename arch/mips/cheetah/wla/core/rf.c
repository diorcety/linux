/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_rf.c
*   \brief  wla wlan RF control function.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
#include <rf.h>
#include <ip301.h>
#include <wla_bb.h>
#include <wla_debug.h>

struct rf_init_tbl rf_init_tbls[] = {
		{mt301_init},
		{ip301_b_init},
		{no_rf_init},
};

struct rf_dev rf_dev_g = 
	{	
		.chip_ver = 0,
		.spi_idx = 1,
  		.spi_clk = 0x3, // system clk * 1/(4+2n)
		.internal_ldo = true,
	};
struct rf_dev *my_rf_dev = &rf_dev_g;
struct rf_regs rf_tbl[MAX_RF_REG_NUM]; 
u32 rf_reg7_txvag[MAX_RF_K_CH_NUM];
u32 rf_reg11_freq_ofs;

unsigned int rf_read(char reg)
{
	return my_rf_dev->read(reg);
}

void rf_write(char reg, int val)
{
	if(my_rf_dev->write)
		my_rf_dev->write(reg, val);

//	WLA_DBG(WLADEBUG, "rf_write reg %d val 0x%0x\n", reg, val);
}

void rf_set_40mhz_channel(int primary_ch_freq, int channel_type, int level)
{
	if(my_rf_dev->set_40mhz_channel)
		my_rf_dev->set_40mhz_channel(primary_ch_freq, channel_type, level);
}

void rf_set_channel(int channel_freq, int channel_num, int level)
{
	if(my_rf_dev->set_channel)
		my_rf_dev->set_channel(channel_freq, channel_num, level);
}

unsigned int rf_set_tx_gain(unsigned long gain)
{
	if(my_rf_dev->set_tx_gain)
		return my_rf_dev->set_tx_gain(gain);
	return 0;
}

void no_rf_set_40mhz_channel(int primary_ch_freq, int channel_type, int level)
{
	ANAREG(W_DAC_ADC_DIV) &= ~(W_DAC_40M|W_ADC_240M);
}

void no_rf_set_channel(int channel_freq, int channel_num, int level)
{
	ANAREG(W_DAC_ADC_DIV) |= (W_DAC_40M|W_ADC_240M);
}

int no_rf_init(void)
{
	my_rf_dev->chip_ver = RFCHIP_NO_RF;

	my_rf_dev->set_channel = no_rf_set_channel;
	my_rf_dev->set_40mhz_channel = no_rf_set_40mhz_channel;
	my_rf_dev->set_tx_gain = 0;
	my_rf_dev->read = 0;
	my_rf_dev->write = 0;			

	bb_register_write(0x02, 0x31); //Cheetah IC

	WLA_DBG(WLADEBUG, "No RF chip detected, fallback to no RF handler\n");

	return 0;
}

int rf_init(void)
{
	int i;

	my_rf_dev->chip_ver = WLA_CDB_GET($parm_rfchip, 0);
	my_rf_dev->internal_ldo = !WLA_CDB_GET($parm_ex_ldo, 0);
#if defined(CONFIG_FPGA)
	if((WLA_CDB_GET($parm_hw_mode, 0) & 0x1) == 1)
	{
		my_rf_dev->spi_clk = 0x1c;
	}
#endif
	bb_reset();

	for(i=0; i<sizeof(rf_init_tbls)/sizeof(struct rf_init_tbl); i++)
	{
		my_rf_dev->init = rf_init_tbls[i].init;
		if(my_rf_dev->init() == 0)
			break;
	}

 	if(i >= (sizeof(rf_init_tbls)/sizeof(struct rf_init_tbl)))
		return -1;

	/* tuning for per RF IC */
	for(i=0; i<MAX_RF_REG_NUM; i++)
	{
		if(rf_tbl[i].num == 0)
			break;
		WLA_DBG(WLADEBUG, "RF write 0x%02x=0x%02x\n", rf_tbl[i].num, rf_tbl[i].val);
		rf_write(rf_tbl[i].num, rf_tbl[i].val);
	}


	return 0;
}

