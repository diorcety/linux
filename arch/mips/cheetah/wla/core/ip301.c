/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file ip301.c
*   \brief  ip301 driver.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_common.h>
#include <rf.h>
#include <rfc.h>
#include <rfac.h>
#include <ip301.h>

#if 1
#define IP301_MSG WLA_MSG
#else
#define IP301_MSG(x , ...)
#endif
#define RFDBG WLA_MSG

struct rf_ctrl_iface rf_iface;
struct rf_ctrl_iface *rf_if = &rf_iface;

int (*ip301_reg_write)(unsigned char spi_idx, unsigned int reg, unsigned int data);
int (*ip301_reg_read)(unsigned char spi_idx, unsigned int reg, unsigned int * data_ptr);
void (*ip301_spi_init)(void);
void (*ip301_spi_write)(u8 reg, u32 data);
u32 (*ip301_spi_read)(u8 reg);

int ip301_reg_write_w6(unsigned char spi_idx, unsigned int reg, unsigned int data)
{
	int timeout = 0;

	bb_register_write(0xA3, (0xFF & (data >> 16)));
	bb_register_write(0xA4, (0xFF & (data >> 8)));
	bb_register_write(0xA5, (0xFF & data));

	bb_register_write(0xA2, reg);
#if 0
	if(reg==2)
		IP301_MSG("=====> ip301 write reg %x, data %x\n", reg, data);
#endif
    /* read BRA2 and get msb bit (like operation &0x80) until the result is 0x0 */
	while(bb_register_read(0xA2) & 0x80)
	{
		if(timeout++>100000)
		{
			IP301_MSG("===> ip301 write fail!!! reg %x, data %x\n", reg, data);
			return -2;
		}
	}

	return 0;
}

int ip301_reg_read_w6(unsigned char spi_idx, unsigned int reg, unsigned int * data_ptr)
{
	int timeout = 0;

	bb_register_write(0xA2, (reg | 0x20));

    /* read BRA2 and get msb bit (like operation &0x80) until the result is 0x0 */
	while(bb_register_read(0xA2) & 0x80)
	{
		if(timeout++>100000)
		{
			IP301_MSG("===>ip301 read fail!!! reg %x\n", reg);
			return -2;
		}
	}

	*data_ptr = ((bb_register_read(0xA6) << 16) | (bb_register_read(0xA7) << 8) | bb_register_read(0xA8));

	return 0;
}

void ip301_spi_init_w6(void)
{
}

void ip301_spi_write_w6(u8 reg, u32 data)
{
#ifdef RF_SPI_DOUBLE_CONFIRM
	u32 tmp, i=0;
	do{
#endif
	ip301_reg_write(my_rf_dev->spi_idx, reg, data);
#ifdef RF_SPI_DOUBLE_CONFIRM
		tmp = ip301_spi_read(reg);
		if(i++ > 10)
		{
			WLA_DBG(WLADEBUG, "Fail in ip301_spi_write(0x%x), Exp value = 0x%x, read value = 0x%x\n", reg, data, tmp);
			break;
		}

	}while(tmp != data);
#endif
}

u32 ip301_spi_read_w6(u8 reg)
{
	unsigned int value = 0;
#ifdef RF_SPI_DOUBLE_CONFIRM
	u32 tmp, i=0;
	do{
		tmp = value;
#endif

	ip301_reg_read(my_rf_dev->spi_idx, reg, &value);
#ifdef RF_SPI_DOUBLE_CONFIRM
		ip301_reg_read(my_rf_dev->spi_idx, reg, &tmp);

		if(i++ > 10)
		{
			WLA_DBG(WLADEBUG, "Fail in ip301_spi_read(0x%x), last value = 0x%x, Prev. value = 0x%x\n", reg, value, tmp);
			break;
		}

	}while(tmp != value);
#endif

	return (u32) value;
}

#include <gspi.h>

#define RF_SPI_DOUBLE_CONFIRM

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define IP301_READ_CMD 1

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/
int ip301_reg_write_hwspi(unsigned char spi_idx, unsigned int reg, unsigned int data)
{
	int rc;
	unsigned char tx_buf[4];
	
	// prepared data(bit) : cmd(1) addr(5) data(18)
	tx_buf[1] = (0x3FC00&data)>>10;
	tx_buf[2] = (0x3FC&data)>>2;
	tx_buf[3] = (0x3&data)<<6|reg<<1;
	rc = spi_write(spi_idx, 24, tx_buf);
	if (rc)
		IP301_MSG("?????????????????????????ip301 write fail!!!\n");
	return rc;
}

int ip301_reg_read_hwspi(unsigned char spi_idx, unsigned int reg, unsigned int * data_ptr)
{
	int rc;
	unsigned char tx_buf[4];
		
	if (!spi_ready(spi_idx))
		return -2;	
	spi_keep_cs(spi_idx, 1);
	// prepared data(bit) : cmd(1) addr(5)
	tx_buf[3] = reg<<1|IP301_READ_CMD;
	rc = spi_write(spi_idx, 6, tx_buf);
	if (rc)
		IP301_MSG("?????????????????????????ip301 read fail(%d)!!!\n", __LINE__);
	rc = spi_read(spi_idx, 18, (unsigned char *)data_ptr);
	*data_ptr = *data_ptr & 0x0003ffff;	
	if (rc)
		IP301_MSG("?????????????????????????ip301 read fail(%d)!!!\n", __LINE__);
	
	spi_keep_cs(spi_idx, 0);

	return rc;
}

void ip301_spi_init_hwspi(void)
{
	spi_init(my_rf_dev->spi_idx, SPI_DATA_INVERT|(my_rf_dev->spi_clk << 8)|SPI_THREE_WIRE|SPI_FALLING_SAMPLE);
}

void ip301_spi_write_hwspi(u8 reg, u32 data)
{
#ifdef RF_SPI_DOUBLE_CONFIRM
	u32 tmp, i=0;
	do{
#endif
	ip301_reg_write(my_rf_dev->spi_idx, reg, data);
#ifdef RF_SPI_DOUBLE_CONFIRM
		tmp = ip301_spi_read(reg);
		if(i++ > 10)
		{
			WLA_DBG(WLADEBUG, "Fail in ip301_spi_write(0x%x), Exp value = 0x%x, read value = 0x%x\n", reg, data, tmp);
			break;
		}

	}while(tmp != data);
#endif
}

u32 ip301_spi_read_hwspi(u8 reg)
{
	unsigned int value = 0;
#ifdef RF_SPI_DOUBLE_CONFIRM
	u32 tmp, i=0;
	do{
		tmp = value;
#endif

	ip301_reg_read(my_rf_dev->spi_idx, reg, &value);
#ifdef RF_SPI_DOUBLE_CONFIRM
		ip301_reg_read(my_rf_dev->spi_idx, reg, &tmp);

		if(i++ > 10)
		{
			WLA_DBG(WLADEBUG, "Fail in ip301_spi_read(0x%x), last value = 0x%x, Prev. value = 0x%x\n", reg, value, tmp);
			break;
		}

	}while(tmp != value);
#endif

	return (u32) value;
}


int rf_interface_check(void)
{
	u32 pattern = 0x5a5a;
	u32 ret_val = 0;
	
	rf_if->type = RF_CTRL_UNKNOW;
	
	/* check 6 wire architecture */
	bb_register_write(0x1, 0x80);
	bb_register_write(0xF4, 0x00);

	ip301_reg_write = ip301_reg_write_w6;
	ip301_reg_read = ip301_reg_read_w6;
	ip301_spi_init = ip301_spi_init_w6;
	ip301_spi_write = ip301_spi_write_w6;
	ip301_spi_read = ip301_spi_read_w6;

	ip301_spi_write(0, pattern);
	if((ret_val = ip301_spi_read(0)) == pattern)
	{
		rf_if->reg01 = 0x80;
		rf_if->type = RF_CTRL_W6;
		goto end_sect;
	}

	WLA_DBG(WLADEBUG,"ret_val = 0x%x\n", ret_val);

	/* check HW SPI architecture */
	bb_register_write(0x1, 0x00);
	/* reg.0xf4 is fixed as 0 */

	ip301_reg_write = ip301_reg_write_hwspi;
	ip301_reg_read = ip301_reg_read_hwspi;
	ip301_spi_init = ip301_spi_init_hwspi;
	ip301_spi_write = ip301_spi_write_hwspi;
	ip301_spi_read = ip301_spi_read_hwspi;

	ip301_spi_write(0, pattern);
	if(ip301_spi_read(0) == pattern)
	{
		rf_if->reg01 = 0x00;
		rf_if->type = RF_CTRL_HWSPI;
		goto end_sect;
	}

end_sect:
	return rf_if->type;
}

static u32 ip301_channel_data[][3] =
{
    { 1, 2412, 0x02864b },
    { 2, 2417, 0x0288cb },
    { 3, 2422, 0x028b4b },
    { 4, 2427, 0x028dcb },
    { 5, 2432, 0x02804c },
    { 6, 2437, 0x0282cc },
    { 7, 2442, 0x02854c },
    { 8, 2447, 0x0287cc },
    { 9, 2452, 0x028a4c },
    { 10, 2457, 0x028ccc },
    { 11, 2462, 0x028f4c },
    { 12, 2467, 0x0281cd },
    { 13, 2472, 0x02844d },
    { 14, 2484, 0x028a4d },
	{ 255, 2520, 0x028c4e },
};

#define CHANNEL_WIDTH_20MHZ		0
#define CHANNEL_WIDTH_40MHZ		1

int current_channel_width = CHANNEL_WIDTH_20MHZ;

void ip301_set_20mhz(void)
{
	if(current_channel_width == CHANNEL_WIDTH_20MHZ)
		return;
#if 0
	ip301_spi_write(0x8, 0x3200);
	ip301_spi_write(0x8, 0x1200);
#else
	/* for ip301_g patch (tx filter code calibration has problem) */
	if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1))
		ip301_spi_write(0x8, ip301_spi_read(0x8) & 0xfffff3ff);
	else
		ip301_spi_write(0x8, ip301_spi_read(0x8) & 0xffff3fff);
#if !defined(CONFIG_FPGA)
	ANAREG(W_DAC_ADC_DIV) |= (W_DAC_40M|W_ADC_240M);
#endif
#endif 
	current_channel_width = CHANNEL_WIDTH_20MHZ;
}

void ip301_set_40mhz(void)
{
	if(current_channel_width == CHANNEL_WIDTH_40MHZ)
		return;
#if 0
	ip301_spi_write(0x8, 0xF200);
	ip301_spi_write(0x8, 0xD200);
#else
	/* for ip301_g patch (tx filter code calibration has problem) */
	if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1))
		ip301_spi_write(0x8, ip301_spi_read(0x8) | 0x0C00);
	else
		ip301_spi_write(0x8, ip301_spi_read(0x8) | 0xC000);
#if !defined(CONFIG_FPGA)
	ANAREG(W_DAC_ADC_DIV) &= ~(W_DAC_40M|W_ADC_240M);
#endif
#endif 
	current_channel_width = CHANNEL_WIDTH_40MHZ;
}

void ip301_set_channel(int channel_freq, int channel_num, int level)
{
    int cfg_index = -1;
    int i, scale;
	unsigned int reg = 0;
	unsigned int txvga = 0;
    
	if(channel_freq) 
    {
        for(i=0;i<(sizeof(ip301_channel_data)/sizeof(ip301_channel_data[0]));i++)
        {
            if(ip301_channel_data[i][1] == channel_freq)
            {
                cfg_index = i;
                break;
            }
        }
    }
    else
    {
        for(i=0;i<(sizeof(ip301_channel_data)/sizeof(ip301_channel_data[0]));i++)
        {
            if(ip301_channel_data[i][0] == channel_num)
            {
                cfg_index = i;
                break;
            }
        }
    }

    if(cfg_index >= 0)
    {
		ip301_spi_write(0x2, ip301_channel_data[i][2]);

		ip301_set_20mhz();
    }

	// apply calibration txvga value
	if(channel_num <= MAX_RF_K_CH_NUM && rf_reg7_txvag[cfg_index] > 0)
	{	
		reg = rf_read(0x07);
		// apply tx power level, level 0 = 15dbm(default), 1 = 1dbm, 2 = 2dbm...
		if (level != 0 && (15 - level) > 0)
        {
            scale = 3 * (15 - level);
		    if (scale > rf_reg7_txvag[cfg_index])
                txvga = 0;
            else
			    txvga = rf_reg7_txvag[cfg_index] - scale;
		}
        else
			txvga = rf_reg7_txvag[cfg_index];
		reg = (reg & 0x301ff) + ((txvga << 9) & 0xfe00);
		rf_write(0x7, reg);
	}

	// apply calibration freqency offset
	if(rf_reg11_freq_ofs > 0)
	{	
		reg = rf_read(0x11);
		reg = (reg & 0xffffffe0) + (rf_reg11_freq_ofs & 0x1f);
		rf_write(0x11, reg);
	}
}

void ip301_set_40mhz_channel(int channel_num, int mode, int level)
{
	int primary_ch_freq;
	int channel_freq;
	int cfg_index = -1;
	int i;
	unsigned int reg = 0;
	unsigned int txvga = 0;
	
	primary_ch_freq = ip301_channel_data[channel_num - 1][1];

	if(mode == HT40_PLUS)
	{
		channel_freq = primary_ch_freq + 10;
	}
	else if(mode == HT40_MINUS)
	{
		channel_freq = primary_ch_freq - 10;
	}
	else
	{
		return;
	}

	for(i=0;i<(sizeof(ip301_channel_data)/sizeof(ip301_channel_data[0]));i++)
	{
		if(ip301_channel_data[i][1] == channel_freq)
		{
			cfg_index = i;
			break;
		}
	}

	if(cfg_index >= 0)
    {
		ip301_spi_write(0x2, ip301_channel_data[cfg_index][2]);

		ip301_set_40mhz();
	}

	// apply calibration txvga value
	if(channel_num <= MAX_RF_K_CH_NUM && rf_reg7_txvag[cfg_index] > 0)
	{	
		reg = rf_read(0x7);
		// apply tx power level, level 0 = 15dbm(default), 1 = 1dbm, 2 = 2dbm...
		if (level != 0 && (15 - level) > 0)
			txvga = rf_reg7_txvag[cfg_index] - (15 - level) * 3;
		else
			txvga = rf_reg7_txvag[cfg_index];
		reg = (reg & 0x301ff) + ((txvga << 9) & 0xfe00);
		rf_write(0x7, reg);
	}

	// apply calibration freqency offset
	if(rf_reg11_freq_ofs > 0)
	{	
		reg = rf_read(0x11);
		reg = (reg & 0xffffffe0) + (rf_reg11_freq_ofs & 0x1f);
		rf_write(0x11, reg);
	}
	return;
}

const unsigned short gain_level[] = {0x65f8, 0x45f8, 0x25f8, 0x35f8 /* for test */};

unsigned int ip301_set_tx_power(unsigned int level)
{
	#if 0
	if(level < (sizeof(gain_level)/sizeof(unsigned short)))
		ip301_spi_write(0x7, gain_level[level]);
	#endif
	return level;
}

int ip301_b_recover(void)
{
    ip301_spi_write(0x0, IP301_NORMAL_MODE);

	// RF tuning, power value
    ip301_spi_write(0x7, 0x45f8);

    /* maximum bandwidth of rxhp to reduce rxhp setting time */
    ip301_spi_write(0x9, 0x3d9fc);

	ip301_spi_write(0x15, 0);
	ip301_spi_write(0x9, 0x349fc);

	return 0;
}

int ip301_b_init(void)
{
    int ret;

	ip301_spi_init();
	if(my_rf_dev->chip_ver == 0)
	{
		unsigned char rfchip;
		rfchip = ip301_spi_read(0x1f);
		if((rfchip < RFCHIP_IP301_B) || (rfchip >= 0x5a))
			return -1;
		my_rf_dev->chip_ver = rfchip; 
	}

	my_rf_dev->set_channel = ip301_set_channel;
	my_rf_dev->set_40mhz_channel = ip301_set_40mhz_channel;
	my_rf_dev->set_tx_gain = ip301_set_tx_power; 
	my_rf_dev->read = ip301_spi_read;
	my_rf_dev->write = ip301_spi_write;

    ret = rf_calibration(RF_CAL_CH, my_rf_dev->internal_ldo);
    if(0>ret)
        goto Failed;

	ip301_b_recover();

    RFDBG("RF init IP301-%d done\n", my_rf_dev->chip_ver);

	rfc_process_new(0);	
		
	if(IP301_NORMAL_MODE != ip301_spi_read(0x0))
		WLA_DBG(WLAERROR, "????????????????????????????????IP301 REG0 has abnormal value, 0x%02x\n", ip301_spi_read(0x0));

    return 0;

Failed:
    RFDBG("RF init IP301-B failed\n");
    return ret;
}

int mt301_recover(void)
{
	ip301_spi_write(0x0, 0x3E954);
	ip301_spi_write(0x1, 0x8000);
	ip301_spi_write(0x6, 0x10bd);
	ip301_spi_write(0x16, 0x12626); /* Disable internal PA, it will degrade B-mode throughput */
	ip301_spi_write(0x17, 0x30A40);
	ip301_spi_write(0x19, 0x3555);
	//ip301_spi_write(0x7, 0x1CC0);
	ip301_spi_write(0x7, 0x3CC0); /* increase TX power */
	ip301_spi_write(0x9, 0x19040);

    /* maximum bandwidth of rxhp to reduce rxhp setting time */
	ip301_spi_write(0x15, 0x3000);

	return 0;
}

//const unsigned short mt301_gain_level[] = {0x58C0, 0x3CC0, 0x3CC0, 0x3CC0 /* for test */};
/* level 0 means 15dbm */
const unsigned short mt301_gain_level[] = 
		{	0x54C0, /* default */ 
			0x16C0, /* 1dbm */ 
			0x18C0, 
			0x1AC0, 
			0x1EC0, 
			0x20C0, 
			0x22C0, 
			0x26C0, 
			0x28C0, 
			0x30C0, 
			0x32C0, 
			0x3CC0, 
			0x40C0, 
			0x46C0, 
			0x4CC0, 
			0x54C0, 
			0x60C0,
			0x64C0	/* 17dbm */
		};

unsigned int mt301_set_tx_power(unsigned int level)
{
	#if 0
	if(level < (sizeof(mt301_gain_level)/sizeof(unsigned short)))
		ip301_spi_write(0x7, mt301_gain_level[level]);
	else
		ip301_spi_write(0x7, mt301_gain_level[0]);
	#endif
	return level;
}

int mt301_init(void)
{
	int ret;

	ip301_spi_init();
	if(my_rf_dev->chip_ver == 0)
	{
		unsigned char rfchip;
		rfchip = ip301_spi_read(0x1f);
		
		if((rfchip != RFCHIP_MT301_A0) && (rfchip != RFCHIP_MT301_A1))
			return -1;
		my_rf_dev->chip_ver = rfchip; 
	}

	my_rf_dev->set_channel = ip301_set_channel;
	my_rf_dev->set_40mhz_channel = ip301_set_40mhz_channel;
	my_rf_dev->set_tx_gain = mt301_set_tx_power; 
	my_rf_dev->read = ip301_spi_read;
	my_rf_dev->write = ip301_spi_write;

	ret = mt301_calibration(7, my_rf_dev->internal_ldo);

    if(0>ret)
        goto Failed;

	mt301_recover();
	rfc_process_new(0);	
	mt301_recover();

    RFDBG("RF init MT301-%d done\n", my_rf_dev->chip_ver);

#if 0
	if(IP301_NORMAL_MODE != ip301_spi_read(0x0))
		WLA_DBG(WLAERROR, "????????????????????????????????IP301 REG0 has abnormal value, 0x%02x\n", ip301_spi_read(0x0));
#endif

    return 0;

Failed:
    RFDBG("RF init MT301-%d failed\n", my_rf_dev->chip_ver);
    return ret;
}

void ip301x2_set_channel(int channel_freq, int channel_num)
{
	int i;
	for(i=1; i>=0; i--)
	{
		my_rf_dev->spi_idx = i;
		ip301_set_channel(channel_freq, channel_num, 0);
	}
}

void ip301X2_set_40mhz_channel(int channel_num, int mode)
{
	int i;
	for(i=1; i>=0; i--)
	{
		my_rf_dev->spi_idx = i;
		ip301_set_40mhz_channel(channel_num, mode, 0);
	}
}
void ip301x2_set_tx_power(unsigned int level)
{
	int i;
	for(i=1; i>=0; i--)
	{
		my_rf_dev->spi_idx = i;
		ip301_set_tx_power(level);
	}
}

#define TXLOOP 0
#define RXLOOP 1
#define NON_LOOP 2

#if 1
u32 txcaltxvga = 0;
u32 txcalrxvga = 0;
u32 rxcaltxvga = 0;
u32 rxcalrxvga = 0;
#endif

void ip301_set_iqcal(u8 loop_type)
{
    IP301_MSG("MANUAL txcaltxvga %x, txcalrxvga %x, rxcaltxvga %x, rxcalrxvga %x\n",
    		txcaltxvga, txcalrxvga, rxcaltxvga, rxcalrxvga);

	if (TXLOOP == loop_type)
	{
		//Configure TX LOOPBACK
		IP301_MSG("Set ip301 to tx loopback mode\n");
		
		ip301_spi_write(0x0, 0x3ffe0);

		// RX VGA gain setting for TX Calibration
		if(txcalrxvga)
		{
			ip301_spi_write(0x9, txcalrxvga);
		}
		else
		{
			ip301_spi_write(0x9, 0x7c);
		}

		// TX VGA gain setting for TX Calibration
		if(txcaltxvga)
		{
			ip301_spi_write(0x7, txcaltxvga);
		}
		else
		{
			ip301_spi_write(0x7, 0xa1f8);
		}

		// TX IQ Calibration
		ip301_spi_write(0x1, 0xa2a);
	}
	else if  (RXLOOP == loop_type)
	{
		//Configure RX LOOPBACK
		IP301_MSG("Set ip301 to rx loopback mode\n");

		ip301_spi_write(0x0, 0x3fffe);
	
		// RX VGA gain setting for RX Calibration
		if(rxcalrxvga)
		{
			ip301_spi_write(0x9, rxcalrxvga);
		}
		else
		{
			{    
				//ip301_spi_write(0x9, 0x8c);
				ip301_spi_write(0x9, ((9 * 2) << 3));
			}
		}

		// TX VGA gain setting for RX Calibration
		if(rxcaltxvga)
		{
			ip301_spi_write(0x7, rxcaltxvga);
		}
		else
		{
			{    
				ip301_spi_write(0x7, 0x65f8);
				ip301_spi_write(0x7, (12 << 9) | (ip301_spi_read(0x7) & 0x1ff));
			}
		}
		
		// RX IQ Calibration
		ip301_spi_write(0x1, 0x0629);
	}
	else
		IP301_MSG("Not supported loopback type!\n");
}

void ip301_set_iqcal_vga(u8 loop_type, u32 rxvga, u32 txvga)
{
    //IP301_MSG("MANUAL txvga %x, rxvga %x\n", txvga, rxvga);
	int txvga_gain_code[11] = { 12, 15, 19, 24, 30, 37, 47, 60, 76, 96, 127, };
	u32 reg0;
	u32 reg9_o = ip301_spi_read(0x9);
	u32 reg9 = reg9_o & 0xFFFFFE07;
	u32 reg7_o = ip301_spi_read(0x7);

	reg0 = ip301_spi_read(0x0);
	if (TXLOOP == loop_type)
	{
		//Configure TX LOOPBACK
		//IP301_MSG("Set ip301 to tx loopback mode\n");
		
		if((my_rf_dev->chip_ver != RFCHIP_MT301_A0) && (my_rf_dev->chip_ver != RFCHIP_MT301_A1)) 
		{
			// rx lo buffer on (reg.0 bit.2), verified by KC
			ip301_spi_write(0x0, 0x3ffe4);
		}

		// RX VGA gain setting for TX Calibration
		if(txcalrxvga)
		{
			ip301_spi_write(0x9, txcalrxvga | reg9);
		}
		else
		{
			if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1)) 
			{
				/* if ((BW==40Mhz)&&(in tone = 18Mhz)) rxvga = 24;
				   else rxvga = 18;
				   rxvga : unit of dB, located at reg9[8:3] 		*/
				ip301_spi_write(0x9, (reg9_o & 0x3fe07) | (rxvga*2 << 3));
			}
			else
			{
				if(rxvga)
				{
					ip301_spi_write(0x9, ((rxvga*2) << 3) | reg9);
				}
				else
				{
					ip301_spi_write(0x9, 0x7c | reg9);
				}
			}
		}

		// TX VGA gain setting for TX Calibration
		if(txcaltxvga)
		{
			ip301_spi_write(0x7, txcaltxvga);
		}
		else
		{
			if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1)) 
			{
				//  txvga = 60; for all conditions
				//  located at reg7[15:9]
				ip301_spi_write(0x7, (reg7_o& 0x301ff) | (txvga_gain_code[txvga] <<9));
			}
			else
			{
				//ip301_spi_write(0x7, ((txvga_gain_code[txvga] << 9) | (ip301_spi_read(0x7) & 0x1ff)));
				ip301_spi_write(0x7, ((txvga_gain_code[txvga] << 9) | 0x1f8));
			}
		}

		// TX IQ Calibration
		if((my_rf_dev->chip_ver != RFCHIP_MT301_A0) && (my_rf_dev->chip_ver != RFCHIP_MT301_A1)) 
		{
			//ip301_spi_write(0x1, 0xa2a);
			ip301_spi_write(0x1, (ip301_spi_read(0x1) & 0xfffff3fc) | 0x802);
		}	
		else
		{
			ip301_spi_write(0x15, 0x0);
			ip301_spi_write(0x19, 0x3555);
			ip301_spi_write(0x0,  0x3ffe0);
			ip301_spi_write(0x1,  0x8802);
			ip301_spi_write(0x17, 0x0);
		}
	}
	else if  (RXLOOP == loop_type)
	{
		//Configure RX LOOPBACK
		//IP301_MSG("Set ip301 to rx loopback mode\n");

		// all blocks on for tx & rx, verified by KC
		if((my_rf_dev->chip_ver != RFCHIP_MT301_A0) && (my_rf_dev->chip_ver != RFCHIP_MT301_A1)) 
		{
			//ip301_spi_write(0x0, 0x3ffff);
			ip301_spi_write(0x0, 0x3fffe);
		}
	
		// RX VGA gain setting for RX Calibration
		if(rxcalrxvga)
		{
			ip301_spi_write(0x9, rxcalrxvga | reg9);
		}
		else
		{
			if(rxvga)
			{
				if((my_rf_dev->chip_ver != RFCHIP_MT301_A0) && (my_rf_dev->chip_ver != RFCHIP_MT301_A1)) 
					ip301_spi_write(0x9, ((rxvga*2) << 3) | reg9);
				else
				{
					// rxvga : unit of dB, located at reg9[8:3], Lab test : rxvga=20
			        ip301_spi_write(0x9, (reg9_o & 0x3fe07) | (rxvga*2 << 3));
				}
			}
			else
			{
				if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1))
				{
					/* RF CHIP J (MT301) doesn't need this step */
				}
				else
				{    
					//ip301_spi_write(0x9, 0x8c);
					ip301_spi_write(0x9, ((9 * 2) << 3) | reg9);
				}
			}
		}

		// TX VGA gain setting for RX Calibration
		if(rxcaltxvga)
		{
			ip301_spi_write(0x7, rxcaltxvga);
		}
		else
		{
			if((my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1)) 
			{
				//  Lab test : txvga = 10;
				//  located at reg7[15:9]
				ip301_spi_write(0x7, (reg7_o& 0x301ff) | (txvga_gain_code[txvga] <<9));
			}
			else
			{   
				//ip301_spi_write(0x7, 0x65f8);
				//ip301_spi_write(0x7, ((txvga_gain_code[txvga] << 9) | (ip301_spi_read(0x7) & 0x1ff)));
				ip301_spi_write(0x7, ((txvga_gain_code[txvga] << 9) | 0x1f8));
			}
		}
		
		// RX IQ Calibration
		if((my_rf_dev->chip_ver != RFCHIP_MT301_A0) && (my_rf_dev->chip_ver != RFCHIP_MT301_A1)) 
		{
			//ip301_spi_write(0x1, 0x0629);
			ip301_spi_write(0x1, (ip301_spi_read(0x1) & 0xfffff3fc) | 0x401);
		}
		else
		{
			ip301_spi_write(0x15, 0x0);
			ip301_spi_write(0x19, 0x3555);
			ip301_spi_write(0x0,  0x3fffe);
			ip301_spi_write(0x1,  0x8409);
			ip301_spi_write(0x17, 0x0);
		}
	}
	else if(NON_LOOP == loop_type)
	{
		/* Disable TX/RX loopback function */
		//ip301_spi_write(0x0, 0x3e954);
	}
	else
		IP301_MSG("Not supported loopback type!\n");

}

