/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_bb.c
*   \brief  wla wlan baseband functions.
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_ctrl.h>
#include <mac_regs.h>
#include <rf.h>
#include <rfc.h>
#include <wla_debug.h>
#include <wla_bb.h>
#include <wla_api.h>
#include <ip301.h>

//#define BB_SPI_DOUBLE_CONFIRM



struct bb_dev bb_dev_g, *my_bb_dev = &bb_dev_g;

struct bb_regs bb_init_tbl_r5c_e [] = {
    {0x00, 0x01},  // reset BB to default value
	{0x01, 0x80},  // toggle RXHP and old bb_LMAC interface
    {0x02, 0x30},  // TX IQ swap
    {0xf2, 0x80},  // DAC CLK 40MHz
    {0xf3, 0x22},  // ADC CLK 40MHz
    {0x54, 0x2b},  // disable on-fly IQmismatch compenstion (0x23 enable)
    {0x10, 0x00},  // AGC Table 0
    {0x11, 0x78},  // decrease AGC lock time
    {0x10, 0x01},  // AGC Table 1
    {0x11, 0x99},  // decrease AGC lock time
    //{0x10, 0x02},  // AGC Table 2
    //{0x11, 0x30},
    //{0x10, 0x05},  // AGC Table 5
    //{0x11, 0x02}, 
    //{0x10, 0x07},  // AGC Table 7
    //{0x11, 0xed},
    //{0x10, 0x08},  // AGC Table 8
    //{0x11, 0xa8},
    //{0x10, 0x09},  // AGC Table 9
    //{0x11, 0x72},
	{0x10, 0x06},  // AGC Table 6
    {0x11, 0x00},
    {0x10, 0x0a},  // AGC Table a
    {0x11, 0xc4},
    {0x10, 0x0b},  // AGC Table b
    {0x11, 0xb8},
    {0x10, 0x0c},  // AGC Table c
    {0x11, 0x0b},
    {0x10, 0x0d},  // AGC Table d
    {0x11, 0x2b},
    {0x10, 0x0e},  // AGC Table e
    {0x11, 0xf0},  // update LNA gain level transition point
    {0x10, 0x0f},  // AGC Table f
    {0x11, 0x04},
    {0x10, 0x10},  // AGC Table 10
    {0x11, 0x6f},  // update LNA gain level transition point

};

struct bb_regs bb_init_tbl_r5c_j [] = {
    {0x00, 0x01},  // reset BB to default value
	{0x01, 0x80},  // toggle RXHP and old bb_LMAC interface
	{0xF4, 0x00},
#if !defined(CONFIG_FPGA)
    {0x02, 0x31},  // TX IQ swap
#else
    {0x02, 0x30},  // TX IQ swap
#endif
    {0xf2, 0x80},  // DAC CLK 40MHz
    {0xf3, 0x22},  // ADC CLK 40MHz
    {0x54, 0x2b},  // disable on-fly IQmismatch compenstion (0x23 enable)
    {0x05, 0x80},  // To pass TX mask of B mode

	{0x10, 0x00},	// AGC table
	{0x11, 0x78},
	{0x10, 0x01},
	{0x11, 0x9c},
	{0x10, 0x02},
	{0x11, 0x30},
	{0x10, 0x03},
	{0x11, 0x7d},
	{0x10, 0x04},
	{0x11, 0x30},
	{0x10, 0x05},
	{0x11, 0x42},
	{0x10, 0x06},
	{0x11, 0x00},
	{0x10, 0x07},
	{0x11, 0x2c},
	{0x10, 0x08},
	{0x11, 0x46},
	{0x10, 0x09},
	{0x11, 0x72},	/* Change CCA for tx easy  */
	//{0x11, 0xb2}, /* David asks to change CCA threshold to pass CE */
	{0x10, 0x0a},
	{0x11, 0xe4},
	{0x10, 0x0b},
	{0x11, 0xb4},
	{0x10, 0x0c},
	{0x11, 0x0a},
	{0x10, 0x0d},
	{0x11, 0x2b},
	{0x10, 0x0e},
	{0x11, 0x30},
	{0x10, 0x0f},
	{0x11, 0x04},
	{0x10, 0x10},
	{0x11, 0x5b},
};

struct bb_rev_t {
	u8 rf_rev;
	u8 rev;
	u8 size;
	struct bb_regs *tbl;
}bb_revsion_tbl[] = {
	{RFCHIP_IP301_E, 0x5C, sizeof(bb_init_tbl_r5c_e)/sizeof(struct bb_regs), bb_init_tbl_r5c_e},
	{RFCHIP_MT301_A0, 0x5C, sizeof(bb_init_tbl_r5c_j)/sizeof(struct bb_regs), bb_init_tbl_r5c_j},
	{RFCHIP_MT301_A1, 0x5C, sizeof(bb_init_tbl_r5c_j)/sizeof(struct bb_regs), bb_init_tbl_r5c_j},
	{RFCHIP_NO_RF, 0x5c, sizeof(bb_init_tbl_r5c_j)/sizeof(struct bb_regs), bb_init_tbl_r5c_j},
	{RFCHIP_MT301_A1, 0x60, sizeof(bb_init_tbl_r5c_j)/sizeof(struct bb_regs), bb_init_tbl_r5c_j},
	{RFCHIP_MT301_A1, 0x80, sizeof(bb_init_tbl_r5c_j)/sizeof(struct bb_regs), bb_init_tbl_r5c_j},
	{0, 0, 0, 0},
};

struct bb_rev_t bb_init_table[] = {
	//{RFCHIP_IP301_G, 0xff, sizeof(bb_init_tbl_dual)/sizeof(struct bb_regs), bb_init_tbl_dual},
	{0, 0, 0, 0},
};

struct bb_regs rfc_tbl[MAX_RFC_REG_NUM];

u8 bb_register_read(int bb_reg)
{
    u8 value=0; //avoid compiler warning
    u32 data;

#ifdef BB_SPI_DOUBLE_CONFIRM
	u8 tmp, i=0;
	do{
		tmp = value;
#endif
		//WLA_DBG(WLADEBUG, "In bb_register_read(), value = 0x%x, tmp = 0x%x\n", value, tmp);
		data = ( (0x01UL << 17)              /* start SPI */
				| ((bb_reg & 0xff) << 8)    /* register address */
				);

		MACREG_WRITE32(BB_SPI_BASE, data);

		while( (MACREG_READ32(BB_SPI_BASE) & BB_SPI_DONE) != BB_SPI_DONE );

		data = MACREG_READ32(BB_SPI_BASE);

		value = (data & 0xff);

#ifdef BB_SPI_DOUBLE_CONFIRM
		/* Reg.76 will be changed after been read. */
		if(bb_reg == 0x76)
			break;

		if(i++ > 10)
		{
			WLA_DBG(WLADEBUG, "Fail in bb_register_read(0x%x), value = 0x%x, tmp = 0x%x\n", bb_reg, value, tmp);
			break;
		}
	} while(value != tmp);
#endif

    return value;
}

void bb_register_write(int bb_reg, u8 value)
{
    u32 data;

    data = ((0x01UL << 17)              /* start SPI */
            |  (0x01UL << 16)           /* SPI write */
            | ((bb_reg & 0xff) << 8)    /* register address */
            |  value );                 /* data */

#ifdef BB_SPI_DOUBLE_CONFIRM
	u8 tmp=0, i=0;
	//char buf[16];
	do{
#endif
		MACREG_WRITE32(BB_SPI_BASE, data);

		while( (MACREG_READ32(BB_SPI_BASE) & BB_SPI_DONE) != BB_SPI_DONE );
#ifdef BB_SPI_DOUBLE_CONFIRM
		if(bb_reg == 0)
			break;
		if((bb_reg == 0XA2) && (rf_iface->type == RF_CTRL_W6))
			break;
		
		tmp = bb_register_read(bb_reg);
		
		if(i++> 10)
		{
			WLA_DBG(WLADEBUG, "Fail in bb_register_write ");
			WLA_DBG(WLADEBUG, "the write value = 0x%x\n", value);
			WLA_DBG(WLADEBUG, "the bb_register_read(0x%x) = 0x%x\n", bb_reg, tmp);
			break;
			//gets(buf);
		}
	} while(tmp != value);
#endif
	
    return;
}

#define BB_TXPE_PIN     1
#define BB_RXPE_PIN     2

#define BB_TXPE         (0x1UL << BB_TXPE_PIN)
#define BB_RXPE         (0x1UL << BB_RXPE_PIN)

void bb_control_init(void)
{
    static int init = 0;

    if(init == 0) 
    {
        GPREG(GPSEL) |= BB_TXPE | BB_RXPE;
        GPREG(GPDIR) |= BB_TXPE | BB_RXPE;
        GPREG(GPCLR) = BB_TXPE | BB_RXPE;
        init = 1;
    }
}


void bb_txpe_off(void)
{
    bb_control_init();
    GPREG(GPCLR) = BB_TXPE;
}

void bb_rxpe_off(void)
{
    bb_control_init();
    GPREG(GPCLR) = BB_RXPE;
}

void bb_rxpe_on(void)
{
    bb_txpe_off();
    GPREG(GPSET) = BB_RXPE;
}

void bb_txpe_on(void)
{
    bb_rxpe_off();
    GPREG(GPSET) = BB_TXPE;
}

void bb_old_set_40mhz_mode(int channel_type)
{
	/* for 80Mhz mode, reverse ADC clock to meet timing*/
	/* reg.0x2 bit.6 = ADC clock invert */
	bb_register_write(0x02, (bb_register_read(0x02) | 0x40)); /* new ADC/DAC board */

	bb_register_write(0x12, 0x1);		
	bb_register_write(0x13, 0x77);	/* lp2sig fine tuned */	
	bb_register_write(0x12, 0x4);		
	bb_register_write(0x13, 0x40);	/* disable tone-erased around DC for carrier offset */	

	if(!(my_rf_dev->chip_ver == RFCHIP_MT301_A0) && !(my_rf_dev->chip_ver == RFCHIP_MT301_A1))
	{
		bb_register_write(0x10, 0x07);
		bb_register_write(0x11, 0x27);
	}

    if(channel_type == HT40_PLUS)
        bb_register_write(0x01, ( (bb_register_read(0x01) & (~(0x03))) | 0x03));    // HT40+
    else if(channel_type == HT40_MINUS)
        bb_register_write(0x01, ( (bb_register_read(0x01) & (~(0x03))) | 0x02));    // HT40-

    bb_register_write(0xF2, (bb_register_read(0xF2) | 0x02));    // Set BR_F2[1] =1 (DAC 80M enable)
    bb_register_write(0xF3, (bb_register_read(0xF3) | 0x40));    // Set BR_F3[6] = 1 (ADC 80M enable)

	if(!(my_rf_dev->chip_ver == RFCHIP_NO_RF))
	{
		/* config the calibrated rfc related register */
		config_rfc_parm_new(0x101);	// 0x101 : accept manual setting & 40MHz Mode
	}
	else
	{
		bb_register_write(0x12, 0x01); //Cheetah IC
		bb_register_write(0x13, 0x50); //Cheetah IC /* turn off notch filter when no RF*/
	}
}

void bb_old_set_20mhz_mode(void)
{
	/* reg.0x2 bit.6 = ADC clock invert */
	bb_register_write(0x02, (bb_register_read(0x02) & 0xBF)); /* new ADC/DAC board */
			
	bb_register_write(0x12, 0x1);		
	bb_register_write(0x13, 0x57);
	bb_register_write(0x12, 0x4);		
	bb_register_write(0x13, 0x0);

	if(!(my_rf_dev->chip_ver == RFCHIP_MT301_A0) && !(my_rf_dev->chip_ver == RFCHIP_MT301_A1))
	{
		bb_register_write(0x10, 0x07);
		bb_register_write(0x11, 0x29);
	}

    bb_register_write(0x01, (bb_register_read(0x01) & (~(0x03))));
    bb_register_write(0xF2, (bb_register_read(0xF2) & (~(0x02))));
    bb_register_write(0xF3, (bb_register_read(0xF3) & (~(0x40))));

	if(!(my_rf_dev->chip_ver == RFCHIP_NO_RF))
	{
		/* config the calibrated rfc related register */
		config_rfc_parm_new(0x100);	// 0x100 : accept manual setting & 20MHz Mode
	}
	else
	{
		bb_register_write(0x12, 0x01); //Cheetah IC
		bb_register_write(0x13, 0x50); //Cheetah IC /* turn off notch filter when no RF*/
	}
}

u8 bb_rssi_decode(u8 val)
{
	s8 lna_gain_b[4] = {0, 12, 15, 21};
	s32 lna_gain, total_gain, rssi_gdb, max_gdb;

	/* RSSI field{1'b0, 2'b_LNA, 5'b_VGA} */
    lna_gain = lna_gain_b[val >> 5]; 
	max_gdb = lna_gain_b[3] +31;
	
	/* total gain = LNA gain + VGA gain */
	total_gain = lna_gain + (val & 0x1f);
	/* reverse gain for signal strength */
	rssi_gdb = max_gdb - total_gain;
//WLA_DBG(WLADEBUG, "total_gain=%d, rssi_gdb=%d\n", total_gain, rssi_gdb);
	/* 1gdb =2dB */
	return (rssi_gdb*2);
}

u8 bb_rssi_decode1(u8 val)
{
	s8 lna_gain_b[4] = {0, 12, 15, 21};
	s32 lna_gain, total_gain;

	/* RSSI field{1'b0, 2'b_LNA, 5'b_VGA} */
    lna_gain = lna_gain_b[val >> 5]; 
	
	/* total gain = LNA gain + VGA gain */
	total_gain = lna_gain + (val & 0x1f);
	return (total_gain*2);
}

void bb_dump_adc_dc(void)
{
	int i;
	for(i=0x40; i<0x45; i++)
	{
		WLA_DBG(WLADEBUG, "REG%x=%x,", i, bb_register_read(i));
	}
	WLA_DBG(WLADEBUG, "\n");	
}

void bb_reset(void)
{
#if defined(CONFIG_FPGA)
	u32 hw_mode;
#endif
	u32 rf_ctrl_type=0; 

	/* Hardware reset BB */
	MACREG_UPDATE32(HW_RESET, 0, HW_RESET_BB_PAUSE|HW_RESET_BB);
    MACREG_UPDATE32(HW_RESET, -1, HW_RESET_BB);
	//WLA_DBG(WLADEBUG, "RESET BB .. OK\n");

	//MACREG_WRITE32(HW_RESET, 0xffffe0ff);

#if defined(CONFIG_FPGA)
	/* 	$param_hw_mode: 
		bit.0 == 1 : external bb mode
		bit.0 == 0 : internal bb mode */
	hw_mode = WLA_CDB_GET($parm_hw_mode, 0);
	if(hw_mode & 0x1)
	{
		/* Setup External BB Mode  */
		MACREG_WRITE32(FUNC_MODE, 0x90000002UL);
		/* Reduce BB SPI clock rate on ASIC with external BB */
		MACREG_WRITE32(BB_SPI_CLK_DIV, 0x20); 
	}
#endif
	/* reset BB to default value */
	bb_register_write(0x0, 0x1);

	rf_ctrl_type = rf_interface_check();
	WLA_DBG(WLADEBUG, "rf_interface_check return %x\n", rf_ctrl_type);

#if 0
	bb_init();

	if(info->bandwidth_type == BW20_ONLY)
		bb_set_20mhz_mode();
	else	
		bb_set_40mhz_mode(info->bandwidth_type);
#endif
}



void bb_old_init(void)
{
	u32 j;
	u8 rev;
	struct bb_regs *init_tbl = 0;
	struct bb_rev_t *bb_rev_tbl = &bb_revsion_tbl[0];

	WLA_DBG(WLADEBUG, "============================ BB init =============================\n");

#ifdef CONFIG_BB_TEST_EXTERNAL_MODULE
	/* only for Jerry, for external bb mode */
	/* Speed down BB SPI clock */
	*(int *)0xaf0038c4 = 0x20;
#endif

	/* check BB revision */
	rev = bb_register_read(0x00);

	do
	{
		if(bb_rev_tbl->rev == 0)
			break;
		/* RF revision matched ? */
		if(my_rf_dev->chip_ver != bb_rev_tbl->rf_rev)
			continue;
		/* BB revision matched */
		if(bb_rev_tbl->rev != rev)
			continue;
		
		init_tbl = bb_rev_tbl->tbl;
		for(j = 0; j < bb_rev_tbl->size; j++)
		{
			bb_register_write(init_tbl[j].num, init_tbl[j].val);
		}
		
		if(rf_if->type != RF_CTRL_UNKNOW)
			bb_register_write(0x01, rf_if->reg01);

		break;
	}while(bb_rev_tbl += 1);
	
	if(init_tbl == 0)
		WLA_DBG(WLADEBUG, "No suppored RF[%x] and BB version[%x]?,", my_rf_dev->chip_ver, bb_rev_tbl->rev);
		 
}

void bb_register_init(u8 bb_rev, u8 rf_rev)
{
	struct bb_regs *init_tbl = 0;
	struct bb_rev_t *bb_tbl = &bb_init_table[0];
	int j=0;

	WLA_DBG(WLADEBUG, "========================== %s ===========================\n", __FUNCTION__);
	
	do
	{
		if(bb_tbl->rev == 0)
			break;
		/* RF revision matched ? */
		if(rf_rev && (rf_rev != bb_tbl->rf_rev))
			continue;
		/* BB revision matched */
		if(bb_rev && (bb_tbl->rev != bb_rev))
			continue;
		
		init_tbl = bb_tbl->tbl;
		for(j = 0; j < bb_tbl->size; j++)
		{
			bb_register_write(init_tbl[j].num, init_tbl[j].val);
		}
		break;
	}while(bb_tbl += 1);

}

void bb_init(void)
{
	WLA_DBG(WLADEBUG, "========================== %s ===========================\n", __FUNCTION__);
	bb_old_init();
	my_bb_dev->set_20mhz_mode = bb_old_set_20mhz_mode;
	my_bb_dev->set_40mhz_mode = bb_old_set_40mhz_mode;
}

void bb_rx_counter_ctrl(u32 op)
{
	switch(op)
	{
		case BB_RX_CNT_RESET:
			bb_register_write(0x80, 0xC0);
			break;
		case BB_RX_CNT_ENABLE:
			bb_register_write(0x80, 0x80);
			break;
		case BB_RX_CNT_DISABLE:
			bb_register_write(0x80, 0x00);
			break;
	}
}

u32 bb_rx_counter_read(u8 addr)
{
	/***** 
		Just for active counter & busy counter now.
		If add new counter read action, the progress should be modified.
	 *****/

	u8 i;
	u32 val=0;

	for(i=0; i<3; i++)
	{
		/* set the counter address to BB Reg.0x14 */
		bb_register_write(0x14, addr + i);
		/* read counter value from BB Reg.0x15 */
		val = (val << 8) | bb_register_read(0x15);
	}

	return val;
}

u8 bb_read_noise_floor(void)
{
	u8 val;

	val = bb_register_read(0x1E);

	return val;
}
