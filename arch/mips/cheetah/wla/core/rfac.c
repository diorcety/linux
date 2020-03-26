/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfac.c
*   \brief
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <ip301.h>
#include <math.h>
#include <rf.h>
#include <rfc_comm.h>
#include <rfac.h>


#ifdef CONFIG_RFAC

#if defined(RFAC_ANALYSIS)
#define MAX_RFAC_ITERATIONS  1000
static int rfac_test_case_no __attribute__ ((section(".rfc_data")));
static u8 __vco_curves[MAX_RFAC_ITERATIONS][10] __attribute__ ((section(".rfc_data")));
static u32 __i_path_txlo_result[MAX_RFAC_ITERATIONS] __attribute__ ((section(".rfc_data")));
static u32 __q_path_txlo_result[MAX_RFAC_ITERATIONS] __attribute__ ((section(".rfc_data")));
static u32 __i_path_rxdc_result[MAX_RFAC_ITERATIONS][RF_DC_LEVEL] __attribute__ ((section(".rfc_data")));
static u32 __q_path_rxdc_result[MAX_RFAC_ITERATIONS][RF_DC_LEVEL] __attribute__ ((section(".rfc_data")));
double __tx_filter_chk[MAX_RFAC_ITERATIONS][4] __attribute__ ((section(".rfc_data")));       // 0:i_f0, 1:i_f1, 2:q_f0, 3:q_f1
double __rx_filter_chk[MAX_RFAC_ITERATIONS][4] __attribute__ ((section(".rfc_data")));       // 0:i_f0, 1:i_f1, 2:q_f0, 3:q_f1
#endif

int vco_calibration(int internal_ldo)
{
	// VCO auto-calibration
	// Relevant registers: reg6, reg0, reg1, reg12, reg4, reg5
	u32 reg12, reg5;
	u32 i;

	u8 lastval;
	u8 vco_curve[10];

    RFC_DBG("VCO auto-calibration: using %s\n", internal_ldo ? "Internal LDO" : "External LDO");

    // setup internal LDO on/off
    if(internal_ldo)
    {
        ip301_spi_write(0x6, 0x3698);
        ndelay(2000);
    }
    else
    {
        ip301_spi_write(0x6, 0x369f);
        ndelay(2000);
    }

    ip301_spi_write(0x11, 0x80b);
    ndelay(200);

    // setup sub-blocks on/off
    ip301_spi_write(0x0, 0x3ffef);      // VCO on, TX on, RX on
    ndelay(2000);

    // disable then enable VCO auto-calibration
    ip301_spi_write(0x1, 0x000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x200);
	ndelay(20000000);

    reg12 = ip301_spi_read(0x12);
    if(0x3!=(0x3 & reg12))       // check reg12[0] and reg12[1]
    {
        RFC_DBG("VCO auto-calibration: reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    // read 10-curve register value
    for(i=0;i<5;i++)
    {
        //ip301_spi_update(0x4, (i << 11), 0x3800);
   		ip301_spi_write(0x4, (ip301_spi_read(0x4) & ~0x3800) | (i << 11));
 
        reg5 = ip301_spi_read(0x5);

        if(0==(reg5 & 0x20000))
        {
            RFC_DBG("VCO auto-calibration: reg5 BIT17 not ok (0x%x), i = %d\n", reg5, i);
            return -2;
        }

        if(0==(reg5 & 0x00100))
        {
            RFC_DBG("VCO auto-calibration: reg5 BIT8 not ok (0x%x), i = %d\n", reg5, i);
            return -3;
        }

        vco_curve[i*2] = ((reg5 >> 9) & 0xff);
        vco_curve[i*2 + 1] = (reg5 & 0xff);
    }

    RFC_DBG("VCO auto-calibration: done, ");
    for(i=0;i<10;i++)
    {
        RFC_DBG(" %d", vco_curve[i]);
    }
    RFC_DBG("\n\n");

#if defined(RFAC_ANALYSIS)
    for(i=0;i<10;i++)
    {
        __vco_curves[rfac_test_case_no][i] = vco_curve[i];
    }
#endif

    // check the results
    lastval = 0xFF;
    for(i=0;i<10;i++)
    {
        if(lastval < vco_curve[i])
        {
            RFC_DBG("VCO auto-calibration: possibly wrong result\n");
            return -4;
        }
        else
        {
            lastval = vco_curve[i];
        }
    }

    ip301_spi_write(0x1, 0x200);
    return 0;
}

int tx_rx_filter_calibration(u8 bw, u8 tx_filter_code, u8 rx_filter_code)
{
	/* Tx Filter Code Setting : Manually
	   Rx Filter Code Setting : Automatic */

    u32 reg12, reg13, regf, reg8, reg1;
	u32 count, tune_rxfil;

    
	RFC_DBG("%s(): bw = %d, tx filter code = %x, rx filter code = 0x%x\n",
			__FUNCTION__, bw, tx_filter_code, rx_filter_code);

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("%s(): reg13 0x%x before calibration\n", __FUNCTION__, reg13);

	/* ============== RX Auto Calibration ============== */
	reg8 = ip301_spi_read(0x8);
	if(bw)	/* 40MHz */
    	ip301_spi_write(0x8, 0xE000 | ((rx_filter_code & 0x7) << 7) | (reg8 & 0x1C7F));
	else	/* 20MHz */
    	ip301_spi_write(0x8, 0x2000 | ((rx_filter_code & 0x7) << 7) | (reg8 & 0x1C7F));
    ndelay(2000);

    // Start calibration
	reg1 = ip301_spi_read(0x1);
	reg1 = reg1 & ~(0x30);
    ip301_spi_write(0x1, reg1);
    ndelay(50000);
    ip301_spi_write(0x1, reg1 | 0x020);
    ndelay(65000);

    // Check test ready
    reg12 = ip301_spi_read(0x12);
    if(0x4!=(0x4 & reg12))       // check reg12[2], (should be "1" and means calibration done)
    {
        RFC_DBG("%s(): reg12 not ready (0x%x)\n", __FUNCTION__, reg12);
        return -1; 
    }

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("%s(): reg13 0x%x after rx calibration\n", __FUNCTION__, reg13);

	tune_rxfil = (reg13 & 0x3fe00) >> 9;
	for(count=1; count <= 8; count++)
	{
		if((1 << count) & tune_rxfil)
			break;
	}

	if(count > 8)
	{
        RFC_DBG("%s(): filter_vco_coeff is wrong(0x%x)\n", __FUNCTION__, tune_rxfil);
        return -1;
    }

	tune_rxfil = (count-1)*2 + (tune_rxfil & 0x1);

	reg8 = ((tx_filter_code & 0x7) << 4) | tune_rxfil;
	if(bw)	/* 40MHz */
		reg8 = reg8 | 0xc000;

	ip301_spi_write(0x8, reg8);
    ndelay(2000);
	reg8 = ip301_spi_read(0x8);

	reg1 = ip301_spi_read(0x1);
	reg1 = (reg1 & 0xffffffcf) | 0x10;
	ip301_spi_write(0x1, reg1);
    ndelay(2000);

	regf = ip301_spi_read(0xf);
	regf = regf | 0x3;
	ip301_spi_write(0xf, regf);

    RFC_DBG("%s(): reg8 0x%x after tx & rx reg_tune_fill_bypass_value setup\n\n", __FUNCTION__, reg8);
    RFC_DBG("%s(): reg13 0x%x after tx & rx reg_tune_fill_bypass_value setup\n\n", __FUNCTION__, reg13);

	return 0;
}

int rx_filter_calibration(u8 bw, u8 filter_code)
{
#if 1
    // RX filter auto-calibration (per filter bandwidth)
    // Relevant registers: reg8, reg0, reg1, reg12, reg13
    // Input: bw => 0: 20MHz Mode, 1: 40MHz Mode
	//		  filter_code => RX filter bandwidth selection (calibration for on BW only)
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13, reg8, reg0;

    RFC_DBG("RX filter auto-calibration: bw = %d, filter_code = %x\n", bw, filter_code);

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RX filter auto-calibration: reg13 0x%x before calibration\n", reg13);

    // select RX and setup RX filter bandwidth, reg8[9:7]= 111 max BW, 000 min BW
	// reg8[15] => 0: 20MHz Mode, 1: 40MHz Mode
	reg8 = ip301_spi_read(0x8);
	if(bw)	/* 40MHz */
    	ip301_spi_write(0x8, 0xB000 | ((filter_code & 0x7) << 7) | (reg8 & 0x7C7F));
	else	/* 20MHz */
    	ip301_spi_write(0x8, 0x3000 | ((filter_code & 0x7) << 7) | (reg8 & 0x7C7F));
    ndelay(2000);

    // setup sub-blocks on/off
	reg0 = ip301_spi_read(0x0);
    ip301_spi_write(0x0, 0x3ffef);      // VCO on, TX on, RX on
    ndelay(2000);

    // Start calibration
    ip301_spi_write(0x1, 0x000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x020);
    ndelay(65000);

    // Check test ready
    reg12 = ip301_spi_read(0x12);
    if(0x4!=(0x4 & reg12))       // check reg12[2], (should be "1" and means calibration done)
    {
        RFC_DBG("RX filter auto-calibration: reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RX filter auto-calibration: done, reg13 0x%x after calibration\n\n", reg13);

    ip301_spi_write(0x1, 0x200);
	/* restore the reg 0 value, for kepping the rfc calibration value. By KC. */
    ip301_spi_write(0x0, reg0);
#endif
    return 0;
}

int tx_filter_calibration(u8 bw, u8 filter_code)
{
#if 1
    // Relevant registers: reg8, reg0, reg1, reg12, reg13
    // Input:  bw => 0: 20MHz Mode, 1: 40MHz Mode
	//		   filter_code => TX filter bandwidth selection (calibration for on BW only)
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13, reg8, reg0;

    RFC_DBG("TX filter auto-calibration: bw = %d, filter_code = %x\n", bw, filter_code);

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("TX filter auto-calibration: reg13 0x%x before calibration\n", reg13);

    // select TX and setup TX filter bandwidth, reg8[12:10]= 111 max BW, 000 min BW
	// reg8[14] => 0: 20MHz Mode, 1: 40MHz Mode
	reg8 = ip301_spi_read(0x8);
	if(bw)	/* 40MHz */
    	ip301_spi_write(0x8, 0x4200 | ((filter_code & 0x7) << 10) | (reg8 & 0x83FF));
	else	/* 20MHz */
    	ip301_spi_write(0x8, 0x0200 | ((filter_code & 0x7) << 10) | (reg8 & 0x83FF));
    ndelay(2000);

    // setup sub-blocks on/off
	reg0 = ip301_spi_read(0x0);
    ip301_spi_write(0x0, 0x3ffef);      // VCO on, TX on, RX on
    ndelay(2000);

    // Start calibration
    ip301_spi_write(0x1, 0x000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x020);
    ndelay(65000);

    // Check test ready
    reg12 = ip301_spi_read(0x12);
    if(0x4!=(0x4 & reg12))       // check reg12[2], (should be "1" and means calibration done)
    {
        RFC_DBG("TX filter auto-calibration: reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("TX filter auto-calibration: done, reg13 0x%x after calibration\n\n", reg13);

    ip301_spi_write(0x1, 0x200);
	/* restore the reg 0 value, for kepping the rfc calibration value. By KC */
    ip301_spi_write(0x0, reg0);
#endif
    return 0;
}

#if 1 // for defined(RFAC_ANALYSIS)
#define TXLO_CALIBRATION_MAX_RETRY  30
int txlo_calibration_max_retry = TXLO_CALIBRATION_MAX_RETRY;
#endif

int txlo_calibration(int channel)
{
    // Relevant registers: reg2, reg0, reg7, reg1, reg12
    // Input: Select an arbitrary channel
    // Assume VCO has been selected and calibrated

    u32 reg12;
    u32 i_path_txlo_result, q_path_txlo_result;
    int retry = 0;

    u32 i_path_txlo_result_prev = 0xffff;
    u32 q_path_txlo_result_prev = 0xffff;

Retry:

    RFC_DBG("TXLO auto-calibration: channel %d\n", channel);

    reg12 = ip301_spi_read(0x12);
    RFC_DBG("TXLO auto-calibration: reg12 0x%x before calibration\n", reg12);

    // Set up DAC: DAC turn on; DAC = 0;
    bb_register_write(0x1c, 0xee);          // Set BB register 0x1C to the value of 0xEE

    // select channel
    ip301_set_channel(0, channel, 0);
	ndelay(30000);      // Wait 30us for PLL locking time

    // setup sub-blocks on/off
    ip301_spi_write(0x0, 0x3fe00);      // VCO on, TX on, RX off
    ndelay(2000);

    // TX VGA setting, VGA_vc<6:0>=1111111
    ip301_spi_write(0x7, 0xffc0);
    ndelay(2000);

    // I path calibration, reg1[6]=1 and reg1[8] = 0
    ip301_spi_write(0x1, 0x000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x040);
    ndelay(20000);

    // Check I-path test ready and save I-path result
    reg12 = ip301_spi_read(0x12);
    if(0x10!=(0x10 & reg12))       // check reg12[4], (should be "1" and means calibration done)
    {
        RFC_DBG("TXLO auto-calibration: I-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    i_path_txlo_result = (reg12 >> 5) & 0xff;       // I_path_TX_LO_result = read reg12[12:5]

    // Q path calibration, reg1[6]=1 and reg1[8] = 1
    ip301_spi_write(0x1, 0x000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x140);
    ndelay(20000);

    // Check Q-path test ready and save Q-path result
    reg12 = ip301_spi_read(0x12);
    if(0x10!=(0x10 & reg12))       // check reg12[4], (should be "1" and means calibration done)
    {
        RFC_DBG("TXLO auto-calibration: Q-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    q_path_txlo_result = (reg12 >> 5) & 0xff;       // Q_path_TX_LO_result = read reg12[12:5]

    RFC_DBG("TXLO auto-calibration: done, i_path_txlo_result 0x%x, q_path_txlo_result 0x%x\n\n", i_path_txlo_result, q_path_txlo_result);

    ip301_spi_write(0x1, 0x200);

#if defined(RFAC_ANALYSIS)
    __i_path_txlo_result[rfac_test_case_no] = i_path_txlo_result;
    __q_path_txlo_result[rfac_test_case_no] = q_path_txlo_result;
#endif

    if(txlo_calibration_max_retry)
    {
		s32 delta_i = (s32)i_path_txlo_result_prev - (s32)i_path_txlo_result;
		s32 delta_q = (s32)q_path_txlo_result_prev - (s32)q_path_txlo_result;
		if((-1 > delta_i) || (delta_i > 1) || (-1 > delta_q) || (delta_q > 1))
        {
            if(retry < txlo_calibration_max_retry)
            {
                i_path_txlo_result_prev = i_path_txlo_result;
                q_path_txlo_result_prev = q_path_txlo_result;
                RFC_DBG("TXLO auto-calibration: retry %d\n", retry);
                retry++;
                goto Retry;
            }
            else
            {
                RFC_DBG("TXLO auto-calibration: failed\n");
                return -1;
            }
        }
    }

    return 0;
}

int rxdc_calibration_old(u8 gain)
{
    // Relevant registers: reg7, reg0, reg9, reg1, reg12, reg13
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13;
    u32 i_path_rxdc_result, q_path_rxdc_result;

    RFC_DBG("RXDC auto-calibration: %s gain mode\n", (gain==0) ? "high" : ((gain==1) ? "middle" : "ultra-low"));

    // Read reg13[8:0]
    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RXDC auto-calibration: reg13 0x%x before calibration\n", reg13);

    // setup LNA gain mode
    if(gain==0)
        ip301_spi_write(0x7, 0x38);         // high gain; gpk=111
    else if(gain==1)
        ip301_spi_write(0x7, 0x3c);         // middle gain
    else
        ip301_spi_write(0x7, 0x3f);         // ultra-low gain
    ndelay(1000);

    // VCO on, RX on, RX_SVLP off, TX off
    ip301_spi_write(0x0, 0x3e0ff);
    ndelay(2000);

    // I path, reg9[17:16]=01 for I path and set RGCL<5:0>=111111
    ip301_spi_write(0x9, 0x101fc);
    ndelay(200000);

    // Set reg1[3:2]=01 to disable rxdc_force but start tuning
    ip301_spi_write(0x1, 0x000);
    ndelay(200000);
    ip301_spi_write(0x1, 0x004);
	ndelay(12000000);

    // Check I-path test ready and save I-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: I-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    i_path_rxdc_result = reg13 & 0x01ff;      // Read reg13[8:0] after calibration

    // Q path, reg9[17:16]=11 for I path and set RGCL<5:0>=111111 
    ip301_spi_write(0x9, 0x301fc);
    ndelay(200000);

    // Start Q-calibration
    ip301_spi_write(0x1, 0x000);
    ndelay(200000);
    ip301_spi_write(0x1, 0x004);
	ndelay(12000000);

    // Check Q-path test ready and save Q-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: Q-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);           // 	Read reg13[8:0] after calibration
    q_path_rxdc_result = reg13 & 0x01ff;

    // paste the I/Q result to regc/regb/rega
    if(gain==0)
        ip301_spi_write(0xc, (q_path_rxdc_result << 9) | i_path_rxdc_result);	// high gain; regc
    else if(gain==1)
        ip301_spi_write(0xb, (q_path_rxdc_result << 9) | i_path_rxdc_result);	// middle gain; regb
    else
        ip301_spi_write(0xa, (q_path_rxdc_result << 9) | i_path_rxdc_result);	// ultra-low gain; rega

    // Disable the calibration
    ip301_spi_write(0x1, 0x000);
    ndelay(200000);

    RFC_DBG("RXDC auto-calibration: done, i_path_rxdc_result 0x%x, q_path_rxdc_result 0x%x\n\n", i_path_rxdc_result, q_path_rxdc_result);

    ip301_spi_write(0x1, 0x200);

#if defined(RFAC_ANALYSIS)
    __i_path_rxdc_result[rfac_test_case_no][gain] = i_path_rxdc_result;
    __q_path_rxdc_result[rfac_test_case_no][gain] = q_path_rxdc_result;
#endif

    return 0;
}

#define RXDC_CALIBRATION_MAX_RETRY  30
int rxdc_calibration_max_retry = RXDC_CALIBRATION_MAX_RETRY;

int rxdc_calibration(u8 gain)
{
    // Relevant registers: reg7, reg0, reg9, reg1, reg12, reg13
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13;
    u32 i_path_rxdc_result, q_path_rxdc_result;
	u32 retry = 0;
    u32 i_path_rxdc_result_prev = 0xffff;
    u32 q_path_rxdc_result_prev = 0xffff;

    RFC_DBG("RXDC auto-calibration: %s gain mode\n", (gain==0) ? "high" : ((gain==1) ? "middle" : "ultra-low"));

rxdc_retry:

    // Read reg13[8:0]
    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RXDC auto-calibration: reg13 0x%x before calibration\n", reg13);

    // setup LNA gain mode
    if(gain==0)
        ip301_spi_write(0x7, 0x38);         // high gain; gpk=111
    else if(gain==1)
        ip301_spi_write(0x7, 0x3c);         // middle gain
	else if(gain==2)
        ip301_spi_write(0x7, 0x3d);         // low gain
    else
        ip301_spi_write(0x7, 0x3f);         // ultra-low gain
    ndelay(1000);

    // VCO on, RX on, RX_SVLP off, TX off
    ip301_spi_write(0x0, 0x3e0ff);
    ndelay(2000);

    // I path, reg9[17:16]=01 for I path and set RGCL<5:0>=111111
    ip301_spi_write(0x9, 0x101fc);
    ndelay(200000);

    // Set reg1[3:2]=01 to disable rxdc_force but start tuning
    ip301_spi_write(0x1, 0x0);
    ndelay(200000);
    ip301_spi_write(0x1, 0x004);
	ndelay(12000000);

    // Check I-path test ready and save I-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: I-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    i_path_rxdc_result = reg13 & 0x01ff;      // Read reg13[8:0] after calibration

    // Q path, reg9[17:16]=11 for I path and set RGCL<5:0>=111111 
    ip301_spi_write(0x9, 0x301fc);
    ndelay(200000);

    // Start Q-calibration
    ip301_spi_write(0x1, 0x0);
    ndelay(200000);
    ip301_spi_write(0x1, 0x004);
	ndelay(12000000);

    // Check Q-path test ready and save Q-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: Q-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);           // 	Read reg13[8:0] after calibration
    q_path_rxdc_result = reg13 & 0x01ff;

    // paste the I/Q result to regc/regb/rega
    if(gain==0)
        ip301_spi_write(0x14, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // high gain; regc
    else if(gain==1)
        ip301_spi_write(0xc, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // middle gain; regb
	else if(gain==2)
        ip301_spi_write(0xb, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // low gain; regb
    else
        ip301_spi_write(0xa, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // ultra-low gain; rega

    // Disable the calibration
    ip301_spi_write(0x1, 0x0);
    ndelay(200000);

    RFC_DBG("RXDC auto-calibration: done, i_path_rxdc_result 0x%x, q_path_rxdc_result 0x%x\n\n", i_path_rxdc_result, q_path_rxdc_result);

    ip301_spi_write(0x1, 0x200);

#if defined(RFAC_ANALYSIS)
    __i_path_rxdc_result[rfac_test_case_no][gain] = i_path_rxdc_result;
    __q_path_rxdc_result[rfac_test_case_no][gain] = q_path_rxdc_result;
#endif

    if(rxdc_calibration_max_retry)
    {
		s32 delta_i = (s32)i_path_rxdc_result_prev - (s32)i_path_rxdc_result;
		s32 delta_q = (s32)q_path_rxdc_result_prev - (s32)q_path_rxdc_result;
		if((-3 > delta_i) || (delta_i > 3) || (-3 > delta_q) || (delta_q > 3))
        {
            if(retry < rxdc_calibration_max_retry)
            {
                i_path_rxdc_result_prev = i_path_rxdc_result;
                q_path_rxdc_result_prev = q_path_rxdc_result;
                RFC_DBG("RXDC auto-calibration: retry %d\n", retry);
                retry++;
                goto rxdc_retry;
            }
            else
            {
                RFC_DBG("RXDC auto-calibration: failed\n");
                return -1;
            }
        }
    }

    return 0;
}

int rf_calibration(int channel, int internal_ldo)
{
    int ret = 0;
    int i;
	unsigned char buf[2];

    ip301_power_on();
   
	RFC_DBG("RF auto-calibration: rfchip=%d, %s, channel %d\n", my_rf_dev->chip_ver, internal_ldo ? "Internal LDO" : "External LDO", channel);

    ret = vco_calibration(internal_ldo);
    if(0>ret)
        goto Failed;

	ret = rx_filter_calibration(0, 4);
	if(0>ret)
		goto Failed;

	ret = tx_filter_calibration(0, 4);
	if(0>ret)
		goto Failed;
    
	ret = txlo_calibration(channel);
    if(0>ret)
        goto Failed;

	/* new RF add low gain calibration */
	for(i=0;i<4;i++)
	{
		ret = rxdc_calibration(i);
		if(0>ret)
			goto Failed;
	}

    RFC_DBG("@@ RF auto-calibration (RFAC): done.\n\n");
    return 0;

Failed:
	while(1)
	{
		RFC_DBG("RF auto-calibration (RFAC) Fail, Please seek assistance from Stan (#2629). Press c to continue.\n\n");
		WLA_GETS(buf);
		if(strncmp(buf, "c", 1) == 0)
			break;
	}
    return ret;
}

int mt301_vco_calibration(int internal_ldo)
{
	// VCO auto-calibration
	// Relevant registers: reg6, reg0, reg1, reg12, reg4, reg5
	u32 reg12, reg5;
	u32 i;

	u8 lastval;
	u8 vco_curve[10];

    RFC_DBG("VCO auto-calibration: using %s\n", internal_ldo ? "Internal LDO" : "External LDO");

	ip301_spi_write(0x15, 0x1005); //shutdown mode
	ip301_spi_write(0x1, 0x8000); // Internal VCO LDO
	ip301_spi_write(0x6, 0x10bd); // Master and DIG LDO control
	ip301_spi_write(0x7, 0x7c0); // 1.8V Slave LDO of PA power 
	ip301_spi_write(0x17, 0x308fe); // Slave LDO control 
	ip301_spi_write(0x3, 0x1284); //Internal loop filter
	ip301_spi_write(0x11, 0x2cb2); //internal loop filter
    ndelay(200);

    // setup sub-blocks on/off
    ip301_spi_write(0x0, 0x3ffef);      // VCO on, TX on, RX on
	ip301_spi_write(0x0, 0x3fe00);      // VCO on, TX on, RX off
    ndelay(2000);

    // disable then enable VCO auto-calibration
    ip301_spi_write(0x1, 0x8200);
	ndelay(20000000);

    reg12 = ip301_spi_read(0x12);
    if(0x3!=(0x3 & reg12))       // check reg12[0] and reg12[1]
    {
        RFC_DBG("VCO auto-calibration: reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    // read 10-curve register value
    for(i=0;i<5;i++)
    {
        //ip301_spi_update(0x4, (i << 11), 0x3800);
   		ip301_spi_write(0x4, (ip301_spi_read(0x4) & ~0x3800) | (i << 11));
 
        reg5 = ip301_spi_read(0x5);

        if(0==(reg5 & 0x20000))
        {
            RFC_DBG("VCO auto-calibration: reg5 BIT17 not ok (0x%x), i = %d\n", reg5, i);
            return -2;
        }

        if(0==(reg5 & 0x00100))
        {
            RFC_DBG("VCO auto-calibration: reg5 BIT8 not ok (0x%x), i = %d\n", reg5, i);
            return -3;
        }

        vco_curve[i*2] = ((reg5 >> 9) & 0xff);
        vco_curve[i*2 + 1] = (reg5 & 0xff);
    }

    RFC_DBG("VCO auto-calibration: done, ");
    for(i=0;i<10;i++)
    {
        RFC_DBG(" %d", vco_curve[i]);
    }
    RFC_DBG("\n\n");

#if defined(RFAC_ANALYSIS)
    for(i=0;i<10;i++)
    {
        __vco_curves[rfac_test_case_no][i] = vco_curve[i];
    }
#endif

    // check the results
    lastval = 0xFF;
    for(i=0;i<10;i++)
    {
        if(lastval < vco_curve[i])
        {
            RFC_DBG("VCO auto-calibration: possibly wrong result\n");
            return -4;
        }
        else
        {
            lastval = vco_curve[i];
        }
    }

    ip301_spi_write(0x1, 0x8000);
    return 0;
}

int mt301_rx_filter_calibration(u8 bw, u8 filter_code)
{
#if 1
    // RX filter auto-calibration (per filter bandwidth)
    // Relevant registers: reg8, reg0, reg1, reg12, reg13
    // Input: bw => 0: 20MHz Mode, 1: 40MHz Mode
	//		  filter_code => RX filter bandwidth selection (calibration for on BW only)
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13, reg8, reg0;

    RFC_DBG("RX filter auto-calibration: bw = %d, filter_code = %x\n", bw, filter_code);

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RX filter auto-calibration: reg13 0x%x before calibration\n", reg13);

    // select RX and setup RX filter bandwidth, reg8[9:7]= 111 max BW, 000 min BW
	// reg8[15] => 0: 20MHz Mode, 1: 40MHz Mode
#if 0	/* FIXME: work around ? */
	reg8 = ip301_spi_read(0x8);
	if(bw)	/* 40MHz */
    	ip301_spi_write(0x8, 0xB000 | ((filter_code & 0x7) << 7) | (reg8 & 0x7C7F));
	else	/* 20MHz */
    	ip301_spi_write(0x8, 0x3000 | ((filter_code & 0x7) << 7) | (reg8 & 0x7C7F));
#else
	/* bw=0, filter_code=4 */
	reg8 = (ip301_spi_read(0x8) & 0x0c00) | 0x28000;
    ip301_spi_write(0x8, reg8);
#endif
    ndelay(2000);

    // setup sub-blocks on/off
	reg0 = ip301_spi_read(0x0);
    ip301_spi_write(0x0, 0x3FE00);      // VCO on, TX on
	ip301_spi_write(0x17, 0x308FE);		// Added
    ndelay(2000);

    // Start calibration
    ip301_spi_write(0x1, 0x8000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x8020);
    ndelay(65000);

    // Check test ready
    reg12 = ip301_spi_read(0x12);
    if(0x4!=(0x4 & reg12))       // check reg12[2], (should be "1" and means calibration done)
    {
        RFC_DBG("RX filter auto-calibration: reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RX filter auto-calibration: done, reg13 0x%x after calibration\n\n", reg13);

	reg13 = (reg13 >>10) & 0x1f;
	reg13 = (reg13 <2) ? 0 : reg13-2;
	reg13 = reg13*33; // *(32 + 1)
	ip301_spi_write(0x8, reg8 +reg13);

    ip301_spi_write(0x1, 0x8000);
	/* restore the reg 0 value, for kepping the rfc calibration value. By KC. */
    ip301_spi_write(0x0, reg0);
#endif
    return 0;
}

int mt301_txlo_calibration(int channel)
{
    // Relevant registers: reg2, reg0, reg7, reg1, reg12
    // Input: Select an arbitrary channel
    // Assume VCO has been selected and calibrated

    u32 reg12;
    u32 i_path_txlo_result, q_path_txlo_result;
    int retry = 0;

    u32 i_path_txlo_result_prev = 0xffff;
    u32 q_path_txlo_result_prev = 0xffff;

Retry:

    RFC_DBG("TXLO auto-calibration: channel %d\n", channel);

    reg12 = ip301_spi_read(0x12);
    RFC_DBG("TXLO auto-calibration: reg12 0x%x before calibration\n", reg12);

    // Set up DAC: DAC turn on; DAC = 0;
    bb_register_write(0x1c, 0xee);          // Set BB register 0x1C to the value of 0xEE

    // select channel
    //ip301_set_channel(0, channel);
	ip301_spi_write(0x2, 0x28c4e);
	ndelay(30000);      // Wait 30us for PLL locking time

    // setup sub-blocks on/off
    ip301_spi_write(0x0, 0x3fe00);      // VCO on, TX on, RX off
    ndelay(2000);

    // TX VGA setting, VGA_vc<6:0>=1111111
	ip301_spi_write(0x7, 0x7c0); // Add, VGA
	ip301_spi_write(0x16, 0x12627); // Add, TXPA
    ndelay(2000);

    // I path calibration, reg1[6]=1 and reg1[8] = 0
    ip301_spi_write(0x1, 0x8000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x8040);
    ndelay(20000);

    // Check I-path test ready and save I-path result
    reg12 = ip301_spi_read(0x12);
    if(0x10!=(0x10 & reg12))       // check reg12[4], (should be "1" and means calibration done)
    {
        RFC_DBG("TXLO auto-calibration: I-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    i_path_txlo_result = (reg12 >> 5) & 0xff;       // I_path_TX_LO_result = read reg12[12:5]

    // Q path calibration, reg1[6]=1 and reg1[8] = 1
    ip301_spi_write(0x1, 0x8000);
    ndelay(5000);
    ip301_spi_write(0x1, 0x8140);
    ndelay(20000);

    // Check Q-path test ready and save Q-path result
    reg12 = ip301_spi_read(0x12);
    if(0x10!=(0x10 & reg12))       // check reg12[4], (should be "1" and means calibration done)
    {
        RFC_DBG("TXLO auto-calibration: Q-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    q_path_txlo_result = (reg12 >> 5) & 0xff;       // Q_path_TX_LO_result = read reg12[12:5]

    RFC_DBG("TXLO auto-calibration: done, i_path_txlo_result 0x%x, q_path_txlo_result 0x%x\n\n", i_path_txlo_result, q_path_txlo_result);

    ip301_spi_write(0x1, 0x8000);

#if defined(RFAC_ANALYSIS)
    __i_path_txlo_result[rfac_test_case_no] = i_path_txlo_result;
    __q_path_txlo_result[rfac_test_case_no] = q_path_txlo_result;
#endif

    if(txlo_calibration_max_retry)
    {
		s32 delta_i = (s32)i_path_txlo_result_prev - (s32)i_path_txlo_result;
		s32 delta_q = (s32)q_path_txlo_result_prev - (s32)q_path_txlo_result;
		if((-1 > delta_i) || (delta_i > 1) || (-1 > delta_q) || (delta_q > 1))
        {
            if(retry < txlo_calibration_max_retry)
            {
                i_path_txlo_result_prev = i_path_txlo_result;
                q_path_txlo_result_prev = q_path_txlo_result;
                RFC_DBG("TXLO auto-calibration: retry %d\n", retry);
                retry++;
                goto Retry;
            }
            else
            {
                RFC_DBG("TXLO auto-calibration: failed\n");
                return -1;
            }
        }
    }

    return 0;
}

int mt301_rxdc_calibration(u8 gain)
{
    // Relevant registers: reg7, reg0, reg9, reg1, reg12, reg13
    // Assume VCO has been selected and calibrated

    u32 reg12, reg13;
    u32 i_path_rxdc_result, q_path_rxdc_result;
	u32 retry = 0;
    u32 i_path_rxdc_result_prev = 0xffff;
    u32 q_path_rxdc_result_prev = 0xffff;

    RFC_DBG("RXDC auto-calibration: %s gain mode\n", (gain==0) ? "high" : ((gain==1) ? "middle" : "ultra-low"));

rxdc_retry:

    // Read reg13[8:0]
    reg13 = ip301_spi_read(0x13);
    RFC_DBG("RXDC auto-calibration: reg13 0x%x before calibration\n", reg13);

    // setup LNA gain mode
    if(gain==0)
        ip301_spi_write(0x7, 0x10020);         // high gain; gpk=111
    else if(gain==1)
        ip301_spi_write(0x7, 0x10024);         // middle gain
	else if(gain==2)
        ip301_spi_write(0x7, 0x10025);         // low gain
    else
        ip301_spi_write(0x7, 0x10027);         // ultra-low gain
    ndelay(1000);

    // VCO on, RX on, RX_SVLP off, TX off
	//ip301_spi_write(0x7, 0x10020); // 1.8V Slave LDO of PA power 
	ip301_spi_write(0x17, 0x37840); // Slave LDO control 
	ip301_spi_write(0x0, 0x3e0ff);
    ndelay(2000);

    // I path, reg9[17:16]=01 for I path and set RGCL<5:0>=111111
    ip301_spi_write(0x9, 0x101fc);
    ndelay(200000);

    // Set reg1[3:2]=01 to disable rxdc_force but start tuning
    ip301_spi_write(0x1, 0x8000);
    ndelay(200000);
    ip301_spi_write(0x1, 0x8004);

#if 0
	int i=0;
	for(i=0; i<=0x1f; i++)
		RFC_DBG("rf reg.0x%02x = 0x%x\n", i, ip301_spi_read(i));
#endif
	
	ndelay(12000000);

    // Check I-path test ready and save I-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: I-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);
    i_path_rxdc_result = reg13 & 0x01ff;      // Read reg13[8:0] after calibration

    // Q path, reg9[17:16]=11 for I path and set RGCL<5:0>=111111 
    ip301_spi_write(0x9, 0x301fc);
    ndelay(200000);

    // Start Q-calibration
    ip301_spi_write(0x1, 0x8000);
    ndelay(200000);
    ip301_spi_write(0x1, 0x8004);

	ndelay(12000000);

    // Check Q-path test ready and save Q-path result
    reg12 = ip301_spi_read(0x12);
    if(0x08!=(0x08 & reg12))
    {
        RFC_DBG("RXDC auto-calibration: Q-path reg12 not ready (0x%x)\n", reg12);
        return -1;
    }

    reg13 = ip301_spi_read(0x13);           // 	Read reg13[8:0] after calibration
    q_path_rxdc_result = reg13 & 0x01ff;

    // paste the I/Q result to regc/regb/rega
    if(gain==0)
        ip301_spi_write(0x14, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // high gain; regc
    else if(gain==1)
        ip301_spi_write(0xc, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // middle gain; regb
	else if(gain==2)
        ip301_spi_write(0xb, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // low gain; regb
    else
        ip301_spi_write(0xa, (q_path_rxdc_result << 9) | i_path_rxdc_result);         // ultra-low gain; rega

    // Disable the calibration
    ip301_spi_write(0x1, 0x8000);
    ndelay(200000);

    RFC_DBG("RXDC auto-calibration: done, i_path_rxdc_result 0x%x, q_path_rxdc_result 0x%x\n\n", i_path_rxdc_result, q_path_rxdc_result);


	i_path_rxdc_result = (i_path_rxdc_result >> 3);
	q_path_rxdc_result = (q_path_rxdc_result >> 3);
#if defined(RFAC_ANALYSIS)
    __i_path_rxdc_result[rfac_test_case_no][gain] = i_path_rxdc_result;
    __q_path_rxdc_result[rfac_test_case_no][gain] = q_path_rxdc_result;
#endif

    if(rxdc_calibration_max_retry)
    {
		s32 delta_i = (s32)i_path_rxdc_result_prev - (s32)i_path_rxdc_result;
		s32 delta_q = (s32)q_path_rxdc_result_prev - (s32)q_path_rxdc_result;
		if((-3 > delta_i) || (delta_i > 3) || (-3 > delta_q) || (delta_q > 3))
        {
            if(retry < rxdc_calibration_max_retry)
            {
                i_path_rxdc_result_prev = i_path_rxdc_result;
                q_path_rxdc_result_prev = q_path_rxdc_result;
                RFC_DBG("RXDC auto-calibration: retry %d\n", retry);
                retry++;
                goto rxdc_retry;
            }
            else
            {
                RFC_DBG("RXDC auto-calibration: failed\n");
                return -1;
            }
        }
    }

    return 0;
}


int mt301_calibration(int channel, int internal_ldo)
{
    int ret = 0;
    int i;
	unsigned char buf[2];

    ip301_power_on();
   
	RFC_DBG("RF auto-calibration: rfchip=%d, %s, channel %d\n", my_rf_dev->chip_ver, internal_ldo ? "Internal LDO" : "External LDO", channel);

	RFC_DBG("~~~~Jerry asks to DAC does not enter sleep mode~~~\n");
	bb_register_write(0x1c, 0xaa);
	
    ret = mt301_vco_calibration(internal_ldo);
    if(0>ret)
        goto Failed;
	ret = mt301_rx_filter_calibration(0, 4);
	if(0>ret)
		goto Failed;

	ret = mt301_txlo_calibration(channel);
    if(0>ret)
        goto Failed;

	/* new RF add low gain calibration */
	for(i=0;i<4;i++)
	{
		ret = mt301_rxdc_calibration(i);
		if(0>ret)
			goto Failed;
	}

	RFC_DBG("~~~Jerry asks to re-enter sleep mode~~~\n");
	bb_register_write(0x1c, 0x0);

    RFC_DBG("@@ RF auto-calibration (RFAC): done.\n\n");
    return 0;

Failed:
	while(1)
	{
		RFC_DBG("RF auto-calibration (RFAC) Fail, Please seek assistance from Stan (#2629). Press c to continue.\n\n");
		WLA_GETS(buf);
		if(strncmp(buf, "c", 1) == 0)
			break;
	}
    
	return ret;
}


#if defined(RFAC_ANALYSIS)

int tx_filter_calibration_check(void)
{
    double ctl_coe;
    unsigned char cal_mode_sel;
    unsigned char tg_a_i, tg_a_q;
    complex I_signal_f0, Q_signal_f0;
    complex I_signal_f1, Q_signal_f1;
#if 0
    int a, b;
#endif

    bb_rfc_reset();

    rx_filter_calibration(0, 7);

    RFC_DBG("tx_filter_calibration_check()\n");

    // Set RX loopback mode
    ip301_set_iqcal(RXLOOP);

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    tg_a_i = 3;
    tg_a_q = 15;
    ctl_coe = 5.0;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x0;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("I: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Set f1 = TX filter 3dB corner frequency tone and send f1 tone; set RFC demodulator to read f1 tone magnitude
    tg_a_i = 3;
    tg_a_q = 15;
    ctl_coe = 8.5;
#if 0
RFC_DBG("I: please enter f1 frequency: ");
WLA_GETS(buf);
if(2==sscanf(buf, "%d.%d", &a, &b))
{   
    ctl_coe = a + ((double)b) / 10;
    RFC_DBG("\nmanually set frequence to %d.%d Mhz\n", a, b);
}
else if(1==sscanf(buf, "%d", &a))
{
    ctl_coe = a;
    RFC_DBG("\nmanually set frequence to %d Mhz\n", a, b);
}
#endif
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("I: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif

    if(read_LPF(&I_signal_f1, &Q_signal_f1, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Check the ratio of f1 tone magnitude to f0 tone magnitude (this ratio should read approximately at -3dB)
    RFC_DBG("I_signal_f0: ");
    RFC_DBG_COMPLEX(I_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&I_signal_f0));
    RFC_DBG(" db");
    RFC_DBG(", I_signal_f1: ");
    RFC_DBG_COMPLEX(I_signal_f1);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&I_signal_f1));
    RFC_DBG(" db");
    RFC_DBG("\n");

    __tx_filter_chk[rfac_test_case_no][0] = __complex_to_db(&I_signal_f0);
    __tx_filter_chk[rfac_test_case_no][1] = __complex_to_db(&I_signal_f1);

    bb_rfc_reset();

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    tg_a_i = 15;
    tg_a_q = 3;
    ctl_coe = 5.0;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x0;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("Q: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Set f1 = TX filter 3dB corner frequency tone and send f1 tone; set RFC demodulator to read f1 tone magnitude
    tg_a_i = 15;
    tg_a_q = 3;
    ctl_coe = 8.5;
#if 0
RFC_DBG("Q: please enter f1 frequency: ");
WLA_GETS(buf);
if(2==sscanf(buf, "%d.%d", &a, &b))
{   
    ctl_coe = a + ((double)b) / 10;
    RFC_DBG("\nmanually set frequence to %d.%d Mhz\n", a, b);
}
else if(1==sscanf(buf, "%d", &a))
{
    ctl_coe = a;
    RFC_DBG("\nmanually set frequence to %d Mhz\n", a, b);
}
#endif
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("Q: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif

    if(read_LPF(&I_signal_f1, &Q_signal_f1, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Check the ratio of f1 tone magnitude to f0 tone magnitude (this ratio should read approximately at -3dB)
    RFC_DBG("Q_signal_f0: ");
    RFC_DBG_COMPLEX(Q_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f0));
    RFC_DBG(" db");
    RFC_DBG(", Q_signal_f1: ");
    RFC_DBG_COMPLEX(Q_signal_f1);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f1));
    RFC_DBG(" db");
    RFC_DBG("\n");

    __tx_filter_chk[rfac_test_case_no][2] = __complex_to_db(&Q_signal_f0);
    __tx_filter_chk[rfac_test_case_no][3] = __complex_to_db(&Q_signal_f1);

    rx_filter_calibration(0, 4);

    bb_rfc_reset();

    return 0;
}

int rx_filter_calibration_check(void)
{
    double ctl_coe;
    unsigned char cal_mode_sel;
    unsigned char tg_a_i, tg_a_q;
    complex I_signal_f0, Q_signal_f0;
    complex I_signal_f1, Q_signal_f1;
#if 0
    int a, b;
#endif

    bb_rfc_reset();

    tx_filter_calibration(0, 7);

    RFC_DBG("rx_filter_calibration_check()\n");

    // Set RX loopback mode
    ip301_set_iqcal(RXLOOP);

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    tg_a_i = 3;
    tg_a_q = 15;
    ctl_coe = 5.0;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x0;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("I: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Set f1 = TX filter 3dB corner frequency tone and send f1 tone; set RFC demodulator to read f1 tone magnitude
    tg_a_i = 3;
    tg_a_q = 15;
    ctl_coe = 8.5;
#if 0
RFC_DBG("please enter f1 frequency: ");
WLA_GETS(buf);
if(2==sscanf(buf, "%d.%d", &a, &b))
{   
    ctl_coe = a + ((double)b) / 10;
    RFC_DBG("\nmanually set frequence to %d.%d Mhz\n", a, b);
}
else if(1==sscanf(buf, "%d", &a))
{
    ctl_coe = a;
    RFC_DBG("\nmanually set frequence to %d Mhz\n", a, b);
}
#endif
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("I: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif

    if(read_LPF(&I_signal_f1, &Q_signal_f1, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Check the ratio of f1 tone magnitude to f0 tone magnitude (this ratio should read approximately at -3dB)
    RFC_DBG("I_signal_f0: ");
    RFC_DBG_COMPLEX(I_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&I_signal_f0));
    RFC_DBG(" db");
    RFC_DBG(", I_signal_f1: ");
    RFC_DBG_COMPLEX(I_signal_f1);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&I_signal_f1));
    RFC_DBG(" db");
    RFC_DBG("\n");

    __rx_filter_chk[rfac_test_case_no][0] = __complex_to_db(&I_signal_f0);
    __rx_filter_chk[rfac_test_case_no][1] = __complex_to_db(&I_signal_f1);


    bb_rfc_reset();

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    tg_a_i = 15;
    tg_a_q = 3;
    ctl_coe = 5.0;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x0;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("Q: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Set f1 = TX filter 3dB corner frequency tone and send f1 tone; set RFC demodulator to read f1 tone magnitude
    tg_a_i = 15;
    tg_a_q = 3;
    ctl_coe = 8.5;
#if 0
RFC_DBG("please enter f1 frequency: ");
WLA_GETS(buf);
if(2==sscanf(buf, "%d.%d", &a, &b))
{   
    ctl_coe = a + ((double)b) / 10;
    RFC_DBG("\nmanually set frequence to %d.%d Mhz\n", a, b);
}
else if(1==sscanf(buf, "%d", &a))
{
    ctl_coe = a;
    RFC_DBG("\nmanually set frequence to %d Mhz\n", a, b);
}
#endif
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

    udelay(1000000);
#if 0
RFC_DBG("Q: press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif

    if(read_LPF(&I_signal_f1, &Q_signal_f1, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    // Check the ratio of f1 tone magnitude to f0 tone magnitude (this ratio should read approximately at -3dB)
    RFC_DBG("Q_signal_f0: ");
    RFC_DBG_COMPLEX(Q_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f0));
    RFC_DBG(" db");
    RFC_DBG(", Q_signal_f1: ");
    RFC_DBG_COMPLEX(Q_signal_f1);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f1));
    RFC_DBG(" db");
    RFC_DBG("\n");

    __rx_filter_chk[rfac_test_case_no][2] = __complex_to_db(&Q_signal_f0);
    __rx_filter_chk[rfac_test_case_no][3] = __complex_to_db(&Q_signal_f1);

    tx_filter_calibration(0, 4);

    bb_rfc_reset();

    return 0;
}

void output_vco_curves_statistics(int samples)
{
    int i;
    int j;
    u8 curr_max;
    u8 curr_min;
    double accmulate;
    double average;
    double variance;

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX       ");
    for(j=0;j<10;j++)
    {
        curr_max = __vco_curves[0][j];

        for(i=0;i<samples;i++)
        {
            if(__vco_curves[i][j]>curr_max)
                curr_max = __vco_curves[i][j];
        }
        RFC_DBG("%02x  ", curr_max);
    }
    RFC_DBG("\n");

    RFC_DBG(" MIN       ");
    for(j=0;j<10;j++)
    {
        curr_min = __vco_curves[0][j];

        for(i=0;i<samples;i++)
        {
            if(__vco_curves[i][j]<curr_min)
                curr_min = __vco_curves[i][j];
        }
        RFC_DBG("%02x  ", curr_min);
    }
    RFC_DBG("\n");

    RFC_DBG(" AVG    ");
    for(j=0;j<10;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __vco_curves[i][j];
        }
        average = accmulate/samples;
        dbg_double_short(average);
        RFC_DBG(" ");
    }
    RFC_DBG("\n");

    RFC_DBG(" DEV    ");
    for(j=0;j<10;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __vco_curves[i][j];
        }
        average = accmulate/samples;

        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += ((__vco_curves[i][j] - average) * (__vco_curves[i][j] - average));
        }
        variance = sqrt((accmulate / samples));
        dbg_double_short(variance);
        RFC_DBG("  ");
    }
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");
}

void output_rx_filter_statistics(int samples)
{
    int i;
    int j;
    double curr_max;
    double curr_min;
    double accmulate;
    double average;
    double variance;

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX       ");
    for(j=0;j<4;j++)
    {
        curr_max = __rx_filter_chk[0][j];
        for(i=0;i<samples;i++)
        {
            if(__rx_filter_chk[i][j]>curr_max)
                curr_max = __rx_filter_chk[i][j];
        }
        RFC_DBG_DOUBLE(curr_max);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" MIN       ");
    for(j=0;j<4;j++)
    {
        curr_min = __rx_filter_chk[0][j];
        for(i=0;i<samples;i++)
        {
            if(__rx_filter_chk[i][j]<curr_min)
                curr_min = __rx_filter_chk[i][j];
        }
        RFC_DBG_DOUBLE(curr_min);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" AVG       ");
    for(j=0;j<4;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __rx_filter_chk[i][j];
        }
        average = accmulate/samples;
        RFC_DBG_DOUBLE(average);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" DEV         ");
    for(j=0;j<4;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __rx_filter_chk[i][j];
        }
        average = accmulate/samples;

        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += ((__rx_filter_chk[i][j] - average) * (__rx_filter_chk[i][j] - average));
        }
        variance = sqrt((accmulate / samples));
        RFC_DBG_DOUBLE(variance);
        RFC_DBG("     ");
    }
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");
}

void output_tx_filter_statistics(int samples)
{
    int i;
    int j;
    double curr_max;
    double curr_min;
    double accmulate;
    double average;
    double variance;

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX       ");
    for(j=0;j<4;j++)
    {
        curr_max = __tx_filter_chk[0][j];
        for(i=0;i<samples;i++)
        {
            if(__tx_filter_chk[i][j]>curr_max)
                curr_max = __tx_filter_chk[i][j];
        }
        RFC_DBG_DOUBLE(curr_max);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" MIN       ");
    for(j=0;j<4;j++)
    {
        curr_min = __tx_filter_chk[0][j];
        for(i=0;i<samples;i++)
        {
            if(__tx_filter_chk[i][j]<curr_min)
                curr_min = __tx_filter_chk[i][j];
        }
        RFC_DBG_DOUBLE(curr_min);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" AVG       ");
    for(j=0;j<4;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __tx_filter_chk[i][j];
        }
        average = accmulate/samples;
        RFC_DBG_DOUBLE(average);
        RFC_DBG(" db  ");
    }
    RFC_DBG("\n");

    RFC_DBG(" DEV         ");
    for(j=0;j<4;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __tx_filter_chk[i][j];
        }
        average = accmulate/samples;

        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += ((__tx_filter_chk[i][j] - average) * (__tx_filter_chk[i][j] - average));
        }
        variance = sqrt((accmulate / samples));
        RFC_DBG_DOUBLE(variance);
        RFC_DBG("     ");
    }
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");
}

void output_txlo_statistics(int samples)
{
    int i;
    u32 curr_max;
    u32 curr_min;
    double accmulate;
    double average;
    double variance;

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX           ");
    curr_max = __i_path_txlo_result[0];
    for(i=0;i<samples;i++)
    {
        if(__i_path_txlo_result[i]>curr_max)
            curr_max = __i_path_txlo_result[i];
    }
    RFC_DBG("%02x             ", curr_max);

    curr_max = __q_path_txlo_result[0];
    for(i=0;i<samples;i++)
    {
        if(__q_path_txlo_result[i]>curr_max)
            curr_max = __q_path_txlo_result[i];
    }
    RFC_DBG("%02x  ", curr_max);
    RFC_DBG("\n");

    RFC_DBG(" MIN           ");
    curr_min = __i_path_txlo_result[0];

    for(i=0;i<samples;i++)
    {
        if(__i_path_txlo_result[i]<curr_min)
            curr_min = __i_path_txlo_result[i];
    }
    RFC_DBG("%02x             ", curr_min);

    curr_min = __q_path_txlo_result[0];

    for(i=0;i<samples;i++)
    {
        if(__q_path_txlo_result[i]<curr_min)
            curr_min = __q_path_txlo_result[i];
    }
    RFC_DBG("%02x  ", curr_min);
    RFC_DBG("\n");

    RFC_DBG(" AVG    ");
    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += __i_path_txlo_result[i];
    }
    average = accmulate/samples;
    RFC_DBG("       ");
    dbg_double_short(average);
    RFC_DBG("          ");

    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += __q_path_txlo_result[i];
    }
    average = accmulate/samples;
    dbg_double_short(average);
    RFC_DBG(" ");
    RFC_DBG("\n");

    RFC_DBG(" DEV    ");
    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += __i_path_txlo_result[i];
    }
    average = accmulate/samples;

    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += ((__i_path_txlo_result[i] - average) * (__i_path_txlo_result[i] - average));
    }
    variance = sqrt((accmulate / samples));
    RFC_DBG("       ");
    dbg_double_short(variance);
    RFC_DBG("          ");

    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += __q_path_txlo_result[i];
    }
    average = accmulate/samples;

    accmulate = 0;
    for(i=0;i<samples;i++)
    {
        accmulate += ((__q_path_txlo_result[i] - average) * (__q_path_txlo_result[i] - average));
    }
    variance = sqrt((accmulate / samples));
    dbg_double_short(variance);
    RFC_DBG("  ");
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");
}

void output_rxdc_statistics(int samples)
{
    int i;
    int j;
    u32 curr_max;
    u32 curr_min;
    double accmulate;
    double average;
    double variance;

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX         ");
    for(j=0;j<RF_DC_LEVEL;j++)
    {
        curr_max = __i_path_rxdc_result[0][j];
        for(i=0;i<samples;i++)
        {
            if(__i_path_rxdc_result[i][j]>curr_max)
                curr_max = __i_path_rxdc_result[i][j];
        }
        RFC_DBG("%03x      ", curr_max);
    }

    for(j=0;j<RF_DC_LEVEL;j++)
    {
        curr_max = __q_path_rxdc_result[0][j];
        for(i=0;i<samples;i++)
        {
            if(__q_path_rxdc_result[i][j]>curr_max)
                curr_max = __q_path_rxdc_result[i][j];
        }
        RFC_DBG("%03x      ", curr_max);
    }
    RFC_DBG("\n");

    RFC_DBG(" MIN         ");
    for(j=0;j<RF_DC_LEVEL;j++)
    {
        curr_min = __i_path_rxdc_result[0][j];
    
        for(i=0;i<samples;i++)
        {
            if(__i_path_rxdc_result[i][j]<curr_min)
                curr_min = __i_path_rxdc_result[i][j];
        }
        RFC_DBG("%03x      ", curr_min);
    }

    for(j=0;j<RF_DC_LEVEL;j++)
    {
        curr_min = __q_path_rxdc_result[0][j];
    
        for(i=0;i<samples;i++)
        {
            if(__q_path_rxdc_result[i][j]<curr_min)
                curr_min = __q_path_rxdc_result[i][j];
        }
        RFC_DBG("%03x      ", curr_min);
    }
    RFC_DBG("\n");

    RFC_DBG(" AVG         ");
    for(j=0;j<RF_DC_LEVEL;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __i_path_rxdc_result[i][j];
        }
        average = accmulate/samples;
        dbg_double_short(average);
        RFC_DBG("   ");
    }

    for(j=0;j<RF_DC_LEVEL;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __q_path_rxdc_result[i][j];
        }
        average = accmulate/samples;
        dbg_double_short(average);
        RFC_DBG("   ");
    }
    RFC_DBG("\n");

    RFC_DBG(" DEV         ");
    for(j=0;j<RF_DC_LEVEL;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __i_path_rxdc_result[i][j];
        }
        average = accmulate/samples;
    
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += ((__i_path_rxdc_result[i][j] - average) * (__i_path_rxdc_result[i][j] - average));
        }
        variance = sqrt((accmulate / samples));
        dbg_double_short(variance);
        RFC_DBG("     ");
    }

    for(j=0;j<RF_DC_LEVEL;j++)
    {
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += __q_path_rxdc_result[i][j];
        }
        average = accmulate/samples;
    
        accmulate = 0;
        for(i=0;i<samples;i++)
        {
            accmulate += ((__q_path_rxdc_result[i][j] - average) * (__q_path_rxdc_result[i][j] - average));
        }
        variance = sqrt((accmulate / samples));
        dbg_double_short(variance);
        RFC_DBG("     ");
    }
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");
}

int rfac_cmd(int argc, char* argv[])
{
    int ret = 0;
    int samples;
    int i;
    int j;
    int error_count = 0;
    int repeat;

    //__bb_init();

    txlo_calibration_max_retry = 0;
	rxdc_calibration_max_retry = 0;
    memset(__vco_curves, 0, sizeof(__vco_curves));
    memset(__i_path_txlo_result, 0, sizeof(__i_path_txlo_result));
    memset(__q_path_txlo_result, 0, sizeof(__q_path_txlo_result));
    memset(__i_path_rxdc_result, 0, sizeof(__i_path_txlo_result));
    memset(__q_path_rxdc_result, 0, sizeof(__q_path_rxdc_result));

    if(argc==2)
    {
        ret = rf_calibration(RF_CAL_CH, my_rf_dev->internal_ldo);
        if(0>ret)
            RFC_DBG("ERROR: RFAC failed (%d)\n", ret);

        samples = 1;
    }
    else
    {
        repeat = atoi(argv[2]);
        samples = repeat;

        for(i=0;i<samples;i++)
        {
            rfac_test_case_no = i;

            RFC_DBG("========================== RFAC Test case %d ==========================\n", i);
            ret = rf_calibration(RF_CAL_CH, my_rf_dev->internal_ldo);
            if(0>ret)
            {    
                RFC_DBG("ERROR: RFAC failed (%d), test case %d\n", ret, i);
                error_count++;
            }

            rx_filter_calibration_check();
            tx_filter_calibration_check();

            ip301_power_off();
            udelay(2000000);
            ip301_power_on();

            //RFC_DBG("press ENTER to continue the test.\n");
            //WLA_GETS(buf);
        }
    }

    RFC_DBG("---------------- VCO curve ----------------------------------------\n");
    RFC_DBG("  NO.       0   1   2   3   4   5   6   7   8   9\n");
    RFC_DBG("-------------------------------------------------------------------\n");
    for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d      ", i);
        for(j=0;j<10;j++)
        {
            RFC_DBG("%02x  ", __vco_curves[i][j]);
        }
        RFC_DBG("\n");
    }
    output_vco_curves_statistics(samples);

    RFC_DBG("---------------- RX filter ----------------------------------------\n");
    RFC_DBG("  NO.          i_f0           i_f1             q_f0             q_f1\n");
    RFC_DBG("-------------------------------------------------------------------\n");
    for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d      ", i);
        RFC_DBG_DOUBLE(__rx_filter_chk[i][0]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__rx_filter_chk[i][1]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__rx_filter_chk[i][2]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__rx_filter_chk[i][3]);
        RFC_DBG(" db  ");
        RFC_DBG("\n");
    }
    output_rx_filter_statistics(samples);

    RFC_DBG("---------------- TX filter ----------------------------------------\n");
    RFC_DBG("  NO.          i_f0           i_f1             q_f0             q_f1\n");
    RFC_DBG("-------------------------------------------------------------------\n");
    for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d      ", i);
        RFC_DBG_DOUBLE(__tx_filter_chk[i][0]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__tx_filter_chk[i][1]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__tx_filter_chk[i][2]);
        RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(__tx_filter_chk[i][3]);
        RFC_DBG(" db  ");
        RFC_DBG("\n");
    }
    output_tx_filter_statistics(samples);

    RFC_DBG("---------------- TXLO ---------------------------------------------\n");
    RFC_DBG("  NO.         i_path        q_path\n");
    RFC_DBG("-------------------------------------------------------------------\n");
    for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d          %02x             %02x\n", i, __i_path_txlo_result[i], __q_path_txlo_result[i]);
    }
    output_txlo_statistics(samples);

    RFC_DBG("---------------- RXDC ---------------------------------------------\n");
    RFC_DBG("  NO.      i_high   i_middle  i_low   q_high   q_middle  q_low\n");
    RFC_DBG("-------------------------------------------------------------------\n");
    for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d        ", i);
        for(j=0;j<RF_DC_LEVEL;j++)
        {
            RFC_DBG("%03x      ", __i_path_rxdc_result[i][j]);
        }
        for(j=0;j<RF_DC_LEVEL;j++)
        {
            RFC_DBG("%03x      ", __q_path_rxdc_result[i][j]);
        }
        RFC_DBG("\n");
    }
    output_rxdc_statistics(samples);

    if(error_count)
    {
        RFC_DBG("ERROR: %d test cases failed\n", error_count);
    }

    txlo_calibration_max_retry = TXLO_CALIBRATION_MAX_RETRY;
	rxdc_calibration_max_retry = RXDC_CALIBRATION_MAX_RETRY;
    return 0;
}

#endif

#ifdef RF_POWER_ONOFF_GPIO
#define RF_POWER_PIN    5
#define RF_POWER        (0x1UL << RF_POWER_PIN)

#define GPSET			1
#define GPCLR			2
#define GPDIR			3
#define GPSEL 			4
#define MI_BASE     	0xaf000000
#define	GPIO_BASE		(MI_BASE+0x5000)
#define	GPREG(reg) 		((volatile unsigned int*)(GPIO_BASE))[reg]
static int power_on = 0;
void ip301_power_on(void)
{
    if(power_on)
        return;

    RFC_DBG("Power on IP301\n");

    GPREG(GPSEL) |= RF_POWER;
    GPREG(GPDIR) |= RF_POWER;
    
    GPREG(GPSET) = RF_POWER;
    
	udelay(5000000);
    power_on = 1;
}

void ip301_power_off(void)
{
    RFC_DBG("Power off IP301\n");

    GPREG(GPSEL) |= RF_POWER;
    GPREG(GPDIR) |= RF_POWER;
    
    GPREG(GPCLR) = RF_POWER;

    power_on = 0;
}
#else //RF_POWER_ONOFF_GPIO
void ip301_power_on(void)
{

}

void ip301_power_off(void)
{

}
#endif   //RF_POWER_ONOFF_GPIO

#endif //CONFIG_RFAC
