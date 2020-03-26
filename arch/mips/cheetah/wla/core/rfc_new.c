/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfc.c
*   \brief
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <mac_regs.h>
#include <complex.h>
#include <wla_bb.h>
#include <ip301.h>
#include <rfc.h>
#include <rfac_rfc_patch.h>
#include <rf.h>
#include <math.h>

//-----------------------------------------------------------------------------------
#ifdef CONFIG_RFC
#define RFC_REG_BIT 8

#define CHANNEL_WIDTH_40MHZ		1
#define PHASE_SHIFTER_TURNON_THRESHOULD  0.02 /* unit: radian, 0.02*180/pi ~= 1.14 degree */

extern int current_channel_width;
struct rfc_cal_reg rfc_result_new[2] ; // [0] : 20MHz, [1] : 40MHz

/* set_sampling_rate(bw): bw=1 => 80MHz, bw=0 => 40MHz */
void set_sampling_rate(int bw)
{
	u32 reg8 = ip301_spi_read(0x8);
	int is_mt301 = (my_rf_dev->chip_ver == RFCHIP_MT301_A0) || (my_rf_dev->chip_ver == RFCHIP_MT301_A1);

	bb_register_write(0x32, ((bb_register_read(0x32) & 0xFE) | bw));
	bb_register_write(0xf2, (((bb_register_read(0xf2)) & 0xfd) | (bw << 1)));

	if(bw)
	{
		if(is_mt301)
			reg8 |= 0x0C00;
		else
			reg8 |= 0xC000;
#if !defined(CONFIG_FPGA)
		ANAREG(W_DAC_ADC_DIV) &= ~(W_DAC_40M|W_ADC_240M);
#endif
	}
	else
	{
		if(is_mt301)
			reg8 &= 0xfffff3ff;
		else
			reg8 &= 0xffff3fff;
#if !defined(CONFIG_FPGA)
		ANAREG(W_DAC_ADC_DIV) |= (W_DAC_40M|W_ADC_240M);
#endif
	}
	
	ip301_spi_write(0x8, reg8);

	/* after bb ver 0x4b */
	bb_register_write(0xf3, (((bb_register_read(0xf3)) & 0xBF) | (bw << 6)));
	bb_register_write(0x1, (((bb_register_read(0x1)) & 0xFD) | (bw << 1)));
}

//-----------------------------------------------------------------------------------

//***********************************Tx Alogorithm***********************************
void modtest_iq_balancer_new(mismatch_info *param, int is_tx)
{
    double alpha, beta;
    double a11, a12, a21, a22, n, m, t;
    double phi;
    complex gain;

    //Input Parameter Structure Construction
    gain = param->gain;
    phi = param->phi;

    //Induced parameters
	if(is_tx)
	{
		/* try the inv sign @ 2013/01/01 */
		//alpha = -sin(phi)*gain.real/gain.imag;
		alpha = sin(phi)*gain.real/gain.imag;
		beta = gain.real*cos(phi)/(gain.imag);
	}
	else	// is_rx
	{
		alpha = tan(phi);
		beta = gain.real/(gain.imag*cos(phi));
	}

    a11 = 1; a12 = 0;
    a21 = alpha; a22 = beta;

    t = max_double(fabs(a11), fabs(a12));
    n = 0;
    while(1)
    {
        if (t <= 1)
            break;
        else
        {
            n = n + 1; 
            t = t/2;
            if (n > 3)
            {    
                RFC_DBG("ERROR: Too large a11 or a12 detected in modtest_iq_balancer_new. (is_tx=%d)\n", is_tx); 
                break;
            }
        }
    }
    a11 = pow(2,-n)*a11; a12 = pow(2,-n)*a12; //Normalized coefficients

    t = max_double(fabs(a21), fabs(a22));
    m = 0;
    while(1)
    {
        if (t <= 1)
            break;
        else
        {
            m = m + 1; 
            t = t/2;
            if (m > 3)
            {    
                WLA_DBG(WLAERROR, "ERROR: Too large a22 detected in modtest_tx_iq_balancer_new.\n"); 
                break;
            }
        }
    }

    a21 = pow(2,-m)*a21; a22 = pow(2,-m)*a22; //Normalized coefficients

	param->n = (char) n; 
	param->m = (char) m;
	param->a11 = 0xff & flt2hex(a11,RFC_REG_BIT); 
	param->a12 = 0xff & flt2hex(a12,RFC_REG_BIT);
	param->a21 = 0xff & flt2hex(a21,RFC_REG_BIT); 
	param->a22 = 0xff & flt2hex(a22,RFC_REG_BIT);
}

double tx_cal_LPFout_new(unsigned char reset, complex y_LPF, mismatch_info *X_tx_iq_balancer, 
							unsigned char tx_cal_state)
{
#define GAIN_I_CAL 0
#define GAIN_Q_CAL 1
#define PHI_CAL_1 2
#define PHI_CAL_2 3
#define PHI_CAL_3 4
    static double phi_est=0, gi=0, gq=0; //tx
    static complex gain_est = { 0, 0, }; //tx
	static double q0_real, q0_imag;
    /* light version add parm */
    static double phase_ref=0;
    double phase_twoside;
    double phase_diff;
    int dirinfo;

    if (reset == 1)
    {
        //Init global variables
        gain_est.real = 1; gain_est.imag = 1; 
        phi_est = 0;
        gi = gain_est.real; gq = gain_est.imag;
        phase_ref = 0;
    }
	else
	{
		switch(tx_cal_state)
		{   //Observation and estimation
				case GAIN_I_CAL: //'gain_i_cal'
					gi = gain_estimator(y_LPF, 0, 0);
					break;
					
				case GAIN_Q_CAL: //'gain_q_cal'
					gq = gain_estimator(y_LPF, 0, 0);
					gain_est.real = gi/gi;
					gain_est.imag = gq/gi;
					X_tx_iq_balancer->gain = gain_est; X_tx_iq_balancer->phi = phi_est;
					modtest_iq_balancer_new(X_tx_iq_balancer, 1);
					phase_ref = calc_angle(y_LPF.real, y_LPF.imag);
					q0_real = y_LPF.real;
					q0_imag = y_LPF.imag;
					break;
					 
				case PHI_CAL_1: //'phi_cal_1'
					phi_est = gain_estimator(y_LPF, gi, 1);
					phase_twoside = calc_angle(y_LPF.real, y_LPF.imag);
					phase_diff = angle_diff(phase_twoside, phase_ref); //calc phase diff
					dirinfo = angle_phase(phase_diff);				   //which dir is phase_diff close to?
					if (dirinfo == ANGLE_90)
						 phi_est = -phi_est;
					else if (dirinfo == ANGLE_270)
						 phi_est = phi_est;
					else
					{
						phi_est = (y_LPF.real*q0_imag - y_LPF.imag*q0_real)/(2*(q0_real*q0_real+q0_imag*q0_imag));
						RFC_DBG("tx_cal_LPF_out : wrong angle\n");
						phase_ref = 370;
					}
					X_tx_iq_balancer->gain = gain_est; X_tx_iq_balancer->phi = phi_est;
					modtest_iq_balancer_new(X_tx_iq_balancer, 1);
					break;
				default:
					RFC_DBG("ERROR: Incorrect tx_cal_state step.\n");
		}
	}

    return phase_ref;
}

double diff_phy(double *array_a, int array_num)
{
	double ret=0, max=array_a[0], min=array_a[0];
	int i;
	
	for(i=1; i<array_num; i++)
	{
		if(array_a[i] > max)
			max = array_a[i];
		else if(array_a[i] < min)
			min = array_a[i];
	}

	ret = max - min;

	return ret;
}

void compute_balancer_new(struct bal_parm *tx_bal, struct bal_parm *rx_bal, struct rfc_cal_parm *result, int freq_quantity, int bw)
{
	int alpha;
	double tx_phi[4], rx_phi[4];
	double phi_diff=0;
	int end_tone;

	if(bw)
		end_tone = FREQ_QUANTITY_40MHZ - 1;
	else
		end_tone = FREQ_QUANTITY_20MHZ - 1;

	tx_phi[0] = tx_bal[0].phi;
	rx_phi[0] = rx_bal[0].phi;
	tx_phi[1] = tx_bal[1].phi;
	rx_phi[1] = rx_bal[1].phi;
	tx_phi[2] = tx_bal[end_tone].phi;
	rx_phi[2] = rx_bal[end_tone].phi;
	tx_phi[3] = tx_bal[end_tone].phi;
	rx_phi[3] = rx_bal[end_tone].phi;

	/* calc the tx/rx gains' & phis' mean */
	bal_parm_mean(tx_bal, rx_bal, result, freq_quantity);
	
	/* setup tx cal data*/
	phi_diff = diff_phy(tx_phi, 4);

	// curve data
	if(phi_diff >= PHASE_SHIFTER_TURNON_THRESHOULD)
	{
		alpha = cmp_phase_shifter_coe_coarse(tx_bal, bw);
		RFC_DBG("coarse tx alpha = ");RFC_DBG_DOUBLE(alpha);RFC_DBG("\n");
		alpha = cmp_phase_shifter_coe_fine(alpha, tx_bal, bw);
		RFC_DBG("fine tx alpha = ");RFC_DBG_DOUBLE(alpha);RFC_DBG("\n");
	}
	else
		alpha = 0;
	result->tx_curve.phase_shifter_coe = fabs(alpha);
	if((alpha == 256) || (alpha == 0))
		result->tx_curve.phase_shifter_ctl = I_Q_NONE;
	else if(alpha>0)
		result->tx_curve.phase_shifter_ctl = I_PATH;
	else
		result->tx_curve.phase_shifter_ctl = Q_PATH;
	result->tx_curve.rolloff_filter_ctl = 0;
	result->tx_curve.rolloff_filter_coe = 0;
	// mismatch_info
	modtest_iq_balancer_new(&result->tx_avg, 1);

	/* setup rx cal data */
	// curve data
	phi_diff = diff_phy(rx_phi, 4);
	if(phi_diff >= PHASE_SHIFTER_TURNON_THRESHOULD)		
	{
		alpha = cmp_phase_shifter_coe_coarse(rx_bal, bw);
		RFC_DBG("coarse rx alpha = ");RFC_DBG_DOUBLE(alpha);RFC_DBG("\n");
		alpha = cmp_phase_shifter_coe_fine(alpha, rx_bal, bw);
		RFC_DBG("fine rx alpha = ");RFC_DBG_DOUBLE(alpha);RFC_DBG("\n");
	}
	else
		alpha = 0;
	result->rx_curve.phase_shifter_coe = fabs(alpha);
	if((alpha == 256) || (alpha == 0))
		result->rx_curve.phase_shifter_ctl = I_Q_NONE;
	else if(alpha>0)
		result->rx_curve.phase_shifter_ctl = I_PATH;
	else
		result->rx_curve.phase_shifter_ctl = Q_PATH;
	result->rx_curve.rolloff_filter_ctl = 0;
	result->rx_curve.rolloff_filter_coe = 0;
	// mismatch_info
	modtest_iq_balancer_new(&result->rx_avg, 0);
}

void transfer_twofilter_regs_new(struct rfc_cal_parm *cal_parm, struct rfc_cal_reg *result)
{
	int phase_shifter_coe_wlen=8;
	
	result->balancer_nm = (cal_parm->tx_avg.n << 6) | (cal_parm->tx_avg.m << 4) | (cal_parm->rx_avg.n << 2) | (cal_parm->rx_avg.m);
	result->tx_a11 = bb_register_read(0x21);
	result->tx_a12 = cal_parm->tx_avg.a12;
	result->tx_a21 = cal_parm->tx_avg.a21;
	result->tx_a22 = cal_parm->tx_avg.a22;
	result->tx_dc_i = bb_register_read(0x25);
	result->rx_a21 = cal_parm->rx_avg.a21;
	result->rx_a22 = cal_parm->rx_avg.a22;
	result->rx_dc_i = 0;
	result->tx_dc_q = bb_register_read(0x30);
	result->rx_dc_q = 0;
	result->filter_switch = (cal_parm->tx_curve.rolloff_filter_ctl << 6) | (cal_parm->tx_curve.phase_shifter_ctl << 4) | (cal_parm->rx_curve.rolloff_filter_ctl << 2) | (cal_parm->rx_curve.phase_shifter_ctl);
	result->rolloff_rx_coe = (cal_parm->rx_curve.rolloff_filter_coe * pow(2, (phase_shifter_coe_wlen-1)));
	result->phaseshifter_rx_alfa = cal_parm->rx_curve.phase_shifter_coe;
	result->rolloff_tx_coe = (cal_parm->tx_curve.rolloff_filter_coe * pow(2, (phase_shifter_coe_wlen-1)));
	result->phaseshifter_tx_alfa = cal_parm->tx_curve.phase_shifter_coe;

	/* patch the issue : 5e = 0 & 5a = 1 => rfc will get wrong result */
	trans_2filter_regs_patch(result);

}

//**********************************Device control***********************************

void bal_regs_new(mismatch_info param, unsigned char mode) //TX_BAL:0 RX_BAL:1
{
#define RFC_TX_N 0x03 //<<6
#define RFC_TX_M 0x03 //<<4
#define RFC_RX_N 0x03 //<<2
#define RFC_RX_M 0x03 //<<0

    char a11, a12, a21, a22;
    char n, m;

    char reg_val;
    unsigned char value20;

    n = param.n; m = param.m;
    a11 = param.a11; a12 = param.a12; 
    a21 = param.a21; a22 = param.a22;

    switch(mode) {
        case TX_BAL: //tx balancer
            //Write N, M
            value20 = bb_register_read(0x20); //get RX N, RX M
            reg_val = (n & RFC_TX_N)<<6 | (m & RFC_TX_M)<<4 | (value20 & 0x0F);
            bb_register_write(0x20, reg_val);
        
            bb_register_write(0x21, a11);
            bb_register_write(0x22, a12);
            bb_register_write(0x23, a21);
            bb_register_write(0x24, a22);

            break;
        case RX_BAL: //rx balancer
            //Write N, M
            value20 = bb_register_read(0x20); //get RX N, RX M
            reg_val = (value20 & 0xF0) | (n & RFC_RX_N)<<2 | (m & RFC_RX_M) ;
            bb_register_write(0x20, reg_val);
        
            bb_register_write(0x26, a11);
            bb_register_write(0x27, a12);
            bb_register_write(0x28, a21);
            bb_register_write(0x29, a22);

            break;
    }
}

int rfc_ht_new(int bw, struct vga_tbl *vga_table, struct rfc_record_parm *parm_record , int test_case_no, int tone_mask, int debug_en)
{
    double tones_freq_20[FREQ_QUANTITY_20MHZ] = RFC_TONE_LIST_20;
    double tones_freq_40[FREQ_QUANTITY_40MHZ] = RFC_TONE_LIST_40;
    double *tones_freq;

    int i=0, j=0, k=0;
	int freq_quantity;
	int agc_en=0, lpf_reset=0, lpf_sel=0;
    double f0;
    double phase_ref=0;
    unsigned char tg_a_i=0, tg_a_q=0;
	unsigned short rxvga=0;

    mismatch_info param;
    complex I_signal, Q_signal;
    complex I_signal_f0, Q_signal_f0;
    complex neg_f0, pos_f0;
	complex *tx_signal, *tx_signal_f0;
    double rx_caldata[2];

	struct bal_parm tx_bal[FREQ_QUANTITY_40MHZ], rx_bal[FREQ_QUANTITY_40MHZ];
	struct rfc_cal_parm calc_ret;

	struct vga_entry *tx_table, *rx_table;
	unsigned char buf[16];
	(void) buf;

	if(bw)	/* 40MHz */
	{
		tx_table = vga_table->txcal_40mhz;
		rx_table = vga_table->rxcal_40mhz;
		freq_quantity = FREQ_QUANTITY_40MHZ;
		tones_freq = tones_freq_40;
		set_sampling_rate(1);
	}
	else	/* 20MHz */
	{
		tx_table = vga_table->txcal_20mhz;
		rx_table = vga_table->rxcal_20mhz;
		freq_quantity = FREQ_QUANTITY_20MHZ;
		tones_freq = tones_freq_20;
		set_sampling_rate(0);
	}

	if(bb_register_read(0x2) & 0x2)
	{
		tx_signal = &I_signal;
		tx_signal_f0 = &I_signal_f0;
	}
	else
	{
		tx_signal = &Q_signal;
		tx_signal_f0 = &Q_signal_f0;
	}
	
	for (i=0; i<freq_quantity; i++)
	{
		if(!(tone_mask & (1 << i)))
			continue;
        
		bb_rfc_reset();
		RFC_DBG("============ RFC test %d : bw = %d, freq. (i) = %d\n", test_case_no, bw, i);
#if 1 // TX CAL.
		/* Should confirm the iq swap setting to use I signal or Q signal in tx calibration */
		
		//----------TX Calibration----------
		ip301_set_iqcal_vga(TXLOOP, tx_table[i].rxvga, tx_table[i].txvga);
	
		//Init TX Calibration variables
		param.gain.real = 1; 
		param.gain.imag = 1; 
		param.phi = 0;
		modtest_iq_balancer_new(&param, 1);
	
		tx_cal_LPFout_new(1, I_signal, &param, -1); //reset tx variables. I_signal, &param, -1 are useless
	
		f0 = tones_freq[i];
		rxvga = tx_table[i].rxvga;

		k = 0;	/* If "wrong angle" happen, re-calibrate one time.*/
		do{
			for (j=0; j<3; j++)
			{
				//RFC_DBG("============ RFC test %d : bw = %d, freq. (i) = %d, j = %d\n", test_case_no, bw, i, j);
				//Generate Tone
				switch(j)
				{
					case 0:
						tg_a_i = tx_table[i].bb_scale;
						tg_a_q = 15;
						agc_en = 1;
						lpf_reset = 1;
						lpf_sel = 0;
						break;
					case 1:
						tg_a_i = 15;
						tg_a_q = tx_table[i].bb_scale;
						agc_en = 0;
						lpf_reset = 1;
						lpf_sel = 0;
						break;
					case 2:
						tg_a_i = tx_table[i].bb_scale;
						tg_a_q = tx_table[i].bb_scale;
						agc_en = 0;
						lpf_reset = 0;
						lpf_sel = 0;
						break;
#if 0
					case 3:
					case 4:
						tg_a_i = tx_table[i].bb_scale;
						tg_a_q = tx_table[i].bb_scale;
						agc_en = 0;
						lpf_reset = 1;
						lpf_sel = 0;
						break;
#endif
				}

				//Program Balancer
				bal_regs_new(param, TX_BAL);
				
				tx_loopback_report_new(bw, i,  tg_a_i,  tg_a_q,  tx_table[i].txvga, rxvga, 65535, 65535, 65535, 65535, 65535, agc_en, NULL, tx_signal, lpf_reset, lpf_sel);
				
				if(agc_en)
					READ_RXVGA(rxvga);

#if defined(RFC_DEBUG)
				if(debug_en & 0x1)
				{
					RFC_DBG("TX CAL. freq %d, j=%d, press ENTER to continue.\n", i, j);
					WLA_GETS(buf);
					if(strncmp(buf, "c", 1) == 0)
						return 1;
				}
#endif	// RFC_DEBUG
				//Calculation gain/phase mismatches
				// j values: GAIN_I_CAL 0 /GAIN_Q_CAL 1 /PHI_CAL_1 2 /PHI_CAL_2 3 /PHI_CAL_3 4
				phase_ref = tx_cal_LPFout_new(0, *tx_signal, &param, j); 
			}
		} while((k++ < 1) && (phase_ref > 360));
		
		if(!(debug_en & 0x2))
		{
			tg_a_q = 1;
			if(tx_fine_tune_new(&param, phase_ref, 5, bw, i, tg_a_q, tx_table[i].txvga, 0.007, 1.008, debug_en))
				return 1;
		}
		
		//Save mismatch results
		tx_bal[i].gain.real = param.gain.real;
		tx_bal[i].gain.imag = param.gain.imag;
		tx_bal[i].phi = param.phi;

		//Program TX Balancer
		bal_regs_new(param, TX_BAL);
#endif 
        //----------RX Calibration----------
#if 1 // RX CAL.
		rx_loopback_report_new(bw, i, rx_table[i].bb_scale, rx_table[i].txvga, rx_table[i].rxvga,
						65535, 65535, 65535, 65535, 65535, 1, &neg_f0, &pos_f0, 1, 0);
#if defined(RFC_DEBUG)
		if(debug_en & 0x1)
		{
			RFC_DBG("RX CAL.,press ENTER to continue.\n");
			WLA_GETS(buf);
			if(strncmp(buf, "c", 1) == 0)
				return 1;
		}
#endif	// RFC_DEBUG
        //Calculation gain/phase mismatches
        cmp_iq_mismatch(pos_f0, neg_f0, rx_caldata);
        param.gain.real = 1;
        param.gain.imag = rx_caldata[0];
        param.phi = rx_caldata[1];

        //Save mismatch results
        rx_bal[i].gain.real = param.gain.real;
        rx_bal[i].gain.imag = param.gain.imag;
        rx_bal[i].phi = param.phi;

        //Reset tx to normal mode
        tx_mux_regs(MUX_BASEBAND, 0, 0, 0); //unsigned char tg_a_i, unsigned char tg_a_q, double ctl_coe are useless
#endif
#if 0
        //Check results
        RFC_DBG("============ RFC test %d result: freq. (i) = %d\n", test_case_no, i);
        RFC_DBG("Tx mismatch : ");
        RFC_DBG_DOUBLE(20*log10(tx_bal[i].gain.imag)); RFC_DBG(" db, ");
        RFC_DBG_DOUBLE((tx_bal[i].phi*180)/3.1415926); RFC_DBG(" degree\n");
        RFC_DBG("Rx mismatch : ");
        RFC_DBG_DOUBLE(20*log10(rx_bal[i].gain.imag)); RFC_DBG(" db, ");
        RFC_DBG_DOUBLE((rx_bal[i].phi*180)/3.1415926); RFC_DBG(" degree\n");
        RFC_DBG("============================================\n\n");
#endif
	}

#ifdef TONE_MASK_ENABLE
	freq_quantity = check_parm_with_mask(tx_bal, rx_bal, tone_mask, 0, freq_quantity);
#endif

	// cal the tx/rx gain & phi mean
	compute_balancer_new(tx_bal, rx_bal, &calc_ret, freq_quantity, bw);

	// Store the tx calibration result
	transfer_twofilter_regs_new(&calc_ret, &rfc_result_new[bw]);
	
	//bb_register_write(0x1c, 0x0);
#if defined(RFC_DEBUG)
	RFC_DBG("txgain=[");
	for(i=0; i<freq_quantity; i++)
	{
        RFC_DBG_DOUBLE(20*log10(tx_bal[i].gain.imag)); RFC_DBG(" \n");
	}
	RFC_DBG("];\n");
	RFC_DBG("txphase=[");
	for(i=0; i<freq_quantity; i++)
	{
        RFC_DBG_DOUBLE((tx_bal[i].phi*180)/3.1415926); RFC_DBG(" \n");
	}
	RFC_DBG("];\n");
	RFC_DBG("rxgain=[");
	for(i=0; i<freq_quantity; i++)
	{
        RFC_DBG_DOUBLE(20*log10(rx_bal[i].gain.imag)); RFC_DBG(" \n ");
	}
	RFC_DBG("];\n");
	RFC_DBG("rxphase=[");
	for(i=0; i<freq_quantity; i++)
	{
        RFC_DBG_DOUBLE((rx_bal[i].phi*180)/3.1415926); RFC_DBG(" \n");
	}
	RFC_DBG("];\n");

	//Check results
    RFC_DBG("\n============ RFC test %d final result:\n", test_case_no);
	RFC_DBG("Tx mismatch : ");
	RFC_DBG_DOUBLE(20*log10(calc_ret.tx_avg.gain.imag)); RFC_DBG(" db, ");
	RFC_DBG_DOUBLE((calc_ret.tx_avg.phi*180)/3.1415926); RFC_DBG(" degree\n");
	RFC_DBG("Rx mismatch : ");
	RFC_DBG_DOUBLE(20*log10(calc_ret.rx_avg.gain.imag)); RFC_DBG(" db, ");
	RFC_DBG_DOUBLE((calc_ret.rx_avg.phi*180)/3.1415926); RFC_DBG(" degree\n");
	RFC_DBG("\n\n\n");
#endif //	RFC_DEBUG

	parm_record->tx_gain = (20*log10(calc_ret.tx_avg.gain.imag));
    parm_record->tx_phase = ((calc_ret.tx_avg.phi*180)/3.1415926);
    parm_record->rx_gain = (20*log10(calc_ret.rx_avg.gain.imag));
    parm_record->rx_phase = ((calc_ret.rx_avg.phi*180)/3.1415926);

	return 0;
}

/* phase_step/gain_step should be large enough to get correctly process */
int tx_fine_tune_new(mismatch_info *tx_result, double phase_ref, int loop_max, int bw, int freq,
		unsigned short tg, unsigned short txvga, double phase_step, double gain_step, int debug_en)
{
	int n_count=0, i=0;
	int dir=0;
	unsigned short rxvga;
	unsigned short tx_nm;
#if defined(RFC_DEBUG)
	unsigned char buf[16];
#endif
	complex read_2f0;
	double phase_diff;
	double phase_out;
	double fine_2f0_mag;
	double new_2f0_mag;
	double a21=0, a22=0;
	mismatch_info  tx_result_copy;

	memcpy(&tx_result_copy, tx_result, (sizeof(mismatch_info)));

#if 0
	RFC_DBG("********** in tx_fine_tune_new() tx_result_copy: **********\n");
	RFC_DBG("gain = ");RFC_DBG_COMPLEX(tx_result_copy.gain);RFC_DBG("\n");
	RFC_DBG("phi = ");RFC_DBG_DOUBLE(tx_result_copy.phi);RFC_DBG("\n");
	RFC_DBG("a11 = ");RFC_DBG_DOUBLE(tx_result_copy.a11);RFC_DBG("\n");
	RFC_DBG("a12 = ");RFC_DBG_DOUBLE(tx_result_copy.a12);RFC_DBG("\n");
	RFC_DBG("a21 = ");RFC_DBG_DOUBLE(tx_result_copy.a21);RFC_DBG("\n");
	RFC_DBG("a22 = ");RFC_DBG_DOUBLE(tx_result_copy.a22);RFC_DBG("\n");
	RFC_DBG("n = ");RFC_DBG_DOUBLE(tx_result_copy.n);RFC_DBG("\n");
	RFC_DBG("m = ");RFC_DBG_DOUBLE(tx_result_copy.m);RFC_DBG("\n");
#endif
	while((n_count++ < loop_max))
	{
		tx_nm = tx_result_copy.n*4 + tx_result_copy.m;

		tx_loopback_report_new(bw, freq,  tg, tg, txvga, 16, 65535,  65535,
							tx_nm, tx_result_copy.a21, tx_result_copy.a22, 1, NULL, 
							&read_2f0, 0, 0);

		fine_2f0_mag = c_square(&read_2f0);

		phase_out = calc_angle(read_2f0.real, read_2f0.imag);
		phase_diff = angle_diff(phase_out, phase_ref);
		dir = angle_phase(phase_diff);

		i=0;
		do {
			a21 = tx_result_copy.a21;
			a22 = tx_result_copy.a22;

		 	// generate new floating point gain/phase coefficient according to last step
			if (dir == ANGLE_180)
				tx_result_copy.gain.imag = tx_result_copy.gain.imag/gain_step;
			else if(dir == ANGLE_270) 
				tx_result_copy.phi = tx_result_copy.phi + phase_step;
			else if(dir == ANGLE_0)
				tx_result_copy.gain.imag = tx_result_copy.gain.imag * gain_step;
			else // (dir == ANGLE_90)
				tx_result_copy.phi = tx_result_copy.phi - phase_step;
			
			// generate new fixed point reg value accroding to last step.
			modtest_iq_balancer_new(&tx_result_copy, 1);

			if(++i >= 10)
				break;
		} while((abs_double(tx_result_copy.a21 - a21) <= 1) && (abs_double(tx_result_copy.a22 - a22) <= 1));
#if 0
		RFC_DBG("The Tx mismatch : ");
		RFC_DBG_DOUBLE(20*log10(tx_result_copy.gain.imag)); RFC_DBG(" db, ");
		RFC_DBG_DOUBLE((tx_result_copy.phi*180)/3.1415926); RFC_DBG(" degree\n");
		RFC_DBG("a12 = ");RFC_DBG_DOUBLE(tx_result_copy.a12);RFC_DBG("\n");
		RFC_DBG("a22 = ");RFC_DBG_DOUBLE(tx_result_copy.a22);RFC_DBG("\n");
		RFC_DBG("n = ");RFC_DBG_DOUBLE(tx_result_copy.n);RFC_DBG("\n");
		RFC_DBG("m = ");RFC_DBG_DOUBLE(tx_result_copy.m);RFC_DBG("\n");
#endif	
		tx_nm = tx_result_copy.n*4 + tx_result_copy.m;

		READ_RXVGA(rxvga);
		tx_loopback_report_new(bw, freq, tg, tg, txvga, rxvga, 65535,  65535,
			tx_nm, tx_result_copy.a21, tx_result_copy.a22, 0, NULL, &read_2f0, 0, 2);
		new_2f0_mag = c_square(&read_2f0);

#if defined(RFC_DEBUG)
		if(debug_en & 0x1)
		{
			RFC_DBG("%s(), press ENTER to continue.\n", __FUNCTION__);
			WLA_GETS(buf);
			if(strncmp(buf, "c", 1) == 0)
				return 1;
		}
#endif //	RFC_DEBUG
		// if new_2f0_mag < fine_2f0_mag  that means new value is better
#if 0
		RFC_DBG("new_2f0_mag = ");RFC_DBG_DOUBLE(new_2f0_mag);
		RFC_DBG(", fine_2f0_mag = ");RFC_DBG_DOUBLE(fine_2f0_mag);RFC_DBG("\n");
#endif
		if (new_2f0_mag < fine_2f0_mag)
		{
			memcpy(tx_result, &tx_result_copy, sizeof(mismatch_info));
			//RFC_DBG("********** end this round : get better gain/phi **********\n");
		}
		else  //(old value is better, fine tune is over)
			break;
	}

	return 0;
}

/* rfc_process_new: for rf calibration when device boot up */
void rfc_process_new(int op)
{
	struct  vga_tbl vga_table;
	//double tx_iq_mis[2][2];
	struct rfc_test_record record;
	int bw=0;
	unsigned int lmac_tstart=0, reg0;
	unsigned char bb_reg_f3=0;

	rfac_rfc_patch_check();

	/* store rf reg 0 & restore it after rfc */
	reg0 = ip301_spi_read(0x0);
	/* store bb reg f3 & restore it after rfc */
	/* FIXME: RF CHIP J move this config in the lmac? */
	bb_reg_f3 = bb_register_read(0xf3);
	if(bb_reg_f3 & 0x40)
		bb_register_write(0xf3, bb_reg_f3 & 0xBF);
	
	/* setup lmac to do calibration */
	if(!op)		// op==1 : rfc_onfly
	{
		lmac_tstart = MACREG_READ32(LMAC_CNTL) & LMAC_CNTL_TSTART;
		MACREG_UPDATE32(LMAC_CNTL, 0, LMAC_CNTL_TSTART);
	}
	
	/* setup rfc environment */
	rfc_env_setup(&vga_table, 0, 0, 0, 0);
	
	/* fine tune tx lo dc */
	txlo_cal_new();

	/* Pre-check the calibration result */
	//tx_cal_result_check(0, &vga_table, &(tx_iq_mis[0][0]), 0);

	for(bw=0; bw<2; bw++)
	{
		/* rfc main function */
		rfc_ht_new(bw, &vga_table, &(record.parm[0]), 0, 0xffff, 0);
		config_rfc_parm_new(bw);
	}

	/* disable the tx/rx loopback */
	ip301_spi_write(0x0, reg0);
	
	/* restore bb reg 0xf3 */
	bb_register_write(0xf3, bb_reg_f3);

	/* restore the 20/40 MHz setting for current environment */
	set_sampling_rate((current_channel_width == CHANNEL_WIDTH_40MHZ));

	/* restore lmac setting to exit calibration */
	if(lmac_tstart && !op)	// op==1 : rfc_onfly
		MACREG_UPDATE32(LMAC_CNTL, lmac_tstart, LMAC_CNTL_TSTART);
}

#ifdef CONFIG_RFC_ONFLY
void rfc_onfly(int channel, int secondary_channel)
{
	WMAC_WAIT_FOREVER((MACREG_READ32(LMAC_CNTL) & LMAC_CNTL_CAL_GNT), 
				"waiting on LMAC_CNTL_CAL_GNT to low before set LMAC_CNTL_CAL_REQ\n");
	MACREG_UPDATE32(LMAC_CNTL, LMAC_CNTL_CAL_REQ, LMAC_CNTL_CAL_REQ);
	WMAC_WAIT_FOREVER(!(MACREG_READ32(LMAC_CNTL) & LMAC_CNTL_CAL_GNT), 
				"waiting on LMAC_CNTL_CAL_GNT to high before rfc calibration\n");
    
	if(rf_calibration(RF_CAL_CH, my_rf_dev->internal_ldo) < 0)
		return;

	ip301_b_recover();
	
	rfc_process_new(1);
	bb_init();
	wla_cfg(SET_CHANNEL, channel, secondary_channel);

	MACREG_UPDATE32(LMAC_CNTL, 0x0, LMAC_CNTL_CAL_REQ);
}
#endif // CONFIG_RFC_ONFLY

/*Linux cannot compile*/
#if defined(CONFIG_RFC_ANALYST) && defined(RFC_DEBUG)
int rfc_ht_cmd_new(int argc, char* argv[])
{
    int samples;
    int i;
	int bw = 0, bw_start = 0, bw_end=1; /* 0: 20MHZ, 1: 40MHz */
	int ret_tx, ret_rx;
	int tx_scale=0, rx_scale=0;
	int tx_txvga=0, rx_txvga=0;
	struct  vga_tbl *vga_table;
	unsigned int lmac_tstart;
	unsigned char bb_reg_f3=0;

	int tone_mask=0xffff, debug_en=0, bandwidth=0x3, repeat=0;
	
	/* For calibration record */
	struct rfc_test_record *record;
	struct mismatch_check *mismatch;
	double tx_iq_mis[2][2];
	
	record = (struct rfc_test_record *) kmalloc(sizeof(struct rfc_test_record)*MAX_RFC_ITERATIONS, GFP_KERNEL);
	mismatch = (struct mismatch_check *) kmalloc(sizeof(struct mismatch_check), GFP_KERNEL);
	vga_table = (struct vga_tbl *) kmalloc(sizeof(struct vga_tbl), GFP_KERNEL);

	//libm_test();

	switch(argc)
	{
		case 10:
			sscanf(argv[9],"%d",&tx_txvga);
		case 9:
			sscanf(argv[8],"%d",&rx_txvga);
		case 8:
			sscanf(argv[7],"%d",&tx_scale);
		case 7:
			sscanf(argv[6],"%d",&rx_scale);
		case 6:
			sscanf(argv[5],"%d",&debug_en);
		case 5:
			sscanf(argv[4],"%d",&bandwidth);
			if(!(bandwidth & 0x1))
				bw_start = 1;
			if(!(bandwidth & 0x2))
				bw_end = 0;
		case 4:
			sscanf(argv[3],"%d",&tone_mask);
		case 3:
			sscanf(argv[2],"%d",&repeat);
		case 2:
			break;
		default:
			RFC_DBG("Wrong argc!!!\n");
			return 0;
	}

	/* store bb reg f3 & restore it after rfc */
	bb_reg_f3 = bb_register_read(0xf3);
	if(bb_reg_f3 & 0x40)
		bb_register_write(0xf3, bb_reg_f3 & 0xBF);

	if(repeat == 65535)
		rx_dc_offset_comp();
		
	RFC_DBG("repeat = %d, tone_mask = 0x%x, bw = %d, bw_start = %d, bw_end = %d, debug = %d, rx_scale = %d, tx_scale = %d\n", repeat, tone_mask, bandwidth, bw_start, bw_end, debug_en, rx_scale, tx_scale);

	lmac_tstart = MACREG_READ32(LMAC_CNTL) & LMAC_CNTL_TSTART;
    MACREG_UPDATE32(LMAC_CNTL, 0x0, LMAC_CNTL_TSTART);
    
	if((argc==2) || (repeat==1))
    {
		rfc_env_setup(vga_table, rx_scale, tx_scale, rx_txvga, tx_txvga);
		/* Before tx calibration, record LPF data. */
		for(bw=bw_start; bw<=bw_end; bw++)
		{	
			set_sampling_rate(bw);
			RFC_DBG("========= RX PreCheck =========\n");
			ret_rx = rx_cal_result_check(bw, vga_table);
			RFC_DBG("========= TX PreCheck =========\n");
			ret_tx = tx_cal_result_check(bw, vga_table, &(tx_iq_mis[0][0]), 0);
		}
		
		for(bw=bw_start; bw<=bw_end; bw++)
		{
		
			if(rfc_ht_new(bw, vga_table, &(record[0].parm[bw]), 0, tone_mask, debug_en))
				return 0;
		
			config_rfc_parm_new(bw);
			
			set_sampling_rate(bw);
			ret_tx = tx_cal_result_check(bw, vga_table, &(tx_iq_mis[0][0]), 1);
			if(ret_tx == 2)
				return 0;
			ret_rx = rx_cal_result_check(bw, vga_table);
			if(ret_rx == 2)
				return 0;

			if(bw)
				RFC_DBG("========= 40MHz Test Result =========\n");
			else
				RFC_DBG("========= 20MHz Test Result =========\n");
			RFC_DBG("TX Gain:"); RFC_DBG_DOUBLE(record[0].parm[bw].tx_gain); RFC_DBG(" db  ");
			RFC_DBG("Phase:");  RFC_DBG_DOUBLE(record[0].parm[bw].tx_phase); RFC_DBG(" degree\n");
			RFC_DBG("RX Gain:"); RFC_DBG_DOUBLE(record[0].parm[bw].rx_gain); RFC_DBG(" db  ");
			RFC_DBG("Phase:");  RFC_DBG_DOUBLE(record[0].parm[bw].rx_phase); RFC_DBG(" degree\n");
			RFC_DBG("=====================================\n");
			RFC_DBG("== The rx calibration result ");
			if(ret_rx)
				RFC_DBG("Fail!!! ==\n");
			else
				RFC_DBG("Success!!! ==\n");
			RFC_DBG("== The tx calibration result ");
			if(ret_tx)
				RFC_DBG("Fail!!! ==\n");
			else
				RFC_DBG("Success!!! ==\n");
			RFC_DBG("=====================================\n");
		}
    }
    else
    {
        if((repeat > 0) && (repeat <= MAX_RFC_ITERATIONS))
        {
			samples = repeat;
			
			for(i=0;i<samples;i++)
			{
				for(bw=bw_start; bw<=bw_end; bw++)
				{
					RFC_DBG("===== Calibration No.%d (bw = %d)=====\n", i, bw);
					
					rfc_env_setup(vga_table, rx_scale, tx_scale, rx_txvga, tx_txvga);

					if(rfc_ht_new(bw, vga_table, &(record[i].parm[bw]), i, tone_mask, debug_en))
						return 0;

					config_rfc_parm_new(bw);
					RFC_DBG("===== Calibration No.%d result (bw = %d) =====\n", i, bw);
					RFC_DBG("TX Gain:"); RFC_DBG_DOUBLE(record[i].parm[bw].tx_gain); RFC_DBG(" db  ");
					RFC_DBG("Phase:");  RFC_DBG_DOUBLE(record[i].parm[bw].tx_phase); RFC_DBG(" degree\n");
					RFC_DBG("RX Gain:"); RFC_DBG_DOUBLE(record[i].parm[bw].rx_gain); RFC_DBG(" db  ");
					RFC_DBG("Phase:");  RFC_DBG_DOUBLE(record[i].parm[bw].rx_phase); RFC_DBG(" degree\n");

					rx_iq_mismatch_check_ht(bw, i, &(mismatch->rx_iq_mismatch[bw].rec[i][0]));
					tx_iq_mismatch_check_ht(bw, i, &(mismatch->tx_iq_mismatch[bw].rec[i][0]));
				}
			}

			for(bw=bw_start; bw<=bw_end; bw++)
				print_rfc_test_result(record, mismatch, samples, bw);
        }
        else
        {
            RFC_DBG("invalid repeat value %d\n", repeat);
        }
    }
	
	kfree(record);
	kfree(mismatch);
	kfree(vga_table);

	MACREG_UPDATE32(LMAC_CNTL, lmac_tstart, LMAC_CNTL_TSTART);
    
	/* restore bb reg 0xf3 */
	bb_register_write(0xf3, bb_reg_f3);
	
	/* restore the 20/40 MHz setting for current environment */
	set_sampling_rate((current_channel_width == CHANNEL_WIDTH_40MHZ));
	
	return 0;
}
#endif
void config_rfc_parm_new(int bw)
{
	struct rfc_cal_reg *reg;
	int manual=0, i=0;
	u8 reg5a;

	//RFC_DBG("Function : %s(), bw = %d\n", __FUNCTION__, bw);
	
	if(bw & 0x100)
		manual = 1;
	
	bw &= 0x1;

	reg = &rfc_result_new[bw];
	reg5a = reg->filter_switch;

	bb_register_write(0x20, reg->balancer_nm);
	bb_register_write(0x21, reg->tx_a11);
	bb_register_write(0x22, reg->tx_a12);
	bb_register_write(0x23, reg->tx_a21);
	bb_register_write(0x24, reg->tx_a22);
	bb_register_write(0x25, reg->tx_dc_i);
	bb_register_write(0x28, reg->rx_a21);
	bb_register_write(0x29, reg->rx_a22);
	bb_register_write(0x2a, reg->rx_dc_i);
	bb_register_write(0x30, reg->tx_dc_q);
	bb_register_write(0x31, reg->rx_dc_q);
	//bb_register_write(0x5a, reg->filter_switch);
//printd("@@@@@@@@@@@ Disable rx phaseshifter BB 0x5a = %x\n", (reg->filter_switch & 0xf0));
	bb_register_write(0x5a, reg5a);
	bb_register_write(0x5b, reg->rolloff_rx_coe);
	bb_register_write(0x5c, reg->phaseshifter_rx_alfa);
	bb_register_write(0x5d, reg->rolloff_tx_coe);
	bb_register_write(0x5e, reg->phaseshifter_tx_alfa);

	/* manual BB setting */ 
	if(manual)
	{
		for(i=0; i<MAX_RFC_REG_NUM; i++)
		{
			if(rfc_tbl[i].num == 0)
				break;
			WLA_DBG(WLADEBUG, "BB write rfc 0x%02x=0x%02x\n", rfc_tbl[i].num,rfc_tbl[i].val);
			bb_register_write(rfc_tbl[i].num, rfc_tbl[i].val);
		}
	}
#if 0
	BBDBG(0x20);
	BBDBG(0x21);
	BBDBG(0x22);
	BBDBG(0x23);
	BBDBG(0x24);
	BBDBG(0x25);
	BBDBG(0x28);
	BBDBG(0x29);
	BBDBG(0x30);
	BBDBG(0x31);
	BBDBG(0x5a);
	BBDBG(0x5b);
	BBDBG(0x5c);
	BBDBG(0x5d);
	BBDBG(0x5e);
#endif
}

void tx_loopback_report_new(int bw, int freq, unsigned short tg_a_i, unsigned short tg_a_q, unsigned short txvga,
						unsigned short rxvga, unsigned short tx_i_dc, unsigned short tx_q_dc, 
						unsigned short tx_nm, unsigned short br23_phase, unsigned short br24_gain,
						int agc_en, complex *read_f0, complex *read_2f0, int lpf_reset, int lpf_sel)
{
	unsigned char val;
    complex I_signal, Q_signal;
	complex *tx_signal;
    double tones_freq_20[FREQ_QUANTITY_20MHZ] = RFC_TONE_LIST_20;
    double tones_freq_40[FREQ_QUANTITY_40MHZ] = RFC_TONE_LIST_40;
	double ctl_coe;

	/* patch for rf noise tone effect, for IP301_G & IP301_E */
	u32 rf_reg6 = 0;
	rf_reg6 = rf_noise_tone_patch(PATCH_START, rf_reg6);

	/* select the signal for calc */
	if(bb_register_read(0x2) & 0x2)
		tx_signal = &I_signal;
	else
		tx_signal = &Q_signal;
#if 0
	RFC_DBG("The tx_loopback_report_new parm: bw = %d, tg_a_i = %d, tg_a_q = %d\n", bw, tg_a_i, tg_a_q);
	RFC_DBG("txvga = %d, rxvga = %d, tx_i_dc = 0x%x, tx_q_dc = 0x%x\n", txvga, rxvga, tx_i_dc, tx_q_dc);
	RFC_DBG("tx_nm = 0x%x, br23_phase = 0x%x, br24_gain = 0x%x, agc_en = %d\n", 
			 tx_nm, br23_phase, br24_gain, agc_en);
	RFC_DBG("lpf_reset = %d, lpf_sel = %d\n", lpf_reset, lpf_sel);
#endif

	if(bw)
		ctl_coe = tones_freq_40[freq];
	else
		ctl_coe = tones_freq_20[freq];
	
	ip301_set_iqcal_vga(TXLOOP, rxvga, txvga);
	tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);
	
	if(agc_en)
	{
		rxvga = rxvga_adjust(RXVGA_OVTH, 85);
		//RFC_DBG("the new rxvga = %d\n", rxvga);
		ip301_set_iqcal_vga(TXLOOP, rxvga, txvga);
	}

	/* setup bb reg */
	if(tx_nm < 65535)
	{
		val = bb_register_read(0x20);
		bb_register_write(0x20, (val & 0xf) | ((tx_nm & 0xf) << 4));
	}
	if(br23_phase < 65535)
		bb_register_write(0x23, br23_phase);
	if(br24_gain < 65535)
		bb_register_write(0x24, br24_gain);
	if(tx_i_dc < 65535)
		bb_register_write(0x25, tx_i_dc);
	if(tx_q_dc < 65535)
		bb_register_write(0x30, tx_q_dc);
		
	if(read_f0)
	{
		rx_demod_regs(0x3, ctl_coe);
		read_LPF(&I_signal, &Q_signal, lpf_reset, lpf_sel);
		memcpy(read_f0, tx_signal, sizeof(complex));
		//RFC_DBG("f0: ");RFC_DBG_COMPLEX(*tx_signal);RFC_DBG("\n");
	}
	if(read_2f0)
	{
		ctl_coe_calc(2, ctl_coe, &ctl_coe);
		rx_demod_regs(0x3, ctl_coe);
		read_LPF(&I_signal, &Q_signal, lpf_reset, lpf_sel);
		memcpy(read_2f0, tx_signal, sizeof(complex));
		//RFC_DBG("2f0: ");RFC_DBG_COMPLEX(*tx_signal);RFC_DBG("\n");
	}
	
	/* revert the patch for rf noise tone effect */
	rf_noise_tone_patch(PATCH_END, rf_reg6);
}

void rx_loopback_report_new(int bw, int freq, unsigned short tg_a,  unsigned short txvga,
						unsigned short rxvga, unsigned short rx_i_dc, unsigned short rx_q_dc,
						unsigned short rx_nm, unsigned short br28_phase, unsigned short br29_gain,
						int agc_en, complex *neg_f0, complex *pos_f0, int lpf_reset, int lpf_sel)
{
    double tones_freq_20[FREQ_QUANTITY_20MHZ] = RFC_TONE_LIST_20;
    double tones_freq_40[FREQ_QUANTITY_40MHZ] = RFC_TONE_LIST_40;
	double ctl_coe;
	int cal_mode_sel = 0x0;
	unsigned char val;
	
	if(bw)
		ctl_coe = tones_freq_40[freq];
	else
		ctl_coe = tones_freq_20[freq];
	
	/* setup bb reg */
	if(rx_nm < 65535)
	{
		val = bb_register_read(0x20);
		bb_register_write(0x20, (val & 0xf0) | (rx_nm & 0xf));
	}
	if(br28_phase < 65535)
		bb_register_write(0x28, br28_phase);
	if(br29_gain < 65535)
		bb_register_write(0x29, br29_gain);
	if(rx_i_dc < 65535)
		bb_register_write(0x2a, rx_i_dc);
	if(rx_q_dc < 65535)
		bb_register_write(0x31, rx_q_dc);
	
	ip301_set_iqcal_vga(RXLOOP, rxvga, txvga);

	//Generate Tone
	tx_mux_regs(MUX_TONEGEN, tg_a, tg_a, ctl_coe);

	if(agc_en)
	{
		rxvga = rxvga_adjust(RXVGA_OVTH, 85);
		//RFC_DBG("the new rxvga = %d\n", rxvga);
		ip301_set_iqcal_vga(RXLOOP, rxvga, txvga);
	}
	
	//Configure RX Demod & LPF
	rx_demod_regs(cal_mode_sel, ctl_coe);

	//Read LPF data
	read_LPF(neg_f0, pos_f0, lpf_reset, lpf_sel);
}

void txlo_cal_new(void)
{
	complex f0_result, f0_result_positive, f0_result_negative;
	complex f02_result, f02_result_positive, f02_result_negative;
	int i=0;
	int bb_scale = 3, txvga = 7, rxvga = 7;
	int tone_freq=5, bw=0;
	int tx_nm=0;
	int br22_phase = 0;
	int br24_gain = 127;
	int bb_max_amp = 512 >> bb_scale;
	char dc_i_norm_amp , dc_q_norm_amp;
	char real_dc_est=0;
	char real_dc_est_last=bb_register_read(0x25);
	char real_dc_est_pos=0;
	char real_dc_est_neg=0;
	char imag_dc_est=0;
	char imag_dc_est_last=bb_register_read(0x30);
	char imag_dc_est_pos=0;
	char imag_dc_est_neg=0;

	/*------------TX LO leakage cancel first step--------*/
	for(i=0; i<2; i++)
	{
	    //----------------send I part first-----------------
		tx_loopback_report_new(bw, tone_freq, bb_scale, 15, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, 
							br22_phase, br24_gain, 1, &f0_result, &f02_result, 1, 0);
	
		//calc dc_i_norm_amp
		dc_i_norm_amp = calc_abs_txlo(bb_max_amp, &f0_result, &f02_result, 1);
		RFC_DBG("!! dc_i_norm_amp = %u\n", dc_i_norm_amp);

		COMPLEX_DIV(f0_result, abs_complex(&f02_result));
			
		//check I part 
		//set real_dc_est= + dc_i_norm_amp
		real_dc_est_pos = real_dc_est_last + dc_i_norm_amp;
		//transfer to 8bit, It will  write to the BB reg later
		real_dc_est = real_dc_est_pos; 

		tx_loopback_report_new(bw, tone_freq, bb_scale, 15, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, br22_phase, br24_gain, 1, &f0_result_positive, &f02_result_positive, 1, 0);

		COMPLEX_DIV(f0_result_positive, abs_complex(&f02_result_positive));

		//set real_dc_est = -dc_i_norm_amp
		real_dc_est_neg = real_dc_est_last - dc_i_norm_amp;        
		//transfer to 8bit, It will  write to the BB reg later
		real_dc_est = real_dc_est_neg;   

		tx_loopback_report_new(bw, tone_freq, bb_scale, 15, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, 
							br22_phase, br24_gain, 1, &f0_result_negative, &f02_result_negative, 1, 0);

		COMPLEX_DIV(f0_result_negative, abs_complex(&f02_result_negative));

		// compare 2 result
		real_dc_est = txlo_pos_neg(real_dc_est_pos, real_dc_est_neg, &f0_result_positive, &f0_result_negative, &f0_result);
		real_dc_est_last = real_dc_est;
		RFC_DBG("!!!! real_dc_est = %u\n", real_dc_est);

		//--------------------send Q part 2nd-------------------
		tx_loopback_report_new(bw, tone_freq, 15, bb_scale, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, 
							br22_phase, br24_gain, 1, &f0_result, &f02_result, 1, 0);

		//calc dc_q_norm_amp
		dc_q_norm_amp = calc_abs_txlo(bb_max_amp, &f0_result, &f02_result, 1);
		RFC_DBG("!! dc_q_norm_amp = %u\n", dc_q_norm_amp);

		COMPLEX_DIV(f0_result, abs_complex(&f02_result));
				
		//check Q part 
		//set imag_dc_est= + dc_i_norm_amp
		imag_dc_est_pos = imag_dc_est_last + dc_q_norm_amp;
		//transfer to 8bit, It will  write to the BB reg later
		imag_dc_est = imag_dc_est_pos;    

		tx_loopback_report_new(bw, tone_freq, 15, bb_scale, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, 
							br22_phase, br24_gain, 1, &f0_result_positive, &f02_result_positive, 1, 0);

		COMPLEX_DIV(f0_result_positive, abs_complex(&f02_result_positive));
			
		//%set imag_dc_est = - dc_q_norm_amp
		imag_dc_est_neg = imag_dc_est_last - dc_q_norm_amp;
		imag_dc_est = imag_dc_est_neg;
			
		tx_loopback_report_new(bw, tone_freq, 15, bb_scale, txvga, rxvga, real_dc_est, imag_dc_est, tx_nm, 
							br22_phase, br24_gain, 1, &f0_result_negative, &f02_result_negative, 1, 0);

		COMPLEX_DIV(f0_result_negative, abs_complex(&f02_result_negative));
				
		//compare 2 result
		imag_dc_est = txlo_pos_neg(imag_dc_est_pos, imag_dc_est_neg, 
									&f0_result_positive, &f0_result_negative, &f0_result );
		RFC_DBG("f0_result_positive = ");RFC_DBG_COMPLEX(f0_result_positive);RFC_DBG("\n");
		RFC_DBG("f0_result_negative = ");RFC_DBG_COMPLEX(f0_result_negative);RFC_DBG("\n");
		RFC_DBG("f0_result = ");RFC_DBG_COMPLEX(f0_result);RFC_DBG("\n");
		imag_dc_est_last = imag_dc_est;
		RFC_DBG("!!!! imag_dc_est = %u\n", imag_dc_est);
	}
	
	RFC_DBG("++++++++ final result ++++++++\n");
	RFC_DBG("real_dc_est = %d\n", real_dc_est);
	RFC_DBG("imag_dc_est = %d\n", imag_dc_est);
	bb_register_write(0x25, (u8) real_dc_est);
	bb_register_write(0x30, (u8) imag_dc_est);
}

#endif //CONFIG_RFC
