/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfc_comm.c
*   \brief
*   \author Montage
*/

/*=============================================================================+
| Included Files
+=============================================================================*/
#include <os_compat.h>
#include <math.h>
#include <complex.h>
#include <ip301.h>
#include <rfc_comm.h>

#define RFC_TG_A_I 0x0F //<<4
#define RFC_TG_A_Q 0x0F //<<0

#define RFC_REG_ENABLE 0x1
#define RFC_REG_DISABLE 0x0

#ifdef CONFIG_RFC

/*** DEBUG FUNCTION SECTOR ***/
#ifdef RFC_DEBUG
void dbg_double(double x)
{
	if(isnan(x))
	{
		RFC_DBG("   NaN   ");
	}
	else if(!finite(x))
	{
		RFC_DBG("   Inf   ");
	}
	else if(x>=0)
	{
		RFC_DBG("%d.%08d", (int) x, (int) ((x - ((int) x)) * 100000000) );
	}
    else
    {
        x = -1 * x;
        RFC_DBG("-%d.%08d", (int) x, (int) ((x - ((int) x)) * 100000000) );
    }
}

void dbg_double_short(double x)
{
    if(x>=0)
    {
        RFC_DBG("%d.%02d", (int) x, (int) ((x - ((int) x)) * 100) );
    }
    else
    {
        x = -1 * x;
        RFC_DBG("-%d.%02d", (int) x, (int) ((x - ((int) x)) * 100) );
    }
}

void print_complex(complex x)
{
    dbg_double(x.real); RFC_DBG("+"); 
    dbg_double(x.imag); RFC_DBG("i");
}
#endif // RFC_DEBUG

/*** MATH/Algorithm FUNCTION SECTOR ***/
int isnormal(double x)
{
    if(isnan(x) || (!finite(x)))
        return 0;
    else
        return 1;
}

double angle_diff(double ang1, double ang2)
{
	double diff=0;

	if(ang1 < 0)
		ang1 += 360;
	if(ang2 < 0)
		ang2 += 360;

	diff = ang1 - ang2;

	RFC_DBG("!!!!!!!!!!!!!!!!!! angle diff = ");
    RFC_DBG_DOUBLE(diff);
	RFC_DBG("\n");

	return diff;
}

int angle_phase(double ang)
{
	int ret=0;

	if(ang < 0)
		ang += 360;

	if((ang <= 45) || (ang > 315))
		ret = ANGLE_0;
	else if(ang <= 135)
		ret = ANGLE_90;
	else if(ang <= 225)
		ret = ANGLE_180;
	else
		ret = ANGLE_270;
	
	return ret;
}

double calc_angle(double real, double image)
{
	double atan_table[10] = {26.565051177077990, 14.036243467926477, 7.125016348901798, 3.576334374997352, 1.789910608246069,  0.895173710211074, 0.447614170860553, 0.223810500368538,  0.111905677066207, 0.055952891893804};
	double temp_x, temp_y, x_arry[10], y_arry[10];
	double ph=0;
	int quadrant=0, xy_switch=0, i=0;

	temp_x = x_arry[0] = real;
	temp_y = y_arry[0] = image;

	//---------ALL 4 quadrant to 1st quadrant-------------------
	if((x_arry[0] > 0) && (y_arry[0] < 0))
	{
		quadrant=4;     
		y_arry[0] = -temp_y;      
	}		
	else if((x_arry[0] < 0) && (y_arry[0] < 0))
	{
		quadrant=3;
		y_arry[0] = -temp_y;
		x_arry[0] = -temp_x;
	}		
	else if((x_arry[0] < 0) && (y_arry[0] > 0))
	{
		quadrant = 2;
		x_arry[0] = -temp_x;
	}		
	else // x>0 && y>0
	{
		quadrant = 1;
	}		

	//%---Now,x and y are all positive (in 1st quadrant)---
	//%--compare who's bigger , x will be the bigger one, y is smaller one
	temp_x = x_arry[0];
	temp_y = y_arry[0];
	if(y_arry[0] > x_arry[0])
	{
		y_arry[0] = temp_x;
		x_arry[0] = temp_y;
		xy_switch = 1;
	}
	else
	{
		xy_switch = 0;
	}

	//%------------cordic loop---------------
	for(i=0; i < 10; i++)
	{
		if(y_arry[i] > 0)
		{
			if(i < 9)
			{
				x_arry[i+1] = x_arry[i] + (y_arry[i] * pow(2, -(i+1)));
				y_arry[i+1] = y_arry[i] - (x_arry[i] * pow(2, -(i+1)));
			}
			ph = ph + atan_table[i];
		}
		else //%y<0
		{
			if(i < 9)
			{
				x_arry[i+1] = x_arry[i] - (y_arry[i] * pow(2, -(i+1)));
				y_arry[i+1] = y_arry[i] + (x_arry[i] * pow(2, -(i+1)));
			}
			ph=ph-atan_table[i];
		}
	}

	//%--------postcordic----------
	if(xy_switch == 1)
		ph = 90 - ph;

	if(quadrant == 3)
		ph=180+ph;
	else if(quadrant == 2)
		ph=180-ph;
	else if(quadrant == 4)
		ph=-ph;

	//%-----------------Output------------------------------------------------
	return ph;
}

double calc_phase_shifter_angle(double alpha, double f0 , double fs)
{
	double A = tan(PI*f0/fs)*(1-alpha)/(1+alpha);
	double real_part = (1 - A*A);
	double imag_part = -2*A;
	
	return (calc_angle(real_part, imag_part)*PI/180);
}

double calc_rolloff_gain(double a, double f0 , double fs)
{
	double response;
	double real, imag;
	
	real = a + (1-2*a)*cos(2*PI*f0/fs) + a*cos(4*PI*f0/fs);
	imag = (1-2*a)*sin(2*PI*f0/fs) + a*sin(4*PI*f0/fs);
	response = real*real + imag*imag;

	RFC_DBG("%s(): response = ", __FUNCTION__); RFC_DBG_DOUBLE(response);
	RFC_DBG(", ");RFC_DBG_DOUBLE((log10(response)*20));RFC_DBG("\n");
	
	return response;
}

double ctl_coe_calc(int m, double f, double *val)
{
	*val = m * f;
	
	return *val;
}

int calc_abs_txlo(int a, complex *res_f0, complex *res_2f0, double reduce_factor)
{
	int dc_norm_amp;

	dc_norm_amp = (0.25*a*(abs_complex(res_f0))*reduce_factor)/(abs_complex(res_2f0));

	return dc_norm_amp;
}

void calc_data2iq(complex *LPF1, complex *LPF2, unsigned char *data)
{
    u32 LPF1_I, LPF1_Q, LPF2_I, LPF2_Q;
	
	LPF1_I = (data[0]&0xFF)<<16 | (data[1]&0xFF)<<8 | (data[2]&0xff) ;
	LPF1_Q = (data[3]&0xFF)<<16 | (data[4]&0xFF)<<8 | (data[5]&0xff) ;
	LPF2_I = (data[6]&0xFF)<<16 | (data[7]&0xFF)<<8 | (data[8]&0xff) ;
	LPF2_Q = (data[9]&0xFF)<<16 | (data[10]&0xFF)<<8 | (data[11]&0xff) ;
	// 16777216 = 2^24
	LPF1->real = hex2flt(LPF1_I, 24)/(16777216);
	LPF1->imag = hex2flt(LPF1_Q, 24)/(16777216);
	LPF2->real = hex2flt(LPF2_I, 24)/(16777216);
	LPF2->imag = hex2flt(LPF2_Q, 24)/(16777216);
}

u8 translate_freq_reg2b(double ctl_coe, int sel)
{
	double tmp;
	u8 ret = 0;

	tmp = (ctl_coe * 10 * 10 * 10)/312.5;

	if(!sel)
		tmp = tmp * 2;

	ret = (u8) tmp;
	
	return ret;
}

#ifdef TONE_MASK_ENABLE
int check_parm_with_mask(struct bal_parm *tx_bal, struct bal_parm *rx_bal, int tone_mask, int tone_start, int quantity)
{
	int i=0, j=quantity;
	int tone_end = tone_start + quantity;
	
	if(((1 << tone_end) - 1) > tone_mask)
	{
		j = 0;
		for(i=0; i<quantity; i++)
		{
			if((1 << (i + tone_start)) & tone_mask)
			{
				tx_bal[j].gain.real = tx_bal[i].gain.real;
				tx_bal[j].gain.imag = tx_bal[i].gain.imag;
				tx_bal[j].phi = tx_bal[i].phi;
				rx_bal[j].gain.real = rx_bal[i].gain.real;
				rx_bal[j].gain.imag = rx_bal[i].gain.imag;
				rx_bal[j].phi = rx_bal[i].phi;
				j++;
			}
		}
	}

	return j;
}
#endif	// TONE_MASK_ENABLE

int cmp_phase_shifter_coe_coarse(struct bal_parm *parm, int bw)
{
	int tone_num=0;
	int edge_loc=0;	//fs=0;
	int l=0, tmp1=0;
	int alpha_coarse;
	double tone_list_20[FREQ_QUANTITY_20MHZ] = RFC_TONE_LIST_20;
	double tone_list_40[FREQ_QUANTITY_40MHZ] = RFC_TONE_LIST_40;
	double *tone_list;
	double slope=0, slope_avg=0, abs_slope_avg=0;
	double cross_corr=0, auto_corr=0;

	if(bw)
	{
		tone_num = FREQ_QUANTITY_40MHZ;
		tone_list = tone_list_40;
		// coarse est 
		edge_loc = 20; 		// pi=20M
		//fs = 80;    
	}
	else
	{
		tone_num = FREQ_QUANTITY_20MHZ;
		tone_list = tone_list_20;
		// coarse est 
		edge_loc = 10;    	// pi=10M
		//fs = 40;
	}
	 
	/* 	phase_diff is in radian representation , not degree
	 	slope_avg is represent in  radian/MHz 					*/
	for(l=0; l < tone_num; l++)
	{
		cross_corr += tone_list[l] * parm[l].phi;
		auto_corr += tone_list[l] * tone_list[l];
	}
	slope_avg = cross_corr / auto_corr;
						 
	/* slope_avg will make the decision which channel will pass through all pass filter and finetune process */
	abs_slope_avg = fabs(slope_avg);
						 
	SIGN(slope_avg, tmp1);

	slope = abs_slope_avg * edge_loc;
	alpha_coarse = ((2 - slope)/(2 + slope))*256;
	alpha_coarse = alpha_coarse * tmp1;
	
	return alpha_coarse;
}

int cmp_phase_shifter_coe_fine(int alpha_coarse, struct bal_parm *parm, int bw)
{
	// decide alpha tunning range, coarse+0.05 ~  coarse-0.05
	int alpha_sign;
	int	tone_num = 0;
	int l=0;
	int search_range=2;
	int alpha_step=2;
	int alpha_coarse_up_lim;
	int alpha_coarse_down_lim;
	int alpha_idx=0;
	int fine_alpha=0;
	double sum_temp=0;
	double min_err_cost=100.0, err_cost=0;
	double ph_response, phase_mean;
	double err=0, err_abs;
	double tone_list_20[FREQ_QUANTITY_20MHZ] = RFC_TONE_LIST_20;
	double tone_list_40[FREQ_QUANTITY_40MHZ] = RFC_TONE_LIST_40;
	double *tone_list;
	double f0=0, fs=0;
	double max_tone_err=0;
	double max_tone_err_ori=0;
	double err_square=0;

	SIGN(alpha_coarse, alpha_sign);
	alpha_coarse = fabs(alpha_coarse);
	alpha_coarse_up_lim = alpha_coarse + search_range;
	alpha_coarse_down_lim = alpha_coarse - search_range;

	//Restriction
	if(alpha_coarse_up_lim > 255)
		alpha_coarse_up_lim	= 255;
	
	if(alpha_coarse_down_lim < 0)
		alpha_coarse_down_lim = 0;

	if(bw==0)   //BW=20MHz
	{
		fs=40;
		tone_list = tone_list_20;
		tone_num = FREQ_QUANTITY_20MHZ;
	}
	else  		//BW=40MHz
	{
		fs=80;    
		tone_list = tone_list_40;
		tone_num = FREQ_QUANTITY_40MHZ;
	}

	/*	calc mean of bal_parm.phase, so the code can be replaced by any other
		function. Just find out the mean					*/
		
	for(l=0; l < tone_num; l++)
		sum_temp = sum_temp + parm[l].phi;
	
	phase_mean = sum_temp/tone_num;

	/*	apply min(max) 1st, LS maybe later
		error is defined as 
		phi_mean=mean(mismatch_info(1:end).phi);
		if slope_avg>0  : error=mismatch_info.phi - phi_mean + ph_response
		elseif slope_avg<0  : error=mismatch_info.phi - phi_mean - ph_response
		[ph_response]=calc_phase_shifter_angle(alpha, f0 ,fs ) */
		
	/*	--------alpha fine tune begin-------------------	*/
	for(alpha_idx=alpha_coarse_down_lim; alpha_idx<=alpha_coarse_up_lim; alpha_idx+=alpha_step)
	{
		max_tone_err = 0;
		err_square = 0;
		
		for(l=0; l < tone_num; l++)
		{
			f0 = tone_list[l];
			ph_response = calc_phase_shifter_angle(((double)alpha_idx/256), f0 , fs);    
			
			err = parm[l].phi - phase_mean + (alpha_sign * ph_response);
			err_abs = fabs(err);
			err_square += err*err;
			if(err_abs > max_tone_err)
			{
				max_tone_err = err_abs;
			}
		}

		//%Find out the min err alpha
		err_cost = err_square;
		if(err_cost < min_err_cost)
		{
			min_err_cost = err_cost;
			fine_alpha = alpha_idx;
			max_tone_err_ori = max_tone_err;
		}
	}																			

	//%restore sign of fine_alpha
	fine_alpha = fine_alpha * alpha_sign;

	/*	%it must be better compared with the no-filter condition
		%find max(bal_parm(l).phase - phase_mean), it can be replaced by any other function 
		%which has the same move 															*/
	max_tone_err=0;
	for(l = 0; l < tone_num; l++)
	{
		err = parm[l].phi - phase_mean;
		err_abs = fabs(err);
		if(err_abs > max_tone_err)
			max_tone_err = err_abs;            
	}
	
	if(max_tone_err < max_tone_err_ori)
	{
		fine_alpha=256;
	}

	return fine_alpha;
}

int cmp_rolloff_filter_coe_coarse(struct bal_parm *parm, int bw)
{
	double gain_th=1.035; // %0.3dB, make it positive gain
	double gain_th_inv, gain_ratio;
	double max_gain, min_gain, center_gain;
	double neg_bandedge_gain, pos_bandedge_gain, neg_bandedge2_gain, pos_bandedge2_gain;
	double edge_gain_mul=0, edge_gain=0, coarse_a=0;
	int neg_edge_rise, neg_edge_fall, pos_edge_rise, pos_edge_fall;
	int goup=0, godown=0;
	int tone_num, tone_center;
	int l, ret;

	gain_th_inv = 1/gain_th;
	
	if(bw == 0)
		tone_num = FREQ_QUANTITY_20MHZ;
	else
		tone_num = FREQ_QUANTITY_40MHZ;
	tone_center = tone_num/2;

	/* 	Do we need turn on roll off filter?  
		if max(gain)/min(gain)<th then we just ignore this filter
		find max/min */
	max_gain = parm[0].gain.imag;
	min_gain = parm[0].gain.imag;
	for(l=1; l < tone_num; l++)
	{
		if(parm[l].gain.imag > max_gain)
			max_gain = parm[l].gain.imag;
						
		if(parm[l].gain.imag < min_gain)
			min_gain = parm[l].gain.imag;
	}
	gain_ratio = max_gain/min_gain;

	/* make sure it's really roll-off */
	if(gain_ratio >= gain_th)
	{
		/* start to calc possible filter coe. 1~8 or 1~16 */
		center_gain = 0.5*(parm[tone_center-1].gain.imag + parm[tone_center].gain.imag);

		/* edge1/center */
		neg_bandedge_gain = parm[0].gain.imag/center_gain;
		pos_bandedge_gain = parm[tone_num-1].gain.imag/center_gain;

		/* edge2/center */
		neg_bandedge2_gain = parm[1].gain.imag/center_gain;
		pos_bandedge2_gain = parm[tone_num-2].gain.imag/center_gain;

		/* Does negative edge rise or fall? */
		neg_edge_rise = (neg_bandedge_gain > gain_th)  || (neg_bandedge2_gain > gain_th);
		neg_edge_fall = (neg_bandedge_gain < gain_th_inv) || (neg_bandedge2_gain < gain_th_inv);

		/* Does positive edge rise or fall? */
		pos_edge_rise = (pos_bandedge_gain > gain_th) || (pos_bandedge2_gain > gain_th);
		pos_edge_fall = (pos_bandedge_gain < gain_th_inv) || (pos_bandedge2_gain < gain_th_inv);
					
		/* Both pos/neg edge are leaving center. We have to know which dir and two side situation. */
		if((neg_edge_rise || neg_edge_fall) && (pos_edge_rise || pos_edge_fall))
		{
			/* make sure two side is on the same dir */
			if(neg_edge_rise && pos_edge_rise)
				goup = 1;
			if(neg_edge_fall && pos_edge_fall)
				godown = 1;
		
			/* make a avg of 4 tones result, prepare for coarse estimation */
			edge_gain = 0.25*(neg_bandedge_gain + pos_bandedge_gain + neg_bandedge2_gain + pos_bandedge2_gain);
				
			if(goup && godown) // go up  and down @ the same time...
			{
				/* find dominant gain up or down */
				edge_gain_mul = neg_bandedge_gain * pos_bandedge_gain * neg_bandedge2_gain * pos_bandedge2_gain;
				if(edge_gain_mul > 1)
					godown = 0;
				else if(edge_gain_mul < 1)
					goup = 0;
				else
					godown = goup = 0;
			}

			if(edge_gain > 1)
				edge_gain = 1/edge_gain;

			/* find appropriate coarse_a */
			coarse_a = (-0.5)*(edge_gain/1.03 - 1);
			if(coarse_a < 0)
				coarse_a = 0;
		}
	}
				
	/* just calc one time? */
	if((!goup && !godown) || (goup && godown))
		coarse_a = 0;

	ret = (int) floor(coarse_a*128);
	return ret;
}

void cmp_iq_mismatch(complex dn, complex vn, double *result)
{
    complex temp, Pdv, K1K2;
    double Pdplusv;

    Pdv = complex_multi(dn,vn);

    //Pdplusv = abs(dn+conj(vn))^2
    temp = complex_add(dn,complex_conj(vn));
    Pdplusv = (temp.real*temp.real)+(temp.imag*temp.imag);

    //complex Pdplusv = complex_conj(vn);
    K1K2.real = Pdv.real/Pdplusv;
    K1K2.imag = Pdv.imag/Pdplusv;

    *result = sqrt(1 - 4*K1K2.real); //g
    *(result+1) = asin(-1*(2/(*result))*K1K2.imag); //phi
}

double gain_estimator(complex y_LPF, double gi, int type)
{
    double A, B, ret;

    //Estimate A (Amplitude of x)
    B = y_LPF.real*y_LPF.real + y_LPF.imag*y_LPF.imag;
    A = 2*sqrt(B);

    if (0 == type)
    {
		//Estimate g (A = g^2/4)
		ret = 2*sqrt(A);
    }
    else //(1 == type)
    {
        //Estimate phi |-sin(phi)/2| = |A|; phi = asin(2*|A|))
        ret = asin(2*fabs(A/(gi*gi)));
        //data in phi.real
    }

	return ret;
}

void bal_parm_mean(struct bal_parm *tx_bal, struct bal_parm *rx_bal, struct rfc_cal_parm *rfc_ret, int freq_quantity)
{
	complex tx_gain, rx_gain;
	double tx_phi, rx_phi;
	int i;

	tx_gain.real = tx_gain.imag = rx_gain.real = rx_gain.imag = tx_phi = rx_phi = 0;

	for(i=0; i<freq_quantity; i++)
	{
		tx_gain.real += tx_bal[i].gain.real;
		tx_gain.imag += tx_bal[i].gain.imag;
		tx_phi += tx_bal[i].phi;
		rx_gain.real += rx_bal[i].gain.real;
		rx_gain.imag += rx_bal[i].gain.imag;
		rx_phi += rx_bal[i].phi;
	}

	rfc_ret->tx_avg.gain.real 	= tx_gain.real/freq_quantity;
	rfc_ret->tx_avg.gain.imag 	= tx_gain.imag/freq_quantity;
	rfc_ret->tx_avg.phi 		= tx_phi/freq_quantity;
	rfc_ret->rx_avg.gain.real 	= rx_gain.real/freq_quantity;
	rfc_ret->rx_avg.gain.imag 	= rx_gain.imag/freq_quantity;
	rfc_ret->rx_avg.phi 		= rx_phi/freq_quantity;
}

void tx_mux_regs(unsigned char mux, unsigned char tg_a_i, unsigned char tg_a_q, double ctl_coe)
{
    unsigned char value20;
    u8 reg_val;

    //Reserved TX-balancer bypass mode and RX-balancer bypass mode
    value20 = bb_register_read(0x2f);

    //Program tx MUX mode and bypass bits
	//Set TX MUX mode 0~15
	if (mux > 15) //BR2F[3:0]
	{
		WLA_DBG(WLAERROR, "ERROR: Too large mux value.\n");
		return;
	}
	reg_val = (value20&RFC_TX_BYPASS) | (value20&RFC_RX_BYPASS) | (mux&RFC_MUX) ;

    //Write TX MUX setting
    bb_register_write(0x2f, reg_val);

    if (mux == 7) //If in tonegen mode, setting tone generator component
    {
        //Convert tone amplitude and program it
        reg_val = (tg_a_i&RFC_TG_A_I)<<4 | (tg_a_q&RFC_TG_A_Q);
        bb_register_write(0x2c, reg_val);

        //Convert tone frequency (tone frequency unit:MHz) and program it
        reg_val = translate_freq_reg2b(ctl_coe, (bb_register_read(0x32) & 0x1));
        bb_register_write(0x2b, reg_val);
    }
}

void rx_demod_regs(unsigned char rx_cal_mode_sel, double ctl_coe)
{
    char reg_val;
	int sel=0;

    //Convert tone frequency (tone frequency unit:MHz) and program it
	if((bb_register_read(0x32) & 0x1))
		sel = 1;
	reg_val = translate_freq_reg2b(ctl_coe, sel);
    bb_register_write(0x2d, reg_val);
    
    //Program Demod and LPF register
    reg_val = (rx_cal_mode_sel&RFC_DEMOD_MUTE)<<2;
    bb_register_write(0x2e, reg_val);
}

void toggle_reg2e(unsigned char msk, unsigned char en)
{
    unsigned char value2e, reg_val;
    //Reserved RX demod configuration
    value2e = bb_register_read(0x2e);

    reg_val = (value2e&(0xFF - msk)) | en;
    bb_register_write(0x2e, reg_val);
}

int read_LPF(complex *LPF1, complex *LPF2, int reset_en, int delay_sel)
{
	/*	Reg.0x2E[0] = latch enable
		if bit.0 = 1 => latch the lpf data
		else => reflect the realtime lpf data	*/
	u8 val = (bb_register_read(0x2e) & (~(RFC_LPF_ALPHA << 4)));
    u8 lpf_alpha1=0;
	u8 data[12];
	unsigned long delay1=0;
	complex lpf1[READ_LPF_CIRCLE], lpf2[READ_LPF_CIRCLE];
	int i, j;

	switch(delay_sel)
	{
		case 0:
			lpf_alpha1 = 10;
			delay1 = 105;
			break;
		case 2:
			lpf_alpha1 = 11;
			delay1 = 1000;
			break;
		default:
			/* for error handle, if the delay_sel value is not expected */
			RFC_DBG("! read_LPF(): delay_sel=%d\n", delay_sel);
			lpf_alpha1 = 10;
			delay1 = 100; 
			break;
	}
			
	bb_register_write(0x2e, val | (lpf_alpha1 << 4));
	
	/* do reset & enable */
	if(reset_en)
		toggle_reg2e(0x3, 0x2);
	
	for(i=0; i < READ_LPF_CIRCLE; i++)
	{
		toggle_reg2e(0x3, RFC_REG_ENABLE);
		udelay(delay1);
		toggle_reg2e(0x1, RFC_REG_DISABLE);

		for(j=0; j<12; j++)
		{
			data[j] = bb_register_read(0x40 + j);
		}

		calc_data2iq(&lpf1[i], &lpf2[i], data);
	}

	if(LPF1)
		*LPF1 = complex_mean(lpf1, READ_LPF_CIRCLE);
	if(LPF2)
		*LPF2 = complex_mean(lpf2, READ_LPF_CIRCLE);

	return 0;
}

void rx_dc_offset_comp(void)
{
	double ctl_coe=0;
	double rx_dc_i=0, rx_dc_q=0;
	unsigned char cal_mode_sel=0;
	complex i_signal_f0, q_signal_f0;
	char val_i, val_q;
    
	bb_register_write(0x2a, 0);
	bb_register_write(0x31, 0);
	
	rx_demod_regs(cal_mode_sel, ctl_coe);

	if(!(read_LPF(&i_signal_f0, &q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL)))
	{
		rx_dc_i = i_signal_f0.real * pow(2,10);
		rx_dc_q = i_signal_f0.imag * pow(2,10);

		//Transform rx_dc_I, rx_dc_Q to signed char
		val_i = rx_dc_i;
		val_q = rx_dc_q;

		RFC_DBG("val_i = %d\n", val_i);
		RFC_DBG("val_q = %d\n", val_q);
		
		//write to reg
		bb_register_write(0x2a, val_i);
		bb_register_write(0x31, val_q);
	}
}

int txlo_pos_neg(int dc_pos, int dc_neg, complex *pos_f0, complex *neg_f0, complex *f0)
{
	int dc_est;
	double abs_f0_pos = abs_complex(pos_f0);
	double abs_f0_neg = abs_complex(neg_f0);
	double abs_f0_new;

	if(abs_f0_pos > abs_f0_neg)
	{
		dc_est = dc_neg;
		abs_f0_new = abs_f0_neg;
	}
	else
	{
		dc_est = dc_pos;
		abs_f0_new = abs_f0_pos;
	}

	if(abs_f0_new > abs_complex(f0))
		dc_est = (dc_pos+dc_neg)/2;

	return dc_est;
}

void bb_rfc_reset(void)
{
    bb_register_write(0x20, 0x00);       
    bb_register_write(0x21, 0x7F);
    bb_register_write(0x22, 0x00);
    bb_register_write(0x23, 0x00);
    bb_register_write(0x24, 0x7F);
    //bb_register_write(0x25, 0x00);
    bb_register_write(0x26, 0x7F);
    bb_register_write(0x27, 0x00);
    bb_register_write(0x28, 0x00); 
    bb_register_write(0x29, 0x7F);
    bb_register_write(0x2a, 0x00);        
    bb_register_write(0x2b, 0x00);      
    bb_register_write(0x2c, 0x00);       
    bb_register_write(0x2d, 0x00);       
    bb_register_write(0x2e, 0x00);
    bb_register_write(0x2f, 0x00);
    bb_register_write(0x31, 0x00);
    bb_register_write(0x5a, 0x00);
    bb_register_write(0x5b, 0x00);
    bb_register_write(0x5c, 0x00);
    bb_register_write(0x5d, 0x00);
    bb_register_write(0x5e, 0x00);
}

unsigned short rxvga_adjust(unsigned short ovth, unsigned short okth)
{
	int adjust_success=0, loop=0;
	unsigned short peak_value=0;
	unsigned short rxvga=0;
	unsigned char sat_trig=0, last_sat=0, last_2low=0;
	unsigned short gain_backoff=3;
	int overflow_count=0, underflow_count=0;
	int gain_setting_time = 800; // ns
	unsigned int count_time_div64 = 32; // counter time = 2048, => 2048/64=32
	unsigned int delay_ns = 102400; // 2048*2*25
	unsigned int reg9=0;

	while(adjust_success == 0 && loop <= 8)
	{
		bb_register_write(0x50, count_time_div64);	// disable peak detector & reset max value
		bb_register_write(0x50, count_time_div64 | 0x80);	// enable peak detector
		ndelay(delay_ns);
		peak_value = bb_register_read(0x55);
		reg9 = ip301_spi_read(0x9);
		rxvga = (reg9 & 0x1f8) >> 4;

		if(peak_value > okth && peak_value < ovth)
		{
			//RFC_DBG("The adjusted rxvga = 0x%x\n", rxvga);
			adjust_success = 1;
		}
		else if(peak_value >= ovth)
		{
			overflow_count++;
			if( rxvga >= gain_backoff )
			{
				if (last_2low==1 && sat_trig==1 &&  peak_value<=124)
			  	{
					adjust_success = 1;     
					//RFC_DBG("last  too low+peak_value<=124, quit loop\n");   
				}
				else
				{
					rxvga = rxvga - gain_backoff;
					last_2low=0;
					sat_trig=1; 
					last_sat=1;	
				}
			}
			else
			{
				rxvga = 0;
				//RFC_DBG("RXVGA is too small for peak detection\n");
				adjust_success=1;
			}
		}
		else // too small
		{
			underflow_count++;
			if(last_sat == 1 && gain_backoff==1)
			{
				adjust_success=1;
				//RFC_DBG("last_sat=1  , gain backoff=1dB , force success=1 to leave loop\n");
			}
			else
			{
				if(sat_trig == 1) 
				{
					rxvga+=1;
					//RFC_DBG("sat triggered!\n");
				}
				else if(peak_value > 71)
					rxvga += 1;
				else if(peak_value > 54)
					rxvga += 2;
				else if(peak_value > 40)
					rxvga += 3;
				else if(peak_value > 32)
					rxvga += 4;
				else if(peak_value > 25)
					rxvga += 5;
				else if(peak_value > 20)
					rxvga += 6;
				else
					rxvga += 8;

				last_sat=0;
				last_2low=1;
				gain_backoff=1;
			}
		}

		if(rxvga > 0x1f)
		{
			//RFC_DBG("RXVGA may too large for RFC system\n");
			rxvga = 0x1f;
		}
		
		/* program rxvga */
		ip301_spi_write(0x9, (((rxvga & 0x1f)*2) << 3) | (reg9 & 0xFFFFFE07));
		ndelay(gain_setting_time);  //delay 600ns after gain change 	
		loop++;
	}

	if(loop > 8)
		RFC_DBG("RXVGA Scan is test more than 9 times!!!!!!!!!!!!\n"); 
	
	//RFC_DBG("the final in rxvga_adjust(), rxvga = %d, peak_value = %d\n", rxvga, peak_value);

	return rxvga;	
}

void setup_vga_tbl(struct vga_tbl *tbl, int rx_scale, int tx_scale, int rx_txvga, int tx_txvga)
{
	unsigned int txcal_vga_20mhz[FREQ_QUANTITY_20MHZ][3] = TXCAL_VGA_20MHZ;
	unsigned int rxcal_vga_20mhz[FREQ_QUANTITY_20MHZ][3] = RXCAL_VGA_20MHZ;
	unsigned int txcal_vga_40mhz[FREQ_QUANTITY_40MHZ][3] = TXCAL_VGA_40MHZ;
	unsigned int rxcal_vga_40mhz[FREQ_QUANTITY_40MHZ][3] = RXCAL_VGA_40MHZ;
	
	int i;

	for(i = 0; i < FREQ_QUANTITY_20MHZ; i++)
	{
		if(tx_scale)
			tbl->txcal_20mhz[i].bb_scale = tx_scale;
		else
			tbl->txcal_20mhz[i].bb_scale = txcal_vga_20mhz[i][0];
		if(rx_scale)
			tbl->rxcal_20mhz[i].bb_scale = rx_scale;
		else
			tbl->rxcal_20mhz[i].bb_scale = rxcal_vga_20mhz[i][0];
		if(tx_txvga)
		{
			tbl->txcal_20mhz[i].rxvga = tx_txvga;
			tbl->txcal_20mhz[i].txvga = tx_txvga;
		}
		else
		{
			tbl->txcal_20mhz[i].rxvga = txcal_vga_20mhz[i][1];
			tbl->txcal_20mhz[i].txvga = txcal_vga_20mhz[i][2];
		}
		if(rx_txvga)
		{
			tbl->rxcal_20mhz[i].rxvga = rx_txvga;
			tbl->rxcal_20mhz[i].txvga = rx_txvga;
		}
		else
		{
			tbl->rxcal_20mhz[i].rxvga = rxcal_vga_20mhz[i][1];
			tbl->rxcal_20mhz[i].txvga = rxcal_vga_20mhz[i][2];
		}
	}

	for(i = 0; i < FREQ_QUANTITY_40MHZ; i++)
	{
		if(tx_scale)
			tbl->txcal_40mhz[i].bb_scale = tx_scale;
		else
			tbl->txcal_40mhz[i].bb_scale = txcal_vga_40mhz[i][0];
		if(rx_scale)
			tbl->rxcal_40mhz[i].bb_scale = rx_scale;
		else
			tbl->rxcal_40mhz[i].bb_scale = rxcal_vga_40mhz[i][0];

		if(tx_txvga)
		{
			tbl->txcal_40mhz[i].rxvga = tx_txvga;
			tbl->txcal_40mhz[i].txvga = tx_txvga;
		}
		else
		{
			tbl->txcal_40mhz[i].rxvga = txcal_vga_40mhz[i][1];
			tbl->txcal_40mhz[i].txvga = txcal_vga_40mhz[i][2];
		}
		if(rx_txvga)
		{
			tbl->rxcal_40mhz[i].rxvga = rx_txvga;
			tbl->rxcal_40mhz[i].txvga = rx_txvga;
		}
		else
		{
			tbl->rxcal_40mhz[i].rxvga = rxcal_vga_40mhz[i][1];
			tbl->rxcal_40mhz[i].txvga = rxcal_vga_40mhz[i][2];
		}
	}
}

void rfc_env_setup(struct vga_tbl *vga_table, int rx_scale, int tx_scale, int rx_txvga, int tx_txvga)
{	
	unsigned int hw_mode;
	unsigned short val;
    /* Should consider to put filter code setting in there. */
	bb_register_write(0x01, (bb_register_read(0x01) | 0x20)); //RXHP always low

	bb_register_write(0x1c, 0xee);
    
	/* disable LPF learning */
	bb_register_write(0x54, 0x2b);

	bb_register_write(0x02, 0x31); //Cheetah IC

#if defined(CONFIG_FPGA)
	/* Old RF board has different ADC format */
	if(bb_register_read(0x00) == 0x5c)
		bb_register_write(0x02, 0x30); //Cheetah IC
#endif

	bb_register_write(0x1D, 0xC6); //close debug port
	//BBDBG(0x1D);
	
	/* setup the bb_scale/rxvga/txvga table */
	if(vga_table)
		setup_vga_tbl(vga_table, rx_scale, tx_scale, rx_txvga, tx_txvga);
	
	/* Setup IQ Swap */
	/* $parm_hw_mode: bit.1 : ADC IQ Swap, bit.2 : DAC IQ Swap */
#if defined(CONFIG_FPGA)
	hw_mode = WLA_CDB_GET($parm_hw_mode, 0x4);
#else
	hw_mode = 0x4;
#endif	
	val = bb_register_read(0x2);
	/* bit.1 : ADC IQ Swap, bit.3 : clock gating disable, bit.5 : DAC IQ Swap */
	val = (val & 0xDD) | (hw_mode & 0x2) | ((hw_mode & 0x4) << 3) | 0x8;
	bb_register_write(0x2, val);
}

/*** RESULT PRINT/CHECK FUNCTION SECTOR ***/

#ifdef RFC_DEBUG
int rx_cal_result_check(int bw, struct vga_tbl *vga_table)
{
	double ctl_coe;
    unsigned char bb_scale;
    complex neg_f0, pos_f0;
	unsigned int txvga, rxvga;
	double rx_result;
	
	/* for break function */
	unsigned char buf[16];
	(void) buf;
	
	if(bw)	/* 40MHz */
	{
		bb_scale = vga_table->rxcal_40mhz[5].bb_scale;
		rxvga = vga_table->rxcal_40mhz[5].rxvga;
		txvga = vga_table->rxcal_40mhz[5].txvga;
	}
	else	/* 20MHz */
	{
		bb_scale = vga_table->rxcal_20mhz[5].bb_scale;
		rxvga = vga_table->rxcal_20mhz[5].rxvga;
		txvga = vga_table->rxcal_20mhz[5].txvga;
	}

    // Set RX loopback mode
	ip301_set_iqcal_vga(RXLOOP, rxvga, txvga);

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    ctl_coe = 2.5;
    tx_mux_regs(MUX_TONEGEN, bb_scale, bb_scale, ctl_coe);

    rx_demod_regs(1, ctl_coe);	// argv[0] = 1 :Dmod input control: from rxbnc

#if defined(INTERACTIVE_MODE)
	RFC_DBG("RX Calibration Result Check, before read_LPF. press ENTER to continue.\n");
	WLA_GETS(buf);
	if(strncmp(buf, "c", 1) == 0)
		return 2;
#endif
    
	if(read_LPF(&neg_f0, &pos_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    //udelay(1000000);
    RFC_DBG("-f0: "); RFC_DBG_COMPLEX(neg_f0); 	RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&neg_f0)); 	RFC_DBG(" db");
    RFC_DBG(", f0: "); RFC_DBG_COMPLEX(pos_f0); 	RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&pos_f0));	RFC_DBG(" db");
    RFC_DBG("\n");    

	rx_result = fabs(__complex_to_db(&neg_f0) - __complex_to_db(&pos_f0));
    RFC_DBG("the |f0 - (-f0)| = ");	RFC_DBG_DOUBLE(rx_result);	RFC_DBG("\n");

    tx_mux_regs(MUX_BASEBAND, 0, 0, 0);

	/* pass criteria: the |f0 - (-f0)| >= 30 */
	if(rx_result >= 30)
		return 0;
	else
		return 1;
}

/*	1.	bw: 0 = 20MHz Mode, 1 = 40MHz Mode 
	2.	*tx_rec: tx_rec[sel][f0/2f0]
		ex: tx_rec[0][0] = before calibration, f0
			tx_rec[1][1] = after calibration, 2f0
	3.	sel: 0 = before calibration, 1 = after calibration		  */
int tx_cal_result_check(int bw, struct vga_tbl *vga_table, double *tx_rec, int sel)
{
    double ctl_coe;
    unsigned char tg_a_i, tg_a_q;
    complex I_signal_f0, Q_signal_f0;
	complex *tx_signal;
	unsigned int txvga, rxvga;
	/* for break function */
	unsigned char buf[16];
	(void) buf;

	if(bw)	/* 40MHz Mode */
	{
		tg_a_i = vga_table->txcal_40mhz[5].bb_scale;
		tg_a_q = vga_table->txcal_40mhz[5].bb_scale;
		rxvga = vga_table->txcal_40mhz[5].rxvga;
		txvga = vga_table->txcal_40mhz[5].txvga;
	}
	else	/* 20MHz Mode */
	{
		tg_a_i = vga_table->txcal_20mhz[5].bb_scale;
		tg_a_q = vga_table->txcal_20mhz[5].bb_scale;
		rxvga = vga_table->txcal_20mhz[5].rxvga;
		txvga = vga_table->txcal_20mhz[5].txvga;
	}

	if(bb_register_read(0x2) & 0x2)
		tx_signal = &I_signal_f0;
	else
		tx_signal = &Q_signal_f0;

	ip301_set_iqcal_vga(TXLOOP, rxvga, txvga);

	// Set f0 = 2.5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    ctl_coe = 2.5;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    rx_demod_regs(0x3, ctl_coe);

	if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    RFC_DBG("f0: ");
    RFC_DBG_COMPLEX(*tx_signal);	RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(tx_signal));	RFC_DBG(" db\n");

    tx_rec[sel*2 + 0] = __complex_to_db(tx_signal);

    ctl_coe *= 2;
    rx_demod_regs(0x3, ctl_coe);

#if defined(INTERACTIVE_MODE)
	RFC_DBG("TX Check. press ENTER to continue.\n");
	WLA_GETS(buf);
	if(strncmp(buf, "c", 1) == 0)
		return 2;
#endif
    
	if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    RFC_DBG("2f0: ");
    RFC_DBG_COMPLEX(*tx_signal);	RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(tx_signal));	RFC_DBG(" db\n");

    tx_rec[sel*2 + 1] = __complex_to_db(tx_signal);

    tx_mux_regs(MUX_BASEBAND, 0, 0, 0);

	/* The pass criteria : the 2f0 after calibration should be smaller than the 2f0 before calibration */
	if(sel)
	{
		RFC_DBG("RFC_DBG INFO: bw= %d, before cal, f0 = ", bw); RFC_DBG_DOUBLE(tx_rec[0]); 
		RFC_DBG(", 2f0 = "); RFC_DBG_DOUBLE(tx_rec[1]); RFC_DBG("\n");
		RFC_DBG("RFC_DBG INFO: bw= %d, after cal, f0 = ", bw); RFC_DBG_DOUBLE(tx_rec[2]); 
		RFC_DBG(", 2f0 = "); RFC_DBG_DOUBLE(tx_rec[3]); RFC_DBG("\n");
		
		if((tx_rec[3] >= tx_rec[1]))
		{
			RFC_DBG("tx_rec[3] = ");RFC_DBG_DOUBLE(tx_rec[3]);
			RFC_DBG(", tx_rec[1] = ");RFC_DBG_DOUBLE(tx_rec[1]);
			RFC_DBG("\n");
			return 1;
		}
	}
    	
    return 0;
}

void check_statistics(int samples, struct result_arry *record)
{
    int i;
    int j;
    double curr_max[2];
    double curr_min[2];
    double accmulate;
    double average[2];
    double variance[2];

	/* calc the max/min/avg/dev value */
	for(j=0;j<2;j++)
    {
        curr_max[j] = record->rec[0][j];
        curr_min[j] = record->rec[0][j];
        accmulate = 0;
        
        for(i=0;i<samples;i++)
        {
            if( record->rec[i][j] > curr_max[j])
                curr_max[j] = record->rec[i][j];
			else if( record->rec[i][j] < curr_min[j])
                curr_min[j] = record->rec[i][j];
            
            accmulate += record->rec[i][j];
        }
        
        average[j] = accmulate/samples;

        accmulate = 0;
		for(i=0;i<samples;i++)
        {
            accmulate += ((record->rec[i][j] - average[j]) * 
						  (record->rec[i][j] - average[j]));
        }
        
        variance[j] = sqrt((accmulate / samples));
    }

	/* print the calculated value */
	for(i=0;i<samples;i++)
    {
        RFC_DBG(" %04d      ", i);
        RFC_DBG_DOUBLE(record->rec[i][0]); RFC_DBG(" db  ");
        RFC_DBG_DOUBLE(record->rec[i][1]); RFC_DBG(" db  ");
        RFC_DBG("\n");
    }

    RFC_DBG("-------------------------------------------------------------------\n");
    RFC_DBG(" MAX       ");
	RFC_DBG_DOUBLE(curr_max[0]); RFC_DBG(" db  ");
	RFC_DBG_DOUBLE(curr_max[1]); RFC_DBG(" db  ");
    RFC_DBG("\n");

    RFC_DBG(" MIN       ");
	RFC_DBG_DOUBLE(curr_min[0]); RFC_DBG(" db  ");
	RFC_DBG_DOUBLE(curr_min[1]); RFC_DBG(" db  ");
    RFC_DBG("\n");

    RFC_DBG(" AVG       ");
	RFC_DBG_DOUBLE(average[0]); RFC_DBG(" db  ");
	RFC_DBG_DOUBLE(average[1]); RFC_DBG(" db  ");
    RFC_DBG("\n");

    RFC_DBG(" DEV         ");
	RFC_DBG_DOUBLE(variance[0]); RFC_DBG("     ");
	RFC_DBG_DOUBLE(variance[1]); RFC_DBG("     ");
    RFC_DBG("\n");

    RFC_DBG("-------------------------------------------------------------------\n");

}

void output_rx_iq_mismatch_check_statistics_ht(int samples, struct result_arry *rx_iq_mismatch)
{
    RFC_DBG("---------RX IQ mismatch check -------------------------------------\n");
    RFC_DBG("  NO.           -f0               f0        \n");
    RFC_DBG("-------------------------------------------------------------------\n");
	
	check_statistics(samples, rx_iq_mismatch);
}

void output_tx_iq_mismatch_check_statistics_ht(int samples, struct result_arry *tx_iq_mismatch)
{
    RFC_DBG("---------TX IQ mismatch check -------------------------------------\n");
    RFC_DBG("  NO.           f0               2f0         \n");
    RFC_DBG("-------------------------------------------------------------------\n");

	check_statistics(samples, tx_iq_mismatch);
}

int check_max_min_avg(double *val_max, double *val_min, double *sum, double *now)
{
	if(!isnormal(*now))
		return 1;
	
	if(*val_max == 0 || *val_min == 0)
	{
		*val_max = *now;
		*val_min = *now;
	}
	else if(*val_max <= *now)
	{
		*val_max = *now;
	}
	else if(*val_min >= *now)
	{
		*val_min = *now;
	}

	*sum = *sum + *now;
	
	return 0;
}

void check_deviation(double val, double *accumlate, double average)
{
	if(!isnormal(val))
		return;

	*accumlate += ((val - average) * (val - average));
}

void print_rfc_test_result(struct rfc_test_record *record, 
						   struct mismatch_check *mismatch,
						   int samples,
						   int bw)
{
	int 	i, num[4];
	double 	accumlate[4], max[4], min[4], dev[4], avg[4];

	if(bw)
		RFC_DBG("========== 40MHz Mode Test Result ==========\n");
	else
		RFC_DBG("========== 20MHz Mode Test Result ==========\n");

	/* Setup parameter */
	for(i=0; i<4; i++)
		accumlate[i] = max[i] = min[i] = num[i] = dev[i] = 0;

	RFC_DBG("-------------------------------------------------------------------\n");
	RFC_DBG("  NO.        TX gain       TX phase        RX gain       RX phase\n");
	RFC_DBG("-------------------------------------------------------------------\n");
	for(i=0;i<samples;i++)
	{
		RFC_DBG(" %04d      ", i);
		RFC_DBG_DOUBLE(record[i].parm[bw].tx_gain); RFC_DBG("     ");
		RFC_DBG_DOUBLE(record[i].parm[bw].tx_phase); RFC_DBG("     ");
		RFC_DBG_DOUBLE(record[i].parm[bw].rx_gain); RFC_DBG("     ");
		RFC_DBG_DOUBLE(record[i].parm[bw].rx_phase); RFC_DBG("\n");
		
		if(check_max_min_avg(&max[0], &min[0], &accumlate[0], &record[i].parm[bw].tx_gain) == 0)
			num[0] += 1;
		if(check_max_min_avg(&max[1], &min[1], &accumlate[1], &record[i].parm[bw].tx_phase) == 0)
			num[1] += 1;
		if(check_max_min_avg(&max[2], &min[2], &accumlate[2], &record[i].parm[bw].rx_gain) == 0)
			num[2] += 1;
		if(check_max_min_avg(&max[3], &min[3], &accumlate[3], &record[i].parm[bw].rx_phase) == 0)
			num[3] += 1;
	}

	for(i=0; i<4; i++)
	{
		avg[i] = accumlate[i]/num[i];
		accumlate[i] = 0;
	}

	for(i=0; i<samples; i++)
	{
		check_deviation(record[i].parm[bw].tx_gain, &accumlate[0], avg[0]);
		check_deviation(record[i].parm[bw].tx_phase, &accumlate[1], avg[1]);
		check_deviation(record[i].parm[bw].rx_gain, &accumlate[2], avg[2]);
		check_deviation(record[i].parm[bw].rx_phase, &accumlate[3], avg[3]);
	}
	for(i=0; i<4; i++)
	{
		dev[i] = sqrt((accumlate[i] / samples));
	}

	RFC_DBG("-------------------------------------------------------------------\n");
	RFC_DBG(" MAX       ");
		RFC_DBG_DOUBLE(max[0]); RFC_DBG("     "); RFC_DBG_DOUBLE(max[1]); RFC_DBG("     ");
		RFC_DBG_DOUBLE(max[2]); RFC_DBG("     "); RFC_DBG_DOUBLE(max[3]); RFC_DBG("     ");
		RFC_DBG("\n");
	RFC_DBG(" MIN       ");
		RFC_DBG_DOUBLE(min[0]); RFC_DBG("     "); RFC_DBG_DOUBLE(min[1]); RFC_DBG("     ");
		RFC_DBG_DOUBLE(min[2]); RFC_DBG("     "); RFC_DBG_DOUBLE(min[3]); RFC_DBG("     ");
		RFC_DBG("\n");
	RFC_DBG(" AVG       ");
		RFC_DBG_DOUBLE(avg[0]); RFC_DBG("     "); RFC_DBG_DOUBLE(avg[1]); RFC_DBG("     ");
		RFC_DBG_DOUBLE(avg[2]); RFC_DBG("     "); RFC_DBG_DOUBLE(avg[3]); RFC_DBG("     ");
		RFC_DBG("\n");
	RFC_DBG(" DEV       ");
		RFC_DBG_DOUBLE(dev[0]); RFC_DBG("     "); RFC_DBG_DOUBLE(dev[1]); RFC_DBG("     ");
		RFC_DBG_DOUBLE(dev[2]); RFC_DBG("     "); RFC_DBG_DOUBLE(dev[3]); RFC_DBG("     ");
		RFC_DBG("\n");
	RFC_DBG("-------------------------------------------------------------------\n");

	if(mismatch)
	{
		output_rx_iq_mismatch_check_statistics_ht(samples, &mismatch->rx_iq_mismatch[bw]);
		output_tx_iq_mismatch_check_statistics_ht(samples, &mismatch->tx_iq_mismatch[bw]);
	}

	RFC_DBG("----------------------------------------------------------------------------------------------\n");

}
#endif // RFC_DEBUG

int tx_iq_mismatch_check_ht(int bw, int rfc_test_case_no, double *tx_iq_mismatch_check)
{
    double ctl_coe;
    unsigned char cal_mode_sel;
    unsigned char tg_a_i, tg_a_q;
    complex I_signal_f0, Q_signal_f0;
	unsigned int txvga, rxvga;
	unsigned int txcal_vga_20mhz[FREQ_QUANTITY_20MHZ][3] = TXCAL_VGA_20MHZ;
	unsigned int txcal_vga_40mhz[FREQ_QUANTITY_40MHZ][3] = TXCAL_VGA_40MHZ;
	/* for break function */
	unsigned char buf[16];
	(void) buf;
    
	if(bw)	/* 40MHz */
	{
		tg_a_i = txcal_vga_40mhz[5][0];
		tg_a_q = txcal_vga_40mhz[5][0];
		rxvga = txcal_vga_40mhz[5][1];
		txvga = txcal_vga_40mhz[5][2];
	}
	else	/* 20MHz */
	{
		tg_a_i = txcal_vga_20mhz[5][0];
		tg_a_q = txcal_vga_20mhz[5][0];
		rxvga = txcal_vga_20mhz[5][1];
		txvga = txcal_vga_20mhz[5][2];
	}
	
	/* need to confirm */
	bb_register_write(0x1c, 0xee);

    RFC_DBG("tx_iq_mismatch_check_ht()\n");

	ip301_set_iqcal_vga(TXLOOP, rxvga, txvga);

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    ctl_coe = 2.5;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

#if 0
RFC_DBG("press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    RFC_DBG("f0: ");
    RFC_DBG_COMPLEX(Q_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f0));
    RFC_DBG(" db\n");

	tx_iq_mismatch_check[0] = __complex_to_db(&Q_signal_f0);

    ctl_coe *= 2;
    cal_mode_sel = 0x3;
    rx_demod_regs(cal_mode_sel, ctl_coe);

#if 0
RFC_DBG("press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    
	if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    RFC_DBG("2f0: ");
    RFC_DBG_COMPLEX(Q_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f0));
    RFC_DBG(" db\n");

	tx_iq_mismatch_check[1] = __complex_to_db(&Q_signal_f0);

    tx_mux_regs(MUX_BASEBAND, 0, 0, 0);

    return 0;
}

int rx_iq_mismatch_check_ht(int bw, int rfc_test_case_no, double *rx_iq_mismatch_check)
{
    double ctl_coe;
    unsigned char cal_mode_sel;
    unsigned char tg_a_i, tg_a_q;
    complex I_signal_f0, Q_signal_f0;
	unsigned int txvga, rxvga;
	unsigned int rxcal_vga_20mhz[FREQ_QUANTITY_20MHZ][3] = RXCAL_VGA_20MHZ;
	unsigned int rxcal_vga_40mhz[FREQ_QUANTITY_40MHZ][3] = TXCAL_VGA_40MHZ;
	/* for break function */
	unsigned char buf[16];
	(void) buf;

	if(bw)	/* 40MHz */
	{
		tg_a_i = rxcal_vga_40mhz[6][0];
		tg_a_q = rxcal_vga_40mhz[6][0];
		rxvga = rxcal_vga_40mhz[6][1];
		txvga = rxcal_vga_40mhz[6][2];
	}
	else	/* 20MHz */
	{
		tg_a_i = rxcal_vga_20mhz[6][0];
		tg_a_q = rxcal_vga_20mhz[6][0];
		rxvga = rxcal_vga_20mhz[6][1];
		txvga = rxcal_vga_20mhz[6][2];
	}
	
    RFC_DBG("rx_iq_mismatch_check_ht()\n");

    // Set RX loopback mode
	ip301_set_iqcal_vga(RXLOOP, rxvga, txvga);

    // Set f0 = 5 MHz tone and send f0 tone; set RFC demodulator to read f0 tone magnitude
    ctl_coe = 5.0;
    tx_mux_regs(MUX_TONEGEN, tg_a_i, tg_a_q, ctl_coe);

	if(bb_register_read(0) >= 0x32) 
		cal_mode_sel = 0x1;  //Dmod input control: from rxbnc 
	else 
		cal_mode_sel = 0x0;  //Dmod input control: normal mode, from ADC 
    rx_demod_regs(cal_mode_sel, ctl_coe);

    //udelay(1000000);
#if 0
RFC_DBG("press ENTER to continue the test.\n");
WLA_GETS(buf);
#endif
    
	if(read_LPF(&I_signal_f0, &Q_signal_f0, LPF_RESET, READ_LPF_DELAY_SEL))
		return 1;

    RFC_DBG("-f0: ");
    RFC_DBG_COMPLEX(I_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&I_signal_f0));
    RFC_DBG(" db");
    RFC_DBG(", f0: ");
    RFC_DBG_COMPLEX(Q_signal_f0);
    RFC_DBG("  ");
    RFC_DBG_DOUBLE(__complex_to_db(&Q_signal_f0));
    RFC_DBG(" db");
    RFC_DBG("\n");    
	
	rx_iq_mismatch_check[0] = __complex_to_db(&I_signal_f0);
	rx_iq_mismatch_check[1] = __complex_to_db(&Q_signal_f0);

    tx_mux_regs(MUX_BASEBAND, 0, 0, 0);

    return 0;
}

#endif // CONFIG_RFC
