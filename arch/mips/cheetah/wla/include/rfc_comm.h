/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfc_comm.h
*   \brief
*   \author Montage
*/

#ifndef __RFC_COMM_H__
#define __RFC_COMM_H__

#include <complex.h>
#include <mac_ctrl.h>
#include <wla_debug.h>

#define LPF_RESET 1
#define MAX_RFC_ITERATIONS  	1000
#define FREQ_QUANTITY_40MHZ 	16
#define FREQ_QUANTITY_20MHZ 	8
#define FREQ_TONE_START_20MHZ 	(FREQ_QUANTITY_40MHZ - FREQ_QUANTITY_20MHZ)/2

/* Config the read loop num in the read_LPF() */
#define READ_LPF_CIRCLE 	5
/* Config the OVTH in the rxvga_adjust() */
#define RXVGA_OVTH			100

/* Config BB Reg.0x1D for debug mode */
#define REG1D_VAL			0x96	// original value = 0x46

#define TXLOOP		0
#define RXLOOP		1
#define NON_LOOP	2

#define ANGLE_0		0
#define ANGLE_90	1
#define ANGLE_180	2
#define ANGLE_270	3

#define MUX_BASEBAND 0
#define MUX_TONEGEN 7

#ifndef PI
#define PI 3.1415926
#endif

#define RXLOOP 1
#define CONFIG_RFC
#define CONFIG_RFC_ANALYST 1
//#define RF_POWER_ONOFF_GPIO
//#define INTERACTIVE_MODE

#ifdef CONFIG_RFC_ANALYST
//#define TONE_MASK_ENABLE
#endif	// CONFIG_RFC_ANALYST


#if defined(RFC_DEBUG)
#define RFC_DBG(_str...)			WLA_DBG(WLADEBUG, _str)
#define RFC_DBG_ERROR(_str...)		WLA_DBG(WLAERROR, _str)
#define RFC_DBG_WARN(_str...)		WLA_DBG(WLAWARN, _str)
#define RFC_DBG_DOUBLE(x)			dbg_double(x)
#define BBDBG(x)					WLA_DBG(WLADEBUG, "Read BB %x %x\n", x, bb_register_read(x))
#define RFC_DBG_COMPLEX(x)  print_complex(x)
#else
#define RFC_DBG(...)
#define RFC_DBG_ERROR(...)
#define RFC_DBG_WARN(...)
#define RFC_DBG_DOUBLE(...)
#define BBDBG(...)
#define RFC_DBG_COMPLEX(...)
#define dbg_double_short(...)
#endif

#define RFC_TONE_LIST_20 {-7.5, -4.0625, -2.5, -1.25, 1.25, 2.5, 4.0625, 7.5}
#define RFC_TONE_LIST_40 {-16.25, -13.75, -12.5, -11.25, -7.5, -4.0625, -2.5, -1.25, 1.25, 2.5, 4.0625, 7.5, 11.25, 12.5, 13.75, 16.25}
#define READ_LPF_DELAY_SEL 	0
/* the bb version that supports rx ADC sampling rate 80M */
#define RX_80M_VER 0x4b
/* TX/RX Calibration Table*/
#define TXCAL_VGA_20MHZ {{2, 9, 5}, {2, 7, 5}, {2, 5, 5}, {2, 4, 5}, \
						 {2, 4, 5}, {2, 5, 5}, {2, 7, 5}, {2, 9, 5}}
#define RXCAL_VGA_20MHZ	{{2, 2, 3}, {2, 2, 3}, {2, 2, 3}, {2, 2, 3}, \
						 {2, 2, 3}, {2, 2, 3}, {2, 2, 3}, {2, 2, 3}}
#define TXCAL_VGA_40MHZ	{{1, 9, 4}, {1, 11, 4}, {1, 12, 4}, {1, 12, 4},\
						 {2, 12, 4}, {2, 10, 4}, {2, 8, 4}, {2, 7, 4}, \
						 {2, 7, 4}, {2, 8, 4}, {2, 10, 4}, {2, 12, 4}, \
						 {1, 12, 4}, {1, 12, 4}, {1, 11, 4}, {1, 9, 4}}
#define RXCAL_VGA_40MHZ	{{2, 8, 3}, {2, 8, 3}, {2, 8, 3}, {2, 8, 3}, \
						 {2, 5, 3}, {2, 4, 3}, {2, 4, 3}, {2, 4, 3}, \
						 {2, 4, 3}, {2, 4, 3}, {2, 4, 3}, {2, 5, 3}, \
						 {2, 8, 3}, {2, 8, 3}, {2, 8, 3}, {2, 8, 3}}

#define TX_BAL 0
#define RX_BAL 1

#define RFC_LPF_ALPHA 0x0F
#define RFC_DEMOD_MUTE 0x03
#define RFC_MUX 0x0F
#define RFC_TX_BYPASS 1 <<5
#define RFC_RX_BYPASS 1 <<4

enum 
{
	I_Q_NONE = 0,
	Q_PATH = 1,
	I_PATH = 2,
};

struct vga_entry {
	unsigned short bb_scale;
	unsigned short rxvga;
	unsigned short txvga;
};

struct vga_tbl {
	struct vga_entry txcal_20mhz[FREQ_QUANTITY_20MHZ];
	struct vga_entry rxcal_20mhz[FREQ_QUANTITY_20MHZ];
	struct vga_entry txcal_40mhz[FREQ_QUANTITY_40MHZ];
	struct vga_entry rxcal_40mhz[FREQ_QUANTITY_40MHZ];
};

typedef struct {
    //u8 dc_offset;
    complex gain;
    double phi;
    char a11, a12, a21, a22, n, m;
} mismatch_info;

struct bal_parm{
	complex gain;
	double phi;
};

typedef struct mismatch_curve_data{
	u8 	phase_shifter_ctl;
	double 	phase_shifter_coe;
	u8 	rolloff_filter_ctl;
	double 	rolloff_filter_coe;
} mismatch_curve;

struct rfc_cal_parm {
	mismatch_curve tx_curve;
	mismatch_curve rx_curve;
	mismatch_info tx_avg;
	mismatch_info rx_avg;
};

struct rfc_cal_reg {
	u8 balancer_nm;				//bb20;
	u8 tx_a11;  				//bb21;
	u8 tx_a12; 					//bb22;
	u8 tx_a21; 					//bb23;
	u8 tx_a22; 					//bb24;
	u8 tx_dc_i; 				//bb25;
	u8 rx_a21;  				//bb28;
	u8 rx_a22; 					//bb29;
	u8 rx_dc_i; 				//bb2a;
	u8 tx_dc_q; 				//bb30;
	u8 rx_dc_q; 				//bb31;
	u8 filter_switch; 			//bb5a;
	u8 rolloff_rx_coe; 			//bb5b;
	u8 phaseshifter_rx_alfa;	//bb5c;
	u8 rolloff_tx_coe;			//bb5d;
	u8 phaseshifter_tx_alfa;	//bb5e;
};

struct rfc_record_parm {
	double tx_gain;
	double tx_phase;
	double rx_gain;
	double rx_phase;
	//complex iq_signal[4];
	//struct test_reports abnormal;
};

struct result_arry {
	double rec[MAX_RFC_ITERATIONS][2];
};

struct mismatch_check {
	/* result_arry[0] : 20MHz mode, result_arry[1] : 40MHz mode */
	struct result_arry tx_iq_mismatch[2];	/* rec[0]: f0, rec[1]: 2f0 */ 
	struct result_arry rx_iq_mismatch[2];	/* rec[0]: -f0, rec[1]: f0 */ 
};

struct rfc_test_record {
	/* parm[0] : 20MHz mode, parm[1]: 40MHz mode*/
	struct rfc_record_parm parm[2];	
};

#define READ_RXVGA(x)								\
	do {											\
		x = (ip301_spi_read(0x9) & 0x1f8) >> 4;		\
	} while(0)

#define COMPLEX_DIV(COMPLEX, DIV)			\
	do{										\
		COMPLEX.real = COMPLEX.real/DIV;	\
		COMPLEX.imag = COMPLEX.imag/DIV;	\
	} while(0)

#define SIGN(x, y)			\
	do{						\
		if(x >= 0) 	y = 1;	\
		else	y = -1;		\
	} while(0)

#define MAX_NUM(arry_a, num, val, ret)		\
	do{										\
		ret = arry_a[0];					\
		for(val = 1; val < num; val++)		\
		{									\
			if(arry_a[val] > ret) 			\
				ret = arry_a[val];			\
		}									\
	} while(0)

#define MIN_NUM(arry_a, num, val, ret)		\
	do{										\
		ret = arry_a[0];					\
		for(val = 1; val < num; val++)		\
		{									\
			if(arry_a[val] < ret) 			\
				ret = arry_a[val];			\
		}									\
	} while(0)


/*** DEBUG FUNCTION ***/
#ifdef RFC_DEBUG
void dbg_double(double x) __attribute__ ((section(".rfc")));
void dbg_double_short(double x) __attribute__ ((section(".rfc")));
void print_complex(complex x) __attribute__ ((section(".rfc")));
#endif

/*** MATH/Algorithm FUNCTION ***/
int isnormal(double x) __attribute__ ((section(".rfc")));
double angle_diff(double ang1, double ang2) __attribute__ ((section(".rfc")));
int angle_phase(double ang) __attribute__ ((section(".rfc")));
double calc_angle(double real, double image) __attribute__ ((section(".rfc")));
double calc_phase_shifter_angle(double alpha, double f0 , double fs) __attribute__ ((section(".rfc")));
double calc_rolloff_gain(double a, double f0 , double fs) __attribute__ ((section(".rfc")));
double ctl_coe_calc(int m, double f, double *val) __attribute__ ((section(".rfc")));
int calc_abs_txlo(int a, complex *res_f0, complex *res_2f0, double reduce_factor) __attribute__ ((section(".rfc")));
void calc_data2iq(complex *LPF1, complex *LPF2, unsigned char *data) __attribute__ ((section(".rfc")));
u8 translate_freq_reg2b(double ctl_coe, int sel) __attribute__ ((section(".rfc")));
int cmp_phase_shifter_coe_coarse(struct bal_parm *parm, int bw) __attribute__ ((section(".rfc")));
int cmp_phase_shifter_coe_fine(int alpha_coarse, struct bal_parm *parm, int bw) __attribute__ ((section(".rfc")));
int cmp_rolloff_filter_coe_coarse(struct bal_parm *parm, int bw) __attribute__ ((section(".rfc")));
void cmp_iq_mismatch(complex dn, complex vn, double *result) __attribute__ ((section(".rfc")));
double gain_estimator(complex y_LPF, double gi, int type) __attribute__ ((section(".rfc")));
void bal_parm_mean(struct bal_parm *tx_bal, struct bal_parm *rx_bal, struct rfc_cal_parm *rfc_ret, int freq_quantity) __attribute__ ((section(".rfc")));
void tx_mux_regs(unsigned char mux, unsigned char tg_a_i, unsigned char tg_a_q, double ctl_coe) __attribute__ ((section(".rfc")));
void rx_demod_regs(unsigned char rx_cal_mode_sel, double ctl_coe) __attribute__ ((section(".rfc")));
void toggle_reg2e(unsigned char msk, unsigned char en) __attribute__ ((section(".rfc")));
int read_LPF(complex *LPF1, complex *LPF2, int reset_en, int delay_sel) __attribute__ ((section(".rfc")));
void rx_dc_offset_comp(void) __attribute__ ((section(".rfc")));
int txlo_pos_neg(int dc_pos, int dc_neg, complex *pos_f0, complex *neg_f0, complex *f0) __attribute__ ((section(".rfc")));
void bb_rfc_reset(void) __attribute__ ((section(".rfc")));
unsigned short rxvga_adjust(unsigned short ovth, unsigned short okth) __attribute__ ((section(".rfc")));
void setup_vga_tbl(struct vga_tbl *tbl, int rx_scale, int tx_scale, int rx_txvga, int tx_txvga) __attribute__ ((section(".rfc")));
void rfc_env_setup(struct vga_tbl *vga_table, int rx_scale, int tx_scale, int rx_txvga, int tx_txvga) __attribute__ ((section(".rfc")));
int tx_cal_result_check(int bw, struct vga_tbl *vga_table, double *tx_rec, int sel) __attribute__ ((section(".rfc")));
int rx_cal_result_check(int bw, struct vga_tbl *vga_table)  __attribute__ ((section(".rfc")));
//void tx_balancer_reverse(mismatch_info *mismatch)  __attribute__ ((section(".rfc")));

/*** RESULT PRINT/CHECK FUNCTION ***/
void check_statistics(int samples, struct result_arry *record) __attribute__ ((section(".rfc")));
void output_rx_iq_mismatch_check_statistics_ht(int samples, struct result_arry *rx_iq_mismatch) __attribute__ ((section(".rfc")));
void output_tx_iq_mismatch_check_statistics_ht(int samples, struct result_arry *tx_iq_mismatch) __attribute__ ((section(".rfc")));
int check_max_min_avg(double *val_max, double *val_min, double *sum, double *now) __attribute__ ((section(".rfc")));
void check_deviation(double val, double *accumlate, double average) __attribute__ ((section(".rfc")));
void print_rfc_test_result(struct rfc_test_record *record, struct mismatch_check *mismatch, int samples, int bw) __attribute__ ((section(".rfc")));
int tx_iq_mismatch_check_ht(int bw, int rfc_test_case_no, double *tx_iq_mismatch_check) __attribute__ ((section(".rfc")));
int rx_iq_mismatch_check_ht(int bw, int rfc_test_case_no, double *rx_iq_mismatch_check) __attribute__ ((section(".rfc")));

#ifdef TONE_MASK_ENABLE
int check_parm_with_mask(struct bal_parm *tx_bal, struct bal_parm *rx_bal, int tone_mask, int tone_start, int quantity) __attribute__ ((section(".rfc")));
#endif	// TONE_MASK_ENABLE

#endif // __RFC_COMM_H__
