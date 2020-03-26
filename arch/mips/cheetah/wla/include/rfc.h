/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfc.h
*   \brief
*   \author Montage
*/

#ifndef __RFC_H__
#define __RFC_H__

#include <complex.h>
#include <rfc_comm.h>


/*============================ 1x1 BB (ver < 0x50) ============================*/
void modtest_iq_balancer_new(mismatch_info *param, int is_tx) __attribute__ ((section(".rfc")));
double tx_cal_LPFout_new(unsigned char reset, complex y_LPF, mismatch_info *X_tx_iq_balancer, unsigned char tx_cal_state) __attribute__ ((section(".rfc")));
void compute_balancer_new(struct bal_parm *tx_bal, struct bal_parm *rx_bal, struct rfc_cal_parm *result, int freq_quantity, int bw) __attribute__ ((section(".rfc")));
void transfer_twofilter_regs_new(struct rfc_cal_parm *cal_parm, struct rfc_cal_reg *result) __attribute__ ((section(".rfc")));
void bal_regs_new(mismatch_info param, unsigned char mode) __attribute__ ((section(".rfc")));
int rfc_ht_new(int bw, struct vga_tbl *vga_table, struct rfc_record_parm *parm_record , int test_case_no, int tone_mask, int debug_en) __attribute__ ((section(".rfc")));
int tx_fine_tune_new(mismatch_info *tx_result, double phase_ref, int loop_max, int bw, int freq, unsigned short tg, unsigned short txvga, double phase_step, double gain_step, int debug_en) __attribute__ ((section(".rfc")));
void rfc_process_new(int op) __attribute__ ((section(".rfc")));
void config_rfc_parm_new(int bw) __attribute__ ((section(".rfc")));
void tx_loopback_report_new(int bw, int freq, unsigned short tg_a_i, unsigned short tg_a_q, unsigned short txvga, unsigned short rxvga, unsigned short tx_i_dc, unsigned short tx_q_dc, unsigned short tx_nm, unsigned short br23_phase, unsigned short br24_gain, int agc_en, complex *read_f0, complex *read_2f0, int lpf_reset, int lpf_sel) __attribute__ ((section(".rfc")));
void rx_loopback_report_new(int bw, int freq, unsigned short tg_a,  unsigned short txvga, unsigned short rxvga, unsigned short rx_i_dc, unsigned short rx_q_dc, unsigned short rx_nm, unsigned short br28_phase, unsigned short br29_gain, int agc_en, complex *neg_f0, complex *pos_f0, int lpf_reset, int lpf_sel) __attribute__ ((section(".rfc")));
void txlo_cal_new(void) __attribute__ ((section(".rfc")));

#ifdef CONFIG_RFC_ANALYST
int rfc_ht_cmd_new(int argc, char* argv[]) __attribute__ ((section(".rfc")));
#endif	// CONFIG_RFC_ANALYST

#endif // __RFC_H__
