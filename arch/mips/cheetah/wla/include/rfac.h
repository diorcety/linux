/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file rfac.h
*   \brief
*   \author Montage
*/

#ifndef __RFAC_H__
#define __RFAC_H__


#define CONFIG_RFAC

/* setup the used channel number in the rfac*/
#define RF_CAL_CH 	255

#define RF_DC_LEVEL	4

void ip301_power_on(void) __attribute__ ((section(".rfc")));
void ip301_power_off(void) __attribute__ ((section(".rfc")));

int vco_calibration(int internal_ldo) __attribute__ ((section(".rfc")));
int rx_filter_calibration(u8 bw, u8 filter_code) __attribute__ ((section(".rfc")));
int tx_filter_calibration(u8 bw, u8 filter_code) __attribute__ ((section(".rfc")));
int tx_rx_filter_calibration(u8 bw, u8 tx_filter_code, u8 rx_filter_code) __attribute__ ((section(".rfc")));

int txlo_calibration(int channel) __attribute__ ((section(".rfc")));
int rxdc_calibration_old(u8 gain) __attribute__ ((section(".rfc")));
int rxdc_calibration(u8 gain) __attribute__ ((section(".rfc")));
int rf_calibration(int channel, int internal_ldo) __attribute__ ((section(".rfc")));
int mt301_vco_calibration(int internal_ldo) __attribute__ ((section(".rfc")));
int mt301_rx_filter_calibration(u8 bw, u8 filter_code) __attribute__ ((section(".rfc")));
//int mt301_tx_filter_calibration(u8 bw, u8 filter_code) __attribute__ ((section(".rfc")));
int mt301_txlo_calibration(int channel) __attribute__ ((section(".rfc")));
int mt301_rxdc_calibration(u8 gain) __attribute__ ((section(".rfc")));
int mt301_calibration(int channel, int internal_ldo) __attribute__ ((section(".rfc")));


#ifdef RFAC_ANALYSIS
int tx_filter_calibration_check(void) __attribute__ ((section(".rfc")));
int rx_filter_calibration_check(void) __attribute__ ((section(".rfc")));
void output_vco_curves_statistics(int samples) __attribute__ ((section(".rfc")));
void output_rx_filter_statistics(int samples) __attribute__ ((section(".rfc")));
void output_tx_filter_statistics(int samples) __attribute__ ((section(".rfc")));
void output_txlo_statistics(int samples) __attribute__ ((section(".rfc")));
void output_rxdc_statistics(int samples) __attribute__ ((section(".rfc")));
int rfac_cmd(int argc, char* argv[]) __attribute__ ((section(".rfc")));
#endif // RFAC_ANALYSIS

#endif // __RFAC_H__
