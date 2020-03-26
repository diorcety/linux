/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_bb.h
*   \brief  API definition for BB
*   \author Montage
*/

#ifndef __WLA_BB_H_
#define __WLA_BB_H_

void bb_init(void);
void bb_reset(void);

u8 bb_register_read(int bb_reg);
void bb_register_write(int bb_reg, u8 value);
u8 bb_rssi_decode(u8 val);

/* for debug */
void bb_dump_rx_auto_rfc(void);
void bb_dump_adc_dc(void);
void bb_enable_dump_adc_dc(void);

void bb_control_init(void);
void bb_txpe_off(void);
void bb_rxpe_off(void);
void bb_rxpe_on(void);
void bb_txpe_on(void);

#define MAX_RFC_REG_NUM		50

struct bb_regs {
	u8 num;
	u8 val;
};

struct bb_dev {
	void (*set_20mhz_mode)(void);
	void (*set_40mhz_mode)(int channel_type);
};

extern struct bb_regs rfc_tbl[MAX_RFC_REG_NUM];
extern struct bb_dev *my_bb_dev;
#endif
