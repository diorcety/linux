#ifndef __AD83586_H__
#define __AD83586_H__

struct ad83586_platform_data {
    int (*init_func)(void);
    int *custom_init_value_table;
    int init_value_table_len;
    int *custom_drc2_table;
    int custom_drc2_table_len;
    int *custom_drc2_tko_table;
    int custom_drc2_tko_table_len;
    int *custom_sub_bq_table;
    int custom_sub_bq_table_len;
    int disable_ch2_drc;
    int enable_hardmute;
    int i2c_addr;
};

#define AD83586_STATE_CTRL1							0x00
#define AD83586_STATE_CTRL2							0x01
#define AD83586_STATE_CTRL3							0x02
#define AD83586_MASTER_VOL							0x03
#define AD83586_CHANNEL1_VOL						0x04
#define AD83586_CHANNEL2_VOL						0x05
#define AD83586_CHANNEL3_VOL						0x06
#define AD83586_BASS								0x07
#define AD83586_TREBLE															0x08	
#define AD83586_BASS_MAN														0x09
#define AD83586_STATE_CTRL4													0x0A
#define AD83586_CH_CONF1														0x0B
#define AD83586_CH_CONF2	                          0x0C
#define AD83586_CH_CONF3		                        0x0D
#define AD83586_DRCL                          			0x0E
#define AD83586_STATE_CTRL5													0x11	
#define AD83586_PVDD																0x12
#define AD83586_STATE_CTRL6													0x13
#define AD83586_PWR_SAVE														0x2A
#define AD83586_VOL_TUNE														0x2B
#define AD83586_RESET																0x34

#define AD83586_ID                          				0x2D
//#define AD83586_NUM_BYTE_REG                      	0x2D
#define AD83586_NUM_BYTE_REG                      	0x2E

#endif
