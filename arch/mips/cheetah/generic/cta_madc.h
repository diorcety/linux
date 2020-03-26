#ifndef __CTA_MADC_H__
#define __CTA_MADC_H__

#define MAX_CHAN_NUM        2
#define MAXKEYNUM           8

#define CTA_KEY_A           65
#define CTA_KEY_B           66
#define CTA_KEY_C           67
#define CTA_KEY_D           68
#define CTA_KEY_E           69
#define CTA_KEY_F           70
#define CTA_KEY_G           71
#define CTA_KEY_H           72
#define CTA_KEY_I           73
#define CTA_KEY_J           74
#define CTA_KEY_K           75
#define CTA_KEY_L           76
#define CTA_KEY_M           77
#define CTA_KEY_N           78
#define CTA_KEY_O           79
#define CTA_KEY_P           80
#define CTA_KEY_Q           81
#define CTA_KEY_R           82
#define CTA_KEY_S           83
#define CTA_KEY_T           84
#define CTA_KEY_U           85
#define CTA_KEY_V           86
#define CTA_KEY_W           87
#define CTA_KEY_X           88
#define CTA_KEY_Y           89
#define CTA_KEY_Z           90

struct madc_platform_data {
	char name[32];
	char chan;
	char type;
	char misc;      // for special usage each type: EV_KEY, EV_ABS, ...
                    // EV_KEY: keycode offset
	char valid;
};

struct madc_range
{
	unsigned int max;
	unsigned int min;
};

struct madc_mon;

struct madc_type
{
	char type;
	char key;
	char chan;
	char misc;
	char valid;
	unsigned int oval;
	unsigned long last;
	unsigned char state;
	unsigned char stype;

	struct madc_range range[MAXKEYNUM];
	struct delayed_work queue_work;

	struct madc_mon *madc;
	struct input_dev *input;
};

struct madc_mon
{
	struct madc_type madct[MAX_CHAN_NUM];
	unsigned char dbg;
	unsigned int allchan;
	unsigned int polling;
	struct delayed_work madc_work;
};

struct madc_setup {
    const char *name;
    void (*handler)(struct madc_type *madct, char *val,unsigned int total);
	char *val;
};

#endif /* __CTA_MADC_H__ */
