/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file arthur.h
*   \brief 
*   \author Montage
*/

#ifndef __ECOS_WLA_H__
#define __ECOS_WLA_H__


#define MAX_2GHZ_CHANNELS	16
#define MAX_5GHZ_CHANNELS	32
#define MAX_RATES		32

#ifndef	HAL_READ_WLANMAC
#define HAL_READ_WLANMAC cdb_get_mac
#endif

#define DEV_INIT		0x8000
#define DEV_ACTIVE		0x4000

#define MBSS_MAX_NUM					3

struct wla_dev {
	MAC_INFO mac_info;
	struct wif_vif vifs[MBSS_MAX_NUM];			/* virtual interface */
	
	unsigned short flags;
	unsigned short irq;
	cyg_handle_t    wla_int_handle;	
	cyg_interrupt   wla_int_obj;

	OS_MUTEX_ID mutex;
	OS_THREAD_ID wla_lock_thread;
	int wla_lock_count;

	//unsigned char macaddr[6];
	char wmac_polling;
};

extern struct wla_dev *my_wla;
#endif // __ECOS_WLA_H__

