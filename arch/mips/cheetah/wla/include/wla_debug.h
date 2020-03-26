/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_debug.h
*   \brief  wla debug function.
*   \author Montage
*/

#ifndef __WLA_DEBUG_H_
#define __WLA_DEBUG_H_

#define WLADEBUG		0x1
#define WLAWARN			0x2
#define WLAERROR		0x4

#ifdef DEBUG_WLA_MIN_LEVEL
#define WLA_DBG(_mode, ...) \
	do {                           \
		if ((_mode) >= DEBUG_WLA_MIN_LEVEL) \
			DBG(__VA_ARGS__);     \
	} while(0)
#if DEBUG_WLA_MIN_LEVEL > WLAWARN
#define WLA_DUMP_BUF(x, y) \
	(void)(x); \
	(void)(y)
#endif
#else
#define WLA_DBG(_mode, ...)	DBG(__VA_ARGS__)
#endif

#define WLA_MSG	DBG
void reg_dump(struct seq_file *s, unsigned char *addr,int size);
void dump_bcap(struct seq_file *s, MAC_INFO* info, int index, char is_ds);
void dump_sta(struct seq_file *s, MAC_INFO* info, int sta_idx, char rate_tbl);
void dump_ba(struct seq_file *s, MAC_INFO* info, int sta_idx);

#ifdef DEBUG_TPUT
void tput_info(MAC_INFO *info);

#define WLA_TIMESTAMP_TRACE(_dir, _n) \
	do { \
		if(_dir) \
			info->tput_dp[_n] = getmicrotime(); \
	} while(0)

#define WLA_TIMESTAMP_DELTA_CAL(_dir) \
	do { \
		if(_dir) \
		tput_info(info); \
	} while(0)
#else
#define WLA_TIMESTAMP_TRACE(_dir, _n)
#define WLA_TIMESTAMP_DELTA_CAL(_dir)
#endif

#endif //__WLA_DEBUG_H_
