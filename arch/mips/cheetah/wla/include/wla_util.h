/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file wla_util.h
*   \brief  wla utility of driver.
*   \author Montage
*/

#ifndef _WLA_UTIL_H_
#define _WLA_UTIL_H_

#include <mac_ctrl.h>

/*
 * TIME_AFTER(a,b) returns true if the time a is after time b.
 *
 * Do this with "<0" and ">=0" to only test the sign of the result. A
 * good compiler would generate better code (and a really good compiler
 * wouldn't care). Gcc is currently neither.
 */
#define TIME_AFTER(a,b)		((long)(b) - (long)(a) < 0)
#define TIME_BEFORE(a,b)	TIME_AFTER(b,a)

#define TIME_AFTER_EQ(a,b)	((long)(a) - (long)(b) >= 0)
#define TIME_BEFORE_EQ(a,b)	TIME_AFTER_EQ(b,a)

int bhdr_to_idx(MAC_INFO* info, buf_header *bhdr);
int hw_sw_bhdr_to_idx(MAC_INFO* info, buf_header *bhdr, char is_hwbuf);
buf_header *idx_to_bhdr(MAC_INFO* info, int index);
buf_header *bhdr_find_tail(MAC_INFO* info, buf_header *bhdr);
struct wbuf *dptr_to_wb(char *dptr);
char *wb_to_dptr(struct wbuf *wb);
void fill_bhdr(buf_header *bhdr, struct wbuf *wb, unsigned short index, unsigned short ep);
void fill_bhdr_offset(buf_header *bhdr, unsigned short offset);
unsigned char get_bhdr_offset(buf_header *bhdr);
char *wb_to_llc(struct wbuf *wb);
buf_header* bhdr_get_first(MAC_INFO* info);
void bhdr_insert_tail(MAC_INFO *info, int head, int tail);
struct wbuf *copy_bhdr_to_wb(MAC_INFO* info, unsigned short head, unsigned short tail);

#endif
