/*
 * =====================================================================================
 *
 *       Filename:  cta_pwm.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/03/2016 03:04:41 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef _CTA_PWM_H
#define _CTA_PWM_H
enum {
    PWM_A = 0,
    PWM_B,
    PWM_NUM,
};

struct pwm_dev {
    int enable[PWM_NUM];
    int mode[PWM_NUM];
    int duty[PWM_NUM];
    int sweep[PWM_NUM];
    int polarity[PWM_NUM];
};

void pwm_update(void);

#endif
