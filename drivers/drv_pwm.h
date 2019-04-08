#ifndef __PWM_H_
#define __PWM_H_

/*
******************************************************************************
  * @file  pwm.h
  * @author  xin
  * @version V1.0
  * @date    16-May-2017
  * @brief   Null
  ******************************************************************************
*/
void pwm_set_duty_ratio(int channel, int ratio);
int pwm_init(void);
#endif
