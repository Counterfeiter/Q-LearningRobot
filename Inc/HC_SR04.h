/*
 * HC_SR04.h
 *
 *  Created on: 18.10.2016
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_

extern void hcsr04_startMeasure(void);
extern uint32_t hcsr04_getLastDistance_mm(void);
extern void hcsr04_cb_timeroverflow(void);
extern void hcsr04_cb_pin_fallingedge(void);

#endif /* HC_SR04_H_ */
