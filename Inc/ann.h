/*
 * ann.h
 *
 *  Created on: 22.08.2016
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */

#ifndef ANN_H_
#define ANN_H_

#ifndef FIXEDFANN
#include "floatfann.h"
#else
#include "fixedfann.h"
#endif

//set this define to train more complex led pattern
//undefine to choose just on led to blink
#define ANN_FEEDBACK_OUT_TO_IN

extern void 		ann_start_qlearning(int epochs, float gamma, float epsilon, int steps_ms);
extern void 		ann_executing(struct fann *ann, int steps_ms);
extern void 		ann_set_new_motor(int output);

#endif /* ANN_H_ */
