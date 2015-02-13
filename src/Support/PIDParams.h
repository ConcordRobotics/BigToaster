/*
 * PIDParams.h
 *
 *  Created on: Feb 10, 2015
 *      Author: forsytjr
 */

#ifndef PIDPARAMS_H_
#define PIDPARAMS_H_

class PIDParams {
public:
	float pGain;
	float iGain;
	float dGain;
	float fGain;
	PIDParams (float p, float i = 0.0, float d = 0.0, float f = 0.0);
};




#endif /* SRC_SUPPORT_PIDPARAMS_H_ */
