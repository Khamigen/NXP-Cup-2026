/*
 * calculateSteer.h
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */

#include <stdbool.h>
#include <PopCycle/eBuffer.h>

#ifndef POPCYCLE_CALCULATESTEER_H_
#define POPCYCLE_CALCULATESTEER_H_


float calculateSteer(eBuffer &eb);
float calculateSteer(eBuffer &eb, float *kPot1);


#endif /* POPCYCLE_CALCULATESTEER_H_ */
