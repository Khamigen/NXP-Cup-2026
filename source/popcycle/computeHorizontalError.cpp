/*
 * computeHorizontalError.cpp
 *
 *  Created on: 22 Feb 2026
 *      Author: brunofigura
 */

#include <Popcycle/computeHorizontalError.h>

#define FRAME_MIDDLE_X 39

float computeHorizontalError(int laneCenterX){
	return (float)(laneCenterX - FRAME_MIDDLE_X);
}
