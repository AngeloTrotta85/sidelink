/*
 * Generic.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Generic.h"

double Generic::getTime2Travel(MyCoord start, MyCoord end) {
	return (start.distance(end) / maxVelocity);
}
