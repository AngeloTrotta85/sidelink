/*
 * Radio.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Radio.h"

void Radio::sendMessage(int tk, UAV *uSnd, UAV *uRcv, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec) {
	uRcv->rcvMessage(tk, uSnd, y_vec, z_vec, s_vec);
}