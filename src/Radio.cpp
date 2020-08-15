/*
 * Radio.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Radio.h"

void Radio::sendMessage(int tk, UAV *uSnd, UAV *uRcv,
		std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
		std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec) {
	uRcv->rcvMessage(tk, uSnd, y_vec, z_vec, s_vec, tx_y_vec, tx_z_vec, tx_s_vec);
}

void Radio::registerUAV(UAV *u) {
	uavList.push_back(u);
}

void Radio::sendBroadcast (int tk, UAV *uSnd,
			std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
			std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec) {
	for (auto& snd : uavList) {
		if (snd->id == uSnd->id) {
			for (auto& rcv : uavList) {
				if (snd->id != rcv->id) {	// send to all the others
					if (checkCBBAmsg(snd, rcv)) {
						rcv->rcvMessage(tk, snd, y_vec, z_vec, s_vec, tx_y_vec, tx_z_vec, tx_s_vec);
					}
				}
			}
			break;
		}
	}
}

bool Radio::checkCBBAmsg(UAV *s, UAV *r) {
	return true;
}






