/*
 * Radio.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <vector>       // std::vector
#include <list>       // std::list
#include <map>       // std::list

#include "RandomGenerator.h"
#include "UAV.h"

class Radio {
public:
	static Radio& getInstance(void) {
		static Radio    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Radio(void){};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Radio(Radio const&)	= delete;
	void operator=(Radio const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status

public:
	static double linear2dBm(double x) {
		return (10.0 * log10(x * 1000.0));
	}

	static double getUAVChannelLoss (double freq, double tx_height, double rx_height, double dist) {
		double C = 0;
		double temp = rx_height * (1.1 * log10(freq) - 0.7) - (1.56 * log10(freq) - 0.8);
		double sigma = 8; // Standard Deviation for Shadowing Effect

		double path_loss = 46.3 + 33.9 * log10(freq) - 13.82 * log10(tx_height) - temp + log10(dist/1000.0)*(44.9 - 6.55 * log10(tx_height))+C;
		double channel_loss = -path_loss + (-1 * sigma * RandomGenerator::getInstance().getRealNormal(0, 1));

		return channel_loss;
	}

public:
	void registerUAV(UAV *u);
	bool checkCBBAmsg(UAV *s, UAV *r);

	void sendMessage(int tk, UAV *uSnd, UAV *uRcv,
			std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
			std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec);

	void sendBroadcast (int tk, UAV *uSnd,
			std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
			std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec);

private:
	std::list<UAV *> uavList;
};

#endif /* RADIO_H_ */
