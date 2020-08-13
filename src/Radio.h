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
	void registerUAV(UAV *u);
	bool checkCBBAmsg(UAV *s, UAV *r);

	void sendMessage(int tk, UAV *uSnd, UAV *uRcv, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec);

	void sendBroadcast (int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec);

private:
	std::list<UAV *> uavList;
};

#endif /* RADIO_H_ */
