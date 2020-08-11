/*
 * UAV.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "RandomGenerator.h"
#include "UAV.h"

int UAV::idUAVGen = 0;

UAV::UAV(MyCoord recCoord) {
	actual_coord = recCoord;
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, int id_new) {
	actual_coord = recCoord;
	id = id_new;
}


void UAV::generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < nu; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);
		double intrange = ((double) ss) / 100.0;
		UAV *newU = new UAV(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange))
		);
		pl.push_back(newU);
		//std::cout << "UAV: " << i << " --> " << newU->recharge_coord << " - Energy:" << newU->max_energy << std::endl;
	}
}

