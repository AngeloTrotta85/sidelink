/*
 * PoI.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "RandomGenerator.h"
#include "PoI.h"

int PoI::idPoIGen = 0;

PoI::PoI(MyCoord posCoord) {
	actual_coord = posCoord;
	id = idPoIGen++;
}

PoI::PoI(MyCoord posCoord, int id_new) {
	actual_coord = posCoord;
	id = id_new;
}


void PoI::generateRandomPoIs(std::list<PoI *> &pl, int ss, int np) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < np; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);
		double intrange = ((double) ss) / 1.0;
		PoI *newP = new PoI(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange))
		);
		pl.push_back(newP);
		//std::cout << "UAV: " << i << " --> " << newU->recharge_coord << " - Energy:" << newU->max_energy << std::endl;
	}
}

