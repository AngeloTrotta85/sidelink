/*
 * UAV.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef UAV_H_
#define UAV_H_

#include "MyCoord.h"

class UAV {
public:
	UAV(MyCoord recCoord);
	UAV(MyCoord recCoord, int id_new);

public:
	static void generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu);

public:
	MyCoord actual_coord;

	int id;
	static int idUAVGen;
};

#endif /* UAV_H_ */
