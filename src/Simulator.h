/*
 * Simulator.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list

#include "Generic.h"
#include "UAV.h"
#include "PoI.h"
#include "Radio.h"

class Simulator {
public:
	static Simulator& getInstance(void) {
		static Simulator    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Simulator(void);         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Simulator(Simulator const&)	= delete;
	void operator=(Simulator const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:

	void init(int stime, int etime);

	void setUAVs(std::list<UAV *> &uList) {uavsList = uList;};
	void setPoIs(std::list<PoI *> &pList) {poisList = pList;};

	void run(void);

private:
	std::list<UAV *> uavsList;
	std::list<PoI *> poisList;
	MyCoord basestation;

	int simulation_time;
	int end_time;
	bool endSimulation;

	int timeSlot;


protected:

private:

};

#endif /* SIMULATOR_H_ */
