/*
 * Generic.h
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#ifndef GENERIC_H_
#define GENERIC_H_

#include "list"
#include "map"

#include "MyCoord.h"

class PoI;

class Generic {
public:
	static Generic& getInstance(void) {
		static Generic    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Generic(void){
		timeSlot = 1;
		maxVelocity = 3;
		commRange = 100;
	};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Generic(Generic const&)	= delete;
	void operator=(Generic const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:
	void init(double ts) {
		timeSlot = ts;
	}

	void setUAVParam(double maxVel) {
		maxVelocity = maxVel;
	}

	void setCommParam(double communicationR) {
		commRange = communicationR;
	}

	double getTime2Travel(MyCoord start, MyCoord end);

	void build_static_positions_task_set(std::list<PoI *> &poisList);
	void build_static_comm_task_set(int nsc, int nsubf_in_supf);

public:
	double timeSlot;

	double maxVelocity;

	double commRange;

	std::map<int, MyCoord> posTasks;
	std::map<int, MyCoord> commTasks;

};

#endif /* GENERIC_H_ */
