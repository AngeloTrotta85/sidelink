/*
 * Simulator.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Simulator.h"

Simulator::Simulator() {
	simulation_time = 0;
	end_time = 0;
	basestation = MyCoord::ZERO;
	timeSlot = 1;
	endSimulation = false;
}


void Simulator::init(int stime, int etime) {
	simulation_time = stime;
	end_time = etime;
}

void Simulator::run() {

	timeSlot = Generic::getInstance().timeSlot;
	endSimulation = false;

	while (((end_time < 0) || (simulation_time < end_time)) && (!endSimulation)) {

		// PHASE 1 - Bundle construction

		for (auto& u : uavsList) {

		}

		// PHASE 2 - Conflict resolution

		simulation_time += timeSlot;
	}

}
