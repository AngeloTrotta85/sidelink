/*
 * Simulator.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Simulator.h"

using namespace std;

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
	//int logTime = 1000;

	while (((end_time < 0) || (simulation_time <= end_time)) && (!endSimulation)) {

		//cout << "SimTime: " << simulation_time << endl << endl;

		CommunicationManager::getInstance().update(simulation_time);

		//PoI packets generator
		for (auto& p : poisList){
			p->generatePackets_check(simulation_time);
		}


		// PHASE 1 - Bundle construction
		for (auto& u : uavsList) {
			u->executePhase1_check(simulation_time);
			//u->executePhase1(simulation_time);
		}

		//communication
		//for (auto& u1 : uavsList) {
		//	for (auto& u2 : uavsList) {
		//		if (u1->id != u2->id) {
		//			Radio::getInstance().sendMessage(simulation_time, u1, u2, u1->mov_y_vec, u1->mov_z_vec, u1->mov_s_vec);
		//		}
		//	}
		//}

		//communicate control
		for (auto& u : uavsList) {
			u->comm_cbba(simulation_time);
		}

		//communicate data
		for (auto& u : uavsList) {
			u->comm_data(simulation_time);
		}

		//move
		for (auto& u : uavsList) {
			u->move(simulation_time);
		}

		// PHASE 2 - Conflict resolution
		//for (auto& u : uavsList) {
		//	u->executePhase2();
		//}

		//if ((simulation_time % logTime) == 0) {
			//for (auto& u : uavsList) {
				//u->log_cout(simulation_time);
			//}
		//}

		//cout << endl;
		//simulation_time += timeSlot;
		++simulation_time;
	}
}


