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
	bool logSF = false;

	while (((end_time < 0) || (simulation_time <= end_time)) && (!endSimulation)) {

		//cout << "SimTime: " << simulation_time << endl << endl;

		if (logSF) {cout << "Simulation check 1" << endl; fflush(stdout);}

		CommunicationManager::getInstance().update(simulation_time);

		if (logSF) {cout << "Simulation check 2" << endl; fflush(stdout);}

		//PoI packets generator
		for (auto& p : poisList){
			p->generatePackets_check(simulation_time);
		}


		if (logSF) {cout << "Simulation check 3" << endl; fflush(stdout);}


		// PHASE 1 - Bundle construction
		for (auto& u : uavsList) {
			//u->executePhase1_check(simulation_time);
			////u->executePhase1(simulation_time);
		}


		if (logSF) {cout << "Simulation check 4" << endl; fflush(stdout);}

		for (auto& u : uavsList) {
			u->executePhase1_comm_check(simulation_time);
			//u->executePhase1(simulation_time);
		}


		if (logSF) {cout << "Simulation check 5" << endl; fflush(stdout);}

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


		if (logSF) {cout << "Simulation check 6" << endl; fflush(stdout);}

		//communicate data
		for (auto& u : uavsList) {
			u->comm_data(simulation_time);
		}
		CommunicationManager::getInstance().manageTransmissionsTimeSlot(simulation_time);


		if (logSF) {cout << "Simulation check 7" << endl; fflush(stdout);}

		//move
		//for (auto& u : uavsList) {
			//u->move(simulation_time);
		//}


		if (logSF) {cout << "Simulation check 8" << endl; fflush(stdout);}

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
		if (simulation_time > 1000) exit(0);
	}
}


