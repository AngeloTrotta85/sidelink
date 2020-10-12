/*
 * Simulator.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Simulator.h"
#include "Generic.h"

using namespace std;

Simulator::Simulator() {
	simulation_time = 0;
	end_time = 0;
	stat_time = 0;
	basestation = MyCoord::ZERO;
	timeSlot = 1;
	endSimulation = false;
}


void Simulator::init(int stime, int etime, int staTime) {
	simulation_time = stime;
	end_time = etime;
	stat_time = staTime;

}

void Simulator::run() {

	timeSlot = Generic::getInstance().timeSlot;
	endSimulation = false;
	int logTime = 1000*2;
	bool logSF = true;

	while (((end_time < 0) || (simulation_time <= end_time)) && (!endSimulation)) {

		//cout << "SimTime: " << simulation_time << endl << endl;

		if (logSF) {cout << "Simulation check 1 - time " << simulation_time << endl; fflush(stdout);}

		CommunicationManager::getInstance().update(simulation_time);

		if (logSF) {cout << "Simulation check 1.1" << endl; fflush(stdout);}

		Generic::getInstance().checkStat(simulation_time);

		if (logSF) {cout << "Simulation check 2" << endl; fflush(stdout);}

		//PoI packets generator
		for (auto& p : poisList){
			p->generatePackets_check(simulation_time);
		}


		if (logSF) {cout << "Simulation check 3" << endl; fflush(stdout);}


		// PHASE 1 - Bundle construction
		//for (auto& u : uavsList) {
			//u->executePhase1_check(simulation_time);
			////u->executePhase1(simulation_time);
		//}


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
		if (Generic::getInstance().aType == Generic::CBBA) {
			for (auto& u : uavsList) {
				u->comm_cbba(simulation_time);
			}
		}


		if (logSF) {cout << "Simulation check 6" << endl; fflush(stdout);}

		//communicate data
		for (auto& u : uavsList) {
			u->comm_data(simulation_time);
		}
		if (logSF) {cout << "Simulation check 6+" << endl; fflush(stdout);}
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

		if ((simulation_time % logTime) == 0) {
			cerr << "SimTime: " << simulation_time << endl;
			//for (auto& u : uavsList) {
				//u->log_cout(simulation_time);
			//}
		}

		//cout << endl;
		//simulation_time += timeSlot;
		++simulation_time;
		//if (simulation_time > 1000) exit(0);
	}

	//make stats
	double ok = CommunicationManager::getInstance().sumPacketDelivered ;
	double dc = CommunicationManager::getInstance().sumPacketDropped ;
	double dq = CommunicationManager::getInstance().sumPacketDroppedQueue ;
	double avgMultipleTx = 0;
	if (CommunicationManager::getInstance().countMultipleTx > 0) {
		avgMultipleTx = CommunicationManager::getInstance().sunMultipleTx / CommunicationManager::getInstance().countMultipleTx;
	}
	//cout << "Packets delivered: " << ok << endl;
	//cout << "Packets dropped comm: " << dc << endl;
	//cout << "Packets dropped queue: " << dq << endl;
	double pdr = 0;
	if (ok != 0) {
		pdr = ok / (ok + dc + dq);
	}
	/*if (ok == 0) {
		//cout << "PDR = 0" << endl;
		cout << "0";
	}
	else {
		//cout << "PDR = " << (ok / (ok + dc + dq)) << endl;
		cout << (ok / (ok + dc + dq));
	}*/

	cout << uavsList.size()
			<< " " << Generic::getInstance().pktLoad
			<< " " << Generic::getInstance().uavDist
			<< " " << avgMultipleTx
			<< " " << pdr
			<< endl;

	cout << uavsList.size()
						<< " " << Generic::getInstance().getDataGen()
						<< " " << Generic::getInstance().getDataArrivedAtBs()
						<< " " << Generic::getInstance().getDataFailedDrop()
						<< " " << Generic::getInstance().getDataFailedTx()
						<< " " << Generic::getInstance().getDataFailedRoute()
						<< " " << (Generic::getInstance().getDataArrivedAtBs() / Generic::getInstance().getDataGen())
						<< " " << Generic::getInstance().getDelayAvg()
						<< endl;

	for (unsigned int i = 0; i < Generic::getInstance().getTxCompetition().size(); i++) {
		//cout << i << ":" << Generic::getInstance().getTxCompetition()[i] << "-";
		cout << Generic::getInstance().getTxCompetition()[i] << " ";
	}
	cout << endl;

	for (auto& el : Generic::getInstance().getDataGenTime()) cout << el << " ";
	cout << endl;

	for (auto& el : Generic::getInstance().getDataArrivedAtBsTime()) cout << el << " ";
	cout << endl;

	for (auto& el : Generic::getInstance().getDataFailedDropTime()) cout << el << " ";
	cout << endl;

	for (auto& el : Generic::getInstance().getDataFailedTxTime()) cout << el << " ";
	cout << endl;

	for (auto& el : Generic::getInstance().getDataFailedRouteTime()) cout << el << " ";
	cout << endl;

	auto itGen = Generic::getInstance().getDataGenTime().begin();
	auto itFailTx = Generic::getInstance().getDataFailedTxTime().begin();
	while (
			(itGen != Generic::getInstance().getDataGenTime().end()) &&
			(itFailTx != Generic::getInstance().getDataFailedTxTime().end())
	) {

		cout << ((*itFailTx) / (*itGen)) << " ";

		itGen++;
		itFailTx++;
	}
	cout << endl;

	itGen = Generic::getInstance().getDataGenTime().begin();
	auto itArrived = Generic::getInstance().getDataArrivedAtBsTime().begin();
	while (
			(itGen != Generic::getInstance().getDataGenTime().end()) &&
			(itArrived != Generic::getInstance().getDataArrivedAtBsTime().end())
	) {

		cout << ((*itArrived) / (*itGen)) << " ";

		itGen++;
		itArrived++;
	}
	cout << endl;
}


