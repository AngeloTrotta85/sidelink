/*
 * CommunicationManager.cpp
 *
 *  Created on: Aug 14, 2020
 *      Author: angelo
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list

#include "CommunicationManager.h"
#include "RandomGenerator.h"

using namespace std;

void CommunicationManager::init(std::list<UAV *> &ul, std::list<PoI *> &pl, double cuu, double cpu, double cub) {
	for (auto& u : ul) {
		uavList[u->id] = u;
		uavLtMap[u->id] = 0;
	}
	for (auto& p : pl) {
		poiList[p->id] = p;
	}

	commRange_u2u = cuu;
	commRange_p2u = cpu;
	commRange_u2bs = cub;
}

void CommunicationManager::update(int tk) {
	std::list<UAV *> tmpUAVList;

	if (logSF) {cout << "CommunicationManager::update BEGIN" << endl; fflush(stdout);}

	connGraph.clear();

	for (auto& u : uavList) {
		tmpUAVList.push_back(u.second);

		u.second->father = nullptr;
		u.second->childUAV.clear();
		u.second->childPoI.clear();
	}
	for (auto& p : poiList) {
		p.second->father = nullptr;
	}
	specialUAV_BS->childUAV.clear();
	specialUAV_BS->childPoI.clear();

	if (logSF) {cout << "CommunicationManager::update 1" << endl; fflush(stdout);}

	auto itU = tmpUAVList.begin();
	while (itU != tmpUAVList.end()) {
	//for (auto& u : uavList) {
		//UAV *actUAV = u.second;
		UAV *actUAV = *itU;

		if (actUAV->actual_coord.distance(MyCoord::ZERO) <= commRange_u2bs) {
			node_t nu; // = {UAV_T, actUAV->id, -1, false};
			node_t bs; // = {BS_T, -1, -1, true};

			nu.t = UAV_T;
			nu.uav = actUAV->id;

			bs.t = BS_T;
			bs.uav = BS_ID;

			connGraph.push_back(std::make_pair(nu, bs));
			actUAV->father = specialUAV_BS;
			specialUAV_BS->childUAV.push_back(actUAV);

			itU = tmpUAVList.erase(itU);
		}
		else {
			itU++;
		}
	}

	if (logSF) {
		cout << "CommunicationManager::update 2" << endl;
		cout << "tmpUAVList.size() " << tmpUAVList.size() << endl;
		fflush(stdout);
	}

	while (tmpUAVList.size() > 0) {
		double distmin = std::numeric_limits<double>::max();
		auto itUmin = tmpUAVList.end();
		UAV *uav_father = nullptr;

		if (logSF) {cout << "CommunicationManager::update 2_1" << endl; fflush(stdout);}

		auto u_out = tmpUAVList.begin();
		while (u_out != tmpUAVList.end()) {
		//for (auto& u_out : tmpUAVList){

			if (logSF) {cout << "CommunicationManager::update 2_1_1" << endl; fflush(stdout);}

			UAV *u_in_c = *u_out;

			if (logSF) {cout << "CommunicationManager::update 2_1_2: " << u_in_c->id << endl; fflush(stdout);}
			for (auto& u_in_map : connGraph){

				if (uavList.count(u_in_map.first.uav) != 0) {
					UAV *u_in1 = uavList[u_in_map.first.uav];
					if ((u_in_c->actual_coord.distance(u_in1->actual_coord) < distmin) && (u_in_map.first.t == UAV_T)) {
						distmin = u_in_c->actual_coord.distance(u_in1->actual_coord);
						uav_father = u_in1;
						itUmin = u_out;
					}
				}
				if (uavList.count(u_in_map.second.uav) != 0) {
					UAV *u_in2 = uavList[u_in_map.second.uav];
					if ((u_in_c->actual_coord.distance(u_in2->actual_coord) < distmin) && (u_in_map.second.t == UAV_T)) {
						distmin = u_in_c->actual_coord.distance(u_in2->actual_coord);
						uav_father = u_in2;
						itUmin = u_out;
					}
				}
			}

			u_out++;
		}

		if (logSF) {cout << "CommunicationManager::update 2_2" << endl; fflush(stdout);}

		if (itUmin != tmpUAVList.end()) {
			UAV *minUAV = *itUmin;

			node_t nu_child; // = {UAV_T, actUAV->id, -1, false};
			node_t nu_father; // = {BS_T, -1, -1, true};

			nu_child.t = UAV_T;
			nu_child.uav = minUAV->id;

			nu_father.t = UAV_T;
			nu_father.uav = uav_father->id;

			connGraph.push_back(std::make_pair(nu_child, nu_father));
			minUAV->father = uav_father;
			uav_father->childUAV.push_back(minUAV);


			tmpUAVList.erase(itUmin);
		}
		else {
			//std::cerr << "Error in CommunicationManager::update" << std::endl;

			//for (auto& u : uavList) {
			//	cout << "UAV " << u.first
			//			<< " - Time: " << tk
			//			<< " " << u.second->actual_coord << " dist: " << u.second->actual_coord.length()
			//			<< " - min: " << commRange_u2bs << endl;
			//}
			//exit (EXIT_FAILURE);
		}

		if (logSF) {cout << "CommunicationManager::update 2_3" << endl; fflush(stdout);}
	}

	if (logSF) {cout << "CommunicationManager::update 3" << endl; fflush(stdout);}

	for (auto& p : poiList) {
		double distmin = std::numeric_limits<double>::max();
		UAV *uav_father = nullptr;
		for (auto& u : uavList) {
			if (u.second->actual_coord.distance(p.second->actual_coord) < distmin) {
				distmin = u.second->actual_coord.distance(p.second->actual_coord);
				uav_father = u.second;
			}
		}
		if (uav_father != nullptr) {
			node_t nu_child; // = {UAV_T, actUAV->id, -1, false};
			node_t nu_father; // = {BS_T, -1, -1, true};

			nu_child.t = POI_T;
			nu_child.poi = p.first;

			nu_father.t = UAV_T;
			nu_father.uav = uav_father->id;

			connGraph.push_back(std::make_pair(nu_child, nu_father));
			uav_father->childPoI.push_back(p.second);
			p.second->father = uav_father;
		}
	}

	if (logSF) {cout << "CommunicationManager::update 4" << endl; fflush(stdout);}

	/*cout << "Communication tree: " << endl;
	for (auto& el : connGraph) {
		cout <<
				"<" <<
				((el.first.t == UAV_T) ? "UAV" : ((el.first.t == POI_T) ? "PoI" : "BS")) <<
				((el.first.t == UAV_T) ? el.first.uav : ((el.first.t == POI_T) ? el.first.poi : 0)) <<
				((el.first.t == UAV_T) ? uavList[el.first.uav]->actual_coord : ((el.first.t == POI_T) ? poiList[el.first.poi]->actual_coord : MyCoord::ZERO)) <<
				" > " <<
				"<" <<
				((el.second.t == UAV_T) ? "UAV" : ((el.second.t == POI_T) ? "PoI" : "BS")) <<
				((el.second.t == UAV_T) ? el.second.uav : ((el.second.t == POI_T) ? el.second.poi : 0)) <<
				((el.second.t == UAV_T) ? uavList[el.second.uav]->actual_coord : ((el.first.t == POI_T) ? poiList[el.second.poi]->actual_coord : MyCoord::ZERO)) <<
				" > " <<
				endl;
	}
	cout << endl;*/
	//exit (EXIT_SUCCESS);


	updateLt();

	if (logSF) {cout << "CommunicationManager::update END" << endl; fflush(stdout);}
}

void CommunicationManager::updateLt(void) {
	// init all to 0
	for (auto& u : uavLtMap) {
		u.second = 0;
	}

	for (auto& p : poiList) {
		UAV *f = p.second->father;

		bool reachBS = false;
		while (f != nullptr) {
			if (f->id == BS_ID) {
				reachBS = true;
				break;
			}
			f = f->father;
		}
		if (reachBS) {
			f = p.second->father;
			if (f->id == BS_ID) {
				break;
			}
			else {
				uavLtMap[f->id] += p.second->packetPerSecond;
			}
			f = f->father;
		}
	}
}

void CommunicationManager::sendPacketFromPoI(Packet *p, int tk) {
	double distmin = std::numeric_limits<double>::max();
	UAV *destUAV = nullptr;

	for (auto& u : uavList) {
		double dist = u.second->actual_coord.distance(poiList[p->sourcePoI]->actual_coord);

		if (dist <= commRange_p2u) {
			if (dist < distmin) {
				distmin = dist;
				destUAV = u.second;
			}
		}
	}

	if (destUAV != nullptr) {
		destUAV->rcvPacketFromPoI(p, tk);
	}
	else {
		// packet not generated because no one is covering
	}
}


int CommunicationManager::get_tx_lt(UAV *u) {
	return uavLtMap[u->id];
}

bool CommunicationManager::isDirect (int idu) {
	for (auto& e : connGraph) {
		if (e.first.uav == idu) {
			return (e.second.t == BS_T);
		}
	}
	return false;
}

double CommunicationManager::getRSSIhistory(int sub_frame, int sub_channel) {
	return (rand() % 10);
}

void CommunicationManager::sendDataRB(UAV *u, Packet *p, int timek, int channel) {

	pktToSend_t newp;
	newp.u = u;
	newp.p = p;
	newp.tk = timek;
	newp.sub_ch = channel;

	if (packetToSend.count(channel) == 0) {
		packetToSend[channel] = std::list<pktToSend_t>();
	}
	packetToSend[channel].push_back(newp);
}

void CommunicationManager::manageTransmissionsTimeSlot(int timek) {
	for (auto& el : packetToSend) {
		int channel = el.first;
		for (auto& p : el.second) {
			Packet *pkt = p.p;
			UAV *u = p.u;
			//int sc = p.sub_ch;
			double rssi = 0;
			bool okTx = checkTxSD(pkt->id, el.second, rssi);
			u->updateRssi(timek, channel, rssi);

			if (okTx) {
				for (auto& e : connGraph) {
					if ((e.first.uav == u->id) && (e.second.t == UAV_T)) {
						UAV *destUav = uavList[e.second.uav];
						destUav->rcvPacketFromUAV(pkt, timek);

						break;
					}
				}

			}
		}
	}

	//clear
	for (auto& l : packetToSend) {
		l.second.clear();
	}
	packetToSend.clear();
}

bool CommunicationManager::checkTxSD (int pktID, std::list<pktToSend_t> &allPkst, double &rssi) {
	rssi = RandomGenerator::getInstance().getRealUniform(-100, -60);
	return true;
}





