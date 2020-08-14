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

using namespace std;

void CommunicationManager::init(std::list<UAV *> &ul, std::list<PoI *> &pl, double cuu, double cpu, double cub) {
	for (auto& u : ul) {
		uavList[u->id] = u;
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

	connGraph .clear();

	for (auto& u : uavList) {
		tmpUAVList.push_back(u.second);
	}

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

			connGraph.push_back(std::make_pair(nu, bs));

			itU = tmpUAVList.erase(itU);
		}
		else {
			itU++;
		}
	}

	while (tmpUAVList.size() > 0) {
		double distmin = std::numeric_limits<double>::max();
		auto itUmin = tmpUAVList.end();
		UAV *uav_father = nullptr;

		auto u_out = tmpUAVList.begin();
		while (u_out != tmpUAVList.end()) {
		//for (auto& u_out : tmpUAVList){
			UAV *u_in_c = *u_out;
			for (auto& u_in_map : connGraph){
				UAV *u_in1 = uavList[u_in_map.first.uav];
				UAV *u_in2 = uavList[u_in_map.second.uav];

				if (u_in_c->actual_coord.distance(u_in1->actual_coord) < distmin) {
					distmin = u_in_c->actual_coord.distance(u_in1->actual_coord);
					uav_father = u_in1;
					itUmin = u_out;
				}

				if (u_in_c->actual_coord.distance(u_in2->actual_coord) < distmin) {
					distmin = u_in_c->actual_coord.distance(u_in2->actual_coord);
					uav_father = u_in2;
					itUmin = u_out;
				}
			}

			u_out++;
		}

		if (itUmin != tmpUAVList.end()) {
			UAV *minUAV = *itU;

			node_t nu_child; // = {UAV_T, actUAV->id, -1, false};
			node_t nu_father; // = {BS_T, -1, -1, true};

			nu_child.t = UAV_T;
			nu_child.uav = minUAV->id;

			nu_father.t = UAV_T;
			nu_father.uav = uav_father->id;

			connGraph.push_back(std::make_pair(nu_child, nu_father));

			tmpUAVList.erase(itUmin);
		}
		else {
			std::cerr << "Error in CommunicationManager::update" << std::endl;
			exit (EXIT_FAILURE);
		}
	}

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
		}
	}

	cout << "Communication tree: " << endl;
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
	cout << endl;
	exit (EXIT_SUCCESS);
}

void CommunicationManager::sendPacketFromPoI(Packet *p) {

}
