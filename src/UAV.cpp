/*
 * UAV.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "RandomGenerator.h"
#include "UAV.h"

using namespace std;

int UAV::idUAVGen = 0;

UAV::UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt) {
	initVars(recCoord, poiList, nu, movNt, movLt, txNt, txLt);
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt, int id_new) {
	initVars(recCoord, poiList, nu, movNt, movLt, txNt, txLt);
	id = id_new;
}

void UAV::initVars(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt) {
	actual_coord = recCoord;

	basestation = MyCoord::ZERO;

	for (auto& el : poiList) {
		poisList.push_back(el);
	}

	mov_nu = nu;
	mov_nt = movNt;
	mov_lt = movLt;
	tx_nu = nu;
	tx_nt = txNt;
	tx_lt = txLt;

	mov_y_vec.resize(mov_nt, 0);
	mov_z_vec.resize(mov_nt, -1);
	//mov_z_vec.resize(lt, -1);
	//mov_z_vec.resize(lt, -1);
	mov_s_vec.resize(mov_nu, -1);

	tx_y_vec.resize(tx_nt, 0);
	tx_z_vec.resize(tx_nt, -1);
	//tx_z_vec.resize(lt, -1);
	//tx_z_vec.resize(lt, -1);
	tx_s_vec.resize(tx_nu, -1);

	//std::vector<double> y_vec;
	//std::vector<int> z_vec;
	//std::vector<int> b_vec;
	//std::vector<int> p_vec;
}

void UAV::init(void) {
	buildTaskMap();
}

void UAV::buildTaskMap(void) {
	if (poisList.size() == 1) {
		MyCoord target = (*(poisList.begin()))->actual_coord;
		//double interdist = target.distance(basestation) / (((double) mov_nu) + 1.0);
		MyCoord base = target / (((double) mov_nu) + 1.0);

		for (int n = 0; n < mov_nu; n++) {
			taskMap[n] = base * n;
		}
	}

	cout << "taskMap ["; for (auto const &el : taskMap) cout << el.second << " "; cout << "]" << endl;
}


void UAV::generateRandomUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < nu; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);
		double intrange = ((double) ss) / 100.0;
		UAV *newU = new UAV(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange)),
				poiList,
				nu, movNt, movLt, txNt, txLt
		);
		pl.push_back(newU);
		//std::cout << "UAV: " << i << " --> " << newU->recharge_coord << " - Energy:" << newU->max_energy << std::endl;
	}
}

void UAV::executePhase1(void) {
	cout << "Starting phase 1 for UAV" << id << " with position " << actual_coord << endl;

	//movement task
	cout << "Movement task with lt " << mov_lt << endl;
	while (((int) (mov_b_vec.size())) < mov_lt) {
		std::vector<double> c(mov_nt, 0);
		std::vector<double> h(mov_nt, 0);

		for (int j = 0; j < mov_nt; j++) {
			double maxGval = -1;
			for (unsigned int n = 0; n <= mov_p_vec.size(); n++) {
				double actVal = calcMovIncreaseReward(mov_p_vec, n, j);
				if (actVal > maxGval) {
					maxGval = actVal;
				}
			}
			c[j] = maxGval;
		}
		cout << "c ["; for (auto const &el : c) cout << el << " "; cout << "]" << endl;

		for (int j = 0; j < mov_nt; j++) {
			h[j] = (c[j] > mov_y_vec[j]) ? 1 : 0;
		}
		cout << "h ["; for (auto const &el : h) cout << el << " "; cout << "]" << endl;

		int maxJ = -1;
		double maxJval = -1;
		for (int j = 0; j < mov_nt; j++) {
			if ((c[j] * h[j]) > maxJval ) {
				maxJval = c[j] * h[j];
				maxJ = j;
			}
		}
		cout << "maxJ: " << maxJ << endl;

		int maxN = -1;
		double maxNval = -1;
		for (unsigned int n = 0; n <= mov_p_vec.size(); n++) {
			double maxAdd = calcMovIncreaseReward(mov_p_vec, n, maxJ);
			if (maxAdd > maxNval ) {
				maxNval = maxAdd;
				maxN = n;
			}
		}
		cout << "maxN: " << maxN << endl;

		mov_b_vec.push_back(maxJ);
		cout << "b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;

		if (maxN == ((int) (mov_p_vec.size()))) {
			mov_p_vec.push_back(maxJ);
		}
		else if (maxN == 0) {
			mov_p_vec.push_front(maxJ);
		}
		else {
			auto it = mov_p_vec.begin(); // initializing list iterator to beginning
			advance(it, maxN); // iterator to point to maxN-th position
			mov_p_vec.insert(it, maxJ); // using insert to insert maxJ element at the maxN-th
		}
		cout << "p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;

		mov_y_vec[maxJ] = c[maxJ];
		cout << "y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;

		mov_z_vec[maxJ] = id;
		cout << "z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;
	}
}



double UAV::calcMovIncreaseReward(std::list<int> &p_vec, int n, int j) {
	std::list<int> tmp_vec;

	auto it = p_vec.begin();
	for (int i = 0; i <= ((int) (p_vec.size())); i++) {
		if (i == n) {
			tmp_vec.push_back(j);
		}
		else {
			tmp_vec.push_back(*it);
			it++;
		}
	}

	double with = calcMovReward(tmp_vec);
	double without = calcMovReward(p_vec);
	return (with - without);
}

double UAV::calcMovReward(std::list<int> &p_vec) {
	std::list<MyCoord> coord_vec;

	for (auto& in : p_vec) {
		if (taskMap.count(in) == 0) {
			fflush(stdout);
			cerr << "Error in taskMap" << endl; fflush(stderr);
			exit(EXIT_FAILURE);
		}
		else {
			coord_vec.push_back(taskMap[in]);
		}
	}

	if (p_vec.size() == 0) {
		return 0;
	}
	else if (p_vec.size() == 1) {
		MyCoord taskCoord = *(coord_vec.begin());
		double dist = actual_coord.distance(taskCoord);
		if (dist == 0) {
			return std::numeric_limits<double>::max();
		}
		else {
			return (1.0 / dist);
		}
	}
	else {
		cerr << "Error in taskMap. p_vec size: " << p_vec.size() << endl; fflush(stderr);
		exit(EXIT_FAILURE);
		return 0;
	}

}

void UAV::rcvMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec) {

	//update S
	mov_s_vec[id] = tk;
	for (int k = 0; k < ((int)(mov_s_vec.size())); k++) {
		if (k == uSnd->id) {
			mov_s_vec[k] = tk;
		}
		else {
			mov_s_vec[k] = (s_vec[k] > mov_s_vec[k]) ? s_vec[k] : mov_s_vec[k];
		}
	}

	for (int j = 0; j < ((int)(z_vec.size())); j++) {

		if (z_vec[j] == uSnd->id) {					//k (sender) thinks z_kj is k
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				if (y_vec[j] > mov_y_vec[j]) UPDATE;
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				UPDATE;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				UPDATE;
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if ((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) || (y_vec[j] > mov_y_vec[j])) UPDATE;
			}
		}
		else if (z_vec[j] == id) {					//k (sender) thinks z_kj is i
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				//LEAVE
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				RESET;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				//LEAVE
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) RESET;
			}
		}
		else if (z_vec[j] == -1) {  				//k (sender) thinks z_kj is none
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				//LEAVE
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				UPDATE;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				//LEAVE
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) UPDATE;
			}
		}
		else {										//k (sender) thinks z_kj is m neq {i,k}
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				if ((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) || (y_vec[j] > mov_y_vec[j])) UPDATE;
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]){
					UPDATE;
				}
				else {
					RESET;
				}
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) UPDATE;
			}
			else if (mov_z_vec[j] == z_vec[j]) {  		//i (receiver) thinks z_ij is m
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) UPDATE;
			}
			else {										//i (receiver) thinks z_ij is n neq {m,i,k}
				if ((s_vec[z_vec[j]] > mov_s_vec[z_vec[j]]) && (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]])) {
					UPDATE;
				}
				else if((s_vec[z_vec[j]] > mov_s_vec[z_vec[j]]) && (y_vec[j] > mov_y_vec[j])) {
					UPDATE;
				}
				else if((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) && (mov_s_vec[z_vec[j]] > s_vec[z_vec[j]])) {
					RESET;
				}
			}
		}
	}
}

void UAV::executePhase2(void) {

}









