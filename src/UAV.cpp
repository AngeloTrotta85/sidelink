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

UAV::UAV(MyCoord recCoord, int nu, int movNt, int movLt, int txNt, int txLt) {
	initVars(recCoord, nu, movNt, movLt, txNt, txLt);
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, int nu, int movNt, int movLt, int txNt, int txLt, int id_new) {
	initVars(recCoord, nu, movNt, movLt, txNt, txLt);
	id = id_new;
}

void UAV::initVars(MyCoord recCoord, int nu, int movNt, int movLt, int txNt, int txLt) {
	actual_coord = recCoord;

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

	tx_y_vec.resize(tx_nt, 0);
	tx_z_vec.resize(tx_nt, -1);
	//tx_z_vec.resize(lt, -1);
	//tx_z_vec.resize(lt, -1);

	//std::vector<double> y_vec;
	//std::vector<int> z_vec;
	//std::vector<int> b_vec;
	//std::vector<int> p_vec;
}


void UAV::generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu, int movNt, int movLt, int txNt, int txLt) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < nu; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);
		double intrange = ((double) ss) / 100.0;
		UAV *newU = new UAV(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange)),
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

		for (int j = 0; j < mov_nt; j++) {
			h[j] = (c[j] > mov_y_vec[j]) ? 1 : 0;
		}

		cout << "c ["; for (auto const &el : c) cout << el << " "; cout << "]" << endl;
		cout << "h ["; for (auto const &el : h) cout << el << " "; cout << "]" << endl;

		int maxJ = -1;
		double maxJval = -1;
		for (int j = 0; j < mov_nt; j++) {
			if ((c[j] * h[j]) > maxJval ) {
				maxJval = c[j] * h[j];
				maxJ = j;
			}
		}

		int maxN = -1;
		double maxNval = -1;
		for (unsigned int n = 0; n <= mov_p_vec.size(); n++) {
			double maxAdd = calcMovIncreaseReward(mov_p_vec, n, maxJ);
			if (maxAdd > maxNval ) {
				maxNval = maxAdd;
				maxN = n;
			}
		}

		mov_b_vec.push_back(maxJ);

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

		mov_y_vec[maxJ] = c[maxJ];
		mov_z_vec[maxJ] = id;
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
			cerr << "Error in taskMap" << endl; fflush(stderr);
			exit(EXIT_FAILURE);
		}
		else {
			coord_vec.push_back(taskMap[in]);
		}
	}


	return 0;
}

void UAV::executePhase2(void) {

}









