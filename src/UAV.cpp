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
#include "Radio.h"

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

void UAV::init(double ts, double velMS, double cbbaMSGsec, double cbbaMSGvar, double phase1MSGsec, double phase1MSGvar) {

	tslot = ts;
	vel_ms = velMS;

	cbba_beacon_interval_sec = cbbaMSGsec;
	cbba_beacon_interval_var = cbbaMSGvar;
	double nexttk;
	do {
		nexttk = RandomGenerator::getInstance().getRealNormal(cbba_beacon_interval_sec, cbba_beacon_interval_var);
	} while (nexttk <= 0);
	next_cbba_beacon_tk = floor(nexttk / ts); //convert in time slot

	phase1_interval_sec = phase1MSGsec;
	phase1_interval_var = phase1MSGvar;
	do {
		nexttk = RandomGenerator::getInstance().getRealNormal(phase1_interval_sec, phase1_interval_var);
	} while (nexttk <= 0);
	next_phase1_tk = floor(nexttk / ts); //convert in time slot

	buildTaskMap();

	Radio::getInstance().registerUAV(this);
}

void UAV::buildTaskMap(void) {
	if (poisList.size() == 1) {
		MyCoord target = (*(poisList.begin()))->actual_coord;
		//double interdist = target.distance(basestation) / (((double) mov_nu) + 1.0);
		MyCoord base = target / (((double) mov_nu) + 2.0);

		for (int n = 0; n < mov_nu; n++) {
			taskMap[n] = base * (n + 1);
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

void UAV::move(int tk) {

	// check if I have a mov-task where to move
	if (mov_b_vec.size() == 1) {
		int target_task = *(mov_b_vec.begin());
		if (taskMap.count(target_task) == 0) {
			fflush(stdout);
			cerr << "Error in move: taskMap" << endl; fflush(stderr);
			exit(EXIT_FAILURE);
		}
		else {
			MyCoord dest = taskMap[target_task];
			MyCoord diff = dest - actual_coord;
			double t, r;
			MyCoord::cartesian2polar(diff.x, diff.y, r, t);
			r = vel_ms * tslot;

			if (r >= diff.length()) {
				actual_coord = dest;
			}
			else {
				MyCoord step = MyCoord::ZERO;
				MyCoord::polar2cartesian(r, t, step.x, step.y);
				actual_coord = actual_coord + step;
			}
		}
	}
	//if (id == 0) {cout << "UAV" << id << " Time: " << tk << " - Position " << actual_coord << endl;}
}

void UAV::executePhase1_check(int tk) {
	if (next_phase1_tk <= tk) {

		//cout << "UAV" << id << " - Sending broadcast at time slot " << tk << endl;

		executePhase1(tk);

		double nexttk;
		do {
			nexttk = RandomGenerator::getInstance().getRealNormal(phase1_interval_sec, phase1_interval_var);
		} while (nexttk <= 0);
		next_phase1_tk = floor(((double) nexttk) / tslot) + tk; //convert in time slot

		//cout << "UAV" << id << " - Time " << tk << " - Next send time slot: " << next_cbba_beacon_tk << endl;
	}
}

void UAV::executePhase1(int tk) {
	if (id == 0) {cout << "TIME: " << tk << " Starting phase 1 for UAV" << id << " with position " << actual_coord << endl;}

	if (id == 0) {cout << "UAV" << id << " - BEFORE b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - BEFORE p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - BEFORE y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - BEFORE z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - BEFORE s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

	// DYNAMIC-VERSION
	// if I have all the Lt tasks assigned, check if my bids are changed

	if (((int) (mov_b_vec.size())) == mov_lt) {
		std::map<int, bool> mapChange;	//contains the map of changes

		for (int j = 0; j < ((int) (mov_z_vec.size())); j++) {
			if (mov_z_vec[j] == id) {
				double actual_gain = update_calcMovIncreaseReward(mov_p_vec, j);
				if (actual_gain >= mov_y_vec[j]) {
					mov_y_vec[j] = actual_gain; //UPDATE ONLY
				}
				else {
					mov_y_vec[j] = 0; //RESET
					mov_z_vec[j] = -1;

					mapChange[j] = true;
				}
			}
		}
		if (mapChange.size() > 0) {		//something changed and have to remove
			bool is_the_first = false;

			auto it_b = mov_b_vec.begin();
			while (it_b != mov_b_vec.end()) {
				for (auto& mc : mapChange) {
					if (mc.first == *it_b) {
						is_the_first = true;
						break;
					}
				}
				if (is_the_first) {		// remove all the b from now on (and the respectively in p)

					while (it_b != mov_b_vec.end()) {	//remove all the choice after the first
						for (auto it_p = mov_p_vec.begin(); it_p != mov_p_vec.end(); it_p++) {		//search for the same task in p
							if (*it_p == *it_b) {
								mov_p_vec.erase(it_p);
								break;
							}
						}

						it_b = mov_b_vec.erase(it_b);
					}

					break; // break the while. I removed everything here
				}

				it_b++;
			}
		}
	}


	//movement task
	//cout << "Movement task with lt " << mov_lt << endl;
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
		//cout << "UAV" << id << " - c ["; for (auto const &el : c) cout << el << " "; cout << "]" << endl;

		for (int j = 0; j < mov_nt; j++) {
			h[j] = (c[j] > mov_y_vec[j]) ? 1 : 0;
		}
		//cout << "UAV" << id << " - h ["; for (auto const &el : h) cout << el << " "; cout << "]" << endl;

		int maxJ = -1;
		double maxJval = -1;
		for (int j = 0; j < mov_nt; j++) {
			if ((c[j] * h[j]) > maxJval ) {
				maxJval = c[j] * h[j];
				maxJ = j;
			}
		}
		//cout << "UAV" << id << " - maxJ: " << maxJ << endl;

		int maxN = -1;
		double maxNval = -1;
		for (unsigned int n = 0; n <= mov_p_vec.size(); n++) {
			double maxAdd = calcMovIncreaseReward(mov_p_vec, n, maxJ);
			if (maxAdd > maxNval ) {
				maxNval = maxAdd;
				maxN = n;
			}
		}
		//cout << "UAV" << id << " - maxN: " << maxN << endl;

		mov_b_vec.push_back(maxJ);
		//cout << "UAV" << id << " - b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;

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
		//cout << "UAV" << id << " - p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;

		mov_y_vec[maxJ] = c[maxJ];
		//cout << "UAV" << id << " - y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;

		mov_z_vec[maxJ] = id;
		//cout << "UAV" << id << " - z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;
	}

	if (id == 0) {cout << "UAV" << id << " - AFTER  b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER  p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER  y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER  z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER  s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << endl;}
}

void UAV::log_cout(int tk) {
	//cout << "UAV" << id << " - TIME " << tk << " - LOG  b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;
	//cout << "UAV" << id << " - TIME " << tk << " - LOG  p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;
	//cout << "UAV" << id << " - TIME " << tk << " - LOG  y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;
	//cout << "UAV" << id << " - TIME " << tk << " - LOG  z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;
	//cout << "UAV" << id << " - TIME " << tk << " - LOG  s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;
	if (id == 0) {cout << "UAV" << id << " - TIME " << tk << " - LOG  pos: " << actual_coord << endl;}
	if (id == 0) {cout << endl;}
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

double UAV::calcMovIncreaseReward_minusN(std::list<int> &p_vec, int n) {
	std::list<int> tmp_vec;

	auto it = p_vec.begin();
	for (int i = 0; i < ((int) (p_vec.size())); i++) {
		if (i != n) {
			tmp_vec.push_back(*it);
		}
		it++;
	}

	double without = calcMovReward(tmp_vec);
	double with = calcMovReward(p_vec);
	return (with - without);
}

double UAV::update_calcMovIncreaseReward(std::list<int> &p_vec, int j) {
	std::list<int> tmp_vec;

	auto it = p_vec.begin();
	for (int i = 0; i < ((int) (p_vec.size())); i++) {
		if (*it != j) {
			tmp_vec.push_back(*it);
		}
		it++;
	}

	double without = calcMovReward(tmp_vec);
	double with = calcMovReward(p_vec);
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

void UAV::cbba_update(int j, double ykj, int zkj) {
	mov_y_vec[j] = ykj;
	mov_z_vec[j] = zkj;
}

void UAV::cbba_reset(int j) {
	mov_y_vec[j] = 0;
	mov_z_vec[j] = -1;
}

void UAV::rcvMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec) {

	if (id == 0) {cout << "UAV" << id << " - TIME: " << tk << " - received packet from UAV" << uSnd->id << endl;}
	if (id == 0) {cout << "UAV" << id << " - RECEIVED y ["; for (auto const &el : y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - RECEIVED z ["; for (auto const &el : z_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - RECEIVED s ["; for (auto const &el : s_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - MY       b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - MY       p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - MY       y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - MY       z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - MY       s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

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

	if (id == 0) {cout << "UAV" << id << " - UPDATED  s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

	//variable backup
	std::vector<int> bkp_mov_z_vec(mov_z_vec);
	std::vector<double> bkp_mov_y_vec(mov_y_vec);
	std::list<int> bkp_mov_b_vec(mov_b_vec);
	std::list<int> bkp_mov_p_vec(mov_p_vec);

	for (int j = 0; j < ((int)(z_vec.size())); j++) {

		if (z_vec[j] == uSnd->id) {					//k (sender) thinks z_kj is k
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				if (y_vec[j] > mov_y_vec[j]) cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if ((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) || (y_vec[j] > mov_y_vec[j]))
					cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
		}
		else if (z_vec[j] == id) {					//k (sender) thinks z_kj is i
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				//LEAVE
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				cbba_reset(j); //RESET;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				//LEAVE
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) cbba_reset(j); //RESET;
			}
		}
		else if (z_vec[j] == -1) {  				//k (sender) thinks z_kj is none
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				//LEAVE
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				//LEAVE
			}
			else {										//i (receiver) thinks z_ij is m neq {i,k}
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
		}
		else {										//k (sender) thinks z_kj is m neq {i,k}
			if (mov_z_vec[j] == id) {					//i (receiver) thinks z_ij is i
				if ((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) || (y_vec[j] > mov_y_vec[j]))
					cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else if (mov_z_vec[j] == uSnd->id) {		//i (receiver) thinks z_ij is k
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]){
					cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
				}
				else {
					cbba_reset(j); //RESET;
				}
			}
			else if (mov_z_vec[j] == -1) {  			//i (receiver) thinks z_ij is none
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else if (mov_z_vec[j] == z_vec[j]) {  		//i (receiver) thinks z_ij is m
				if (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
			}
			else {										//i (receiver) thinks z_ij is n neq {m,i,k}
				if ((s_vec[z_vec[j]] > mov_s_vec[z_vec[j]]) && (s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]])) {
					cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
				}
				else if((s_vec[z_vec[j]] > mov_s_vec[z_vec[j]]) && (y_vec[j] > mov_y_vec[j])) {
					cbba_update(j, y_vec[j], z_vec[j]); //UPDATE;
				}
				else if((s_vec[mov_z_vec[j]] > mov_s_vec[mov_z_vec[j]]) && (mov_s_vec[z_vec[j]] > s_vec[z_vec[j]])) {
					cbba_reset(j); //RESET;
				}
			}
		}
	}

	if (id == 0) {cout << "UAV" << id << " - AFTER UP b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER UP p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER UP y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - AFTER UP z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}

	//variable to check changes
	std::map<int, bool> mapChange;	//contains the map of changes
	//cout << "UAV" << id << " - BEGIN The tasks that have changed ["; for (auto const &el : mapChange) cout << el.first << " "; cout << "]" << endl;
	for (int j = 0; j < ((int) (mov_z_vec.size())); j++) {
		//cout << "UAV" << id << " - Checking old value: " << bkp_mov_z_vec[j] << " with the new value: " << mov_z_vec[j] << " with my ID: " << id << endl;
		if ((bkp_mov_z_vec[j] == id) && (mov_z_vec[j] != id)) {
			mapChange[j] = true;
			//cout << "UAV" << id << " - Task " << j << " has changed" << endl;
		}
	}
	//cout << "UAV" << id << " - AFTER The tasks that have changed ["; for (auto const &el : mapChange) cout << el.first << " "; cout << "]" << endl;
	if (mapChange.size() > 0) {		//something changed and have to remove
		bool is_the_first = false;

		auto it_b = mov_b_vec.begin();
		while (it_b != mov_b_vec.end()) {
			for (auto& mc : mapChange) {
				if (mc.first == *it_b) {
					is_the_first = true;
					break;
				}
			}
			if (is_the_first) {		// remove all the b from now on (and the respectively in p)

				while (it_b != mov_b_vec.end()) {	//remove all the choice after the first
					for (auto it_p = mov_p_vec.begin(); it_p != mov_p_vec.end(); it_p++) {		//search for the same task in p
						if (*it_p == *it_b) {
							mov_p_vec.erase(it_p);
							break;
						}
					}

					it_b = mov_b_vec.erase(it_b);
				}

				break; // break the while. I removed everything here
			}

			it_b++;
		}
	}


	if (id == 0) {cout << "UAV" << id << " - FINAL    b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - FINAL    p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - FINAL    y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << "UAV" << id << " - FINAL    z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (id == 0) {cout << endl;}
}

void UAV::comm_cbba(int tk) {

	if (next_cbba_beacon_tk <= tk) {

		if (id == 0) {cout << "UAV" << id << " - Sending broadcast at time slot " << tk << endl << endl;}

		Radio::getInstance().sendBroadcast(tk, this, mov_y_vec, mov_z_vec, mov_s_vec);

		double nexttk;
		do {
			nexttk = RandomGenerator::getInstance().getRealNormal(cbba_beacon_interval_sec, cbba_beacon_interval_var);
		} while (nexttk <= 0);
		next_cbba_beacon_tk = floor(((double) nexttk) / tslot) + tk; //convert in time slot

		//cout << "UAV" << id << " - Time " << tk << " - Next send time slot: " << next_cbba_beacon_tk << endl;
	}
}

void UAV::comm_data(int tk) {

}









