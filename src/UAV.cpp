/*
 * UAV.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream


#include "CommunicationManager.h"
#include "Generic.h"
#include "RandomGenerator.h"
#include "UAV.h"
#include "Radio.h"

using namespace std;

int UAV::idUAVGen = 0;

UAV::UAV(MyCoord recCoord) {
	actual_coord = recCoord;
	id = BS_ID;
	father = nullptr;
	logUAV = 0;
}

UAV::UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt) {
	initVars(recCoord, poiList, nu, movNt, movLt, txNt, txLt);
	id = idUAVGen++;
}

UAV::UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt, int id_new) {
	initVars(recCoord, poiList, nu, movNt, movLt, txNt, txLt);
	id = id_new;
}

UAV::UAV(int newid) {
	id = newid;
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

	logUAV = 0;
	sumLastFrameReceived = 0;

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

	phase1_comm_interval_sec = phase1MSGsec;
	phase1_comm_interval_var = phase1MSGvar;
	do {
		nexttk = RandomGenerator::getInstance().getRealNormal(phase1_comm_interval_sec, phase1_comm_interval_var);
	} while (nexttk <= 0);
	next_phase1_comm_tk = floor(nexttk / ts); //convert in time slot

	//buildTaskMap();

	Radio::getInstance().registerUAV(this);
}

void UAV::initTasks(std::map<int, MyCoord> &tm) {
	for (auto& el : tm) {
		taskMap[el.first] = el.second;
	}
}

void UAV::initComTasks(std::map<int, MyCoord> &tm, int x_frame, int y_channel, int max_queue) {
	for (auto& el : tm) {
		taskComMap[el.first] = el.second;
	}

	rssiRCV.resize(x_frame);
	for (int x = 0; x < x_frame; x++) {
		rssiRCV[x] = std::vector<double>();
		rssiRCV[x].resize(y_channel);
		for (int y = 0; y < y_channel; y++) {
			rssiRCV[x][y] = 0;
		}
	}

	rssiRCV_dBm.resize(x_frame);
	for (int x = 0; x < x_frame; x++) {
		rssiRCV_dBm[x] = std::vector<double>();
		rssiRCV_dBm[x].resize(y_channel);
		for (int y = 0; y < y_channel; y++) {
			rssiRCV_dBm[x][y] = 0;
		}
	}

	maxPacketQueue = max_queue;
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

void UAV::generateChainUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt) {
	PoI *target = *(poiList.begin());
	MyCoord startCoord = MyCoord::ZERO;
	MyCoord diff = target->actual_coord - startCoord;
	double dist = target->actual_coord.distance(startCoord);

	double r, t;
	MyCoord step = MyCoord::ZERO;
	MyCoord::cartesian2polar(diff.x, diff.y, r, t);
	double stepSize = dist / ((double) (nu + 1));
	//double stepSize = dist / ((double) nu);
	MyCoord::polar2cartesian(stepSize, t, step.x, step.y);

	std::cerr << "Target --> " << target->actual_coord << std::endl;
	for (int i = 1; i <= nu; i++) {
		MyCoord uavPos = startCoord + (step * i);
		if ((uavPos.x < 0.001) && (uavPos.x > -0.001)) uavPos.x = 0;
		if ((uavPos.y < 0.001) && (uavPos.y > -0.001)) uavPos.y = 0;
		UAV *newU = new UAV(uavPos, poiList, nu, movNt, movLt, txNt, txLt);
		pl.push_back(newU);
		std::cerr << "UAV: " << newU->id << " --> " << newU->actual_coord << std::endl;
	}
}

void UAV::generateRandomUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < nu; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);
		//double intrange = ((double) ss) / 200.0;
		double intrange = 20;
		//double intrange = ((double) ss) / 1.0;
		UAV *newU = new UAV(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange)),
				poiList,
				nu, movNt, movLt, txNt, txLt
		);
		pl.push_back(newU);
		std::cout << "UAV: " << newU->id << " --> " << newU->actual_coord << std::endl;
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
	if (logUAV > 0 && id == 0) {cout << "TIME: " << tk << " Starting phase 1 for UAV" << id << " with position " << actual_coord << endl;}

	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - BEFORE b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - BEFORE p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - BEFORE y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - BEFORE z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - BEFORE s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

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
		cout << "UAV" << id << " - c ["; for (auto const &el : c) cout << el << " "; cout << "]" << endl;

		for (int j = 0; j < mov_nt; j++) {
			h[j] = (c[j] > mov_y_vec[j]) ? 1 : 0;
		}
		cout << "UAV" << id << " - h ["; for (auto const &el : h) cout << el << " "; cout << "]" << endl;

		int maxJ = -1;
		double maxJval = -1;
		for (int j = 0; j < mov_nt; j++) {
			if ((c[j] * h[j]) > maxJval ) {
				maxJval = c[j] * h[j];
				maxJ = j;
			}
		}
		cout << "UAV" << id << " - maxJ: " << maxJ << endl;

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

	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - AFTER  b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - AFTER  p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - AFTER  y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - AFTER  z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << "UAV" << id << " - AFTER  s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0 && id == 0) {cout << endl;}
}

void UAV::executePhase1_comm_check(int tk) {
	if (	((tk % Generic::getInstance().superFrame) == 0)
			//&&
			//(!CommunicationManager::getInstance().isDirect(id))
	) {
		executePhase1_comm(tk);
	}

	/*if (next_phase1_comm_tk <= tk) {

		//cout << "UAV" << id << " - Sending broadcast at time slot " << tk << endl;

		executePhase1_comm(tk);

		double nexttk;
		do {
			nexttk = RandomGenerator::getInstance().getRealNormal(phase1_comm_interval_sec, phase1_comm_interval_var);
		} while (nexttk <= 0);
		next_phase1_comm_tk = floor(((double) nexttk) / tslot) + tk; //convert in time slot

		//cout << "UAV" << id << " - Time " << tk << " - Next send time slot: " << next_cbba_beacon_tk << endl;
	}*/

}
void UAV::executePhase1_comm(int tk) {
	//int my_tx_lt = CommunicationManager::getInstance().get_tx_lt(this);
	int my_tx_lt = pktQueue.size() + sumLastFrameReceived;

	//cout << "UAV" << id << " - My TX_Lt: " << my_tx_lt << endl;


//		if (my_tx_lt == 0) {
//			return;
//		}

	if (logUAV > 0) {cout << "TIME: " << tk << " Starting phase_comm 1 for UAV" << id << " with position " << actual_coord << endl;}

	if (logUAV > 0) {cout << "UAV" << id << " - BEFORE b ["; for (auto const &el : tx_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - BEFORE p ["; for (auto const &el : tx_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - BEFORE y ["; for (auto const &el : tx_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - BEFORE z ["; for (auto const &el : tx_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - BEFORE s ["; for (auto const &el : tx_s_vec) cout << el << " "; cout << "]" << endl;}

	// DYNAMIC-VERSION
	// if I have all the Lt tasks assigned, check if my bids are changed

	if (logUAV > 1) {cout << "executePhase1_comm 1" << endl; fflush(stdout);}

	// check il If have more
	if (((int) (tx_b_vec.size())) > my_tx_lt) {
		auto it = tx_b_vec.begin(); // initializing list iterator to beginning
		advance(it, my_tx_lt); // iterator to point to size-th position

		while (it != tx_b_vec.end()) {	//remove all the choice after the first
			for (auto it_p = tx_p_vec.begin(); it_p != tx_p_vec.end(); it_p++) {		//search for the same task in p
				if (*it_p == *it) {
					tx_p_vec.erase(it_p);
					break;
				}
			}

			it = tx_b_vec.erase(it);
		}
	}

	if (logUAV > 1) {cout << "executePhase1_comm 2" << endl; fflush(stdout);}


	if (my_tx_lt == 0) {
		return;
	}


	//cout << "UAV" << id << " - step1 " << my_tx_lt << endl;

	//check for changes
	if (tx_b_vec.size() > 0) {
		std::map<int, bool> mapChange;	//contains the map of changes
		for (int j = 0; j < ((int) (tx_z_vec.size())); j++) {
			if (tx_z_vec[j] == id) {
				double actual_gain = update_calcTxIncreaseReward(tx_p_vec, j);
				if (actual_gain >= tx_y_vec[j]) {
					tx_y_vec[j] = actual_gain; //UPDATE ONLY
				}
				else {
					tx_y_vec[j] = 0; //RESET
					tx_z_vec[j] = -1;

					mapChange[j] = true;
				}
			}
		}
		if (mapChange.size() > 0) {		//something changed and have to remove
			bool is_the_first = false;

			auto it_b = tx_b_vec.begin();
			while (it_b != tx_b_vec.end()) {
				for (auto& mc : mapChange) {
					if (mc.first == *it_b) {
						is_the_first = true;
						break;
					}
				}
				if (is_the_first) {		// remove all the b from now on (and the respectively in p)

					while (it_b != tx_b_vec.end()) {	//remove all the choice after the first
						for (auto it_p = tx_p_vec.begin(); it_p != tx_p_vec.end(); it_p++) {		//search for the same task in p
							if (*it_p == *it_b) {
								tx_p_vec.erase(it_p);
								break;
							}
						}

						it_b = tx_b_vec.erase(it_b);
					}

					break; // break the while. I removed everything here
				}

				it_b++;
			}
		}
	}

	if (logUAV > 1) {cout << "executePhase1_comm 3" << endl; fflush(stdout);}

	//cout << "UAV" << id << " - step2 " << my_tx_lt << endl;


	//tx task
	//cout << "Movement task with lt " << tx_lt << endl;
	while (((int) (tx_b_vec.size())) < my_tx_lt) {
		std::vector<double> c(tx_nt, 0);
		std::vector<double> h(tx_nt, 0);

		//cout << "UAV" << id << " - step3 " << tx_b_vec.size() << endl;
		//cout << "UAV" << id << " - step3 1" << endl;

		if (logUAV > 1) {cout << "executePhase1_comm 3 1 - " << tx_b_vec.size() << " " << my_tx_lt << endl; fflush(stdout);}

		double gainP = calcTxReward(tx_p_vec);

		//cout << "UAV" << id << " - step3 2" << endl;

		for (int j = 0; j < tx_nt; j++) {
			double maxGval = -1;
			bool trovato = false;
			for (auto& ta : tx_p_vec) {
				if (ta == j) {
					trovato = true;
					break;
				}
			}
			if (!trovato) {
				for (unsigned int n = 0; n <= tx_p_vec.size(); n++) {
					double actVal = calcTxIncreaseReward_opt(j);
					//double actVal = calcTxIncreaseReward(gainP, tx_p_vec, n, j);
					if (actVal > maxGval) {
						maxGval = actVal;
					}
				}
			}
			else {
				maxGval = 0;
			}
			c[j] = maxGval;
		}
		//cout << "UAV" << id << " - step3 3" << endl;
		//cout << "UAV" << id << " - c ["; for (auto const &el : c) cout << el << " "; cout << "]" << endl;

		for (int j = 0; j < tx_nt; j++) {
			h[j] = (c[j] > tx_y_vec[j]) ? 1 : 0;
		}
		//cout << "UAV" << id << " - step3 4" << endl;
		//cout << "UAV" << id << " - h ["; for (auto const &el : h) cout << el << " "; cout << "]" << endl;

		int maxJ = -1;
		std::list<int> maxJs;
		double maxJval = -1;
		for (int j = 0; j < tx_nt; j++) {
			if ((c[j] * h[j]) > maxJval ) {
				maxJval = c[j] * h[j];
				maxJ = j;
				maxJs.clear();
				maxJs.push_back(j);
			}
			else if ((c[j] * h[j]) == maxJval) {
				maxJs.push_back(j);
			}
		}
		if (maxJs.size() > 1) {
			auto it = maxJs.begin(); // initializing list iterator to beginning
			advance(it, RandomGenerator::getInstance().getIntUniform(0, maxJs.size()-1)); // iterator to point to size-th position
			maxJ = *it;
		}
		//cout << "UAV" << id << " - step3 5" << endl;
		//cout << "UAV" << id << " - maxJ: " << maxJ << endl;

		int maxN = -1;
		double maxNval = -1;
		for (unsigned int n = 0; n <= tx_p_vec.size(); n++) {
			double maxAdd = calcTxIncreaseReward(gainP, tx_p_vec, n, maxJ);
			if (maxAdd > maxNval ) {
				maxNval = maxAdd;
				maxN = n;
			}
		}
		//cout << "UAV" << id << " - step3 6" << endl;
		//cout << "UAV" << id << " - maxN: " << maxN << endl;

		tx_b_vec.push_back(maxJ);
		//cout << "UAV" << id << " - b ["; for (auto const &el : tx_b_vec) cout << el << " "; cout << "]" << endl;

		if (maxN == ((int) (tx_p_vec.size()))) {
			tx_p_vec.push_back(maxJ);
		}
		else if (maxN == 0) {
			tx_p_vec.push_front(maxJ);
		}
		else {
			auto it = tx_p_vec.begin(); // initializing list iterator to beginning
			advance(it, maxN); // iterator to point to maxN-th position
			tx_p_vec.insert(it, maxJ); // using insert to insert maxJ element at the maxN-th
		}
		//cout << "UAV" << id << " - p ["; for (auto const &el : tx_p_vec) cout << el << " "; cout << "]" << endl;

		tx_y_vec[maxJ] = c[maxJ];
		//cout << "UAV" << id << " - y ["; for (auto const &el : tx_y_vec) cout << el << " "; cout << "]" << endl;

		tx_z_vec[maxJ] = id;
		//cout << "UAV" << id << " - z ["; for (auto const &el : tx_z_vec) cout << el << " "; cout << "]" << endl;
	}

	if (logUAV > 1) {cout << "executePhase1_comm 4" << endl; fflush(stdout);}

	//reset counter
	sumLastFrameReceived = 0;

	if (logUAV > 0) {cout << "UAV" << id << " - AFTER  b ["; for (auto const &el : tx_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER  p ["; for (auto const &el : tx_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER  y ["; for (auto const &el : tx_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER  z ["; for (auto const &el : tx_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER  s ["; for (auto const &el : tx_s_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << endl;}

	//cout << "UAV" << id << " - Finish phase1: " << my_tx_lt << endl;
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

double UAV::calcTxIncreaseReward_opt(int j) {
	MyCoord act = taskComMap[j];
	return calcSingleTxReward (act.x, act.y);
}

double UAV::calcTxIncreaseReward(double without, std::list<int> &p_vec, int n, int j) {
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

	double with = calcTxReward(tmp_vec);
	return (with - without);
}

double UAV::calcTxIncreaseReward(std::list<int> &p_vec, int n, int j) {
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

	double with = calcTxReward(tmp_vec);
	double without = calcTxReward(p_vec);
	return (with - without);
}

double UAV::update_calcTxIncreaseReward(std::list<int> &p_vec, int j) {
	std::list<int> tmp_vec;

	auto it = p_vec.begin();
	for (int i = 0; i < ((int) (p_vec.size())); i++) {
		if (*it != j) {
			tmp_vec.push_back(*it);
		}
		it++;
	}

	double without = calcTxReward(tmp_vec);
	double with = calcTxReward(p_vec);
	return (with - without);
}

double UAV::calcTxReward(std::list<int> &p_vec) {
	std::list<MyCoord> coord_vec;

	for (auto& in : p_vec) {
		if (taskComMap.count(in) == 0) {
			fflush(stdout);
			cerr << "Error in taskMap" << endl; fflush(stderr);
			exit(EXIT_FAILURE);
		}
		else {
			coord_vec.push_back(taskComMap[in]);
		}
	}

	if (p_vec.size() == 0) {
		return 0;
	}
	else {
		double ris = 0;
		for (auto& act : coord_vec) {
			ris += calcSingleTxReward (act.x, act.y);
			//ris += 1.0 / (1.0 + CommunicationManager::getInstance().getRSSIhistory(act.x, act.y));
		}
		return ris;
	}
}

double UAV::calcSingleTxReward(double xframe, double ychannel) {
	//return (1.0 / (1.0 + CommunicationManager::getInstance().getRSSIhistory(xframe, ychannel)));
	//return (1.0 / (1.0 + rssiRCV[xframe][ychannel]));
	double rssi = rssiRCV[xframe][ychannel];
	double rssi_dbm = 10.0 * log10(1000.0*rssi);
	double lb = rssi_dbm - (-95);
	if (lb < 0) {
		return 1;
	}
	else {
		double x = lb * 0.2;
		return (1.0 / (1.0 + x));
	}
	//return (1.0 / (1.0 + rssiRCV[xframe][ychannel]));
}

void UAV::cbba_update(int j, double ykj, int zkj) {
	mov_y_vec[j] = ykj;
	mov_z_vec[j] = zkj;
}

void UAV::cbba_reset(int j) {
	mov_y_vec[j] = 0;
	mov_z_vec[j] = -1;
}

void UAV::rcvMessage(int tk, UAV *uSnd,
			std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec,
			std::vector<double> &tx_y_vec, std::vector<int> &tx_z_vec, std::vector<int> &tx_s_vec) {
	rcvMovMessage(tk, uSnd, y_vec, z_vec, s_vec);
	rcvTxMessage(tk, uSnd, tx_y_vec, tx_z_vec, tx_s_vec);
}

void UAV::rcvTxMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec) {

}

void UAV::rcvMovMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec) {

	if (logUAV > 0) {cout << "UAV" << id << " - TIME: " << tk << " - received packet from UAV" << uSnd->id << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - RECEIVED y ["; for (auto const &el : y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - RECEIVED z ["; for (auto const &el : z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - RECEIVED s ["; for (auto const &el : s_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - MY       b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - MY       p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - MY       y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - MY       z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - MY       s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

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

	if (logUAV > 0) {cout << "UAV" << id << " - UPDATED  s ["; for (auto const &el : mov_s_vec) cout << el << " "; cout << "]" << endl;}

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

	if (logUAV > 0) {cout << "UAV" << id << " - AFTER UP b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER UP p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER UP y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - AFTER UP z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}

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


	if (logUAV > 0) {cout << "UAV" << id << " - FINAL    b ["; for (auto const &el : mov_b_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - FINAL    p ["; for (auto const &el : mov_p_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - FINAL    y ["; for (auto const &el : mov_y_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << "UAV" << id << " - FINAL    z ["; for (auto const &el : mov_z_vec) cout << el << " "; cout << "]" << endl;}
	if (logUAV > 0) {cout << endl;}
}

void UAV::comm_cbba(int tk) {

	if (next_cbba_beacon_tk <= tk) {

		if (logUAV > 0) {cout << "UAV" << id << " - Sending broadcast at time slot " << tk << endl << endl;}

		Radio::getInstance().sendBroadcast(tk, this, mov_y_vec, mov_z_vec, mov_s_vec, tx_y_vec, tx_z_vec, tx_s_vec);

		double nexttk;
		do {
			nexttk = RandomGenerator::getInstance().getRealNormal(cbba_beacon_interval_sec, cbba_beacon_interval_var);
		} while (nexttk <= 0);
		next_cbba_beacon_tk = floor(((double) nexttk) / tslot) + tk; //convert in time slot

		//cout << "UAV" << id << " - Time " << tk << " - Next send time slot: " << next_cbba_beacon_tk << endl;
	}
}

void UAV::rcvPacketFromPoI(Packet *p, int tk) {
	pktInfo_t new_pkt;

	new_pkt.pk = p;
	new_pkt.time_drop_tk = -1;
	new_pkt.time_sent_tk = -1;
	new_pkt.time_received_tk = tk;

	pktQueue.push_back(new_pkt);

	sumLastFrameReceived++;

}

void UAV::rcvPacketFromUAV(Packet *p, int tk) {
	if (((int) pktQueue.size()) >= maxPacketQueue) {
		CommunicationManager::getInstance().packetDroppedQueue(p);
		delete p;
	}
	else {
		pktInfo_t new_pkt;

		new_pkt.pk = p;
		new_pkt.time_drop_tk = -1;
		new_pkt.time_sent_tk = -1;
		new_pkt.time_received_tk = tk;

		pktQueue.push_back(new_pkt);
	}

	sumLastFrameReceived++;
}

void UAV::updateRssi(int tk, int channel, double rssi) {
	if ( (((int) rssiRCV.size()) != Generic::getInstance().superFrame) ||
			(((int) rssiRCV[tk % Generic::getInstance().superFrame].size()) < channel) ) {
		cerr << "Error in UAV::updateRssi" << endl;
		exit(EXIT_FAILURE);
	}
	rssiRCV[tk % Generic::getInstance().superFrame][channel] = rssi;
	rssiRCV_dBm[tk % Generic::getInstance().superFrame][channel] = 10.0 * log10(rssi * 1000.0);
}

void UAV::comm_data(int tk) {
	if (CommunicationManager::getInstance().isDirect(id)) {
		//cout << "UAV" << id << " - Time " << tk << " - I'm directly connected to the BS" << endl;
		comm_directBS(tk);
	}
	else {
		//cout << "UAV" << id << " - Time " << tk << " - Sending mulihop" << endl;
		comm_multihop(tk);
	}
}

void UAV::comm_directBS(int tk) {
	while (pktQueue.size() > 0) {
		pktInfo_t p = pktQueue.front();
		pktQueue.pop_front();

		CommunicationManager::getInstance().packetDeliveretToBS(p.pk);

		std::cout << "PK:" << p.pk->sourcePoI << ":" << p.pk->genTime << " - UAV" << id << " I'm sending direct to the BS at time " << tk << std::endl;

		ofstream f_out(Generic::getInstance().traceOutString, ofstream::out | ofstream::app);
		if (f_out.is_open()) {
			f_out <<
					"U:" << id << ";" <<
					"B:" << 0 << ";" <<
					"TS:" << tk << ";" <<
					"CH:" << 1 << ";" <<
					"POI:" << p.pk->sourcePoI << ";" <<
					"TX:" << p.pk->genTime <<
					std::endl;

			f_out.close();
		}

		delete (p.pk);
	}
}

void UAV::comm_multihop(int tk) {
	if (pktQueue.size() > 0) {
		//cout << "UAV" << id << " - Time " << tk << " - Multihop comm. I have " << pktQueue.size() << " packets to send" << endl;
		std::list<int> bookedCh;
		checkBookForNow(bookedCh, tk%Generic::getInstance().superFrame);
		if (bookedCh.size() > 0) {
			//cout << "UAV" << id << " - Time " << tk << " - Multihop comm. I have " << bookedCh.size() << " channel booked this time slot" << endl;
			for (auto& c : bookedCh) {
				if (pktQueue.size() > 0) {
					pktInfo_t pi = pktQueue.front();
					//Packet *ps = pktQueue.front();
					pktQueue.pop_front();

					std::cout << "PK:" << pi.pk->sourcePoI << ":" << pi.pk->genTime << " - UAV" << id << " I'm sending multi-hop at time slot " << tk << std::endl;

					CommunicationManager::getInstance().sendDataRB(this, pi.pk, tk, c);

					/*ofstream f_out(Generic::getInstance().traceOutString, ofstream::out);
					if (f_out.is_open()) {
						f_out <<
								"U:" << id << ";" <<
								"U:" << xxx << ";" <<
								"TS:" << tk << ";" <<
								"CH:" << c << ";" <<
								"POI:" << pi.pk->sourcePoI << ";" <<
								"TX:" << pi.pk->genTime <<
								std::endl;

						f_out.close();
					}*/
				}
				else {
					break;
				}
			}
		}
	}
}

void UAV::checkBookForNow (std::list<int> &bc, int tkmod) {
	for (auto& b : tx_b_vec) {
		MyCoord frame_channel = taskComMap[b];
		if (frame_channel.x == tkmod) {
			bc.push_back(frame_channel.y);
		}
	}
}







