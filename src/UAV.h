/*
 * UAV.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef UAV_H_
#define UAV_H_

#include <map>

#include "MyCoord.h"
#include "PoI.h"

class UAV {
public:
	UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt);
	UAV(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt, int id_new);

public:
	static void generateRandomUAVs(std::list<UAV *> &pl, std::list<PoI *> &poiList, int ss, int nu, int movNt, int movLt, int txNt, int txLt);

	void executePhase1(void);
	void executePhase2(void);

	void rcvMessage(int tk, UAV *uSnd, std::vector<double> &y_vec, std::vector<int> &z_vec, std::vector<int> &s_vec);

	double calcMovReward(std::list<int> &p_vec);
	double calcMovIncreaseReward(std::list<int> &p_vec, int n, int j);

	void init(void);
	void buildTaskMap(void);

private:
	void initVars(MyCoord recCoord, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt);

public:
	MyCoord actual_coord;

	std::list<PoI *> poisList;
	MyCoord basestation;

	int id;
	static int idUAVGen;


	std::vector<double> mov_y_vec;
	std::vector<int> mov_z_vec;
	std::list<int> mov_b_vec;
	std::list<int> mov_p_vec;
	std::vector<int> mov_s_vec;
	int mov_nu;
	int mov_nt;
	int mov_lt;
	std::map<int, MyCoord> taskMap;

	std::vector<double> tx_y_vec;
	std::vector<int> tx_z_vec;
	std::list<int> tx_b_vec;
	std::list<int> tx_p_vec;
	std::vector<int> tx_s_vec;
	int tx_nu;
	int tx_nt;
	int tx_lt;
};

#endif /* UAV_H_ */
