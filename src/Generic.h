/*
 * Generic.h
 *
 *  Created on: Dec 19, 2018
 *      Author: angelo
 */

#ifndef GENERIC_H_
#define GENERIC_H_

#include "list"
#include "map"
#include "vector"

#include "UAV.h"
#include "MyCoord.h"


class PoI;

class SinrLimits {
public:
	double k1snr;
	double k2snr;
	double l1snr;
	double l2snr;

	double k1sinr;
	double k2sinr;
	double l1sinr;
	double l2sinr;

	double distMaxUAV;
	double distMaxInterf;
	double distMaxBS;

	bool useInterferenceLimits;
};

class Generic {
public:
	typedef enum {
		CBBA,
		RANDOM
	} AlgoType;
public:
	static Generic& getInstance(void) {
		static Generic    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	Generic(void){
		timeSlot = 1;
		maxVelocity = 3;
		commRange = 100;

		uavDist = 1000;
		pktLoad = 100;
		superFrame = 20;
		numSubChannels = 5;

		signalLim = nullptr;
		aType = CBBA;

		dataGen = 0;
		dataArrivedAtBS = 0;
		dataFailed_Drop = 0;
		dataFailed_Tx = 0;
		dataFailed_Route = 0;

		start_time_stats = 0;

		txCompetition.resize(30);
		for (unsigned int ii = 0; ii < txCompetition.size(); ii++) {
			txCompetition[ii] = 0;
		}

//		next_dataGen_time = 0;
//		next_dataArrivedAtBS_time = 0;
//		next_dataFailed_Drop_time = 0;
//		next_dataFailed_Tx_time = 0;
//		next_dataFailed_Route_time = 0;
		next_stat_time = 0;

		delay_sum = 0;
		delay_count = 0;

	};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	Generic(Generic const&)	= delete;
	void operator=(Generic const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:
	void init(double ts, int time_stat) {
		timeSlot = ts;
		start_time_stats = time_stat;
	}

	void setUAVParam(double maxVel) {
		maxVelocity = maxVel;
	}

	void setCommParam(double communicationR, int nsc, int supFrame, int pkt_interval_npkt, double singlePoItest_distance) {
		commRange = communicationR;
		superFrame = supFrame;
		numSubChannels = nsc;
		pktLoad = pkt_interval_npkt;
		uavDist= singlePoItest_distance;
	}

	void setMiscParam(std::string traceString, AlgoType at);
	void setLimitsParam(SinrLimits *sl) {
		signalLim = sl;
	}

	double getTime2Travel(MyCoord start, MyCoord end);

	void build_static_positions_task_set(std::list<PoI *> &poisList);
	void build_static_comm_task_set(int nsc, int nsubf_in_supf);

	double getDataGen() const { return dataGen;	}
	void setDataGen(double incr_dataGen, int tk) {
		if (tk >= start_time_stats) {
			dataGen += incr_dataGen;
		}
//		if (tk == next_dataGen_time) {
//			dataGen_time.push_back(0);
//			next_dataGen_time += 1000;
//		}
		*std::prev(dataGen_time.end()) += incr_dataGen;
	}

	double getDataArrivedAtBs() const { return dataArrivedAtBS;	}
	void setDataArrivedAtBs(double incr_dataArrivedAtBs, int tk) {
		if (tk >= start_time_stats) {
			dataArrivedAtBS += incr_dataArrivedAtBs;
		}
//		if (tk == next_dataArrivedAtBS_time) {
//			dataArrivedAtBS_time.push_back(0);
//			next_dataArrivedAtBS_time += 1000;
//		}
		*std::prev(dataArrivedAtBS_time.end()) += incr_dataArrivedAtBs;
	}

	double getDataFailedDrop() const { return dataFailed_Drop; }
	void setDataFailedDrop(double incr_dataFailedDrop, int tk) {
		if (tk >= start_time_stats) {
			dataFailed_Drop += incr_dataFailedDrop;
		}
//		if (tk == next_dataFailed_Drop_time) {
//			dataFailed_Drop_time.push_back(0);
//			next_dataFailed_Drop_time += 1000;
//		}
		*std::prev(dataFailed_Drop_time.end()) += incr_dataFailedDrop;
	}

	double getDataFailedRoute() const {	return dataFailed_Route; }
	void setDataFailedRoute(double incr_dataFailedRoute, int tk) {
		if (tk >= start_time_stats) {
			dataFailed_Route += incr_dataFailedRoute;
		}
//		if (tk == next_dataFailed_Route_time) {
//			dataFailed_Route_time.push_back(0);
//			next_dataFailed_Route_time += 1000;
//		}
		*std::prev(dataFailed_Route_time.end()) += incr_dataFailedRoute;
	}

	double getDataFailedTx() const { return dataFailed_Tx; }
	void setDataFailedTx(double incr_dataFailedTx, int tk) {
		if (tk >= start_time_stats) {
			dataFailed_Tx += incr_dataFailedTx;
		}
//		if (tk == next_dataFailed_Tx_time) {
//			dataFailed_Tx_time.push_back(0);
//			next_dataFailed_Tx_time += 1000;
//		}
		*std::prev(dataFailed_Tx_time.end()) += incr_dataFailedTx;
	}

	void checkStat(int tk) {
		if (tk == next_stat_time) {

			dataGen_time.push_back(0);
			dataArrivedAtBS_time.push_back(0);
			dataFailed_Drop_time.push_back(0);
			dataFailed_Route_time.push_back(0);
			dataFailed_Tx_time.push_back(0);

			next_stat_time += 1000;
		}
	}

	const std::vector<int>& getTxCompetition() const { return txCompetition; }
	void setTxCompetition(int idx, int incr_val, int tk) {
		if ( (tk >= start_time_stats) && (idx >= 0) && (idx < ((int) txCompetition.size())) ) {
			txCompetition[idx] += incr_val;
		}
	}

	const std::list<double>& getDataArrivedAtBsTime() const {		return dataArrivedAtBS_time;	}
	const std::list<double>& getDataFailedDropTime() const {		return dataFailed_Drop_time;	}
	const std::list<double>& getDataFailedRouteTime() const {		return dataFailed_Route_time;	}
	const std::list<double>& getDataFailedTxTime() const {		return dataFailed_Tx_time;}
	const std::list<double>& getDataGenTime() const {		return dataGen_time;}

	void addDelay(double delay) {
		delay_sum += delay;
		delay_count += 1;
	}
	double getDelayAvg(void) {
		if (delay_count > 0) {
			return (delay_sum / delay_count);
		}
		else {
			return 0;
		}
	}

public:
	double timeSlot;

	double maxVelocity;

	int start_time_stats;

	double commRange;
	int numSubChannels;
	int superFrame;
	int pktLoad;
	double uavDist;

	SinrLimits *signalLim;

	std::string traceOutString;

	AlgoType aType;

	std::map<int, MyCoord> posTasks;
	std::map<int, MyCoord> commTasks;

private:
	//stats
	double dataGen;
	double dataArrivedAtBS;
	double dataFailed_Drop;
	double dataFailed_Tx;
	double dataFailed_Route;

	std::vector<int> txCompetition;

	int next_stat_time;

	std::list<double> dataGen_time;
	//int next_dataGen_time;

	std::list<double> dataArrivedAtBS_time;
	//int next_dataArrivedAtBS_time;

	std::list<double> dataFailed_Drop_time;
	//int next_dataFailed_Drop_time;

	std::list<double> dataFailed_Tx_time;
	//int next_dataFailed_Tx_time;

	std::list<double> dataFailed_Route_time;
	//int next_dataFailed_Route_time;

	double delay_sum;
	double delay_count;
};

#endif /* GENERIC_H_ */
