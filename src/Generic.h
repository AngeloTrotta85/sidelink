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

		txCompetition.resize(20);
		for (unsigned int ii = 0; ii < txCompetition.size(); ii++) {
			txCompetition[ii] = 0;
		}

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
	void init(double ts) {
		timeSlot = ts;
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

public:
	double timeSlot;

	double maxVelocity;

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

	//stats
	double dataGen;
	double dataArrivedAtBS;
	double dataFailed_Drop;
	double dataFailed_Tx;
	double dataFailed_Route;

	std::vector<int> txCompetition;

};

#endif /* GENERIC_H_ */
