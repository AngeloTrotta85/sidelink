//============================================================================
// Name        : sidelink.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list
#include <cstdlib>
#include <ctime>
#include <random>
#include <chrono>

#include "CommunicationManager.h"
#include "RandomGenerator.h"
#include "Simulator.h"
#include "Generic.h"
#include "UAV.h"
#include "PoI.h"

using namespace std;


class InputParser{
public:
	InputParser (int &argc, char **argv){
		for (int i=1; i < argc; ++i)
			this->tokens.push_back(std::string(argv[i]));
	}
	const std::string& getCmdOption(const std::string &option) const{
		std::vector<std::string>::const_iterator itr;
		itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end()){
			return *itr;
		}
		static const std::string empty_string("");
		return empty_string;
	}
	bool cmdOptionExists(const std::string &option) const{
		return std::find(this->tokens.begin(), this->tokens.end(), option)
		!= this->tokens.end();
	}
private:
	std::vector <std::string> tokens;
};

int main(int argc, char **argv) {

	std::list<UAV *> uavsList;
	std::list<PoI *> poisList;

	int time_N = 100000;
	int scenarioSize = 1000;
	int nUAV = 4;
	int nPoI = 1;
	int nTasks_mov = 4;
	int nLt_mov = 1;
	int nTasks_tx = 100;
	int nLt_tx = 1;
	double timeSlot = 0.001;

	//Communication
	double commRange = 100;

	//UAV
	double velocity_ms = 10;

	//PoI
	int pkt_interval_npkt = 10;
	int pkt_interval_slots = 100;

	//CBBA
	double phase1_interval_sec = 3;
	double phase1_interval_var = 1;
	double cbba_beacon_interval_sec = 3;
	double cbba_beacon_interval_var = 0.1;

	//channels
	int nsc = 30;
	int nsubf_in_supf = 60;


	InputParser input(argc, argv);

	const std::string &seedUser = input.getCmdOption("-seed");
	if (!seedUser.empty()) {
		int seedR = atoi(seedUser.c_str());
		RandomGenerator::getInstance().setSeed(seedR);
	}
	else {
		unsigned seedR = std::chrono::system_clock::now().time_since_epoch().count();
		RandomGenerator::getInstance().setSeed(seedR);
	}
	const std::string &inputTimeSim = input.getCmdOption("-time");
	if (!inputTimeSim.empty()) {
		time_N = atoi(inputTimeSim.c_str());
	}
	const std::string &inputNumUAV = input.getCmdOption("-nu");
	if (!inputNumUAV.empty()) {
		nUAV = atoi(inputNumUAV.c_str());
	}
	const std::string &inputNumPoI = input.getCmdOption("-np");
	if (!inputNumPoI.empty()) {
		nPoI = atoi(inputNumPoI.c_str());
	}
	const std::string &inputNumTasksM = input.getCmdOption("-ntM");
	if (!inputNumTasksM.empty()) {
		nTasks_mov = atoi(inputNumTasksM.c_str());
	}
	const std::string &inputNumNLM = input.getCmdOption("-ltM");
	if (!inputNumNLM.empty()) {
		nLt_mov = atoi(inputNumNLM.c_str());
	}
	const std::string &inputNumTasksT = input.getCmdOption("-ntT");
	if (!inputNumTasksT.empty()) {
		nTasks_tx = atoi(inputNumTasksT.c_str());
	}
	const std::string &inputNumNLT = input.getCmdOption("-ltT");
	if (!inputNumNLT.empty()) {
		nLt_tx = atoi(inputNumNLT.c_str());
	}
	const std::string &scenarioMaxVal = input.getCmdOption("-scenario");
	if (!scenarioMaxVal.empty()) {
		scenarioSize = atoi(scenarioMaxVal.c_str());
	}
	const std::string &timeSlot_string = input.getCmdOption("-tSlot");
	if (!timeSlot_string.empty()) {
		timeSlot = atof(timeSlot_string.c_str());
	}
	const std::string &cbbaSec_string = input.getCmdOption("-cbbaS");
	if (!cbbaSec_string.empty()) {
		cbba_beacon_interval_sec = atof(cbbaSec_string.c_str());
	}
	const std::string &cbbaVar_string = input.getCmdOption("-cbbaV");
	if (!cbbaVar_string.empty()) {
		cbba_beacon_interval_var = atof(cbbaVar_string.c_str());
	}
	const std::string &uavVel_string = input.getCmdOption("-vel");
	if (!uavVel_string.empty()) {
		velocity_ms = atof(uavVel_string.c_str());
	}
	const std::string &p1Sec_string = input.getCmdOption("-p1S");
	if (!p1Sec_string.empty()) {
		phase1_interval_sec = atof(p1Sec_string.c_str());
	}
	const std::string &p1Var_string = input.getCmdOption("-p1V");
	if (!p1Var_string.empty()) {
		phase1_interval_var = atof(p1Var_string.c_str());
	}
	const std::string &comRange_string = input.getCmdOption("-cr");
	if (!comRange_string.empty()) {
		commRange = atof(comRange_string.c_str());
	}

	const std::string &numberSubChannels_string = input.getCmdOption("-nsc");
	if (!numberSubChannels_string.empty()) {
		nsc = atoi(numberSubChannels_string.c_str());
	}
	const std::string &numberSubFrames_string = input.getCmdOption("-nsf");
	if (!numberSubFrames_string.empty()) {
		nsubf_in_supf = atoi(numberSubFrames_string.c_str());
	}
	const std::string &nPacket_string = input.getCmdOption("-npktint");
	if (!nPacket_string.empty()) {
		pkt_interval_npkt = atoi(nPacket_string.c_str());
	}
	const std::string &packetSlots_string = input.getCmdOption("-npktslot");
	if (!packetSlots_string.empty()) {
		pkt_interval_slots = atoi(packetSlots_string.c_str());
	}


	Generic::getInstance().init(timeSlot);
	Generic::getInstance().setUAVParam(velocity_ms);
	Generic::getInstance().setCommParam(commRange);

	PoI::generateRandomPoIs(poisList, scenarioSize, nPoI);
	for (auto& p : poisList) {
		p->init(pkt_interval_npkt, pkt_interval_slots);
	}

	Generic::getInstance().build_static_positions_task_set(poisList);
	nTasks_mov = Generic::getInstance().posTasks.size();

	Generic::getInstance().build_static_comm_task_set(nsc, nsubf_in_supf);
	nTasks_mov = Generic::getInstance().posTasks.size();

	UAV::generateRandomUAVs(uavsList, poisList, scenarioSize, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
	for (auto& u : uavsList) {
		u->init(timeSlot, velocity_ms,
				cbba_beacon_interval_sec, cbba_beacon_interval_var, phase1_interval_sec, phase1_interval_var);
		u->initTasks(Generic::getInstance().posTasks);
	}

	CommunicationManager::getInstance().init(uavsList, poisList, commRange, commRange, commRange);

	Simulator::getInstance().init(0, time_N);
	Simulator::getInstance().setUAVs(uavsList);
	Simulator::getInstance().setPoIs(poisList);
	Simulator::getInstance().run();


	return 0;
}
