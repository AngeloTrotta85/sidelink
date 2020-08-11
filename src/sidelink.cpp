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

	int time_N = 10;
	int scenarioSize = 10000;
	int nUAV = 4;
	int nPoI = 1;
	int nTasks_mov = 4;
	int nLt_mov = 1;
	int nTasks_tx = 100;
	int nLt_tx = 1;
	double timeSlot = 1;


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

	Generic::getInstance().init(timeSlot);

	PoI::generateRandomPoIs(poisList, scenarioSize, nPoI);
	UAV::generateRandomUAVs(uavsList, poisList, scenarioSize, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
	for (auto& u : uavsList) {
		u->init();
	}

	Simulator::getInstance().init(0, time_N);
	Simulator::getInstance().setUAVs(uavsList);
	Simulator::getInstance().setPoIs(poisList);
	Simulator::getInstance().run();


	return 0;
}
