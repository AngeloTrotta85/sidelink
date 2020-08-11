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

	int time_N = 3600;
	int scenarioSize = 10000;
	int nUAV = 8;
	double timeSlot = 1;


	InputParser input(argc, argv);

	const std::string &inputTimeSim = input.getCmdOption("-time");
	if (!inputTimeSim.empty()) {
		time_N = atoi(inputTimeSim.c_str());
	}
	const std::string &inputNumUAV = input.getCmdOption("-nu");
	if (!inputNumUAV.empty()) {
		nUAV = atoi(inputNumUAV.c_str());
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

	UAV::generateRandomUAVs(uavsList, scenarioSize, nUAV);
	PoI::generateRandomPoIs(poisList, scenarioSize, nUAV);

	Simulator::getInstance().init(0, time_N);
	Simulator::getInstance().setUAVs(uavsList);
	Simulator::getInstance().setPoIs(poisList);
	Simulator::getInstance().run();


	return 0;
}
