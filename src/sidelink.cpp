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

#include <complex>      // std::complex, std::real

#include <boost/algorithm/string.hpp>

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

double linear2dBm(double x) {
	return (10.0 * log10(x * 1000.0));
}

double getUAVChannelLoss_old (double freq, double tx_height, double rx_height, double dist) {
	double C = 0;
	double temp = rx_height * (1.1 * log10(freq) - 0.7) - (1.56 * log10(freq) - 0.8);
	double sigma = 8; // Standard Deviation for Shadowing Effect

	double path_loss = 46.3 + 33.9 * log10(freq) - 13.82 * log10(tx_height) - temp + log10(dist/1000.0)*(44.9 - 6.55 * log10(tx_height))+C;
	double channel_loss = -path_loss + (-1 * sigma * RandomGenerator::getInstance().getRealNormal(0, 1));

	return channel_loss;
}

double getUAVChannelLoss (double freq, double tx_height, double rx_height, double dist) {
	//double C = 0;
	//double temp = rx_height * (1.1 * log10(freq) - 0.7) - (1.56 * log10(freq) - 0.8);
	double sigma = 8; // Standard Deviation for Shadowing Effect

	//double path_loss = 41.1 + 20.9 * log10(dist);
	//double path_loss = 41.1 + 41.8 * log10(dist);
	double path_loss = 41.1 + 41.8 * log10(dist);

	//double path_loss = 46.3 + 33.9 * log10(freq) - 13.82 * log10(tx_height) - temp + log10(dist/1000.0)*(44.9 - 6.55 * log10(tx_height))+C;
	double channel_loss = -path_loss + (-1 * sigma * RandomGenerator::getInstance().getRealNormal(0, 1));

	return channel_loss;
}
void test(MyCoord rcv, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt) {

	nu = movNt = movLt = txNt = txLt = 0;

	cout << "test 1" << endl; fflush(stdout);

	UAV *u1 = new UAV(MyCoord::ZERO, poiList, nu, movNt, movLt, txNt, txLt);
	UAV *u2 = new UAV(rcv, poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u2 = new UAV(MyCoord(0,50), poiList, nu, movNt, movLt, txNt, txLt);

	double UAV_TX_pt = 0.2512; //%%%% 0.2512 Watt = 24 dBm; % All UAVs transmit with same Power (24 dBm)
	double UAV_TX_pt_db = 24;
	double GAIN_ag = pow(10.0, 0.6);  //% Transmiter Antenna Gain (6 dB)
	double GAIN_ag_db = 6;
	double freq = 3410;
	double tx_height = 30;
	double rx_height = 30;
	double distance = u1->actual_coord.distance(u2->actual_coord);
	double loss_dB = getUAVChannelLoss(freq,tx_height,rx_height,distance);

	cout << "Loss at distance: " << distance << ": " << loss_dB << endl;

	double fading_variance = 1.59; // Fading model is Complex Gaussian Random Variable
	double chan_real = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_complex = RandomGenerator::getInstance().getRealNormal(0, 1);
	std::complex<double> chan (chan_real, chan_complex);
	//chan = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1) + 1i*RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_value = std::abs(chan); // fading loss

	cout << "chan: " << chan << endl;
	cout << "chan_value: " << chan_value << endl;

	cout << "UAV_TX_pt: " << UAV_TX_pt << endl;
	cout << "GAIN_ag: " << GAIN_ag << endl;
	cout << "pow(10.0, loss_dB/10.0): " << pow(10.0, loss_dB/10.0) << endl;

	double receivedPower_db = UAV_TX_pt_db + GAIN_ag_db + loss_dB;
	double receivedPower = pow(10.0, receivedPower_db / 10.0) / 1000.0;
	//double receivedPower = UAV_TX_pt * GAIN_ag * pow(10.0, loss_dB/10.0) ;//(10.^((loss_dB)/10)); // received power including path loss,shadowing
	receivedPower *= chan_value;

	cout << "receivedPower_db: " << receivedPower_db << endl;
	cout << "receivedPower: " << receivedPower << endl;

	double interference = 0;

	// Calculate Noise Parameters
	double temperature = 290; // Kelvin
	double k = 1.3806488 * pow(10.0, -23.0); // Boltzman Constant
	double bw = 9*1e6; // Efective Bandwidth of channel (9 MHz)
	double ue_noise_figure = 7 ; // 7 dB noise figure is considered
	double noise = linear2dBm(k * temperature * bw);
	double total_noise_dBm = ue_noise_figure + noise;
	double total_noise = pow(10.0, total_noise_dBm/10.0) / 1000.0;

	cout << "total_noise_dBm: " << total_noise_dBm << endl;
	cout << "total_noise: " << total_noise << endl;

	double sinr = receivedPower / (interference + total_noise);
	double sinr_db = linear2dBm(sinr);

	cout << "sinr: " << sinr << endl;
	cout << "sinr_db: " << sinr_db << endl;

	double sinr_low = -5;
	double sinr_high = 25;
	double total_samples = sinr_high - sinr_low;

	double prob = 0;
	if (sinr_db <= sinr_low) {
		prob = 0;
	}
	else if (sinr_db >= sinr_high) {
		prob = 1;
	}
	else {
		prob = (sinr_db - sinr_low) / total_samples;
	}

	cout << "prob: " << prob << endl;
}

int main(int argc, char **argv) {

	std::list<UAV *> uavsList;
	std::list<PoI *> poisList;

	string configFile = "";
	string traceFile = "";

	int time_N = 1 * 60 * 60 * 1000;
	int scenarioSize = 1000;
	int nUAV = 4;
	//int nPoI = 1;
	int nTasks_mov = 4;
	int nLt_mov = 1;
	int nTasks_tx = 100;
	int nLt_tx = 1;
	double timeSlot = 0.001;
	//int supFrame = 1000;

	//Communication
	double commRange = 100;
	SinrLimits *signalLimits = new SinrLimits();

	//UAV
	double velocity_ms = 10;
	double max_queue = 1000;

	//PoI
	int pkt_interval_npkt = 20;
	int pkt_interval_slots = 1000;

	//CBBA
	double phase1_interval_sec = 1;
	double phase1_interval_var = 0.1;
	double cbba_beacon_interval_sec = 0.2;
	double cbba_beacon_interval_var = 0.01;

	Generic::AlgoType algoType = Generic::CBBA;

	//channels
	int nsc = 5;
	int nsubf_in_supf = 100;

	double singlePoItest_distance = 100;

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
	const std::string &conf_String = input.getCmdOption("-conf");
	if (!conf_String.empty()) {
		configFile = conf_String;
	}
	const std::string &trace_String = input.getCmdOption("-trace");
	if (!trace_String.empty()) {
		traceFile = trace_String;
	}
	const std::string &inputTimeSim = input.getCmdOption("-time");
	if (!inputTimeSim.empty()) {
		time_N = atoi(inputTimeSim.c_str());
	}
	const std::string &inputNumUAV = input.getCmdOption("-nu");
	if (!inputNumUAV.empty()) {
		nUAV = atoi(inputNumUAV.c_str());
	}
	const std::string &str_algoType = input.getCmdOption("-at");
	if (!str_algoType.empty()) {
		int aatt = atoi(str_algoType.c_str());
		switch (aatt) {
			case 0:
				algoType = Generic::CBBA;
				break;

			default:
				algoType = Generic::RANDOM;
				break;
		}

	}
	//const std::string &inputNumPoI = input.getCmdOption("-np");
	//if (!inputNumPoI.empty()) {
	//	nPoI = atoi(inputNumPoI.c_str());
	//}
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
	//const std::string &superFrame_string = input.getCmdOption("-supF");
	//if (!superFrame_string.empty()) {
	//	supFrame = atoi(superFrame_string.c_str());
	//}
	const std::string &singlePoIdist_string = input.getCmdOption("-sPoIdist");
	if (!singlePoIdist_string.empty()) {
		singlePoItest_distance = atof(singlePoIdist_string.c_str());
	}
	const std::string &maxQueue_string = input.getCmdOption("-uavMaxQ");
	if (!maxQueue_string.empty()) {
		max_queue = atoi(maxQueue_string.c_str());
	}

	Generic::getInstance().init(timeSlot);
	Generic::getInstance().setUAVParam(velocity_ms);
	Generic::getInstance().setCommParam(commRange, nsc, nsubf_in_supf, pkt_interval_npkt, singlePoItest_distance);
	Generic::getInstance().setMiscParam(traceFile, algoType);

	if (configFile.length() > 0) {
		ifstream infile_pos;
		infile_pos.open (configFile, std::ifstream::in);
		if (infile_pos.is_open()){
			bool continue_read = true;
			std::string line;

			//cout << "FILE " << fin_pos << endl;

			while ( (std::getline(infile_pos, line)) && (continue_read) ) {
				std::string delimiter_field = ";";
				std::string delimiter_eq = ":";

				cout << "Line: " << line << endl;fflush(stdout);

				bool isUAVdef = false;
				bool isPOIdef = false;

				//if (line.rfind("U:", 0) != 0) {
				if (line.find("U:") != std::string::npos) {
					if (line.find("x") != std::string::npos) {
						isUAVdef = true; // found
					}
					else if (line.find("POI:") != std::string::npos) {
						isPOIdef = true; // found
					}
					else {
						break;
					}
				}

				cout << "isUAVdef " << isUAVdef << endl;fflush(stdout);
				cout << "isPOIdef " << isPOIdef << endl;fflush(stdout);

				vector<string> strs;
				boost::split(strs, line, boost::is_any_of(";"));

				//int uavIdx = -1;
				UAV *newUAV = nullptr;
				PoI *newPOI = nullptr;

				for (auto& var : strs) {
					//cout << var << endl;

					vector<string> strs_var;
					boost::split(strs_var, var, boost::is_any_of(":"));

					//for (auto& el : strs_var) {
					//	cout << el << endl;
					//}

					if (strs_var.size() == 2) {
						if (strs_var[0].compare("U") == 0) {
							//uavIdx = stoi(strs_var[1]);
							//uavPos[uavIdx] = MyCoord::ZERO;
							if (isUAVdef) {
								newUAV = new UAV(stoi(strs_var[1]) - 1);
							}
							else {
								// in POI def
								if (isPOIdef) {
									//cout << "For PoI" << newPOI->id << " looking for UAV" << strs_var[1] << endl;
									for (auto& uu : uavsList) {
										if (uu->id == (stoi(strs_var[1]) - 1)) {
											newPOI->actual_coord = uu->actual_coord;
											break;
										}
									}
									if (newPOI->actual_coord == MyCoord::ZERO) {
										cerr << "PoI should be positioned" << endl;
									}
								}
								else {
									cerr << "PoI should be defined" << endl;
								}
							}
						}
						else if (strs_var[0].compare("x") == 0) {
							//uavPos[uavIdx].x = stod(strs_var[1]);
							if (isUAVdef) {
								newUAV->actual_coord.x = stod(strs_var[1]);
							}
						}
						else if (strs_var[0].compare("y") == 0) {
							//uavPos[uavIdx].y = stod(strs_var[1]);
							if (isUAVdef) {
								newUAV->actual_coord.y = stod(strs_var[1]);
							}
						}
						else if (strs_var[0].compare("POI") == 0) {
							if (isPOIdef) {
								newPOI = new PoI(stoi(strs_var[1]) - 1);
							}
						}
						else if (strs_var[0].compare("BS") == 0) {
							break;
						}
						else if (strs_var[0].compare("NC") == 0) {
							nsc = stoi(strs_var[1]);
						}
						else if (strs_var[0].compare("DR") == 0) {
							for (auto& pp : poisList) {
								pp->init(stoi(strs_var[1]));
							}
						}
						else if (strs_var[0].compare("DB") == 0) {
							signalLimits->distMaxBS = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("DI") == 0) {
							signalLimits->distMaxInterf = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("DM") == 0) {
							signalLimits->distMaxUAV = stod(strs_var[1]);
							commRange = stod(strs_var[1]);

							//continue_read = false;
						}
						else if (strs_var[0].compare("K1SNR") == 0) {
							signalLimits->k1snr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("K2SNR") == 0) {
							signalLimits->k2snr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("L1SNR") == 0) {
							signalLimits->l1snr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("L2SNR") == 0) {
							signalLimits->l2snr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("K1SINR") == 0) {
							signalLimits->k1sinr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("K2SINR") == 0) {
							signalLimits->k2sinr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("L1SINR") == 0) {
							signalLimits->l1sinr = stod(strs_var[1]);
						}
						else if (strs_var[0].compare("L2SINR") == 0) {
							signalLimits->l2sinr = stod(strs_var[1]);
						}
					}
				}

				if (newUAV != nullptr) {
					uavsList.push_back(newUAV);
				}
				if (newPOI != nullptr) {
					poisList.push_back(newPOI);
				}
			}

			Generic::getInstance().build_static_positions_task_set(poisList);
			nTasks_mov = Generic::getInstance().posTasks.size();

			Generic::getInstance().build_static_comm_task_set(nsc, nsubf_in_supf);
			nTasks_tx = Generic::getInstance().commTasks.size();

			/*cout << "Created a comm task of " << nTasks_tx << " tasks";
			cout << " starting from " << nsc << " channels and " << nsubf_in_supf << " timeslots" << endl;
			for (auto& t : Generic::getInstance().commTasks) {
				cout << t.first << " ---> " << t.second << endl;
			}

			fflush (stdout);
			exit(0);*/

			for (auto& u : uavsList) {
				/*for (auto& pp : poisList) {
					u->poisList.push_back(pp);
				}*/

				u->initVars(u->actual_coord, poisList, uavsList.size(), nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
			}
			for (auto& u : uavsList) {
				u->init(timeSlot, velocity_ms,
						cbba_beacon_interval_sec, cbba_beacon_interval_var, phase1_interval_sec, phase1_interval_var);
				u->initTasks(Generic::getInstance().posTasks);
				u->initComTasks(Generic::getInstance().commTasks, nsubf_in_supf, nsc, max_queue);
			}

			for (auto& u : uavsList) {
				cout << "UAV" << u->id << " at pos: " << u->actual_coord << endl;
			}
			for (auto& p : poisList) {
				cout << "POI" << p->id << " at pos: " << p->actual_coord << endl;
			}

			//UAV *newU = new UAV(uavPos, poiList, nu, movNt, movLt, txNt, txLt);

			Generic::getInstance().setLimitsParam(signalLimits);

			infile_pos.close();
		}
	}
	else {
		//PoI::generateRandomPoIs(poisList, scenarioSize, nPoI);
		PoI::generateSinglePoI(poisList, scenarioSize, singlePoItest_distance, nUAV);
		for (auto& p : poisList) {
			p->init(pkt_interval_npkt, pkt_interval_slots);
		}

		Generic::getInstance().build_static_positions_task_set(poisList);
		nTasks_mov = Generic::getInstance().posTasks.size();

		Generic::getInstance().build_static_comm_task_set(nsc, nsubf_in_supf);
		nTasks_tx = Generic::getInstance().commTasks.size();

		//UAV::generateRandomUAVs(uavsList, poisList, scenarioSize, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
		UAV::generateChainUAVs(uavsList, poisList, scenarioSize, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
		for (auto& u : uavsList) {
			u->init(timeSlot, velocity_ms,
					cbba_beacon_interval_sec, cbba_beacon_interval_var, phase1_interval_sec, phase1_interval_var);
			u->initTasks(Generic::getInstance().posTasks);
			u->initComTasks(Generic::getInstance().commTasks, nsubf_in_supf, nsc, max_queue);
		}

	}
	//exit(0);

	/*for (int d = 100; d <= 1000; d+=100) {
		MyCoord rcv(0, d);
		test(rcv, poisList, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
		cout << endl;
	}*/
	//test(MyCoord(0, 4), poisList, nUAV, nTasks_mov, nLt_mov, nTasks_tx, nLt_tx);
	//exit(EXIT_FAILURE);

	CommunicationManager::getInstance().init(uavsList, poisList, commRange, commRange, commRange, nsc);

	Simulator::getInstance().init(0, time_N);
	Simulator::getInstance().setUAVs(uavsList);
	Simulator::getInstance().setPoIs(poisList);
	Simulator::getInstance().run();


	return 0;
}
