/*
 * CommunicationManager.cpp
 *
 *  Created on: Aug 14, 2020
 *      Author: angelo
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <stack>
#include <map>       // std::list


#include <complex>      // std::complex, std::real

#include "CommunicationManager.h"
#include "RandomGenerator.h"

using namespace std;

void CommunicationManager::init(std::list<UAV *> &ul, std::list<PoI *> &pl, double cuu, double cpu, double cub, int nsc) {
	for (auto& u : ul) {
		uavList[u->id] = u;
		uavLtMap[u->id] = 0;
	}
	for (auto& p : pl) {
		poiList[p->id] = p;
	}

	commRange_u2u = cuu;
	commRange_p2u = cpu;
	commRange_u2bs = cub;

	for (int channel = 0; channel < nsc; channel++) {
		packetToSend[channel] = std::list<pktToSend_t>();
	}
}

void CommunicationManager::update(int tk) {
	std::list<UAV *> tmpUAVList;

	if (logSF) {cout << "CommunicationManager::update BEGIN" << endl; fflush(stdout);}

	connGraph.clear();

	for (auto& u : uavList) {
		tmpUAVList.push_back(u.second);

		u.second->father = nullptr;
		u.second->childUAV.clear();
		u.second->childPoI.clear();
	}
	for (auto& p : poiList) {
		p.second->father = nullptr;
	}
	specialUAV_BS->childUAV.clear();
	specialUAV_BS->childPoI.clear();

	if (logSF) {cout << "CommunicationManager::update 1" << endl; fflush(stdout);}

	auto itU = tmpUAVList.begin();
	double distmin_bs = std::numeric_limits<double>::max();
	auto itU_rm = tmpUAVList.end();
	while (itU != tmpUAVList.end()) {
		UAV *actUAV = *itU;
		if (actUAV->actual_coord.distance(MyCoord::ZERO) < distmin_bs) {
			distmin_bs = actUAV->actual_coord.distance(MyCoord::ZERO);
			itU_rm = itU;
		}
		itU++;
	}
	if (itU_rm != tmpUAVList.end()) {
		UAV *actUAV = *itU_rm;

		node_t nu; // = {UAV_T, actUAV->id, -1, false};
		node_t bs; // = {BS_T, -1, -1, true};

		nu.t = UAV_T;
		nu.uav = actUAV->id;

		bs.t = BS_T;
		bs.uav = BS_ID;

		connGraph.push_back(std::make_pair(nu, bs));
		actUAV->father = specialUAV_BS;
		specialUAV_BS->childUAV.push_back(actUAV);

		tmpUAVList.erase(itU_rm);
	}
	else {
		cerr << "Errore in CommunicationManager::update" << endl;
		exit(EXIT_FAILURE);
	}

	/*
	auto itU = tmpUAVList.begin();
	while (itU != tmpUAVList.end()) {
	//for (auto& u : uavList) {
		//UAV *actUAV = u.second;
		UAV *actUAV = *itU;

		if (actUAV->actual_coord.distance(MyCoord::ZERO) <= commRange_u2bs) {
			node_t nu; // = {UAV_T, actUAV->id, -1, false};
			node_t bs; // = {BS_T, -1, -1, true};

			nu.t = UAV_T;
			nu.uav = actUAV->id;

			bs.t = BS_T;
			bs.uav = BS_ID;

			connGraph.push_back(std::make_pair(nu, bs));
			actUAV->father = specialUAV_BS;
			specialUAV_BS->childUAV.push_back(actUAV);

			itU = tmpUAVList.erase(itU);
		}
		else {
			itU++;
		}
	}
	*/

	if (logSF) {
		cout << "CommunicationManager::update 2" << endl;
		cout << "tmpUAVList.size() " << tmpUAVList.size() << endl;
		fflush(stdout);
	}

	while (tmpUAVList.size() > 0) {
		double distmin = std::numeric_limits<double>::max();
		auto itUmin = tmpUAVList.end();
		UAV *uav_father = nullptr;

		if (logSF) {cout << "CommunicationManager::update 2_1" << endl; fflush(stdout);}

		auto u_out = tmpUAVList.begin();
		while (u_out != tmpUAVList.end()) {
		//for (auto& u_out : tmpUAVList){

			if (logSF) {cout << "CommunicationManager::update 2_1_1" << endl; fflush(stdout);}

			UAV *u_in_c = *u_out;

			if (logSF) {cout << "CommunicationManager::update 2_1_2: " << u_in_c->id << endl; fflush(stdout);}
			for (auto& u_in_map : connGraph){

				if (uavList.count(u_in_map.first.uav) != 0) {
					UAV *u_in1 = uavList[u_in_map.first.uav];
					if ((u_in_c->actual_coord.distance(u_in1->actual_coord) < distmin) && (u_in_map.first.t == UAV_T)) {
						distmin = u_in_c->actual_coord.distance(u_in1->actual_coord);
						uav_father = u_in1;
						itUmin = u_out;
					}
				}
				if (uavList.count(u_in_map.second.uav) != 0) {
					UAV *u_in2 = uavList[u_in_map.second.uav];
					if ((u_in_c->actual_coord.distance(u_in2->actual_coord) < distmin) && (u_in_map.second.t == UAV_T)) {
						distmin = u_in_c->actual_coord.distance(u_in2->actual_coord);
						uav_father = u_in2;
						itUmin = u_out;
					}
				}
			}

			u_out++;
		}

		if (logSF) {cout << "CommunicationManager::update 2_2" << endl; fflush(stdout);}

		if (itUmin != tmpUAVList.end()) {
			UAV *minUAV = *itUmin;

			node_t nu_child; // = {UAV_T, actUAV->id, -1, false};
			node_t nu_father; // = {BS_T, -1, -1, true};

			nu_child.t = UAV_T;
			nu_child.uav = minUAV->id;

			nu_father.t = UAV_T;
			nu_father.uav = uav_father->id;

			connGraph.push_back(std::make_pair(nu_child, nu_father));
			minUAV->father = uav_father;
			uav_father->childUAV.push_back(minUAV);


			tmpUAVList.erase(itUmin);
		}
		else {
			//std::cerr << "Error in CommunicationManager::update" << std::endl;

			//for (auto& u : uavList) {
			//	cout << "UAV " << u.first
			//			<< " - Time: " << tk
			//			<< " " << u.second->actual_coord << " dist: " << u.second->actual_coord.length()
			//			<< " - min: " << commRange_u2bs << endl;
			//}
			//exit (EXIT_FAILURE);
		}

		if (logSF) {cout << "CommunicationManager::update 2_3" << endl; fflush(stdout);}
	}

	if (logSF) {cout << "CommunicationManager::update 3" << endl; fflush(stdout);}

	for (auto& p : poiList) {
		double distmin = std::numeric_limits<double>::max();
		UAV *uav_father = nullptr;
		for (auto& u : uavList) {
			if (u.second->actual_coord.distance(p.second->actual_coord) < distmin) {
				distmin = u.second->actual_coord.distance(p.second->actual_coord);
				uav_father = u.second;
			}
		}
		if (uav_father != nullptr) {
			node_t nu_child; // = {UAV_T, actUAV->id, -1, false};
			node_t nu_father; // = {BS_T, -1, -1, true};

			nu_child.t = POI_T;
			nu_child.poi = p.first;

			nu_father.t = UAV_T;
			nu_father.uav = uav_father->id;

			connGraph.push_back(std::make_pair(nu_child, nu_father));
			uav_father->childPoI.push_back(p.second);
			p.second->father = uav_father;
		}
	}

	if (logSF) {cout << "CommunicationManager::update 4" << endl; fflush(stdout);}

	if (tk == 0) {
		cerr << "Communication tree: " << endl;
		for (auto& el : connGraph) {
			cerr <<
					"<" <<
					((el.first.t == UAV_T) ? "UAV" : ((el.first.t == POI_T) ? "PoI" : "BS")) <<
					((el.first.t == UAV_T) ? el.first.uav : ((el.first.t == POI_T) ? el.first.poi : 0)) <<
					((el.first.t == UAV_T) ? uavList[el.first.uav]->actual_coord : ((el.first.t == POI_T) ? poiList[el.first.poi]->actual_coord : MyCoord::ZERO)) <<
					" > " <<
					"<" <<
					((el.second.t == UAV_T) ? "UAV" : ((el.second.t == POI_T) ? "PoI" : "BS")) <<
					((el.second.t == UAV_T) ? el.second.uav : ((el.second.t == POI_T) ? el.second.poi : 0)) <<
					((el.second.t == UAV_T) ? uavList[el.second.uav]->actual_coord : ((el.first.t == POI_T) ? poiList[el.second.poi]->actual_coord : MyCoord::ZERO)) <<
					" > " <<
					endl;
		}
		cerr << endl;
	}
	//exit (EXIT_SUCCESS);


	updateLt();

	if (logSF) {cout << "CommunicationManager::update END" << endl; fflush(stdout);}
}

void CommunicationManager::updateLt(void) {
	// init all to 0
	for (auto& u : uavLtMap) {
		u.second = 0;
	}

	for (auto& p : poiList) {
		UAV *f = p.second->father;

		bool reachBS = false;
		while (f != nullptr) {
			if (f->id == BS_ID) {
				reachBS = true;
				break;
			}
			f = f->father;
		}
		if (reachBS) {
			f = p.second->father;
			if (f->id == BS_ID) {
				break;
			}
			else {
				uavLtMap[f->id] += p.second->packetPerSecond;
			}
			f = f->father;
		}
	}
}

void CommunicationManager::sendPacketFromPoI(Packet *p, int tk) {

	double distmin = std::numeric_limits<double>::max();
	UAV *destUAV = nullptr;

	for (auto& u : uavList) {
		double dist = u.second->actual_coord.distance(poiList[p->sourcePoI]->actual_coord);

		//if (dist <= commRange_p2u) {
			if (dist < distmin) {
				distmin = dist;
				destUAV = u.second;
			}
		//}
	}

	if (destUAV != nullptr) {
		destUAV->rcvPacketFromPoI(p, tk);
	}
	else {
		// packet not generated because no one is covering
	}
}


int CommunicationManager::get_tx_lt(UAV *u) {
	return uavLtMap[u->id];
}

bool CommunicationManager::isDirect (int idu) {
	for (auto& e : connGraph) {
		if (e.first.uav == idu) {
			return (e.second.t == BS_T);
		}
	}
	return false;
}

double CommunicationManager::getRSSIhistory(int sub_frame, int sub_channel) {
	return (rand() % 10);
}

void CommunicationManager::sendDataRB(UAV *u, Packet *p, int timek, int channel) {

	pktToSend_t newp;
	newp.u = u;
	newp.p = p;
	newp.tk = timek;
	newp.sub_ch = channel;

	if (packetToSend.count(channel) == 0) {
		cerr << "errore init packetToSend" << endl;
		exit (EXIT_FAILURE);
	}
	packetToSend[channel].push_back(newp);
}

void CommunicationManager::manageTransmissionsTimeSlot(int timek) {
	if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot BEGIN" << endl; fflush(stdout);}

	for (auto& el : packetToSend) {
		int channel = el.first;
		std::map<int, double> uav_interference_Map;

		if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot 1" << endl; fflush(stdout);}

		//calculate the interference RSSI for each UAV
		for (auto& u_rcv : uavList){
			double sumInterf = 0;
			for (auto& p : el.second) {
				if (p.u->id != u_rcv.second->id) {
					sumInterf += calcReceivedPower(p.u->actual_coord.distance(u_rcv.second->actual_coord));
				}
			}
			uav_interference_Map[u_rcv.second->id] = 0;
		}

		if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot 2" << endl; fflush(stdout);}

		if (el.second.size() > 0) {
			countMultipleTx += 1;
			sunMultipleTx += el.second.size();
		}

		for (auto& p : el.second) {
			Packet *pkt = p.p;
			UAV *u_src = p.u;
			//int sc = p.sub_ch;
			//double rssi = 0;

			UAV *u_dst = nullptr;
			for (auto& e : connGraph) {
				if ((e.first.uav == u_src->id) && (e.second.t == UAV_T)) {
					u_dst = uavList[e.second.uav];
					break;
				}
			}
			if (u_dst != nullptr) {
				bool okTx = checkTxSD(p, el.second, u_dst, uav_interference_Map);
				//u_src->updateRssi(timek, channel, rssi);

				if (okTx) {
					u_dst->rcvPacketFromUAV(pkt, timek);
				}
				else {
					// drop packet not arrived
					packetDropped(pkt);
					delete (pkt);
				}
			}
			else {
				// drop, no connection
				packetDropped(pkt);
				delete (pkt);
			}

		}

		if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot 3" << endl; fflush(stdout);}

		for (auto& um : uavList){
			um.second->updateRssi(timek, channel, uav_interference_Map[um.second->id]);
		}

		if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot 4" << endl; fflush(stdout);}
	}

	if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot ALMOST" << endl; fflush(stdout);}

	//clear
	for (auto& l : packetToSend) {
		l.second.clear();
	}
	//packetToSend.clear();

	if (logSF) {cout << "CommunicationManager::manageTransmissionsTimeSlot END" << endl; fflush(stdout);}
}

double CommunicationManager::linear2dBm(double x) {
	return (10.0 * log10(x * 1000.0));
}


double CommunicationManager::getUAVChannelLoss (double freq, double tx_height, double rx_height, double dist) {
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

double CommunicationManager::calcReceivedPower (double distance) {
	//cout << "test 1" << endl; fflush(stdout);

	//UAV *u1 = pkt.u;//new UAV(MyCoord::ZERO, poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u2 = u_dst;//new UAV(rcv, poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u2 = new UAV(MyCoord(0,50), poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u_src = pkt.u;

	//double UAV_TX_pt = 0.2512; //%%%% 0.2512 Watt = 24 dBm; % All UAVs transmit with same Power (24 dBm)
	double UAV_TX_pt_db = 24;
	//double GAIN_ag = pow(10.0, 0.6);  //% Transmiter Antenna Gain (6 dB)
	double GAIN_ag_db = 6;
	double freq = 3410;
	double tx_height = 30;
	double rx_height = 30;
	//double distance = u_src->actual_coord.distance(u_dst->actual_coord);
	double loss_dB = getUAVChannelLoss(freq,tx_height,rx_height,distance);

	//cout << "Loss at distance: " << distance << ": " << loss_dB << endl;

	double fading_variance = 1.59; // Fading model is Complex Gaussian Random Variable
	double chan_real = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_complex = RandomGenerator::getInstance().getRealNormal(0, 1);
	std::complex<double> chan (chan_real, chan_complex);
	//chan = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1) + 1i*RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_value = std::abs(chan); // fading loss

	//cout << "chan: " << chan << endl;
	//cout << "chan_value: " << chan_value << endl;

	//cout << "UAV_TX_pt: " << UAV_TX_pt << endl;
	//cout << "GAIN_ag: " << GAIN_ag << endl;
	//cout << "pow(10.0, loss_dB/10.0): " << pow(10.0, loss_dB/10.0) << endl;

	double receivedPower_db = UAV_TX_pt_db + GAIN_ag_db + loss_dB;
	double receivedPower = pow(10.0, receivedPower_db / 10.0) / 1000.0;
	//double receivedPower = UAV_TX_pt * GAIN_ag * pow(10.0, loss_dB/10.0) ;//(10.^((loss_dB)/10)); // received power including path loss,shadowing
	receivedPower *= chan_value;

	return receivedPower;
}

//double CommunicationManager::checkTxSD(MyCoord rcv, std::list<PoI *> &poiList, int nu, int movNt, int movLt, int txNt, int txLt) {
bool CommunicationManager::checkTxSD (pktToSend_t pkt, std::list<pktToSend_t> &allPkst, UAV *u_dst, std::map<int, double> &uav_interference_Map) {

	/*
	//cout << "test 1" << endl; fflush(stdout);

	//UAV *u1 = pkt.u;//new UAV(MyCoord::ZERO, poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u2 = u_dst;//new UAV(rcv, poiList, nu, movNt, movLt, txNt, txLt);
	//UAV *u2 = new UAV(MyCoord(0,50), poiList, nu, movNt, movLt, txNt, txLt);
	UAV *u_src = pkt.u;

	//double UAV_TX_pt = 0.2512; //%%%% 0.2512 Watt = 24 dBm; % All UAVs transmit with same Power (24 dBm)
	double UAV_TX_pt_db = 24;
	//double GAIN_ag = pow(10.0, 0.6);  //% Transmiter Antenna Gain (6 dB)
	double GAIN_ag_db = 6;
	double freq = 3410;
	double tx_height = 30;
	double rx_height = 30;
	double distance = u_src->actual_coord.distance(u_dst->actual_coord);
	double loss_dB = getUAVChannelLoss(freq,tx_height,rx_height,distance);

	//cout << "Loss at distance: " << distance << ": " << loss_dB << endl;

	double fading_variance = 1.59; // Fading model is Complex Gaussian Random Variable
	double chan_real = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_complex = RandomGenerator::getInstance().getRealNormal(0, 1);
	std::complex<double> chan (chan_real, chan_complex);
	//chan = sqrt(fading_variance/2) * RandomGenerator::getInstance().getRealNormal(0, 1) + 1i*RandomGenerator::getInstance().getRealNormal(0, 1);
	double chan_value = std::abs(chan); // fading loss

	//cout << "chan: " << chan << endl;
	//cout << "chan_value: " << chan_value << endl;

	//cout << "UAV_TX_pt: " << UAV_TX_pt << endl;
	//cout << "GAIN_ag: " << GAIN_ag << endl;
	//cout << "pow(10.0, loss_dB/10.0): " << pow(10.0, loss_dB/10.0) << endl;

	double receivedPower_db = UAV_TX_pt_db + GAIN_ag_db + loss_dB;
	double receivedPower = pow(10.0, receivedPower_db / 10.0) / 1000.0;
	//double receivedPower = UAV_TX_pt * GAIN_ag * pow(10.0, loss_dB/10.0) ;//(10.^((loss_dB)/10)); // received power including path loss,shadowing
	receivedPower *= chan_value;
	*/

	UAV *u_src = pkt.u;
	double distance = u_src->actual_coord.distance(u_dst->actual_coord);
	double receivedPower = calcReceivedPower(distance);

	//cout << "receivedPower_db: " << receivedPower_db << endl;
	//cout << "receivedPower: " << receivedPower << endl;

	double interference = uav_interference_Map[u_src->id];

	// Calculate Noise Parameters
	double temperature = 290; // Kelvin
	double k = 1.3806488 * pow(10.0, -23.0); // Boltzman Constant
	double bw = 9*1e6; // Efective Bandwidth of channel (9 MHz)
	double ue_noise_figure = 7 ; // 7 dB noise figure is considered
	double noise = linear2dBm(k * temperature * bw);
	double total_noise_dBm = ue_noise_figure + noise;
	double total_noise = pow(10.0, total_noise_dBm/10.0) / 1000.0;

	//cout << "total_noise_dBm: " << total_noise_dBm << endl;
	//cout << "total_noise: " << total_noise << endl;

	double sinr = receivedPower / (interference + total_noise);
	double sinr_db = linear2dBm(sinr);

	//cout << "sinr: " << sinr << endl;
	//cout << "sinr_db: " << sinr_db << endl;

	double sinr_low = -5;
	double sinr_high = 25;
	double total_samples = sinr_high - sinr_low;

	double prob = 0;
	bool ris = false;
	if (sinr_db <= sinr_low) {
		prob = 0;
		ris = false;
	}
	else if (sinr_db >= sinr_high) {
		prob = 1;
		ris = true;
	}
	else {
		prob = (sinr_db - sinr_low) / total_samples;
		if (RandomGenerator::getInstance().getRealUniform(0, 1) <= prob) {
			ris = true;
		}
		else {
			ris = false;
		}
	}

	//cout << "prob: " << prob << endl;
	return ris;
}

bool CommunicationManager::chekcTxNoInterference (UAV *u_src, UAV *u_dest) {
	double distance = u_src->actual_coord.distance(u_dest->actual_coord);
	double receivedPower = calcReceivedPower(distance);

	//cout << "receivedPower_db: " << receivedPower_db << endl;
	//cout << "receivedPower: " << receivedPower << endl;

	double interference = 0;

	// Calculate Noise Parameters
	double temperature = 290; // Kelvin
	double k = 1.3806488 * pow(10.0, -23.0); // Boltzman Constant
	double bw = 9*1e6; // Efective Bandwidth of channel (9 MHz)
	double ue_noise_figure = 7 ; // 7 dB noise figure is considered
	double noise = linear2dBm(k * temperature * bw);
	double total_noise_dBm = ue_noise_figure + noise;
	double total_noise = pow(10.0, total_noise_dBm/10.0) / 1000.0;

	//cout << "total_noise_dBm: " << total_noise_dBm << endl;
	//cout << "total_noise: " << total_noise << endl;

	double sinr = receivedPower / (interference + total_noise);
	double sinr_db = linear2dBm(sinr);

	//cout << "sinr: " << sinr << endl;
	//cout << "sinr_db: " << sinr_db << endl;

	double sinr_low = -5;
	double sinr_high = 25;
	double total_samples = sinr_high - sinr_low;

	double prob = 0;
	bool ris = false;
	if (sinr_db <= sinr_low) {
		prob = 0;
		ris = false;
	}
	else if (sinr_db >= sinr_high) {
		prob = 1;
		ris = true;
	}
	else {
		prob = (sinr_db - sinr_low) / total_samples;
		if (RandomGenerator::getInstance().getRealUniform(0, 1) <= prob) {
			ris = true;
		}
		else {
			ris = false;
		}
	}

	//cout << "prob: " << prob << endl;
	return ris;
}

void CommunicationManager::packetDeliveretToBS(Packet *p) {
	sumPacketDelivered++;
}

void CommunicationManager::packetDropped(Packet *p) {
	sumPacketDropped++;
}

void CommunicationManager::packetDroppedQueue(Packet *p) {
	sumPacketDroppedQueue++;
}

