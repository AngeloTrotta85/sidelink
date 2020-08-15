/*
 * PoI.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "Generic.h"
#include "RandomGenerator.h"
#include "CommunicationManager.h"
#include "Packet.h"
#include "PoI.h"

int PoI::idPoIGen = 0;

PoI::PoI(MyCoord posCoord) {
	actual_coord = posCoord;
	id = idPoIGen++;

	father = nullptr;
	packetPerSecond = 100;
	nPacket2Generate = 10;
	generationIntervalSlots = 100;
	next_packet_generation_tk = 0;
}

PoI::PoI(MyCoord posCoord, int id_new) {
	actual_coord = posCoord;
	id = id_new;

	father = nullptr;
	packetPerSecond = 100;
	nPacket2Generate = 10;
	generationIntervalSlots = 100;
	next_packet_generation_tk = 0;
}


void PoI::generateRandomPoIs(std::list<PoI *> &pl, int ss, int np) {
	//for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
	for (int i = 0; i < np; i++) {
		//UAV *newU = new UAV(
		//		MyCoord(RandomGenerator::getInstance().getRealUniform(0, ss), RandomGenerator::getInstance().getRealUniform(0, ss))
		//);

		int rndtry = 100;
		double distance = 0;
		MyCoord poipos = MyCoord::ZERO;
		while ((rndtry > 0) && (poipos.length() < Generic::getInstance().commRange)) {
			double intrange = ((double) ss) / 1.0;
			poipos.x = RandomGenerator::getInstance().getRealUniform(-intrange, intrange);
			poipos.y = RandomGenerator::getInstance().getRealUniform(-intrange, intrange);
			rndtry--;
		}
		if (rndtry > 0) {
			PoI *newP = new PoI(poipos);
			pl.push_back(newP);
		}
		else {
			std::cerr << "Error generating PoIs" << std::endl;
			exit(EXIT_FAILURE);
		}
		//std::cout << "UAV: " << i << " --> " << newU->recharge_coord << " - Energy:" << newU->max_energy << std::endl;
	}
}


void PoI::init(int npkt, int slots) {
	next_packet_generation_tk = RandomGenerator::getInstance().getIntUniform(0, 1000);

	nPacket2Generate = npkt;
	generationIntervalSlots = slots;

	packetPerSecond = nPacket2Generate * (1000 / generationIntervalSlots);
}

void PoI::generatePackets_check(int tk) {
	if (next_packet_generation_tk <= tk) {

		generatePackets(tk);

		next_packet_generation_tk = tk + generationIntervalSlots;
	}
}

void PoI::generatePackets(int tk) {
	for (int i = 0; i < nPacket2Generate; i++) {
		Packet *newp = new Packet(id);

		CommunicationManager::getInstance().sendPacketFromPoI(newp, tk);
	}

}




