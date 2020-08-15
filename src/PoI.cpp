/*
 * PoI.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "RandomGenerator.h"
#include "CommunicationManager.h"
#include "Packet.h"
#include "PoI.h"

int PoI::idPoIGen = 0;

PoI::PoI(MyCoord posCoord) {
	actual_coord = posCoord;
	id = idPoIGen++;

	nPacket2Generate = 10;
	generationIntervalSlots = 100;
	next_packet_generation_tk = 0;
}

PoI::PoI(MyCoord posCoord, int id_new) {
	actual_coord = posCoord;
	id = id_new;

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
		double intrange = ((double) ss) / 1.0;
		PoI *newP = new PoI(
				MyCoord(RandomGenerator::getInstance().getRealUniform(-intrange, intrange),
						RandomGenerator::getInstance().getRealUniform(-intrange, intrange))
		);
		pl.push_back(newP);
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




