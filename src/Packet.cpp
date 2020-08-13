/*
 * Packet.cpp
 *
 *  Created on: Aug 13, 2020
 *      Author: angelo
 */

#include "Packet.h"

int Packet::idPKTGen = 0;

Packet::Packet() {
	// TODO Auto-generated constructor stub
	id = idPKTGen++;
}

Packet::~Packet() {
	// TODO Auto-generated destructor stub
}

