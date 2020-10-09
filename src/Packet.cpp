/*
 * Packet.cpp
 *
 *  Created on: Aug 13, 2020
 *      Author: angelo
 */

#include "Packet.h"

int Packet::idPKTGen = 0;

Packet::Packet(int idPoI, int timeg) {
	id = idPKTGen++;

	sourcePoI = idPoI;
	genTime = timeg;
}
