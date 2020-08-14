/*
 * Packet.cpp
 *
 *  Created on: Aug 13, 2020
 *      Author: angelo
 */

#include "Packet.h"

int Packet::idPKTGen = 0;

Packet::Packet(int idPoI) {
	id = idPKTGen++;

	sourcePoI = idPoI;
}
