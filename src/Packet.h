/*
 * Packet.h
 *
 *  Created on: Aug 13, 2020
 *      Author: angelo
 */

#ifndef PACKET_H_
#define PACKET_H_

class Packet {
public:
	Packet(int idPoI, int timeg);
	virtual ~Packet() {};

public:
	int id;
	static int idPKTGen;

	int sourcePoI;
	int genTime;
};

#endif /* PACKET_H_ */
