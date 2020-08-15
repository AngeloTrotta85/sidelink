/*
 * CommunicationManager.h
 *
 *  Created on: Aug 14, 2020
 *      Author: angelo
 */

#ifndef COMMUNICATIONMANAGER_H_
#define COMMUNICATIONMANAGER_H_

#include <list>
#include <map>

#include "Packet.h"
#include "UAV.h"
#include "PoI.h"


class CommunicationManager {
	public:
	static CommunicationManager& getInstance(void) {
		static CommunicationManager    instance; 	// Guaranteed to be destroyed.

		// Instantiated on first use.
		return instance;
	}
private:
	CommunicationManager(void){
		commRange_u2u = 10;
		commRange_p2u = 10;
		commRange_u2bs = 10;

		specialUAV_BS = new UAV(MyCoord::ZERO);

		logSF = false;

	};         // Constructor? (the {} brackets) are needed here.

	// C++ 11
	// =======
	// We can use the better technique of deleting the methods
	// we don't want.
public:
	CommunicationManager(CommunicationManager const&)	= delete;
	void operator=(CommunicationManager const&)  = delete;

	// Note: Scott Meyers mentions in his Effective Modern
	//       C++ book, that deleted functions should generally
	//       be public as it results in better error messages
	//       due to the compilers behavior to check accessibility
	//       before deleted status


public:
	typedef enum typ {
		NONE_T,
		UAV_T,
		POI_T,
		BS_T
	} typ_t;
	typedef struct node {
		typ_t t = NONE_T;
		int uav = -1;
		int poi = -1;
	} node_t;

public:
	void init(std::list<UAV *> &ul, std::list<PoI *> &pl, double cuu, double cpu, double cub);
	void sendPacketFromPoI(Packet *p, int tk);

	void updateLt(void);

	void update(int tk);

	int get_tx_lt(UAV *u);

	bool isDirect (int idu);

	double getRSSIhistory(int sub_frame, int sub_channel);

public:
	UAV *specialUAV_BS;
	std::map<int, UAV *> uavList;
	std::map<int, int> uavLtMap;

	std::map<int, PoI *> poiList;

	std::list<std::pair <node_t, node_t> > connGraph;

	double commRange_u2u;
	double commRange_p2u;
	double commRange_u2bs;

	bool logSF;
};

#endif /* COMMUNICATIONMANAGER_H_ */
