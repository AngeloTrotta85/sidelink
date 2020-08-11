/*
 * PoI.h
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#ifndef POI_H_
#define POI_H_

#include "MyCoord.h"

class PoI {
public:
	PoI(MyCoord posCoord);
	PoI(MyCoord posCoord, int id_new);

public:
	static void generateRandomPoIs(std::list<PoI *> &pl, int ss, int np);

public:
	MyCoord actual_coord;

	int id;
	static int idPoIGen;
};

#endif /* POI_H_ */
