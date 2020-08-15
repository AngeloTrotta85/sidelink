/*
 * Generic.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: angelo
 */

#include "Generic.h"
#include "PoI.h"

double Generic::getTime2Travel(MyCoord start, MyCoord end) {
	return (start.distance(end) / maxVelocity);
}

void Generic::build_static_positions_task_set(std::list<PoI *> &poisList) {
	if (poisList.size() == 0) {
		return;
	}

	int taskID = 0;
	double comD = Generic::getInstance().commRange;

	for (auto& z : poisList) {
		MyCoord target = z->actual_coord;
		MyCoord actPoint = MyCoord::ZERO;
		MyCoord diff = target - actPoint;
		double r, t;
		MyCoord::cartesian2polar(diff.x, diff.y, r, t);
		r = comD;
		MyCoord step = MyCoord::ZERO;
		MyCoord::polar2cartesian(r, t, step.x, step.y);

		while (actPoint.distance(target) > comD) {
			actPoint += step;
			posTasks[taskID++] = actPoint;
		}

	}

}

void Generic::build_static_comm_task_set(int nsc, int nsubf_in_supf) {
	int taskID = 0;

	for (int x_nsubf = 0; x_nsubf < nsubf_in_supf; x_nsubf++) {
		for (int y_nsc = 0; y_nsc < nsc; y_nsc++) {
			commTasks[taskID++] = MyCoord(x_nsubf, y_nsc);
		}
	}
}


