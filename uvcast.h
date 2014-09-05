#ifndef UVCAST_H_
#define UVCAST_H_

#include "gissumo.h"
#define PI 3.14159265
extern bool gm_debug;

// Returns the list of angles to each neighbor as required by the gift-wrapping algorithm.
vector<float> UVCAST_computeAngles(RoadObject* src, RoadObject* self, vector<Vehicle*> neighbors);

// Given a list of angles, returns the result of the gift-wrapping algorithm.
bool UVCAST_determineSCFtask(vector<float> angles);

#endif /* UVCAST_H_ */
