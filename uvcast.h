#ifndef UVCAST_H_
#define UVCAST_H_

#include "gissumo.h"
#include <vector>
#include <cmath>

using namespace std;

#define PI 3.14159265

// from gissumo.cpp
extern bool m_debug;


vector<float> UVCAST_computeAngles(Vehicle src, Vehicle self, vector<Vehicle> neighbors);
bool UVCAST_determineSCFtask(vector<float> angles);




#endif /* UVCAST_H_ */
