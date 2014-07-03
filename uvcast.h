#ifndef UVCAST_H_
#define UVCAST_H_

#include "gissumo.h"
#include <list>
#include <cmath>

using namespace std;

#define PI 3.14159265

// from gissumo.cpp
extern bool m_debug;


list<float> UVCAST_computeAngles(Vehicle src, Vehicle self, list<Vehicle> neighbors);
bool UVCAST_determineSCFtask(list<float> angles);




#endif /* UVCAST_H_ */
