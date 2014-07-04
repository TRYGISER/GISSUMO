#ifndef NETWORK_H_
#define NETWORK_H_

#include "gissumo.h"

void processNetwork(pqxx::connection &c, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList);

#endif /* NETWORK_H_ */
