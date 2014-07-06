#ifndef NETWORK_H_
#define NETWORK_H_

#include "gissumo.h"
#include "gis.h"

// Called from main, handles the transmission of packets.
void processNetwork(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList);

// Vehicle veh sends its message to all neighbors.
void rebroadcastPacket(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &veh);


#endif /* NETWORK_H_ */
