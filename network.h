#ifndef NETWORK_H_
#define NETWORK_H_

#include <map>
#include "gissumo.h"
#include "gis.h"
#include "uvcast.h"

// Called from main, handles the transmission of packets.
void processNetwork(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList);

// Vehicle veh sends its message to all neighbors.
void rebroadcastPacket(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &veh);

// Simulates an accident on Vehicle accidentSource, gets UVCAST going.
void simulateAccident(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &accidentSource);

// An initial broadcast is recursive, and will call itself for all vehicles that are part of a cluster.
void initialBroadcast(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &selfVeh, Vehicle &srcVeh);

#endif /* NETWORK_H_ */
