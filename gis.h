#ifndef GIS_H_
#define GIS_H_

#include "gissumo.h"
extern bool m_debug;

// Returns geographic coordinates of a point given its GID.
void GIS_getPointCoords(pqxx::connection &c, unsigned int gid, float &xgeo, float &ygeo);

// Returns the GIDs of all points in a given range of a given point.
vector<unsigned int> GIS_getPointsInRange(pqxx::connection &c, float xcenter, float ycenter, unsigned short range);

// Returns the distance from a set of coordinates to a given point by GID.
unsigned short GIS_distanceToPointGID(pqxx::connection &c, float xx, float yy, unsigned int targetgid);

// Returns false if the path between (x1,y1) and (x2,y2) is obstructed, true otherwise.
bool GIS_isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

// Returns true if the point at (xx,yy) is intersecting with something.
bool GIS_isPointObstructed(pqxx::connection &c, float xx, float yy);

// Adds a new point to the database. Returns the unique identifier 'gid'. Sets 'feattype' to 2222.
unsigned int GIS_addPoint(pqxx::connection &c, float xx, float yy, unsigned short id);

// Updates the coordinates of a point via GID.
void GIS_updatePoint(pqxx::connection &c, float xx, float yy, unsigned int gid);

// Removes all POINTs from the database (feattyp 2222).
void GIS_clearAllPoints(pqxx::connection &c);

// Adds an RSU to the database and GIS.
void addNewRSU(pqxx::connection &conn, std::list<RSU> &rsuList, unsigned short id, float xgeo, float ygeo, bool active);

// Returns a list of pointers to vehicles (not RSUs) that we can communicate with.
vector<Vehicle*> getVehiclesInRange(pqxx::connection &conn, list<Vehicle> &vehiclesOnGIS, const RoadObject src);

// Returns a list of pointers to vehicles in a range [range] of [xgeo,ygeo].
vector<Vehicle*> getVehiclesNearPoint(pqxx::connection &conn, list<Vehicle> &vehiclesOnGIS, const float xgeo, const float ygeo, const unsigned short range);

// Returns a list of pointers to RSUs that we can communicate with.
vector<RSU*> getRSUsInRange(pqxx::connection &conn, list<RSU> &rsuList, const RoadObject src);

#endif /* GIS_H_ */
