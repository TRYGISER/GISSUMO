#ifndef GIS_H_
#define GIS_H_

#include "gissumo.h"

// Returns geographic coordinates of a point given its GID.
void GIS_getPointCoords(pqxx::connection &c, unsigned short gid, float &xgeo, float &ygeo);

// Returns the GIDs of all points in a given range of a given point.
vector<unsigned short> GIS_getPointsInRange(pqxx::connection &c, float xcenter, float ycenter, unsigned short range);

// Returns the distance from a set of coordinates to a given point by GID.
unsigned short GIS_distanceToPointGID(pqxx::connection &c, float xx, float yy, unsigned short targetgid);

// Returns false if the path between (x1,y1) and (x2,y2) is obstructed, true otherwise.
bool GIS_isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

// Returns true if the point at (xx,yy) is intersecting with something.
bool GIS_isPointObstructed(pqxx::connection &c, float xx, float yy);

// Adds a new point to the database. Returns the unique identifier 'gid'. Sets 'feattype' to 2222.
unsigned short GIS_addPoint(pqxx::connection &c, float xx, float yy, unsigned short id);

// Updates the coordinates of a point via GID.
void GIS_updatePoint(pqxx::connection &c, float xx, float yy, unsigned short GID);

// Removes all POINTs from the database (feattyp 2222).
void GIS_clearAllPoints(pqxx::connection &c);

#endif /* GIS_H_ */
