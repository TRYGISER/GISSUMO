#ifndef GISSUMO_H_
#define GISSUMO_H_

#include <iostream>
#include <string>
#include <vector>
#include <pqxx/pqxx>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// Returns true if the path between (x1,y1) and (x2,y2) is obstructed, false otherwise.
bool isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

// Returns true if the point at (xx,yy) is intersecting with something
bool isPointObstructed(pqxx::connection &c, float xx, float yy);

struct Vehicle
{
	unsigned short id;
	double x;
	double y;
	double speed;
};

struct Timestep
{
	float time;
	std::vector<Vehicle> vehiclelist;
};


#endif /* GISSUMO_H_ */
