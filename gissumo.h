#ifndef GISSUMO_H_
#define GISSUMO_H_

#include <iostream>
#include <string>
#include <vector>
#include <pqxx/pqxx>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

bool isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

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
