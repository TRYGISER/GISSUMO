#ifndef GISSUMO_H_
#define GISSUMO_H_

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <pqxx/pqxx>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace std;
using namespace boost;
using namespace boost::property_tree;



/* Definitions
   ----------- */

/* These are the top left origin points for our map, in WGS84 coordinates
 * From pre-analysis of our map we've determined the limits to be:
 * Top Left: 41.16884 -8.622678
 * Bottom Right: 41.160837 -8.609375
 */
// TODO: add margins for RSU coverage, 5-6 seconds ?
#define XREFERENCE 41.16884
#define YREFERENCE -8.622678

// This is the coverage map size, in cells, of an RSU (e.g. '11' means 5 cell radius, RSU at center cell)
#define PARKEDCELLCOVERAGE 11

// The size of the city map in cells
// TODO: add margins for RSU coverage, 5-6 seconds ?
#define CITYWIDTH 47 // xx
#define CITYHEIGHT 28 // yy



/* Functions
   --------- */

// Returns true if the path between (x1,y1) and (x2,y2) is obstructed, false otherwise.
bool isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

// Returns true if the point at (xx,yy) is intersecting with something
bool isPointObstructed(pqxx::connection &c, float xx, float yy);

// Given a WGS84 pair of coordinates, return an integer cell position
void determineCellFromWGS84 (float xgeo, float ygeo, unsigned short &xcell, unsigned short &ycell);

// Returns the distance in seconds between two coordinates on the same bearing (two latitudes or two longitudes)
unsigned int deltaSeconds(float c1, float c2);



/* Classes and Structs
   ------------------- */

class CityMap {
public:
	array< array<unsigned short,CITYHEIGHT>,CITYWIDTH > map;	// city coverage map, (0,0) on Top Left

	CityMap() { for(int i=0; i<CITYWIDTH; i++) map[i].fill(0); }
};

class RSU {
public:
	unsigned short id;		// numeric identifier
	bool active;			// active status
	unsigned short xcell;	// x,y position in a cell map
	unsigned short ycell;
	float xgeo;				// x,y geographic position
	float ygeo;
	array< array<unsigned short,PARKEDCELLCOVERAGE>,PARKEDCELLCOVERAGE > coverage;	// coverage map, vehicle is at center cell

	// this initialization takes ID and geographic coordinates, and completes the cell coordinates automatically
	RSU(unsigned short iid, float xx, float yy)
	{
		id=iid;
		active = false;
		xcell=0; ycell=0;
		xgeo=xx; ygeo=yy;
		determineCellFromWGS84(xgeo,ygeo,xcell,ycell);
		for(int i=0; i<PARKEDCELLCOVERAGE; i++) coverage[i].fill(0);
	}

	RSU()
	{
		id=0;
		active = false;
		xcell=0; ycell=0;
		xgeo=0; ygeo=0;
		for(int i=0; i<PARKEDCELLCOVERAGE; i++) coverage[i].fill(0);
	}
};


struct Vehicle
{
	unsigned short id = 0;
	double x = 0;
	double y = 0;
	double speed = 0;
};

struct Timestep
{
	float time = 0;
	std::vector<Vehicle> vehiclelist;
};


#endif /* GISSUMO_H_ */
