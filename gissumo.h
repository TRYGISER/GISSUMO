#ifndef GISSUMO_H_
#define GISSUMO_H_

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

#include <pqxx/pqxx>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;
using namespace boost::property_tree;
using namespace boost::program_options;


/* Definitions
   ----------- */

/* These are the top left origin points for our map, in WGS84 coordinates
 * From pre-analysis of our map we've determined the limits to be:
 * Top Left: 41.16884 -8.622678			(29 cells Y)
 * Bottom Right: 41.160837 -8.609375	(48 cells X)
 *
 * 6 seconds (6 cells) worth of margin were added to the following coordinates
 * (direction top left) to account for radio range.
 */
#define YREFERENCE 41.17056 	// 41°10′14.0″N
#define XREFERENCE -8.62444		// 008°37′28.0″W

// This is the coverage map size, in cells, of an RSU (e.g. '11' means 5 cell radius, RSU at center cell)
#define PARKEDCELLCOVERAGE 11
#define PARKEDCELLRANGE 5

// The size of the city map in cells
// 6 cell margins were added (thus, +12 cells on each bearing)
#define CITYHEIGHT 41	// yy
#define CITYWIDTH 60	// xx

/* At our location: 1" latitude: 30.89m; 1" longitude: 23.25m.
 * Ideally we would be using SRID 27492 which would give us equal axis, but that would require
 * converting the WGS84 coordinates from SUMO, which is nontrivial.
 * We're defining the conversion between meters and degrees as: 1/(3600*30.89)
 */
#define METERSTODEGREES 0.0000089925

// Max range of an RSU, in meters. 5 cells: 154.45m (5/0.0000089925*3600)
#define MAXRANGE 155

/* Functions
   --------- */

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

// Given a WGS84 pair of coordinates, return an integer cell position.
void determineCellFromWGS84 (float xgeo, float ygeo, unsigned short &xcell, unsigned short &ycell);

// Returns the distance in seconds between two coordinates on the same bearing (two latitudes or two longitudes).
unsigned int deltaSeconds(float c1, float c2);

// Prints a char CityMap to the terminal.
class CityMapChar; class CityMapNum;
void printCityMap (CityMapChar cmap);
void printCityMap (CityMapNum cmap);

// Returns the signal quality on a 1-5 scale based on distance and Line of Sight.
unsigned short getSignalQuality(unsigned short distance, bool lineOfSight);

// Applies the coverage map of an RSU to a global city map.
class RSU; class CityMapNum;
void applyCoverageToCityMap(RSU rsu, CityMapNum &city);

// Prints ASCII of a local coverage map.
void printLocalCoverage(array< array<unsigned short,PARKEDCELLCOVERAGE>,PARKEDCELLCOVERAGE > coverage);

// Prints all details of a vehicle.
struct Vehicle;
void printVehicleDetails(Vehicle veh);

// Adds an RSU to the database and GIS.
void addNewRSU(pqxx::connection &conn, std::vector<RSU> &rsuList, unsigned short id, float xgeo, float ygeo, bool active);

// Returns a list of nearby vehicles (not RSUs) that we can communicate with.
vector<Vehicle> getVehiclesInRange(pqxx::connection &conn, vector<Vehicle> vehiclesOnGIS, Vehicle src);

/* Classes and Structs
   ------------------- */

// A char map to keep data for 2D visualization (terminal output)
class CityMapChar {
public:
	array< array<char,CITYHEIGHT>,CITYWIDTH > map;	// city coverage map, (0,0) on Top Left

	CityMapChar() { for(int i=0; i<CITYWIDTH; i++) map[i].fill(' '); }
	CityMapChar(char fill) { for(int i=0; i<CITYWIDTH; i++) map[i].fill(fill); }
};

// A numeric map for keeping data in memory, such as coverage levels
class CityMapNum {
public:
	array< array<int,CITYHEIGHT>,CITYWIDTH > map;	// city coverage map, (0,0) on Top Left

	CityMapNum() { for(int i=0; i<CITYWIDTH; i++) map[i].fill(0); }
	CityMapNum(int fill) { for(int i=0; i<CITYWIDTH; i++) map[i].fill(fill); }
};


class RSU {
public:
	unsigned short id;		// numeric identifier
	unsigned short gid;		// GIS numeric identifier
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
		id=0; gid=0;
		active = false;
		xcell=0; ycell=0;
		xgeo=0; ygeo=0;
		for(int i=0; i<PARKEDCELLCOVERAGE; i++) coverage[i].fill(0);
	}
};

struct Packet
{
	unsigned short m_src;
	unsigned short m_id;
	float m_timestamp;
};

struct Vehicle
{
	unsigned short id=0;	// SUMO identifier
	unsigned short gid=0;	// GIS identifier
	bool parked=false;		// Parked status

	unsigned short xcell=0;	// x,y position in a cell map
	unsigned short ycell=0;
	float xgeo=0;				// x,y geographic position
	float ygeo=0;

	float speed = 0;
	vector<Packet> p_buffer;	// packet storage
};

struct Timestep
{
	float time = 0;
	std::vector<Vehicle> vehiclelist;
};








#endif /* GISSUMO_H_ */
