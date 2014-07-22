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

// Map center coordinates
#define YCENTER 41.163535
#define XCENTER -8.617485

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

// Emergency message code
#define EMERGENCYID 31337

// GIS feattyp codes
#define CAR_FEATTYP 2222	// Vehicle
#define RSU_FEATTYP 2223	// RSU
#define BLD_FEATTYP 9790	// Building

/* Functions
   --------- */

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

// Prints the list<Vehicle> of vehicles.
void printListOfVehicles(list<Vehicle> &vehiclesOnGIS);


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


/* Packet. Network layer.
 */
struct Packet
{
	unsigned short packetSrc=0;
	unsigned short packetID=0;
	float packetTime=0;
};


/* Elementary road object with a radio, a physical presence, and coordinates.
 */
class RoadObject {
public:
	enum RoadObjectType {VEHICLE, RSU};

	// Characteristics and identifiers
	RoadObjectType type;	// the type of this entity
	unsigned short id;		// numeric identifier
	unsigned int gid;		// GIS numeric identifier
	bool active;			// active status

	// Location and cells
	unsigned short xcell;	// x,y position in a cell map
	unsigned short ycell;
	float xgeo;				// x,y geographic position
	float ygeo;

	// Network layer
	Packet packet;		// store a single packet for now
};


/* Vehicle. Can move, park, and be selected by UVCAST.
 */
class Vehicle : public RoadObject {
public:
	bool parked;	// Parking status
	bool scf;		// Store-carry-forward task
	float speed;	// Vehicle speed
};


/* RSU. Features a coverage map.
 */
class RSU : public RoadObject {
public:
	// Coverage map, RSU is at the center cell
	array< array<unsigned short,PARKEDCELLCOVERAGE>,PARKEDCELLCOVERAGE > coverage;

	// Initialize the coverage map on creation
	RSU() { for(int i=0; i<PARKEDCELLCOVERAGE; i++) coverage[i].fill(0); }
};


/* A timestep with a list of vehicles, for reading XML FCD data into.
 */
struct Timestep
{
	float time = 0;
	std::vector<Vehicle> vehiclelist;
};








#endif /* GISSUMO_H_ */
