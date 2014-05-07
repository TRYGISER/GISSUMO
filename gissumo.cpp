#include "gissumo.h"

#define XML_PATH "./fcdoutput.xml"

const ptree& empty_ptree(){
    static ptree t;
    return t;
}

int main(int argc, char *argv[])
{
	/* Process command-line options
	 * Using boost::program_options
	 */

	// Defaults
	bool m_printVehicleMap = false;
	bool m_validVehicle = false;
	bool m_debug = false;

	// List of command line options
	options_description cliOptDesc("Options");
	cliOptDesc.add_options()
		("print-vehicle-map", "prints an ASCII map of vehicle positions")
		("check-valid-vehicles", "counts number of vehicles in the clear")
	    ("debug", "enable debug mode (very verbose)")
	    ("help", "give this help list")
	;

	// Parse options
	variables_map varMap;
	store(parse_command_line(argc, argv, cliOptDesc), varMap);
	notify(varMap);

	// Process options
	if (varMap.count("debug")) 					m_debug=true;
	if (varMap.count("print-vehicle-map")) 		m_printVehicleMap=true;
	if (varMap.count("check-valid-vehicles"))	m_validVehicle=true;
	if (varMap.count("help")) 					{ cout << cliOptDesc; return 1; }


	/* Parse SUMO logs
	 * This expects SUMO's floating car data (FCD) output with geographic
	 * coordinates (--fcd-output.geo=true)
	 * 
	 * The resulting file is XML, with syntax as follows:
	 * <fcd-export>
	 *  <timestep time="">
	 *   <vehicle id="" x="" y="" ... />
	 * 
	 * To parse, we're using boost::propertytree, which runs on top of RapidXML.
	 */

	// using boost and vector structs
	ptree tree;
	read_xml(XML_PATH, tree);
	std::vector<Timestep> fcd_output;

	// traverse tree and fill fcd_output with timesteps
	BOOST_FOREACH ( ptree::value_type const& iterTimestep, tree.get_child("fcd-export") )
	{
		if(iterTimestep.first == "timestep")
		{
			Timestep t;

			// get time attribute
			t.time = iterTimestep.second.get<float>("<xmlattr>.time",0);

			// get second level tree 'vehicles'
			BOOST_FOREACH ( ptree::value_type const& iterVehicle, iterTimestep.second )
			{
				if(iterVehicle.first == "vehicle")
				{
					Vehicle v;

					v.id = iterVehicle.second.get<unsigned short>("<xmlattr>.id",0);
					v.xgeo = iterVehicle.second.get<double>("<xmlattr>.x",0);
					v.ygeo = iterVehicle.second.get<double>("<xmlattr>.y",0);
					v.speed = iterVehicle.second.get<double>("<xmlattr>.speed",0);

					t.vehiclelist.push_back(v);
				}
			}

			// store this timestep entry
			fcd_output.push_back(t);
		}
	}

	/* Open a connection to PostgreSQL
	 * A password can be added to this string.
	 */
	pqxx::connection conn("dbname=shapefiledb user=abreis");

	// Clear all POINT entities from the database from past simulations.
	GIS_clearAllPoints(conn);

	/* Simulation starts here.
	 * We have a 0.37% mismatch error between the SUMO roads and the Porto shapefile data.
	 * This causes vehicles to be inside buildings every now and then.
	 * When a vehicle parks and is marked as an RSU, we must check to see if this isn't happening,
	 * otherwise the RSU will always return NLOS to all vehicles.
	 */

	// setup vectors to keep the list of RSUs and their status
	std::vector<RSU> rsuList;			// vector to hold list of RSUs
	CityMapChar vehicleLocations; 		// 2D map for vehicle locations
//	CityMapNum globalCoverage;

	// TEST: add an active RSU
	RSU testRSU;
	testRSU.id=1;
	testRSU.ygeo=41.164798;
	testRSU.xgeo=-8.616050;
	testRSU.active=true;
	// check to see if the RSU is in a valid location
	if(GIS_isPointObstructed(conn,testRSU.xgeo,testRSU.ygeo))
		{ cerr << "ERROR: RSU is inside a building." << endl; return 1; }
	// get cell coordinates from WGS84
	determineCellFromWGS84(testRSU.xgeo,testRSU.ygeo,testRSU.xcell,testRSU.ycell);
	// add RSU to GIS and get GIS unique id (gid)
	testRSU.gid = GIS_addPoint(conn,testRSU.xgeo,testRSU.ygeo,testRSU.id);
	// add RSU to list of RSUs
	rsuList.push_back(testRSU);

//	vector<unsigned short> neighList = GIS_getPointsInRange(conn,testRSU.xgeo,testRSU.ygeo,100);
//	unsigned short distTest = GIS_distanceToPointGID(conn,-8.6160498,41.165799,testRSU.gid);
//	for(vector<unsigned short>::iterator iter=neighList.begin(); iter!=neighList.end(); iter++)
//		cout << "\tNeighbor gid: " << *iter << '\n';

	// Run through every time step
	for(std::vector<Timestep>::iterator
			iterTime = fcd_output.begin();
			iterTime != fcd_output.end();
			iterTime++ )
	{
		/*
		 * Beginning of each time step
		 */
		if(m_debug) cout << "Time step: " << iterTime->time << endl;

		// run through each vehicle
		for(std::vector<Vehicle>::iterator
				iterVeh=iterTime->vehiclelist.begin();
				iterVeh!=iterTime->vehiclelist.end();
				iterVeh++)
		{
			/*
			 * Beginning of each vehicle
			 */
			// check the cell location of this vehicle
			unsigned short xcell=0, ycell=0;
			determineCellFromWGS84 (iterVeh->xgeo, iterVeh->ygeo, xcell, ycell);

			// TODO: update the positions of each vehicle in GIS and create new vehicles as needed


			if(m_printVehicleMap)	vehicleLocations.map[xcell][ycell]='o';	// tag the vehicle citymap

		}	// end for(vehicle)

		/*
		 * End of each time step
		 */

		if(m_printVehicleMap)	// --print-vehicle-map
		{
			for(vector<RSU>::iterator iter=rsuList.begin(); iter!=rsuList.end(); iter++)
				if(iter->active)
					vehicleLocations.map[iter->xcell][iter->ycell]='R';		// overlay RSUs on the map
			printCityMap(vehicleLocations);			// print the vehicle map
			for(short xx=0;xx<CITYWIDTH;xx++)		// clean map
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(vehicleLocations.map[xx][yy]=='o' || vehicleLocations.map[xx][yy]=='R')
						vehicleLocations.map[xx][yy]='.';
		}
	}	// end for(timestep)



	// DEBUG: go through every vehicle position and see if it's not inside a building.
	if(m_validVehicle)
	{
		unsigned int bumpcount=0, clearcount=0;
		for(std::vector<Timestep>::iterator iter1=fcd_output.begin(); iter1 != fcd_output.end(); iter1++)
			for(std::vector<Vehicle>::iterator iter2=iter1->vehiclelist.begin(); iter2!=iter1->vehiclelist.end(); iter2++)
				if( GIS_isPointObstructed(conn,iter2->xgeo,iter2->ygeo) ) bumpcount++; else clearcount++;
		cout << "bump " << bumpcount << " clear " << clearcount << endl;
	}


	// Clear all POINT entities from the database from past simulations.
//	GIS_clearAllPoints(conn);

	return 0;
}


/* * */

vector<unsigned short> GIS_getPointsInRange(pqxx::connection &c, float xcenter, float ycenter, unsigned short range)
{
	float wgs84range = range*METERSTODEGREES;
	
	pqxx::work txn(c);
	pqxx::result r = txn.exec(
		"SELECT gid "
		"FROM edificios "
		"WHERE ST_DWithin(geom,ST_GeomFromText('POINT("
			+ pqxx::to_string(xcenter) + " "
			+ pqxx::to_string(ycenter) + ")',4326)"
			+ "," + pqxx::to_string(wgs84range) + ")"
			+ " and feattyp='2222'"
	);
	txn.commit();
	
	vector<unsigned short> neighbors;

	for(pqxx::result::iterator iter=r.begin(); iter != r.end(); iter++)
		neighbors.push_back(iter[0].as<unsigned short>());

	return neighbors;
}

unsigned short GIS_distanceToPointGID(pqxx::connection &c, float xx, float yy, unsigned short targetgid)
{
	// first get the target point as WKT
	pqxx::work txn1(c);
	pqxx::result r1 = txn1.exec(
			"SELECT ST_AsText(geom) "
			"FROM edificios "
			"WHERE gid=" + pqxx::to_string(targetgid)
		);
	txn1.commit();

	std::string targetWKT(r1[0][0].as<string>());
	cout << targetWKT << endl;


	// now get the distance, convert to meters, and return
	pqxx::work txn2(c);
	pqxx::result r2 = txn2.exec(
		"SELECT ST_Distance('POINT("
			+ pqxx::to_string(xx) + " "
			+ pqxx::to_string(yy) + ")', '"
			+ targetWKT + "')"
	);
	txn2.commit();

	return (unsigned short) (r2[0][0].as<float>()/METERSTODEGREES);
}

bool GIS_isLineOfSight (pqxx::connection &c, float x1, float y1, float x2, float y2)
{
	pqxx::work txn(c);

	pqxx::result r = txn.exec(
		"SELECT COUNT(id) "
		"FROM edificios "
		"WHERE ST_Intersects(geom, ST_GeomFromText('LINESTRING("
			+ pqxx::to_string(x1) + " "
			+ pqxx::to_string(y1) + ","
			+ pqxx::to_string(x2) + " "
			+ pqxx::to_string(y2) + ")',4326))"
	);
	txn.commit();

	if(r[0][0].as<int>() > 0) return 1; else return 0;
}

bool GIS_isPointObstructed(pqxx::connection &c, float xx, float yy)
{
	pqxx::work txn(c);
	pqxx::result r = txn.exec(
		"SELECT COUNT(gid) "
		"FROM edificios "
		"WHERE ST_Intersects(geom, ST_GeomFromText('POINT("
			+ pqxx::to_string(xx) + " "
			+ pqxx::to_string(yy) + ")',4326))"
	);
	txn.commit();

	if(r[0][0].as<int>() > 0) return 1; else return 0;
}

unsigned short GIS_addPoint(pqxx::connection &c, float xx, float yy, unsigned short id)
{
	pqxx::work txnInsert(c);
	pqxx::result r = txnInsert.exec(
			"INSERT INTO edificios(id, geom, feattyp) "
			"VALUES ("
				+ pqxx::to_string(id)
				+ ", ST_GeomFromText('POINT("
				+ pqxx::to_string(xx) + " "
				+ pqxx::to_string(yy) + ")',4326), 2222) RETURNING gid"
		);
	txnInsert.commit();

	return r[0][0].as<int>();
}

void GIS_updatePoint(pqxx::connection &c, float xx, float yy, unsigned short id)
{
	pqxx::work txnUpdate(c);
	txnUpdate.exec(
			"UPDATE edificios SET geom=ST_GeomFromText('POINT("
			+ pqxx::to_string(xx) + " "
			+ pqxx::to_string(yy) + ")',4326) WHERE id="
			+ to_string(id)
		);
	txnUpdate.commit();
}


void GIS_clearAllPoints(pqxx::connection &c)
{
	pqxx::work txn(c);
	txn.exec( "DELETE FROM edificios WHERE feattyp='2222'");
	txn.commit();
}

void determineCellFromWGS84 (float xgeo, float ygeo, unsigned short &xcell, unsigned short &ycell)
{
	xcell=deltaSeconds(xgeo,XREFERENCE);
	ycell=deltaSeconds(ygeo,YREFERENCE);
}

unsigned int deltaSeconds(float c1, float c2)
{
	return (unsigned int) floor(fabs(c1-c2)*3600);
}

void printCityMap (CityMapChar cmap)
{
	for(short yy=0;yy<CITYHEIGHT;yy++)
	{
		for(short xx=0;xx<CITYWIDTH;xx++)
			cout << cmap.map[xx][yy] << ' ';
		cout << '\n';
	}
}


/* Code examples and debug
 */

//	// DEBUG, print all timesteps and vehicles
//	for(std::vector<Timestep>::iterator iter1=fcd_output.begin(); iter1 != fcd_output.end(); iter1++)
//	{
//		cout << "time " << iter1->time << '\n';
//
//		for(std::vector<Vehicle>::iterator iter2=iter1->vehiclelist.begin(); iter2!=iter1->vehiclelist.end(); iter2++)
//			cout << std::setprecision(8)
//					<< "\tvehicle"
//					<< " id " << iter2->id
//					<< " x " << iter2->x
//					<< " y " << iter2->y
//					<< " speed " << iter2->speed
//					<< '\n';
//	}


//	// DEBUG: go through every vehicle position and see if it's not inside a building.
//	unsigned int bumpcount=0, clearcount=0;
//	for(std::vector<Timestep>::iterator iter1=fcd_output.begin(); iter1 != fcd_output.end(); iter1++)
//	{
//		cout << "time " << iter1->time << '\n';
//
//
//		for(std::vector<Vehicle>::iterator iter2=iter1->vehiclelist.begin(); iter2!=iter1->vehiclelist.end(); iter2++)
//			if( isPointObstructed(conn,iter2->x,iter2->y) ) bumpcount++; else clearcount++;
//
//	}
//	cout << "bump " << bumpcount << " clear " << clearcount << endl;


//	if(GIS_isLineOfSight(conn,-8.620598,41.164310,-8.619410,41.164179)) cout << "NLOS\n"; else cout << "LOS\n";


//	// DEBUG deltaSeconds
//	// -8.61250 (08º36"45'W) to -8.62083 (08º37"15'W) should be 30"
//	cout << "city height: " << deltaSeconds(41.16884,41.160837) << endl;
//	cout << "city width: " << deltaSeconds(-8.622678,-8.609375) << endl;

//			// DEBUG, check what cells each vehicle is on
//			unsigned short xcell=0, ycell=0;
//			cout << "xgeo " << iterVeh->xgeo << " ygeo " << iterVeh->ygeo << '\n';
//			determineCellFromWGS84 (iterVeh->xgeo, iterVeh->ygeo, xcell, ycell);
//			cout << xcell << '\t' << ycell << '\n';
