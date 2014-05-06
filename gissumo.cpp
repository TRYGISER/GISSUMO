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


	/* Simulation starts here.
	 * We have a 0.37% mismatch error between the SUMO roads and the Porto shapefile data.
	 * This causes vehicles to be inside buildings every now and then.
	 * When a vehicle parks and is marked as an RSU, we must check to see if this isn't happening,
	 * otherwise the RSU will always return NLOS to all vehicles.
	 */

	// setup vectors to keep the list of RSUs and their status
	std::vector<RSU> rsuList;			// vector to hold list of RSUs
	CityMapChar vehicleLocations; 		// setup 2D map for vehicle locations
//	CityMapNum globalCoverage;

	// TEST: add an active RSU
	RSU testRSU;
	testRSU.id=0;
	testRSU.ygeo=41.164798;
	testRSU.xgeo=-8.616050;
	determineCellFromWGS84(testRSU.xgeo,testRSU.ygeo,testRSU.xcell,testRSU.ycell);
	testRSU.active=true;
	rsuList.push_back(testRSU);
	vehicleLocations.map[testRSU.xcell][testRSU.ycell]='R';





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




			if(m_printVehicleMap)	vehicleLocations.map[xcell][ycell]='o';	// tag the vehicle citymap

		}	// end for(vehicle)

		/*
		 * End of each time step
		 */

		if(m_printVehicleMap)	// --print-vehicle-map
		{
			printCityMap(vehicleLocations);			// print the vehicle map
			for(short yy=0;yy<CITYWIDTH;yy++)		// clean map
				for(short xx=0;xx<CITYHEIGHT;xx++)
					if(vehicleLocations.map[yy][xx]=='o')
						vehicleLocations.map[yy][xx]='.';
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
}


/* * */


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
		"SELECT COUNT(id) "
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
	pqxx::work txn(c);
	pqxx::result r = txn.exec(
			"INSERT INTO edificios(id, geom) "
			"VALUES ("
				+ pqxx::to_string(id)
				+ ", ST_GeomFromText('POINT("
				+ pqxx::to_string(xx) + " "
				+ pqxx::to_string(yy) + ")',4326) )"
		);
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
