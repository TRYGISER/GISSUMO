#include "gissumo.h"
#include "gis.h"
#include "network.h"
#include "uvcast.h"

#define XML_PATH "./fcdoutput.xml"

const ptree& empty_ptree(){
    static ptree t;
    return t;
}

// Can extern the debug variable.
bool m_debug = false;
bool m_rsu = false;
// From network
extern map<float,int> s_packetPropagationTime;


int main(int argc, char *argv[])
{
	/* Process command-line options
	 * Using boost::program_options
	 */

	// Defaults
	bool m_printVehicleMap = false;
	bool m_printSignalMap = false;
	bool m_printStatistics = false;
	bool m_printEndStatistics = false;
	bool m_validVehicle = false;
	bool m_debugLocations = false;
	bool m_debugCellMaps = false;
	bool m_networkEnabled = false;
	unsigned short m_accidentTime=60;
	unsigned short m_stopTime=0;
	string m_fcdFile = "./fcdoutput.xml";
	unsigned short m_pause = 0;

	// List of command line options
	options_description cliOptDesc("Options");
	cliOptDesc.add_options()
		("print-vehicle-map", "prints an ASCII map of vehicle positions")
		("print-signal-map", "prints an ASCII map of signal quality")
		("print-statistics", "outputs coverage metrics")
		("print-end-statistics", "output final counts")
		("check-valid-vehicles", "counts number of vehicles in the clear")
		("enable-network", "enables the network layer and packet transmission")
		("enable-rsu", "enables the RSU communication code")
		("accident-time", boost::program_options::value<unsigned short>(), "creates an accident at a specific time")
		("stop-time", boost::program_options::value<unsigned short>(), "stops the simulation at a specific time")
		("pause", boost::program_options::value<unsigned short>(), "pauses for N milliseconds after every timestep")
		("fcd-data", boost::program_options::value<string>(), "floating car data file location")
	    ("debug", "enable debug mode")
	    ("debug-locations", "debug vehicle location updates")
	    ("debug-cell-maps", "debug cell map updates")
	    ("help", "give this help list")
	;

	// Parse options
	variables_map varMap;
	store(parse_command_line(argc, argv, cliOptDesc), varMap);
	notify(varMap);

	if(argc==1) { cout << cliOptDesc; return 1; }

	// Process options
	if (varMap.count("debug")) 					m_debug=true;
	if (varMap.count("debug-locations")) 		m_debugLocations=true;
	if (varMap.count("debug-cell-maps")) 		m_debugCellMaps=true;
	if (varMap.count("print-vehicle-map")) 		m_printVehicleMap=true;
	if (varMap.count("print-signal-map")) 		m_printSignalMap=true;
	if (varMap.count("print-statistics")) 		m_printStatistics=true;
	if (varMap.count("print-end-statistics")) 	m_printEndStatistics=true;
	if (varMap.count("enable-network")) 		m_networkEnabled=true;
	if (varMap.count("enable-rsu")) 			m_rsu=true;
	if (varMap.count("accident-time")) 			m_accidentTime=varMap["accident-time"].as<unsigned short>();
	if (varMap.count("stop-time")) 				m_stopTime=varMap["stop-time"].as<unsigned short>();
	if (varMap.count("check-valid-vehicles"))	m_validVehicle=true;
	if (varMap.count("pause"))					m_pause=varMap["pause"].as<unsigned short>();
	if (varMap.count("fcd-data"))				m_fcdFile=varMap["fcd-data"].as<string>();
	if (varMap.count("help")) 					{ cout << cliOptDesc; return 1; }

	if (m_debug) cout << "BEGIN FCD FILE " << m_fcdFile << endl;

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
	read_xml(m_fcdFile, tree);
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

	// setup vectors and maps
	list<RSU> rsuList;			// vector to hold list of RSUs
	list<Vehicle> vehiclesOnGIS;	// vehicles we've processed from SUMO to GIS
	CityMapChar vehicleLocations; 		// 2D map for vehicle locations
	CityMapNum globalSignal;			// 2D map for global signal quality


	if(m_rsu)
	{
		// Add an RSU
		// Bottom left
		addNewRSU(conn, rsuList, 10000, -8.619278, 41.162600, true);
		// Bottom right
		addNewRSU(conn, rsuList, 10001, -8.614409, 41.162411, true);
		// Top right
		addNewRSU(conn, rsuList, 10002, -8.614507, 41.166282, true);
		// Top left
		addNewRSU(conn, rsuList, 10003, -8.620375, 41.165852, true);
		// North
		addNewRSU(conn, rsuList, 10004, -8.617054, 41.167548, true);
		// East
		addNewRSU(conn, rsuList, 10005, -8.614909, 41.164852, true);
		// South
		addNewRSU(conn, rsuList, 10006, -8.617476, 41.163523, true);
		// West
		addNewRSU(conn, rsuList, 10007, -8.620539, 41.164816, true);
	}

//	vector<unsigned short> neighList = GIS_getPointsInRange(conn,testRSU.xgeo,testRSU.ygeo,100);
//	unsigned short distTest = GIS_distanceToPointGID(conn,-8.6160498,41.165799,testRSU.gid);
//	for(vector<unsigned short>::iterator iter=neighList.begin(); iter!=neighList.end(); iter++)
//		cout << "\tNeighbor gid: " << *iter << '\n';



	// Run through every time step on the FCD XML file
	for(std::vector<Timestep>::iterator
			iterTime = fcd_output.begin();
			iterTime != fcd_output.end();
			iterTime++ )
	{
		/*
		 * Beginning of each FCD XML time step
		 */
		if(m_debug) cout << "\nDEBUG Timestep time=" << iterTime->time << endl;

		/* Mark all vehicles on vehiclesOnGIS as active=false
		 * The next step remarks the ones on the road (XML) as active=true
		 */
		for(list<Vehicle>::iterator
				iter=vehiclesOnGIS.begin();
				iter!=vehiclesOnGIS.end();
				iter++)
			iter->active=false;

		// run through each vehicle
		for(std::vector<Vehicle>::iterator
				iterVeh=iterTime->vehiclelist.begin();
				iterVeh!=iterTime->vehiclelist.end();
				iterVeh++)
		{
			/*
			 * Beginning of each vehicle on the FCD trace (already converted to Vehicle entities)
			 */

			if(m_debugLocations) cout << "DEBUG Vehicle id=" << iterVeh->id << endl;

			// 0 - Always needed: clone the vehicle and update its position in cells
			Vehicle newVehicle = *iterVeh;						// copy vehicle from XML iterator
			determineCellFromWGS84 (newVehicle.xgeo, newVehicle.ygeo,
					newVehicle.xcell, newVehicle.ycell);		// determine vehicle location in cells
			if(m_debugLocations) cout << "DEBUG Vehicle id=" << iterVeh->id << " new xcell=" << newVehicle.xcell << " new ycell=" << newVehicle.ycell << endl;
			if(m_printVehicleMap || m_printStatistics)
				vehicleLocations.map[newVehicle.xcell][newVehicle.ycell]='o';	// tag the vehicle citymap

			// 1 - See if the vehicle is new.
			list<Vehicle>::iterator iterVehicleOnGIS = find_if(
					vehiclesOnGIS.begin(),
					vehiclesOnGIS.end(),
					boost::bind(&Vehicle::id, _1) == iterVeh->id	// '_1' means "substitute with the first input argument"
					);

			if(iterVehicleOnGIS==vehiclesOnGIS.end())
			{
				// 2a - New vehicle. Add it to GIS, get GID, add to our local record.
				newVehicle.gid = GIS_addPoint(conn,newVehicle.xgeo,newVehicle.ygeo,newVehicle.id);
				// Mark as active
				newVehicle.active=true;
				// Add to our local record
				vehiclesOnGIS.push_back(newVehicle);
				// Debug
				if(m_debugLocations) cout << "DEBUG Vehicle id=" << iterVeh->id << " is new, added to GIS with gid=" << newVehicle.gid << endl;
			}
			else
			{
				// 2b - Existing vehicle: update its position on GIS via GID
				GIS_updatePoint(conn,newVehicle.xgeo,newVehicle.ygeo,iterVehicleOnGIS->gid);
				// Update our local copy
				iterVehicleOnGIS->xcell = newVehicle.xcell;
				iterVehicleOnGIS->ycell = newVehicle.ycell;
				iterVehicleOnGIS->xgeo = newVehicle.xgeo;
				iterVehicleOnGIS->ygeo = newVehicle.ygeo;
				iterVehicleOnGIS->speed = newVehicle.speed;
				// Mark as active
				iterVehicleOnGIS->active = true;
				// Debug
				if(m_debugLocations) cout << "DEBUG Vehicle id=" << iterVeh->id << " exists, gid=" << iterVehicleOnGIS->gid << " update xgeo=" << newVehicle.xgeo << " ygeo=" << newVehicle.ygeo << endl;


			}

			/* 3 - Mark vehicles missing from this timestep as 'parked'.
			 * No longer needed. Vehicles that are active=false are no longer a part of the FCD XML output, and thus disappeared from the simulation.
			 * We can decide what to do with inactive cars here. Parked/RSU/Uplink.
			 */

		}	// end for(vehicle)


		/* Vehicles are now in the GIS map as POINTs.
		 * All new vehicles added to GIS, all existing vehicles' positions updated on GIS.
		 * The first level of fcd_output is a time entity, the second level is a vehiclelist
		 * of Vehicle entities from XML.
		 * vehiclesOnGIS is our local database of updated vehicles.
		 * rsuList has our RSUs.
		 */


		/* Go through each RSU and update its coverage map.
		 * This is computed from the vehicles the RSU sees, and their signal strength.
		 */
		for(list<RSU>::iterator iterRSU = rsuList.begin();
			iterRSU != rsuList.end();
			iterRSU++)
		{
			// first get the RSU's neighbors' GIDs
			vector<unsigned short> rsuNeighs = GIS_getPointsInRange(conn,iterRSU->xgeo,iterRSU->ygeo,MAXRANGE);

			// now run through each neighbor
			for(vector<unsigned short>::iterator neighbor=rsuNeighs.begin();
					neighbor != rsuNeighs.end();
					neighbor++)
			{
				// get distance from neighbor to RSU
				unsigned short distneigh = GIS_distanceToPointGID(conn,iterRSU->xgeo,iterRSU->ygeo,*neighbor);

				if(distneigh)	// ignore ourselves (distance==0)
				{
					// carry debug
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " distance " << distneigh << '\n';

					// get the neighbor's coordinates
					float xgeoneigh=0, ygeoneigh=0;
					GIS_getPointCoords(conn, *neighbor, xgeoneigh, ygeoneigh);
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << setprecision(8) << " at xgeo=" << xgeoneigh << " ygeo=" << ygeoneigh << '\n';

					// convert them to cells
					unsigned short xcellneigh=0, ycellneigh=0;
					determineCellFromWGS84(xgeoneigh,ygeoneigh,xcellneigh,ycellneigh);
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " cell coords as xcell=" << xcellneigh << "\t ycell=" << ycellneigh << '\n';

					// determine LOS status
					bool LOSneigh = GIS_isLineOfSight(conn,iterRSU->xgeo,iterRSU->ygeo,xgeoneigh,ygeoneigh);
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " LOS " << (LOSneigh?"true":"false") << '\n';

					// determine signal quality
					unsigned short signalneigh = getSignalQuality(distneigh,LOSneigh);
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " signal " << signalneigh << '\n';

					// update RSU coverage map
					short xrelative = PARKEDCELLRANGE + xcellneigh - iterRSU->xcell;
					short yrelative = PARKEDCELLRANGE + ycellneigh - iterRSU->ycell;
					iterRSU->coverage[xrelative][yrelative]=signalneigh;
					if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " on RSU map at xcell=" << xrelative << " ycell=" << yrelative << '\n';

				}	// end distance!=0
			}	// end for(RSU neighbors)

			// now that the RSU's local map is updated, apply this map to the global signal map
			applyCoverageToCityMap(*iterRSU, globalSignal);

		}	// end for(RSUs)

		/* Network layer.
		 * Act on vehiclesOnGIS and rsuList, and disseminate packets.
		 * Activate UVCAST and designate vehicles as SCF
		 */
		if(m_networkEnabled)
		{
			processNetwork(conn,iterTime->time,vehiclesOnGIS,rsuList);

			// Create an accident in the middle of the map
			// Locate a random vehicle at the center of the map to be the accident source
			// Get a specific vehicle to act as the accident source
			if(iterTime->time==m_accidentTime)
			{
				// Locate a vehicle. Map center is at YCENTER XCENTER
				// we begin with a range of 8, and keep doubling it until one vehicle is found
				vector<Vehicle*> centerVehicles; unsigned short centerRange=8;
				do{
					centerVehicles = getVehiclesNearPoint(conn,vehiclesOnGIS, XCENTER, YCENTER, centerRange);
					centerRange *= 2;
				} while(centerVehicles.size()==0);

				if(m_debug) cout << "DEBUG AccidentSelected on vehicle"
						<< " vID " << (*(centerVehicles.begin()))->id
						<< " xgeo " << (*(centerVehicles.begin()))->xgeo
						<< " ygeo " << (*(centerVehicles.begin()))->xgeo
						<< endl;

						simulateAccident(conn, iterTime->time, vehiclesOnGIS, rsuList, **(centerVehicles.begin()) );
			}
		}



		/* Compute and print statistics.
		 *
		 */
		if(m_printStatistics)
		{
			short countInactive=0, countActive=0;
			for(list<Vehicle>::iterator
					iter=vehiclesOnGIS.begin();
					iter!=vehiclesOnGIS.end();
					iter++)
				if(iter->active) countActive++; else countInactive++;
			cout << "STAT vehicleStatus"
					<< " active " << countActive
					<< " inactive " << countInactive
					<< endl;
		}

		if(m_printStatistics)
		{
			// count the number of 'road' cells
			unsigned short roadCells = 0;
			for(short xx=0;xx<CITYWIDTH;xx++)
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(vehicleLocations.map[xx][yy]!=' ')
						roadCells++;

			// count the number of covered cells
			unsigned short roadCellsCovered = 0;
			for(short xx=0;xx<CITYWIDTH;xx++)
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(globalSignal.map[xx][yy]!=0)
						roadCellsCovered++;

			// determine the mean coverage of all valid cells
			unsigned short roadCellsSignalSum = 0;
			float roadCellsMeanSignal = 0;
			for(short xx=0;xx<CITYWIDTH;xx++)
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(globalSignal.map[xx][yy]!=0)
						roadCellsSignalSum+=globalSignal.map[xx][yy];
			roadCellsMeanSignal = (float)roadCellsSignalSum/(float)roadCells;
			if(m_debug) cout << "DEBUG Road Cell Signal Sum " << roadCellsSignalSum << endl;

			cout << "STAT"
					<< " cells " << roadCells
					<< " cellsCovered " << roadCellsCovered
					<< " cellsMeanSignal " << roadCellsMeanSignal
					<< " cellsCoveredMeanSignal " << ( (float)roadCellsSignalSum/(float)roadCellsCovered )
					<< endl;
		}


		// Wrap up.
		if(m_printSignalMap && m_printVehicleMap)
		{
			// Apply signal map to vehicle map
			for(short xx=0;xx<CITYWIDTH;xx++)
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(globalSignal.map[xx][yy])
						vehicleLocations.map[xx][yy]=boost::lexical_cast<char>(globalSignal.map[xx][yy]);
			// overlay RSUs on the map
			for(list<RSU>::iterator iter=rsuList.begin(); iter!=rsuList.end(); iter++)
				if(iter->active)
					vehicleLocations.map[iter->xcell][iter->ycell]='R';
			// print the vehicle map
			cout << "Timestep: " << iterTime->time << '\n';
			printCityMap(vehicleLocations);
			// clean map
			for(short xx=0;xx<CITYWIDTH;xx++)
				for(short yy=0;yy<CITYHEIGHT;yy++)
					if(vehicleLocations.map[xx][yy]=='o' || vehicleLocations.map[xx][yy]=='R')
						vehicleLocations.map[xx][yy]='.';
		}
		else
		{
			if(m_printSignalMap)
				printCityMap(globalSignal);

			if(m_printVehicleMap)	// --print-vehicle-map
			{
				for(list<RSU>::iterator iter=rsuList.begin(); iter!=rsuList.end(); iter++)
					if(iter->active)
						vehicleLocations.map[iter->xcell][iter->ycell]='R';		// overlay RSUs on the map
				printCityMap(vehicleLocations);			// print the vehicle map
				for(short xx=0;xx<CITYWIDTH;xx++)		// clean map
					for(short yy=0;yy<CITYHEIGHT;yy++)
						if(vehicleLocations.map[xx][yy]=='o' || vehicleLocations.map[xx][yy]=='R')
							vehicleLocations.map[xx][yy]='.';
			}
		}
		if(m_pause)
			{ cout << flush; this_thread::sleep( posix_time::milliseconds(m_pause) ); }

		/*
		 * End of each time step
		 */

		if(m_stopTime && iterTime->time>=m_stopTime)
			break;
	}	// end for(timestep)


	/* Print final count of packet propagation times.
	 */
	if(m_printEndStatistics)
	{
		cout << "STAT PacketPropagationTime"
				<< "\nCount\tTime" << endl;
		for(map<float,int>::iterator mapIter=s_packetPropagationTime.begin();
				mapIter!=s_packetPropagationTime.end();
				mapIter++)
			cout << mapIter->second << '\t' << mapIter->first << '\n';

	}

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
	// Uncomment this to leave the GIS database in a clean state after the simulation.
//	GIS_clearAllPoints(conn);

	return 0;
}


/* * */



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

void printCityMap (CityMapNum cmap)
{
	for(short yy=0;yy<CITYHEIGHT;yy++)
	{
		for(short xx=0;xx<CITYWIDTH;xx++)
			if(cmap.map[xx][yy]>0) cout << cmap.map[xx][yy] << ' ';
			else cout << "  ";
		cout << '\n';
	}
}

unsigned short getSignalQuality(unsigned short distance, bool lineOfSight)
{
	if(lineOfSight)
	{
		if(distance<70) return 5;
		if(distance<115) return 4;
		if(distance<135) return 3;
		if(distance<155) return 2;
	}
	else
	{
		if(distance<58) return 5;
		if(distance<65) return 4;
		if(distance<105) return 3;
		if(distance<130) return 2;
	}
	return 0; 	// no signal
}

void applyCoverageToCityMap (RSU rsu, CityMapNum &city)
{
	for(short xx=0; xx<PARKEDCELLCOVERAGE; xx++)
		for(short yy=0; yy<PARKEDCELLCOVERAGE; yy++)
		{
			short mapX=xx+rsu.xcell-PARKEDCELLRANGE;
			short mapY=yy+rsu.ycell-PARKEDCELLRANGE;

			// 'upgrade' coverage in a given cell if this RSU can cover it better
			if(rsu.coverage[xx][yy] > city.map[mapX][mapY])
				city.map[mapX][mapY] = rsu.coverage[xx][yy];
		}
}

void printLocalCoverage(array< array<unsigned short,PARKEDCELLCOVERAGE>,PARKEDCELLCOVERAGE > coverage)
{
	for(short yy=0;yy<PARKEDCELLCOVERAGE;yy++)
	{
		for(short xx=0;xx<PARKEDCELLCOVERAGE;xx++)
			cout << coverage[yy][xx] << ' ';
		cout << '\n';
	}
}

void printVehicleDetails(Vehicle veh)
{
	cout << "DEBUG Vehicle"
			<< "\n\t id " << veh.id
			<< " gid " << veh.gid
			<< "\n\t parked " << (veh.parked?"true":"false")
			<< "\n\t xcell " << veh.xcell
			<< " ycell " << veh.ycell
			<< "\n\t xgeo " << veh.xgeo
			<< " ygeo " << veh.ygeo
			<< "\n\t speed " << veh.speed
//			<< " packets " << veh.p_buffer.size()
			<< " packetID " << veh.packet.packetID
			<< " packetSrc " << veh.packet.packetSrc
			<< '\n';
}

void printListOfVehicles(list<Vehicle> &vehiclesOnGIS)
{
	cout << "DEBUG VehicleList"
			<< "\n\tID\tGID\tpID\tSCF\n";
	for(list<Vehicle>::iterator iterD=vehiclesOnGIS.begin(); iterD!=vehiclesOnGIS.end(); iterD++)
		cout <<
			'\t' << iterD->id <<
			'\t' << iterD->gid <<
			'\t' << iterD->packet.packetID <<
			'\t' << iterD->scf
			<< endl;
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
