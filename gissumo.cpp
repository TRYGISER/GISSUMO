#include "gissumo.h"
#include "gis.h"
#include "network.h"
#include "uvcast.h"

const ptree& empty_ptree(){
    static ptree t;
    return t;
}

// Global Simulation Time
float g_simulationTime=-1;
// Can extern the debug variable.
unsigned short gm_debug = 0;
bool gm_rsu = false;
// From network
extern map<float,int> gs_packetPropagationTime;

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
	bool m_printMapTime = false;
	bool m_validVehicle = false;
	bool m_debugLocations = false;
	bool m_debugCellMaps = false;
	bool m_debugMapBroadcast = false;
	bool m_networkEnabled = false;
	bool m_bruteforce = false;
	bool m_enableMapSpread = true;
	unsigned short m_accidentTime=60;
	unsigned short m_stopTime=0;
	unsigned short m_rsuLoadTime=0;
	unsigned short m_rsuMapDebugId=0;
	unsigned short m_decisionMode=1;
	uint32_t m_printCombination = 0;
	string m_fcdFile = "./fcdoutput.xml";
	string m_rsuFile = "./rsudata.tsv";
	unsigned short m_pause = 0;

	// List of command line options
	options_description cliOptDesc("Options");
	cliOptDesc.add_options()
		("print-vehicle-map", "prints an ASCII map of vehicle positions")
		("print-signal-map", "prints an ASCII map of signal quality")
		("print-statistics", "outputs coverage metrics")
		("print-end-statistics", "outputs final counts")
	    ("print-map-time", "outputs coverage map time to completeness")
		("print-combination", boost::program_options::value<uint32_t>(), "prints statistics of a specific combination")
		("check-valid-vehicles", "counts number of vehicles in the clear")
		("enable-network", "enables the network layer and packet transmission")
		("enable-rsu", "enables the RSU communication code")
		("decision-mode", boost::program_options::value<unsigned short>(), "1: always deciding, 2: permanent shutdown")
		("disable-map-spread", "stops RSUs from broadcasting their maps")
		("bruteforce", "bruteforces the best RSU coverage combination")
		("accident-time", boost::program_options::value<unsigned short>(), "creates an accident at a specific time")
		("stop-time", boost::program_options::value<unsigned short>(), "stops the simulation at a specific time")
		("pause", boost::program_options::value<unsigned short>(), "pauses for N milliseconds after every timestep")
		("fcd-data", boost::program_options::value<string>(), "floating car data file location")
		("rsu-data", boost::program_options::value<string>(), "tab-separated file with x,y coordinates of RSUs")
		("rsu-load-time", boost::program_options::value<unsigned short>(), "loads the RSUs at a specific time")
		("debug", boost::program_options::value<unsigned short>(), "enable debug mode")
	    ("debug-locations", "debug vehicle location updates")
	    ("debug-cell-maps", "debug cell map updates")
	    ("debug-map-broadcast", "debug coverage map broadcasting")
		("debug-rsu-map", boost::program_options::value<unsigned short>(), "prints the evolution of a specific RSU's map")
	    ("help", "give this help list")
	;

	// Parse options
	variables_map varMap;
	store(parse_command_line(argc, argv, cliOptDesc), varMap);
	notify(varMap);

	if(argc==1) { cout << cliOptDesc; return 1; }

	// Process options
	if (varMap.count("debug")) 					gm_debug=varMap["debug"].as<unsigned short>();
	if (varMap.count("debug-locations")) 		m_debugLocations=true;
	if (varMap.count("debug-cell-maps")) 		m_debugCellMaps=true;
	if (varMap.count("debug-map-broadcast")) 	m_debugMapBroadcast=true;
	if (varMap.count("debug-rsu-map"))			m_rsuMapDebugId=varMap["debug-rsu-map"].as<unsigned short>();
	if (varMap.count("print-vehicle-map")) 		m_printVehicleMap=true;
	if (varMap.count("print-signal-map")) 		m_printSignalMap=true;
	if (varMap.count("print-statistics")) 		m_printStatistics=true;
	if (varMap.count("print-end-statistics")) 	m_printEndStatistics=true;
	if (varMap.count("print-map-time"))		 	m_printMapTime=true;
	if (varMap.count("enable-network")) 		m_networkEnabled=true;
	if (varMap.count("enable-rsu")) 			gm_rsu=true;
	if (varMap.count("decision-mode")) 			m_decisionMode=varMap["decision-mode"].as<unsigned short>();
	if (varMap.count("disable-map-spread")) 	m_enableMapSpread=false;
	if (varMap.count("bruteforce")) 			m_bruteforce=true;
	if (varMap.count("accident-time")) 			m_accidentTime=varMap["accident-time"].as<unsigned short>();
	if (varMap.count("stop-time")) 				m_stopTime=varMap["stop-time"].as<unsigned short>();
	if (varMap.count("rsu-load-time")) 			m_rsuLoadTime=varMap["rsu-load-time"].as<unsigned short>();
	if (varMap.count("check-valid-vehicles"))	m_validVehicle=true;
	if (varMap.count("pause"))					m_pause=varMap["pause"].as<unsigned short>();
	if (varMap.count("print-combination"))		m_printCombination=varMap["print-combination"].as<uint32_t>();
	if (varMap.count("fcd-data"))				m_fcdFile=varMap["fcd-data"].as<string>();
	if (varMap.count("rsu-data"))				m_rsuFile=varMap["rsu-data"].as<string>();
	if (varMap.count("help")) 					{ cout << cliOptDesc; return 1; }

	cout << "BEGIN FCD FILE " << m_fcdFile << endl;

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

	cout << "Read " << fcd_output.size() << " records from " << m_fcdFile << endl;

	/* Open a connection to PostgreSQL
	 * A password can be added to this string.
	 */
	pqxx::connection conn("dbname=shapefiledb user=abreis");

	// Clear all POINT entities from the database from past simulations.
	GIS_clearAllPoints(conn, CAR_FEATTYP);
	GIS_clearAllPoints(conn, RSU_FEATTYP);

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

	// list of events
	EventList events;


	/* * * TIMESTEP LOOP BEGIN * * */


	// Run through every time step on the FCD XML file
	for(std::vector<Timestep>::iterator
			iterTime = fcd_output.begin();
			iterTime != fcd_output.end();
			iterTime++ )
	{

		/*
		 * Beginning of each FCD XML time step
		 */

		// Break if FCD XML has two timesteps with the same time.
		assert(iterTime->time > g_simulationTime);

		// Update global simulation time.
		g_simulationTime = iterTime->time;
		if(gm_debug) cout << "\nDEBUG Timestep time=" << iterTime->time << endl;

		// Signal map needs to be reset on each timestep
		CityMapNum globalSignal;			// 2D map for global signal quality

		/* Only load RSUs at specified time
		 */
		if(iterTime->time==m_rsuLoadTime && gm_rsu)
		{
			// import RSU locations from m_rsuFile, tab-separated [x,y] coordinates

			std::ifstream inFile(m_rsuFile);
			if(gm_debug) cout << "Adding RSUs from " << m_rsuFile << endl;

			short rsuIDcounter = 10000;
			while(inFile)
			{
				string tsvX, tsvY;
				inFile >> tsvX; inFile >> tsvY;
				if(tsvX!="" && tsvY!="")
				{
					float xgeo = std::stof(tsvX);
					float ygeo = std::stof(tsvY);

					addNewRSU(conn, rsuList, ++rsuIDcounter, xgeo, ygeo, true);
					if(gm_debug) cout << "\tLoaded RSU " << rsuIDcounter << " on " << xgeo << '\t' << ygeo << endl;
				}
			}
		}


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
				newVehicle.gid = GIS_addPoint(conn,newVehicle.xgeo,newVehicle.ygeo,newVehicle.id, CAR_FEATTYP);
				// Mark as active
				newVehicle.active = true;
				// Tag creation time
				newVehicle.timeAdded = g_simulationTime;
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
		 * If the RSU is active, trigger map broadcasts on criteria (5 new cells covered).
		 * If the RSU is inactive only the coverage map is updated, nothing else.
		 */
		if(gm_rsu)
			for(list<RSU>::iterator iterRSU = rsuList.begin();
				iterRSU != rsuList.end();
				iterRSU++)
			{
				// first get the RSU's neighbors' GIDs
				vector<unsigned int> rsuNeighs = GIS_getPointsInRange(conn,iterRSU->xgeo,iterRSU->ygeo,MAXRANGE,CAR_FEATTYP);

				if(gm_debug>1) cout << "DEBUG2 got " << rsuNeighs.size() << " neighbors of RSU id=" << iterRSU->id << endl;

				// now run through each neighbor
				for(vector<unsigned int>::iterator neighbor=rsuNeighs.begin();
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

						// determine relative positions for local map position
						short xrelative = PARKEDCELLRANGE + xcellneigh - iterRSU->xcell;
						short yrelative = PARKEDCELLRANGE + ycellneigh - iterRSU->ycell;

						// only count newly covered cells if coverage goes from 0 to positive
						if(iterRSU->coverage.map[xrelative][yrelative]==0 && signalneigh)
						{
							iterRSU->coveredCellCount++;
							// Also record current time, to draw statistics on time to complete coverage map
							iterRSU->lastTimeUpdated=g_simulationTime;
						}
						if(m_debugCellMaps) cout << "DEBUG\t RSU id=" << iterRSU->id << " is now covering " << iterRSU->coveredCellCount << " cells" << endl;

						// update RSU coverage map if we recorded a better signal
						if(signalneigh>iterRSU->coverage.map[xrelative][yrelative])
							iterRSU->coverage.map[xrelative][yrelative]=signalneigh;

						if(m_debugCellMaps) cout << "DEBUG\t neighbor gid=" << *neighbor << " on RSU map at xcell=" << xrelative << " ycell=" << yrelative << '\n';

						// increment number of cells this RSU is covering
					}	// end distance!=0
				}	// end for(RSU neighbors)


				if(iterRSU->active)		// inactive RSUs don't broadcast their maps
				{
					// Coverage map broadcast criteria
					if(m_enableMapSpread)
							if( (iterRSU->coveredCellCount - iterRSU->coveredCellsOnLastBroadcast) > 5)
							{
								if(gm_debug || m_debugMapBroadcast) cout << "DEBUG Criteria triggered, marking RSU id=" << iterRSU->id << " for coverage map broadcast" << endl;
								// reset count
								iterRSU->coveredCellsOnLastBroadcast = iterRSU->coveredCellCount;
								// update flag
								iterRSU->triggerBroadcast=true;
							}

					// now that the RSU's local map is updated, apply this map to the global signal map
					applyCoverageToCityMap(iterRSU->coverage, globalSignal);
				}

				// Debug, print the map of the RSU specified in --debug-rsu-map
				if(m_rsuMapDebugId && m_rsuMapDebugId==iterRSU->id)
				{
					cout << "RSU id=" << iterRSU->id << " covers " << iterRSU->coveredCellCount << " cells:" << endl;
					printLocalCoverage(iterRSU->coverage);
				}
			}	// end for(RSUs)


		/* Go through each RSU.
		 * - If criteria match for a decision, decide.
		 * - If the RSU is going inactive from the decision, prepare to broadcast an empty map.
		 * - If criteria match for the map to be broadcast, broadcast.
		 * No decisions without map spreading.
		 */
		if(gm_rsu && m_enableMapSpread)
		{
			for(list<RSU>::iterator iterRSU = rsuList.begin();
				iterRSU != rsuList.end();
				iterRSU++)
			{
				// Hold the previous status
				bool lastRSUstatus=iterRSU->active;

				/* Run the decision if we got a trigger to broadcast or to decide
				 */
				if( m_decisionMode==1 && (iterRSU->triggerBroadcast || iterRSU->triggerDecision) )
					iterRSU->active = pointDecisionAlgorithm(*iterRSU);		// Use classifier
//					iterRSU->active = decisionAlgorithm(*iterRSU);			// Use cell overlap

				// Inform if the RSU switched states now.
				if(gm_debug)
				{
					if(iterRSU->active==false && lastRSUstatus==true)
						cout << "DEBUG Turning OFF RSU id=" << iterRSU->id << " and broadcasting empty map." << endl;
					else if(iterRSU->active==true && lastRSUstatus==false)
						cout << "DEBUG Turning ON RSU id=" << iterRSU->id << " and broadcasting complete map." << endl;
				}

				/* What triggers a broadcast:
				 * RSU active to inactive
				 * RSU inactive to active
				 * RSU triggerBroadcast & RSU active
				 */
				if( (iterRSU->active==false && lastRSUstatus==true) ||
					(iterRSU->active==true && lastRSUstatus==false) ||
					(iterRSU->triggerBroadcast && iterRSU->active) )
					{
						// The map that we're broadcasting to neighbors.
						CoverageMap mapToBroadcast;

						// Inactive RSUs must always broadcast empty maps
						if(iterRSU->active)
							mapToBroadcast = iterRSU->coverage;	// full map
						else
							mapToBroadcast = CoverageMap();		// empty map

						// Broadcast the map in mapToBroadcast
						// Get the list of all RSU neighbors
						vector<RSU*> neighborRSUs = getRSUsInRange(conn, rsuList, *iterRSU, RSU_ALL);
						for(vector<RSU*>::iterator iterNeigh = neighborRSUs.begin();
							iterNeigh != neighborRSUs.end();
							iterNeigh++)
							{
								(*iterNeigh)->neighborMaps[iterRSU->id] = mapToBroadcast;	// send each neighbor our updated coverage map
								(*iterNeigh)->triggerDecision = true; // trigger a decision on the neighbor
							}
					} 	// end if(broadcast)

				// Reset flags
				iterRSU->triggerBroadcast=false;
				iterRSU->triggerDecision=false;
			}
		}


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

				if(gm_debug) cout << "DEBUG AccidentSelected on vehicle"
						<< " vID " << (*(centerVehicles.begin()))->id
						<< " xgeo " << (*(centerVehicles.begin()))->xgeo
						<< " ygeo " << (*(centerVehicles.begin()))->xgeo
						<< endl;

						simulateAccident(conn, iterTime->time, vehiclesOnGIS, rsuList, *(centerVehicles.begin()) );
			}
		}

		/* Debug coverage map broadcasts.
		 *
		 */
		if(m_debugMapBroadcast)
		{
			cout << "DEBUG MAP BCAST " << '\n';

			for(list<RSU>::iterator iterRSU = rsuList.begin();
				iterRSU != rsuList.end();
				iterRSU++)
			{
				cout << "\tLocal map of RSU id=" << iterRSU->id << '\n';
				printLocalCoverage(iterRSU->coverage);

				for(map<unsigned short, CoverageMap>::iterator iterNeighMap = iterRSU->neighborMaps.begin();
						iterNeighMap != iterRSU->neighborMaps.end();
						iterNeighMap++)
				{
					cout << "\t\tRSU id=" << iterRSU->id << " map of neighbor id=" << iterNeighMap->first << '\n';
					printLocalCoverage(iterNeighMap->second);
				}

			}
		}


		/* Compute and print timestep statistics.
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

			/*
			 */
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
			if(gm_debug) cout << "DEBUG Road Cell Signal Sum " << roadCellsSignalSum << endl;

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
		{
			break;
		}
	}	// end for(timestep)


	/* * * TIMESTEP LOOP END * * */


	/* Print final coverage map, coverage metrics, redundancy, etc.
	 */
	if(gm_rsu && m_enableMapSpread && m_printEndStatistics)
	{

		// Go through active RSUs and place them in the global map
		CityMapNum cityCoverageSignal, cityCoverageCount;

		cout << "Active RSUs: ";
		for(list<RSU>::iterator iterRSU=rsuList.begin(); iterRSU != rsuList.end(); iterRSU++)
			if(iterRSU->active)
			{
				cout << iterRSU->id << ' ';
				applyCountToCityMap(iterRSU->coverage, cityCoverageCount);
				applyCoverageToCityMap(iterRSU->coverage, cityCoverageSignal);
			}
		cout << endl;

		// Compute statistics from the coverage maps
		map<int,unsigned short> covStat = getCoverageStatistics(cityCoverageSignal);
		unsigned short over1 = getOvercoverageMetric(cityCoverageCount, 1);
		unsigned short over2 = getOvercoverageMetric(cityCoverageCount, 2);
		unsigned short over3 = getOvercoverageMetric(cityCoverageCount, 3);


		// Print the statistics
		cout << '\n' << "STAT\tcov0\tcov1\tcov2\tcov3\tcov4\tcov5\tover1\tover2\tover3" << endl;
		cout << '\t'
			<< covStat[0] << '\t'
			<< covStat[1] << '\t'
			<< covStat[2] << '\t'
			<< covStat[3] << '\t'
			<< covStat[4] << '\t'
			<< covStat[5] << '\t'
			<< over1 << '\t'
			<< over2 << '\t'
			<< over3 << endl;

		// Print each RSUs utility scores
		if(gm_debug)
		{
			cout << "RSU scores: \n" << "ID\tactive\tutility\tutilPos\tutilNeg\n";
			for(list<RSU>::iterator iterRSU=rsuList.begin(); iterRSU != rsuList.end(); iterRSU++)
			cout << iterRSU->id
				<< '\t' << (iterRSU->active?"yes":"no")
				<< '\t' << iterRSU->utility
				<< '\t' << iterRSU->utilPos
				<< '\t' << iterRSU->utilNeg
				<< endl;
		}

		// TODO: debug: compute real utility vs local 'believed' utility

		// Print the signal map
		printCityMap(cityCoverageSignal);

		// Print the coverage map
		printCityMap(cityCoverageCount);
	}


	/* Print final count of packet propagation times.
	 */
	if(m_networkEnabled && m_printEndStatistics)
	{
		cout << "STAT PacketPropagationTime"
				<< "\nCount\tTime" << endl;
		for(map<float,int>::iterator mapIter=gs_packetPropagationTime.begin();
				mapIter!=gs_packetPropagationTime.end();
				mapIter++)
			cout << mapIter->second << '\t' << mapIter->first << '\n';
	}


	/* Compute RSU coverage map time (RSU time of creation - RSU last map update time)
	 */
	if(m_printMapTime && gm_rsu)
	{
		// Store total time, number of RSUs considered
		float totalTime=0; unsigned short rsuCount=0;

		// Go through each RSU and compute time between creation and last map update
		cout << "RSU ID\tTime\n";
		for(list<RSU>::iterator iterRSU = rsuList.begin();
			iterRSU != rsuList.end();
			iterRSU++)
			{
				float deltaTime = iterRSU->lastTimeUpdated - iterRSU->timeAdded;

				cout << iterRSU->id << '\t' << deltaTime << '\n';

				if(deltaTime>0)
					{ totalTime+=deltaTime; rsuCount++; }
			}

		if(gm_debug) cout << "DEBUG got total time " << totalTime << " from " << rsuCount << " RSUs" << endl;

		// Compute and print statistic
		float meanMapTime = totalTime / static_cast<float>(rsuCount);
		cout << "STAT Mean RSU Map Time " << meanMapTime << endl;

	}


	// Bruteforce the optimal RSU coverage solution.
	if(m_bruteforce)
	{
		// Clone active RSUs from rsuList to rsuListActive
		vector<RSU> rsuListActive;	// vector required for [] access
		for(list<RSU>::iterator iterRSU = rsuList.begin(); iterRSU != rsuList.end(); iterRSU++)
			if(iterRSU->active) rsuListActive.push_back(*iterRSU);

		cout << "\nBruteforcing optimal RSU coverage solution..."
				<< "\nWorking with " << rsuListActive.size() << " active RSUs" << endl;

		// Don't do more than 32 RSUs.
		assert(rsuListActive.size() <= 32);

		// Sort the list of RSUs by ID
		std::sort(rsuListActive.begin(), rsuListActive.end());

		/* Test all combinations from 1 to 2^RSUs
		 */
		if(gm_debug) cout << "Going through combinations..." << endl;

		// Vector to store the statistics of each combination
		vector<StatEntry> stats;

		// Store the max or min stats that we're keeping in memory. This initializes to max and min values.
		StatEntry limits;

		// Only print statistics every 1/1000th percent
		const unsigned int perMille=pow(2,rsuListActive.size())/1000;
		float percentProgress = 0.0;

		// Go through combinations
		for(uint32_t cID = 1; cID < pow(2,rsuListActive.size()); cID++)
		{
			if(gm_debug)
				if(cID%perMille==0)
				{
					percentProgress += 0.1;
					cout << std::fixed << std::setprecision(1)
						<< "Progress: " << percentProgress << '%'
						<< "\tcID: " << cID
						<< "\tstored: " << stats.size()
						<< endl;
				}

			/*	'cityCoverageSignal': Best signal level at each cell
			 * 	'cityCoverageCount'	: Number of RSUs covering each cell
			 */
			CityMapNum cityCoverageSignal, cityCoverageCount;

			// Look at combination ID (cID) bit by bit
			for (int i=0; i<24; i++)
				if ( ( (cID >> i) & 1) == 1)
				{
					// bit 'i' is a '1'
					// add car 'i' in 'listOfCars' to the maps
					applyCountToCityMap(rsuListActive[i].coverage, cityCoverageCount);
					applyCoverageToCityMap(rsuListActive[i].coverage, cityCoverageSignal);
				}

			// Compute statistics for this combination
			map<int,unsigned short> covStat = getCoverageStatistics(cityCoverageSignal);
			unsigned short over1 = getOvercoverageMetric(cityCoverageCount, 1);
			unsigned short over2 = getOvercoverageMetric(cityCoverageCount, 2);
			unsigned short over3 = getOvercoverageMetric(cityCoverageCount, 3);

			// Make a record of this combination's statistics.
			StatEntry ccomb;
			ccomb.cID = cID;
			ccomb.cov0 = covStat[0];
			ccomb.cov1 = covStat[1];
			ccomb.cov2 = covStat[2];
			ccomb.cov3 = covStat[3];
			ccomb.cov4 = covStat[4];
			ccomb.cov5 = covStat[5];
			ccomb.over1 = over1;
			ccomb.over2 = over2;
			ccomb.over3 = over3;

			/* Can't store all combinations, must only keep the best ones stored.
			 * Discard combinations unless they are better at at least 1 stat on
			 * what we currently have stored.
			 */
				 if(ccomb.cov0 < limits.cov0 )			{ stats.push_back(ccomb); limits.cov0=ccomb.cov0;}
			else if(ccomb.cov1 > limits.cov1 )			{ stats.push_back(ccomb); limits.cov1=ccomb.cov1;}
			else if(ccomb.cov2 > limits.cov2 )			{ stats.push_back(ccomb); limits.cov2=ccomb.cov2;}
			else if(ccomb.cov3 > limits.cov3 )			{ stats.push_back(ccomb); limits.cov3=ccomb.cov3;}
			else if(ccomb.cov4 > limits.cov4 )			{ stats.push_back(ccomb); limits.cov4=ccomb.cov4;}
			else if(ccomb.cov5 > limits.cov5 )			{ stats.push_back(ccomb); limits.cov5=ccomb.cov5;}
			else if(ccomb.over1 < limits.over1 )		{ stats.push_back(ccomb); limits.over1=ccomb.over1;}
			else if(ccomb.over2 < limits.over2 )		{ stats.push_back(ccomb); limits.over2=ccomb.over2;}
			else if(ccomb.over3 < limits.over3 )		{ stats.push_back(ccomb); limits.over3=ccomb.over3;}

		}	// end for(combinations)


		/* 'stats' now has a mix of the best combinations and some redundant results,
		 * but the amount of data we store should be much smaller now.
		 */

		// Print the combinations that were stored.
		cout << '\n' << "cID\tcov0\tcov1\tcov2\tcov3\tcov4\tcov5\tover1\tover2\tover3" << endl;
		for(vector<StatEntry>::iterator iterStat = stats.begin();
				iterStat != stats.end();
				iterStat++)
					cout << iterStat->cID << '\t'
						<< iterStat->cov0 << '\t'
						<< iterStat->cov1 << '\t'
						<< iterStat->cov2 << '\t'
						<< iterStat->cov3 << '\t'
						<< iterStat->cov4 << '\t'
						<< iterStat->cov5 << '\t'
						<< iterStat->over1 << '\t'
						<< iterStat->over2 << '\t'
						<< iterStat->over3 << endl;
	}	// end if(bruteforce)


	/* Print statistics about a specified combination.
	 */
	if(m_printCombination)
	{
		// Clone active RSUs from rsuList to rsuListActive
		vector<RSU> rsuListActive;	// vector required for [] access
		for(list<RSU>::iterator iterRSU = rsuList.begin(); iterRSU != rsuList.end(); iterRSU++)
			if(iterRSU->active) rsuListActive.push_back(*iterRSU);

		// Sort the list of RSUs by ID
		std::sort(rsuListActive.begin(), rsuListActive.end());

		/*	'cityCoverageSignal': Best signal level at each cell
		 * 	'cityCoverageCount'	: Number of RSUs covering each cell
		 */
		CityMapNum cityCoverageSignal, cityCoverageCount;

		// Tag maps and list RSUs
		cout << "cID " << m_printCombination << " has the following RSUs active: ";
		for (int i=0; i<24; i++)
			if ( ( (m_printCombination >> i) & 1) == 1)
			{
				// bit 'i' is a '1'
				// add car 'i' in 'listOfCars' to the maps
				applyCountToCityMap(rsuListActive[i].coverage, cityCoverageCount);
				applyCoverageToCityMap(rsuListActive[i].coverage, cityCoverageSignal);

				cout << rsuListActive[i].id << ' ';
			}
		cout << endl;

		// Compute statistics for this combination
		map<int,unsigned short> covStat = getCoverageStatistics(cityCoverageSignal);
		unsigned short over1 = getOvercoverageMetric(cityCoverageCount, 1);
		unsigned short over2 = getOvercoverageMetric(cityCoverageCount, 2);
		unsigned short over3 = getOvercoverageMetric(cityCoverageCount, 3);

		// Make a record of this combination's statistics.
		StatEntry ccomb;
		ccomb.cID = m_printCombination;
		ccomb.cov0 = covStat[0];
		ccomb.cov1 = covStat[1];
		ccomb.cov2 = covStat[2];
		ccomb.cov3 = covStat[3];
		ccomb.cov4 = covStat[4];
		ccomb.cov5 = covStat[5];
		ccomb.over1 = over1;
		ccomb.over2 = over2;
		ccomb.over3 = over3;

		// Print the combination statistics
		cout << '\n' << "cID\tcov0\tcov1\tcov2\tcov3\tcov4\tcov5\tover1\tover2\tover3" << endl;
		cout << ccomb.cID << '\t'
			<< ccomb.cov0 << '\t'
			<< ccomb.cov1 << '\t'
			<< ccomb.cov2 << '\t'
			<< ccomb.cov3 << '\t'
			<< ccomb.cov4 << '\t'
			<< ccomb.cov5 << '\t'
			<< ccomb.over1 << '\t'
			<< ccomb.over2 << '\t'
			<< ccomb.over3 << endl;

		// Mark the selected RSUs as 'R' in the map. Print routines replace '-1' with 'R'.
		for (int i=0; i<24; i++)
			if ( ( (m_printCombination >> i) & 1) == 1)
				cityCoverageSignal.map[rsuListActive[i].xcell][rsuListActive[i].ycell] = -1;

		// Print the signal map
		printCityMap(cityCoverageSignal);

		// Print the coverage map
		printCityMap(cityCoverageCount);

	}

	// Go through every vehicle position and see if it's not inside a building.
	if(m_validVehicle)
	{
		unsigned int bumpcount=0, clearcount=0;
		for(std::vector<Timestep>::iterator iter1=fcd_output.begin(); iter1 != fcd_output.end(); iter1++)
			for(std::vector<Vehicle>::iterator iter2=iter1->vehiclelist.begin(); iter2!=iter1->vehiclelist.end(); iter2++)
				if( GIS_isPointObstructed(conn,iter2->xgeo,iter2->ygeo) ) bumpcount++; else clearcount++;
		cout << "bump " << bumpcount << " clear " << clearcount << endl;
	}


	// Clear all POINT entities from the database from past simulations.
	// Uncomment to leave the GIS database in a clean state after the simulation.
//	GIS_clearAllPoints(conn, CAR_FEATTYP);
//	GIS_clearAllPoints(conn, RSU_FEATTYP);

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
	for(short dd=0;dd<CITYWIDTH;dd++)
		cout << "--"; cout << endl;
	for(short yy=0;yy<CITYHEIGHT;yy++)
	{
		for(short xx=0;xx<CITYWIDTH;xx++)
			cout << cmap.map[xx][yy] << ' ';
		cout << '\n';
	}
}

void printCityMap (CityMapNum cmap)
{
	for(short dd=0;dd<CITYWIDTH;dd++)
		cout << "--"; cout << endl;
	for(short yy=0;yy<CITYHEIGHT;yy++)
	{
		for(short xx=0;xx<CITYWIDTH;xx++)
			if(cmap.map[xx][yy]>0)
				cout << cmap.map[xx][yy] << ' ';
			else if(cmap.map[xx][yy]==-1)
				cout << "R ";
			else
				cout << "  ";
		cout << '\n';
	}
	for(short dd=0;dd<CITYWIDTH;dd++)
		cout << "--"; cout << endl;
}


void printLocalCoverage(CoverageMap coverage)
{
	for(short dd=0;dd<PARKEDCELLCOVERAGE;dd++)
		cout << "--"; cout << '\n';
	for(short yy=0;yy<PARKEDCELLCOVERAGE;yy++)
	{
		for(short xx=0;xx<PARKEDCELLCOVERAGE;xx++)
			if(coverage.map[xx][yy])
				cout << coverage.map[xx][yy] << ' ';
			else
				cout << "  ";
		cout << '\n';
	}
	for(short dd=0;dd<PARKEDCELLCOVERAGE;dd++)
		cout << "--"; cout << '\n';
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

map<int,unsigned short> getCoverageStatistics (CityMapNum cmap)
{
	// Initialize an STL map with coverage levels 0 through 5
	map<int,unsigned short> stats = {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}};

	// Go through each cell, register its coverage on 'stats'
	for(short yy=0; yy<CITYWIDTH; yy++)
		for(short xx=0; xx<CITYHEIGHT; xx++)
			stats[ cmap.map[yy][xx] ]++;	// e.g. if the cell has coverage 5, then stats[5]++

	return stats;
}

unsigned short getOvercoverageMetric (CityMapNum cmap, short cap)
{
	/* Arguments:
	 *		'cap': above this number, it counts as overcovered
	 */
	unsigned short metric=0;

	for(short yy=0; yy<CITYWIDTH; yy++)
		for(short xx=0; xx<CITYHEIGHT; xx++)
			if(cmap.map[yy][xx] > cap)
				metric += cmap.map[yy][xx]-cap;

	return metric;
}

void applyCoverageToCityMap (CoverageMap coverage, CityMapNum &city)
{
	for(short xx=0; xx<PARKEDCELLCOVERAGE; xx++)
		for(short yy=0; yy<PARKEDCELLCOVERAGE; yy++)
		{
			short mapX=xx+coverage.xcenter-PARKEDCELLRANGE;
			short mapY=yy+coverage.ycenter-PARKEDCELLRANGE;

			// 'upgrade' coverage in a given cell if this RSU can cover it better
			if(coverage.map[xx][yy] > city.map[mapX][mapY])
				city.map[mapX][mapY] = coverage.map[xx][yy];
		}
}

void applyCountToCityMap (CoverageMap coverage, CityMapNum &city)
{
	for(short xx=0; xx<PARKEDCELLCOVERAGE; xx++)
		for(short yy=0; yy<PARKEDCELLCOVERAGE; yy++)
		{
			short mapX=xx+coverage.xcenter-PARKEDCELLRANGE;
			short mapY=yy+coverage.ycenter-PARKEDCELLRANGE;

			// if a cell is covered, increment its coverage count
			if(coverage.map[xx][yy])
				city.map[mapX][mapY]++;
		}
}

bool decisionAlgorithm(RSU &rsu)
{
	// Returns whether the RSU should remain active (true) or not (false).

	/* Should be enough to work with the RSU's local neighbor maps.
	 * For more elaborate algorithms, pass the GIS connection and the list of RSUs.
	 */

	// Create a blank map
	CityMapNum rsuMap;

	// Place neighbor local coverage maps on the full map
	for(map<unsigned short, CoverageMap>::iterator iterNeighMap = rsu.neighborMaps.begin();
			iterNeighMap != rsu.neighborMaps.end();
			iterNeighMap++)
				applyCoverageToCityMap(iterNeighMap->second, rsuMap);

	// Count how many cells we cover that aren't covered already
	unsigned short exclusiveCoverage=0;
	for(short xx=0; xx<PARKEDCELLCOVERAGE; xx++)
		for(short yy=0; yy<PARKEDCELLCOVERAGE; yy++)
			if(rsu.coverage.map[xx][yy])	// if our RSU is covering this cell
				if(!rsuMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy])	// and if it's not covered by someone else already
					exclusiveCoverage++;

	// DEBUG2: print the city map and the local RSU map, and check count
	if(gm_debug>1)
	{
		cout << "DEBUG2 Exclusive coverage count: " << exclusiveCoverage << endl;
		cout << "DEBUG2 Local city map:" << endl;
		printCityMap(rsuMap);
		cout << "DEBUG2 Local RSU map:" << endl;
		printLocalCoverage(rsu.coverage);
	}

	// Decide
	// If > 10% of our cell coverage is exclusive to us, don't shut down.
	float percentExclusive = (float)exclusiveCoverage/(float)rsu.coveredCellCount;

	if(gm_debug) cout << "DEBUG decisionAlgorithm"
			<< " RSU id " << rsu.id
			<< " coveredCellCount " << rsu.coveredCellCount
			<< " exclusiveCoverage " << exclusiveCoverage
			<< " percentExclusive " << percentExclusive
			<< endl;

	if(percentExclusive > 0.10)
		return true;
	else
		return false;
}

bool pointDecisionAlgorithm(RSU &rsu)
{
	// Returns whether the RSU should remain active (true) or not (false).

	/* Points algorithm.
	 * Classify RSU quality.
	 * Covering a new cell: points equal to signal strength.
	 * Improving coverage: points equal to delta between old and new coverage.
	 * Adding redundancy: variable, 1 negative point per increase in redundancy >1.
	 *
	 * Consider limiting redundancy to >2, and increasing redundancy penalty.
	 */
	/* Two possibilities:
	 * - Current impact to the network, considering neighbors.
	 * - Value of the RSU when isolated.
	 */


	// Create a blank map
	CityMapNum signalMap;
	CityMapNum redundancyMap;

	// Place neighbor local coverage maps on the full map. Create a redundancy map too.
	for(map<unsigned short, CoverageMap>::iterator iterNeighMap = rsu.neighborMaps.begin();
			iterNeighMap != rsu.neighborMaps.end();
			iterNeighMap++)
			{
				applyCoverageToCityMap(iterNeighMap->second, signalMap);
				applyCountToCityMap(iterNeighMap->second, redundancyMap);
			}

	// Compute utility
	signed short utility=0;
//	signed short isolatedUtility=0;
	signed short debugPos=0, debugNeg=0;

	for(short xx=0; xx<PARKEDCELLCOVERAGE; xx++)
		for(short yy=0; yy<PARKEDCELLCOVERAGE; yy++)
			if(rsu.coverage.map[xx][yy]) // if we're covering this cell
			{
				// if a neighbor is covering the cell too and is worse than our coverage
				if(signalMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy] < rsu.coverage.map[xx][yy])
				{
					// add delta to the count
					utility += rsu.coverage.map[xx][yy] - signalMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy];
					debugPos += rsu.coverage.map[xx][yy] - signalMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy];
				}
				else
				{
					// just add our own coverage to the count
					utility += rsu.coverage.map[xx][yy];
					debugPos += rsu.coverage.map[xx][yy];
				}

				// Add redundancy penalty. If there's an RSU covering this area already, penalize.
				// Perhaps: square the new redundancy so as to more strongly penalize redundancy.
				// e.g.: going to 4 -> penalty 16
				if(redundancyMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy])
				{
					utility -= redundancyMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy];
					debugNeg += redundancyMap.map[rsu.xcell-PARKEDCELLRANGE+xx][rsu.ycell-PARKEDCELLRANGE+yy];
				}
			}

	// Update stats on the RSU
	rsu.utility = utility;
	rsu.utilPos = debugPos;
	rsu.utilNeg = debugNeg;

	if(gm_debug) cout << "DEBUG CLASSIFIER "
			<< " RSU id " << rsu.id
			<< " score " << utility
			<< " pos " << debugPos
			<< " neg " << debugNeg
			<< endl;

	if(utility>0)
		return true;
	else
		return false;
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
