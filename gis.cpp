#include "gis.h"

void GIS_getPointCoords(pqxx::connection &c, unsigned short gid, float &xgeo, float &ygeo)
{
	pqxx::work txn(c);
	pqxx::result r = txn.exec(
			"SELECT ST_X(geom),ST_Y(geom) "
			"FROM edificios "
			"WHERE gid=" + pqxx::to_string(gid)
		);
	txn.commit();

	xgeo = r[0][0].as<float>();
	ygeo = r[0][1].as<float>();
}


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
			+ pqxx::to_string(y2) + ")',4326)) and feattyp='9790'"
	);
	txn.commit();

	// return false if line interesects, true otherwise
	if(r[0][0].as<int>() > 0) return false; else return true;
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

void GIS_updatePoint(pqxx::connection &c, float xx, float yy, unsigned short gid)
{
	pqxx::work txnUpdate(c);
	txnUpdate.exec(
			"UPDATE edificios SET geom=ST_GeomFromText('POINT("
			+ pqxx::to_string(xx) + " "
			+ pqxx::to_string(yy) + ")',4326) WHERE gid="
			+ pqxx::to_string(gid)
		);
	txnUpdate.commit();
}


void GIS_clearAllPoints(pqxx::connection &c)
{
	pqxx::work txn(c);
	txn.exec( "DELETE FROM edificios WHERE feattyp='2222'");
	txn.commit();
}


void addNewRSU(pqxx::connection &conn, std::list<RSU> &rsuList, unsigned short id, float xgeo, float ygeo, bool active)
{
	RSU testRSU;
	testRSU.id=id;	// building IDs start on #17779, through #35140
	testRSU.xgeo=xgeo;
	testRSU.ygeo=ygeo;
	testRSU.active=active;
	// check to see if the RSU is in a valid location
	if(GIS_isPointObstructed(conn,testRSU.xgeo,testRSU.ygeo))
		{ cerr << "ERROR: RSU is inside a building." << endl; exit(1); }
	// get cell coordinates from WGS84
	determineCellFromWGS84(testRSU.xgeo,testRSU.ygeo,testRSU.xcell,testRSU.ycell);
	// add RSU to GIS and get GIS unique id (gid)
	testRSU.gid = GIS_addPoint(conn,testRSU.xgeo,testRSU.ygeo,testRSU.id);
	// add RSU to list of RSUs
	rsuList.push_back(testRSU);
}


vector<Vehicle*> getVehiclesInRange(pqxx::connection &conn, list<Vehicle> &vehiclesOnGIS, const Vehicle src)
{
	/* Step 1: ask GIS for neighbors
	 * Step 2: match gid to Vehicle objects
	 * Step 3: get distance to neighbors, obstruction status
	 * Step 4: trim based on signal strength (<2 drop)
	 * Note that vehiclesOnGIS does not have RSUs.
	 */
	vector<Vehicle*> neighbors;
	vector<unsigned short> GISneighbors;

	// Step 1
	GISneighbors = GIS_getPointsInRange(conn,src.xgeo,src.ygeo,MAXRANGE);
	GISneighbors.erase(std::remove(GISneighbors.begin(), GISneighbors.end(), src.gid), GISneighbors.end() ); // drop ourselves from the list


	// Step 2
	for(vector<unsigned short>::iterator iter=GISneighbors.begin(); iter != GISneighbors.end(); iter++)
	{
		// find the vehicle by *iter
		list<Vehicle>::iterator iterVehicle = find_if(
				vehiclesOnGIS.begin(),
				vehiclesOnGIS.end(),
				boost::bind(&Vehicle::gid, _1) == *iter	// match Vehicle GID with GID from GIS
				);

		if(iterVehicle != vehiclesOnGIS.end())	// we can get to end() if the neighbor GID was an RSU
		{
			// Step 3
			// get the distance, obstruction, and signal to src
			unsigned short distance = GIS_distanceToPointGID(conn,src.xgeo,src.ygeo,iterVehicle->gid);
			bool isLineOfSight = GIS_isLineOfSight(conn,src.xgeo,src.ygeo,iterVehicle->xgeo,iterVehicle->ygeo);
			unsigned short signal = getSignalQuality(distance, isLineOfSight);

			// Step 4
			if(signal>=2)
				neighbors.push_back( &(*iterVehicle) ); // an iterator is not a pointer to an object. Dereference and rereference.
		}
	}


	if(m_debug)
	{
		cout << "DEBUG getVehiclesInRange valid neighbors " << neighbors.size() << '/' << GISneighbors.size()
					<< ", neighbors of " << src.id << ": " ;
		for(vector<Vehicle*>::iterator iter=neighbors.begin(); iter != neighbors.end(); iter++)
			cout << (*iter)->id << ' ';
		cout << endl;
	}

	return neighbors;
}

vector<Vehicle*> getVehiclesNearPoint(pqxx::connection &conn, list<Vehicle> &vehiclesOnGIS, const float xgeo, const float ygeo, const unsigned short range)
{
	/* Step 1: ask GIS for neighbors
	 * Step 2: match gid to Vehicle objects
	 * Note that vehiclesOnGIS does not have RSUs.
	 */
	vector<Vehicle*> neighbors;
	vector<unsigned short> GISneighbors;

	// Step 1
	GISneighbors = GIS_getPointsInRange(conn,xgeo,ygeo,range);

	// Step 2
	for(vector<unsigned short>::iterator iter=GISneighbors.begin(); iter != GISneighbors.end(); iter++)
	{
		// find the vehicle by *iter
		list<Vehicle>::iterator iterVehicle = find_if(
				vehiclesOnGIS.begin(),
				vehiclesOnGIS.end(),
				boost::bind(&Vehicle::gid, _1) == *iter	// match Vehicle GID with GID from GIS
				);

		if(iterVehicle != vehiclesOnGIS.end())	// we can get to end() if the neighbor GID was an RSU
			neighbors.push_back( &(*iterVehicle) ); // an iterator is not a pointer to an object. Dereference and rereference.
	}


	if(m_debug)
	{
		cout << "DEBUG getVehiclesNearPoint " << GISneighbors.size()
					<< " neighbors of point X=" << xgeo << ",Y=" << ygeo << " range " << range << " IDs " ;
		for(vector<Vehicle*>::iterator iter=neighbors.begin(); iter != neighbors.end(); iter++)
			cout << (*iter)->id << ' ';
		cout << endl;
	}

	return neighbors;
}
