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
