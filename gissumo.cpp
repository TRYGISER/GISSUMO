#include <iostream>
#include <pqxx/pqxx>
#include "tinyxml.h"
#include "tinystr.h"

using namespace std;
using namespace pqxx;
bool isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

int main(int, char *argv[])
{
	/* step 1: parse SUMO logs
	 * This expects SUMO's floating car data (FCD) output method with geographic
	 * coordinates (--fcd-output.geo=true)
	 * 
	 * The resulting file is XML, with syntax as follows:
	 * 
	 * To parse, we're using TinyXML++, or 'ticpp'.
	 */

	// this opens the connection to postgres
	pqxx::connection conn("dbname=shapefiledb user=abreis");

	if(isLineOfSight(conn,-40910,166245,-40810,166230) > 0) cout << "NLOS\n"; else cout << "LOS\n";
}


bool isLineOfSight (pqxx::connection &c, float x1, float y1, float x2, float y2)
{
        pqxx::work txn(c);
	
	pqxx::result r = txn.exec(
		"SELECT COUNT(id) "
		"FROM edificios "
		"WHERE ST_Intersects(geom, ST_GeomFromText('LINESTRING(-40910 166245,-40810 166230)',27492))"
	);
	txn.commit();

	cout << "test " << r[0][0].as<int>() << endl;

	if(r[0][0].as<int>() > 0)
		return 1;
	else
		return 0;
}

