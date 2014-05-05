#include "gissumo.h"

#define XML_PATH "./fcdoutput.xml"

using namespace std;
using namespace boost;
using namespace boost::property_tree;

const ptree& empty_ptree(){
    static ptree t;
    return t;
}

int main(int, char *argv[])
{
	/* Init step 1: parse SUMO logs
	 * This expects SUMO's floating car data (FCD) output method with geographic
	 * coordinates (--fcd-output.geo=true)
	 * 
	 * The resulting file is XML, with syntax as follows:
	 * <fcd-export>
	 *  <timestep time="">
	 *   <vehicle id="" x="" y="" ... />
	 * 
	 * To parse, we're using boost propertytrees, which run on top of RapidXML,
	 * as ticpp kept spewing linker errors.
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
					v.x = iterVehicle.second.get<double>("<xmlattr>.x",0);
					v.y = iterVehicle.second.get<double>("<xmlattr>.y",0);
					v.speed = iterVehicle.second.get<double>("<xmlattr>.speed",0);

					t.vehiclelist.push_back(v);
				}
			} // end BOOST_FOREACH ( iterTimestep.second )

			// store this timestep entry
			fcd_output.push_back(t);

		} // end if(iterTimestep.first == "timestep")
	} // end BOOST_FOREACH ( "fcd-export" )

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


	/* Init step 2: open a connection to PostgreSQL
	 * A password can be added to this string.
	 */
	pqxx::connection conn("dbname=shapefiledb user=abreis");

	/* Simulation starts here.
	 * 
	 *
	 */

	if(isLineOfSight(conn,-8.620151,41.164420,-8.619759,41.164364) > 0) cout << "NLOS\n"; else cout << "LOS\n";
}


bool isLineOfSight (pqxx::connection &c, float x1, float y1, float x2, float y2)
{
	pqxx::work txn(c);
	
	// TODO
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

	if(r[0][0].as<int>() > 0)
		return 1;
	else
		return 0;
}

bool isPointObstructed(pqxx::connection &c, float xx, float yy)
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

	if(r[0][0].as<int>() > 0)
		return 1;
	else
		return 0;

}
