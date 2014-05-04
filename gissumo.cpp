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

	cout << "DEBUG loading XML..." << flush;

	ptree tree;
	read_xml(XML_PATH, tree);
	const ptree & formats = tree.get_child("fcd-export", empty_ptree());

	cout << " done.\n";

    BOOST_FOREACH(const ptree::value_type &f, formats){
        string at = f.first + ".<xmlattr>";
        const ptree & attributes = f.second.get_child("<xmlattr>", empty_ptree());
        BOOST_FOREACH(const ptree::value_type &v, attributes){
            cout << at << ' ' << v.first.data() << '=' << v.second.data() << '\n';
        }
    }


	/* Init step 2: open a connection to PostgreSQL
	 * A password can be added to this string.
	 */
	pqxx::connection conn("dbname=shapefiledb user=abreis");

	/* Simulation starts here.
	 * 
	 *
	 */

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

	if(r[0][0].as<int>() > 0)
		return 1;
	else
		return 0;
}

