#include "uvcast.h"

list<float> UVCAST_computeAngles(Vehicle src, Vehicle self, list<Vehicle> neighbors)
{
	list<float> angles;

	double srcAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;



	// go through our neighbors
	for(list<Vehicle>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
	{
		double neighAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;

	}


	return angles;
}



