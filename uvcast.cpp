#include "uvcast.h"

list<float> UVCAST_computeAngles(Vehicle src, Vehicle self, list<Vehicle> neighbors)
{
	list<float> angles;

	double srcAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;

	// go through our neighbors
	for(list<Vehicle>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
	{
		double neighAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;

		float deltaAngle = srcAngle-neighAngle;
		assert( (deltaAngle <= 180) && (deltaAngle >= -180) );

		angles.push_back(deltaAngle);
	}

	return angles;
}


bool UVCAST_determineSCFtask(list<float> angles)
{
	float min=0, max=0;

	for(list<float>::iterator iter=angles.begin(); iter!=angles.end(); iter++)
	{
		if(*iter<min) min=*iter;
		if(*iter>max) max=*iter;
	}
	assert(min<=0); assert(max>=0);

	if(max-min>180)
		return false;
	else
		return true;
}
