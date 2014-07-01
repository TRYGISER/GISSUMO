#include "uvcast.h"

list<float> UVCAST_computeAngles(Vehicle src, Vehicle self, list<Vehicle> neighbors)
{
	list<float> angles;

	// compute slope with the Src vehicle
	unsigned short srcQuadrant = 0;
		 if( (src.xgeo>=self.xgeo) & (src.ygeo>=self.ygeo)) srcQuadrant=1;
	else if( (src.xgeo<self.xgeo) & (src.ygeo>=self.ygeo)) srcQuadrant=2;
	else if( (src.xgeo<self.xgeo) & (src.ygeo<self.ygeo)) srcQuadrant=3;
	else if( (src.xgeo>=self.xgeo) & (src.ygeo<self.ygeo)) srcQuadrant=4;

	float slopeSrc;

	if(self.xgeo != src.xgeo)
		slopeSrc = (self.ygeo - src.ygeo) / (self.xgeo - src.xgeo);
	else
		// TODO



	// go through our neighbors
	for(list<Vehicle>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
	{
		unsigned short neighQuadrant = 0;
			 if( (iter->xgeo>=self.xgeo) & (iter->ygeo>=self.ygeo)) neighQuadrant=1;
		else if( (iter->xgeo<self.xgeo) & (iter->ygeo>=self.ygeo)) neighQuadrant=2;
		else if( (iter->xgeo<self.xgeo) & (iter->ygeo<self.ygeo)) neighQuadrant=3;
		else if( (iter->xgeo>=self.xgeo) & (iter->ygeo<self.ygeo)) neighQuadrant=4;


		float slope;
		if(self.xgeo != iter->xgeo)
			slope = (self.ygeo - iter->ygeo) / (self.xgeo - iter->xgeo);
		else
			slope = 1000;	// TODO
	}


	return angles;
}



