#include "uvcast.h"

vector<float> UVCAST_computeAngles(Vehicle src, Vehicle self, vector<vector<Vehicle>::iterator> neighbors)
{
	vector<float> angles;

	double srcAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;

	// go through our neighbors
	for(vector<vector<Vehicle>::iterator>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
	{
		double neighAngle = atan2(self.ygeo-src.ygeo, self.xgeo-src.xgeo) * 180 / PI;

		float deltaAngle = srcAngle-neighAngle;
		assert( (deltaAngle <= 180) && (deltaAngle >= -180) );

		angles.push_back(deltaAngle);
	}

	if(m_debug)
	{
		cout << "DEBUG UVCAST ANGLES"
				<< " srcid=" << src.id
				<< " dstid=" << self.id
				<< " neighbors=" << neighbors.size()
				<< '\n';
		cout << "\tangles: ";
		for(vector<float>::iterator iterAngles=angles.begin(); iterAngles!=angles.end(); iterAngles++)
			cout << *iterAngles << ' ';
		cout << endl;
	}


	return angles;
}


bool UVCAST_determineSCFtask(vector<float> angles)
{
	float min=0, max=0;

	for(vector<float>::iterator iter=angles.begin(); iter!=angles.end(); iter++)
	{
		if(*iter<min) min=*iter;
		if(*iter>max) max=*iter;
	}
	assert(min<=0); assert(max>=0);

	if(m_debug) cout << "DEBUG UVCAST SCF delta " << max-min << " SCF " << ( (max-min>180)?"true":"false" ) << endl;

	if(max-min>180)
		return false;
	else
		return true;
}
