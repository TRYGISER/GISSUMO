#include "network.h"

void processNetwork(pqxx::connection &c, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList)
{

	/* UVCAST approach:
	 * Go through each vehicle. If it's tagged as an SCF carrier, broadcast its packet to neighbors.
	 * UVCAST considers that vehicles advertise which emergency messages they already received in their
	 * hello packets, so if a destination neighbor already has our message, don't 'transmit' (no statistics).
	 */
	for(vector<Vehicle>::iterator iterVehicle=vehiclesOnGIS.begin(); iterVehicle!=vehiclesOnGIS.end(); iterVehicle++)
	{
		// we need to differentiate new broadcasts (source isn't an SCF) and run the gift-wrapping algorithm, from
		// messages received from an SCF (don't rebroadcast).
	}

}
