#include "network.h"

// Global statistics
unsigned int s_packetCount = 0;

void processNetwork(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList)
{

	/* UVCAST approach:
	 * Go through each vehicle. If it's tagged as an SCF carrier, broadcast its packet to neighbors.
	 * UVCAST considers that vehicles advertise which emergency messages they already received in their
	 * hello packets, so if a destination neighbor already has our message, don't 'transmit' (no statistics).
	 */
	for(vector<Vehicle>::iterator iterVehicle=vehiclesOnGIS.begin(); iterVehicle!=vehiclesOnGIS.end(); iterVehicle++)
	{
		// We need to differentiate new broadcasts (source isn't an SCF) and run the gift-wrapping algorithm, from
		// messages received from an SCF (don't rebroadcast).




		/* Vehicles assigned the SCF duty rebroadcast their message
		 */
		if(iterVehicle->scf)
			rebroadcastPacket(conn, vehiclesOnGIS, rsuList, *iterVehicle);

	}

}

void rebroadcastPacket(pqxx::connection &conn, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &veh)
{
	// Get our neighbor list. This routine already returns vehicles where communication is possible (signal>=2)
	vector<Vehicle> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, veh);

	// Go through each neighbor. If the packet isn't the same as ours, send our packet to them.
	for(vector<Vehicle>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
		if( iter->packet.m_id != veh.packet.m_id )
			{
				iter->packet = veh.packet;
				s_packetCount++;
			}
}
