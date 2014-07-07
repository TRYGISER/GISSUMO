#include "network.h"

// Global statistics
unsigned int s_packetCount = 0;
map<float,int> s_packetPropagationTime;

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
			rebroadcastPacket(conn, timestep, vehiclesOnGIS, rsuList, *iterVehicle);

	}

}

void rebroadcastPacket(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &veh)
{
	// Get our neighbor list. This routine already returns vehicles where communication is possible (signal>=2)
	vector<vector<Vehicle>::iterator> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, veh);

	// Go through each neighbor. If the packet isn't the same as ours, send our packet to them.
	for(vector<vector<Vehicle>::iterator>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
		if( (*iter)->packet.m_id != veh.packet.m_id )
			{
				(*iter)->packet.m_id = veh.packet.m_id;
				(*iter)->packet.m_src = veh.id;
				(*iter)->packet.m_timestamp = timestep;
				s_packetCount++;
				s_packetPropagationTime[timestep]++;

				if(m_debug)
					cout << "DEBUG rebroadcastPacket"
							<< " from vID " << veh.id
							<< " to vID " << (*iter)->id
							<< endl;
			}
}

void simulateAccident(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &accidentSource)
{
	// An accident begins at accidentSource. We get our neighbors.
	vector<vector<Vehicle>::iterator> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, accidentSource);

	// No neighbors: not what we want to simulate, exit.
	assert(neighbors.size()>0);

	if(m_debug)
		cout << "DEBUG simulateAccident"
				<< " time " << timestep
				<< " srcID " << accidentSource.id
				<< " neighbors " << neighbors.size()
				<< endl;

	// Give the source vehicle an emergency message.
	accidentSource.packet.m_id = EMERGENCYID;
	accidentSource.packet.m_src = accidentSource.id;
	accidentSource.packet.m_timestamp = timestep;

	// Get the message going
	initialBroadcast(conn, timestep, vehiclesOnGIS, rsuList, accidentSource, accidentSource);
}

void initialBroadcast(pqxx::connection &conn, float timestep, vector<Vehicle> &vehiclesOnGIS, vector<RSU> &rsuList, Vehicle &srcVeh, Vehicle &packetSrcVeh)
{
	/* This is a recursive function.
	 * Make sure that the vehicle on the first call has a packet.
	 */
	if(m_debug)
		cout << "DEBUG initialBroadcast BEGIN"
				<< " on vID " << srcVeh.id
				<< " from vID " << packetSrcVeh.id
				<< endl;

	// We get our neighbors.
	vector<vector<Vehicle>::iterator> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, srcVeh);

	// We broadcast the packet. Those who don't have the packet already get initialBroadcast() called on them too.
	for(vector<vector<Vehicle>::iterator>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
	{
		cout << "MASS DEBUG"
				<< "\n\titer id " << (*iter)->id
				<< " iter packetID " << (*iter)->packet.m_id
				<< " iter packetSrc " << (*iter)->packet.m_src
				<< "\n\tsrcVeh id " << srcVeh.id
				<< " srcVeh packetID " << srcVeh.packet.m_id
				<< " srcVeh packetSrc " << srcVeh.packet.m_src
				<< endl;

		if((*iter)->packet.m_id != srcVeh.packet.m_id)
		{
			// Neighbor doesn't have our packet. Give it, and stat.
			(*iter)->packet.m_id = srcVeh.packet.m_id;
			(*iter)->packet.m_src = srcVeh.id;
			(*iter)->packet.m_timestamp = timestep;
			s_packetCount++;
			s_packetPropagationTime[timestep]++;

			if(m_debug)
				cout << "DEBUG initialBroadcast"
						<< " from vID " << srcVeh.id
						<< " to vID " << (*iter)->id
						<< endl;

			// Do initialBroadcast on it.
			initialBroadcast(conn, timestep, vehiclesOnGIS, rsuList, **iter, srcVeh);
		}
	}
	// Call UVCAST and decide SCF function here.
	// UVCAST doesn't work when neighbors < 3 (?)
	if(srcVeh.id != srcVeh.packet.m_src)	// the accident source's packet.m_src is itself, the others aren't.
	{
		if(neighbors.size()<2)
			srcVeh.scf = true;
		else
			srcVeh.scf = UVCAST_determineSCFtask(UVCAST_computeAngles(packetSrcVeh, srcVeh, neighbors));
	}

}
