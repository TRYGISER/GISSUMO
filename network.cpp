#include "network.h"

// Global statistics
unsigned int s_packetCount = 0;
map<float,int> s_packetPropagationTime;
extern bool m_debug;
extern bool m_rsu;

void processNetwork(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, list<RSU> &rsuList)
{
	if(m_debug) cout << "DEBUG processNetwork" << " timestep " << timestep << " RSUs " << (m_rsu?"enabled":"disabled")<< endl;

	// begin packet propagation time collection
	s_packetPropagationTime[timestep]=0;

	/* UVCAST approach:
	 * Go through each vehicle. If it's tagged as an SCF carrier, broadcast its packet to neighbors.
	 * UVCAST considers that vehicles advertise which emergency messages they already received in their
	 * hello packets, so if a destination neighbor already has our message, don't 'transmit' (no statistics).
	 */
	// We need to differentiate new broadcasts (source isn't an SCF) and run the gift-wrapping algorithm, from
	// messages received from an SCF (don't rebroadcast).
	for(list<Vehicle>::iterator iterVehicle=vehiclesOnGIS.begin(); iterVehicle!=vehiclesOnGIS.end(); iterVehicle++)
		if(iterVehicle->scf)
			rebroadcastPacket(conn, timestep, vehiclesOnGIS, rsuList, *iterVehicle);


	/* RSUs with packets rebroadcast their message as well and trigger UVCAST (new source points).
	 */
	if(m_rsu)
		for(list<RSU>::iterator iterRSU=rsuList.begin(); iterRSU!=rsuList.end(); iterRSU++)
			if(iterRSU->packet.packetID)
			{
				/* TODO: RSUs and Vehicles should both be classes derived from
				 * the same main class with common characteristics, id, gid, xgeo, ygeo, active, etc.
				 */
				Vehicle rsuCar;
				rsuCar.xgeo = iterRSU->xgeo;
				rsuCar.ygeo = iterRSU->ygeo;
				rsuCar.id = iterRSU->id;
				rsuCar.gid = iterRSU->gid;
				rsuCar.packet = iterRSU->packet;
				rsuCar.packet.packetSrc = rsuCar.id; 	// required so that UVCAST isn't called on this vehicle
				initialBroadcast(conn, timestep, vehiclesOnGIS, rsuList, rsuCar, rsuCar);
	//			rebroadcastPacket(conn, timestep, vehiclesOnGIS, rsuList, rsuCar);
			}

	// All RSUs share the same packet. This could be improved.
	if(m_rsu)
		for(list<RSU>::iterator iterRSU=rsuList.begin(); iterRSU!=rsuList.end(); iterRSU++)
			if(iterRSU->packet.packetID)
				for(list<RSU>::iterator iterRSU2=rsuList.begin(); iterRSU2!=rsuList.end(); iterRSU2++)
					if(!iterRSU2->packet.packetID)
						iterRSU2->packet=iterRSU->packet;
}


void rebroadcastPacket(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, list<RSU> &rsuList, Vehicle &veh)
{
	// Get our neighbor list. This routine already returns vehicles where communication is possible (signal>=2)
	vector<Vehicle*> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, veh);

	// Go through each neighbor. If the packet isn't the same as ours, send our packet to them.
	for(vector<Vehicle*>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
		if( (*iter)->packet.packetID != veh.packet.packetID )
			{
				(*iter)->packet.packetID = veh.packet.packetID;
				(*iter)->packet.packetSrc = veh.id;
				(*iter)->packet.packetTime = timestep;
				s_packetCount++;
				s_packetPropagationTime[timestep]++;

				if(m_debug)
					cout << "DEBUG rebroadcastPacket"
							<< " from vID " << veh.id
							<< " to vID " << (*iter)->id
							<< endl;
			}

	if(m_rsu)
	{
		// get our RSU neighbor list
		vector<RSU*> RSUneighbors = getRSUsInRange(conn, rsuList, veh);
		// Go through each RSU. If the packet isn't the same as ours, send our packet to it.
		for(vector<RSU*>::iterator iter=RSUneighbors.begin(); iter!=RSUneighbors.end(); iter++)
			if( (*iter)->packet.packetID != veh.packet.packetID )
				{
					(*iter)->packet.packetID = veh.packet.packetID;
					(*iter)->packet.packetSrc = veh.id;
					(*iter)->packet.packetTime = timestep;
					s_packetCount++;
	//				s_packetPropagationTime[timestep]++;

					if(m_debug)
						cout << "DEBUG rebroadcastPacket RSU"
								<< " from vID " << veh.id
								<< " to vID " << (*iter)->id
								<< endl;
				}
	}
}

void simulateAccident(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, list<RSU> &rsuList, Vehicle &accidentSource)
{
	if(m_debug)
		cout << "DEBUG simulateAccident"
				<< " time " << timestep
				<< " srcID " << accidentSource.id
				<< endl;

//	// An accident begins at accidentSource. We get our neighbors.
//	vector<Vehicle*> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, accidentSource);
//
//	// No neighbors: not what we want to simulate, exit.
//	assert(neighbors.size()>0);

	// Give the source vehicle an emergency message.
	accidentSource.packet.packetID = EMERGENCYID;
	accidentSource.packet.packetSrc = accidentSource.id;
	accidentSource.packet.packetTime = timestep;

	// Get the message going
	initialBroadcast(conn, timestep, vehiclesOnGIS, rsuList, accidentSource, accidentSource);
}

void initialBroadcast(pqxx::connection &conn, float timestep, list<Vehicle> &vehiclesOnGIS, list<RSU> &rsuList, Vehicle &selfVeh, Vehicle &srcVeh)
{
	/* This is a recursive function.
	 * Make sure that the vehicle on the first call has a packet.
	 */
	if(m_debug)
		cout << "DEBUG initialBroadcast "
				<< " called by vID " << srcVeh.id
				<< " on vID " << selfVeh.id
				<< endl;

	// We get our neighbors.
	vector<Vehicle*> neighbors = getVehiclesInRange(conn, vehiclesOnGIS, selfVeh);

	// We broadcast the packet. Those who don't have the packet already get initialBroadcast() called on them too.
	for(vector<Vehicle*>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++)
		if((*iter)->packet.packetID != selfVeh.packet.packetID)
		{
			// Neighbor doesn't have our packet. Give it, and stat.
//			if(m_debug) cout << "\tinitialBroadcast giving packet to vID " << (*iter)->id << endl;
			(*iter)->packet.packetID = selfVeh.packet.packetID;
			(*iter)->packet.packetSrc = selfVeh.id;
			(*iter)->packet.packetTime = timestep;
			s_packetCount++;
			s_packetPropagationTime[timestep]++;

			// Do initialBroadcast on it.
			initialBroadcast(conn, timestep, vehiclesOnGIS, rsuList, **iter, selfVeh);
		}

	if(m_rsu)
	{
		/* Get RSU neighbors and pass the message on to them. Don't call InitialBroadcast on RSUs.
		 */
		// Get our RSU neighbor list
		vector<RSU*> RSUneighbors = getRSUsInRange(conn, rsuList, selfVeh);
		// Go through each RSU. If the packet isn't the same as ours, send our packet to it.
		for(vector<RSU*>::iterator iter=RSUneighbors.begin(); iter!=RSUneighbors.end(); iter++)
			if( (*iter)->packet.packetID != selfVeh.packet.packetID )
				{
					(*iter)->packet.packetID = selfVeh.packet.packetID;
					(*iter)->packet.packetSrc = selfVeh.id;
					(*iter)->packet.packetTime = timestep;
					s_packetCount++;
	//				s_packetPropagationTime[timestep]++;

					if(m_debug)
						cout << "DEBUG initialBroadcast RSU"
								<< " from vID " << selfVeh.id
								<< " to vID " << (*iter)->id
								<< endl;
				}
	}

//	printListOfVehicles(vehiclesOnGIS);

	// Call UVCAST and decide SCF function.
	// UVCAST doesn't work when neighbors < 3 (?)
	if(selfVeh.id != selfVeh.packet.packetSrc)	// the accident source's packet.m_src is itself, the others aren't.
	{
		if(neighbors.size()<2)	// If we only have 1 neighbor, that neighbor was the message source, and we're an isolated edge.
		{
			selfVeh.scf = true;
			if(m_debug) cout << "DEBUG UVCAST SCF true" << endl;
		}
		else
			selfVeh.scf = UVCAST_determineSCFtask(UVCAST_computeAngles(srcVeh, selfVeh, neighbors));
	}

}
