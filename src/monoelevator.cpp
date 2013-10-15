#include "monoelevator.h"
#include "utils.h"
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>

#define _unused(x) ((void)x)

#define DOWN 0
#define UP 1
#define OPEN 2
#define UPTRAFFIC 0
#define DOWNTRAFFIC 1
#define BUSYTRAFFIC 2

using namespace std;
using namespace UTILS;

MONO_ELEVATOR::MONO_ELEVATOR(int numFloors, int maxToStay, bool original)
: ELEVATOR(numFloors, 1, maxToStay, original)
{}

MONO_ELEVATOR::MONO_ELEVATOR(const MONO_ELEVATOR& other)
: ELEVATOR(other)
{}

void MONO_ELEVATOR::FromObservation(int observation, int& floorIndex, vector<bool>& pickup, vector<bool>& dropoff) const {
	int numFloors = GetNumFloors();
	int oldObs = observation;

	pickup.assign(numFloors, false);
	dropoff.assign(numFloors, false);

	for( int i = numFloors - 1; i >= 0; i-- ) {
		pickup[i] = (observation & 0x01);
		observation >>= 1;
	}
	for( int i = numFloors - 1; i >= 0; i-- ) {
		dropoff[i] = (observation & 0x01);
		observation >>= 1;
	}

	floorIndex = observation;

	assert(ToObservation(floorIndex, pickup, dropoff) == oldObs);
}

int MONO_ELEVATOR::ToObservation(int floorIndex, vector<bool> pickup, vector<bool> dropoff) const {
	int observation = floorIndex;
	int numFloors = GetNumFloors();
	for( int f = 0; f < numFloors; f++ ) {
		observation <<= 1;
		observation |= dropoff[f];
	}

	for( int f = 0; f < numFloors; f++ ) {
		observation <<= 1;
		observation |= pickup[f];
	}

	return observation;
}

bool MONO_ELEVATOR::Step(STATE& state, int action, int& observation, double& reward) const
{
	ENVIRONMENT_STATE& State = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex           = State.stateIndex;
	int MDPIndex             = State.MDPIndex;
	int timeToStay           = State.timeToStay;
	int numFloors            = GetNumFloors();
	int floorIndex;
	vector<bool> pickup, dropoff;
	FromObservation(stateIndex, floorIndex, pickup, dropoff);

	reward = 0;

	for( int f = 0; f < numFloors; f++ ) {
		if( pickup[f] )
			reward -= 0.25;
		if( dropoff[f] )
			reward -= 0.25;
	}

	if( action == DOWN ) {
		if( floorIndex > 0 )
			floorIndex--;
	} else if( action == UP ) {
		if( floorIndex < (numFloors - 1))
			floorIndex++;
	} else if( action == OPEN) {
		if( pickup[floorIndex] ) {
			reward += 0.25;
			pickup[floorIndex] = false;

			if( floorIndex == 0 )
				dropoff[Random(numFloors-1) + 1] = true;
			else if( floorIndex == (numFloors - 1) )
				dropoff[Random(numFloors-1)] = true;
			else {
				int newfloor = Random(numFloors-1);
				if( newfloor >= floorIndex )
					newfloor++;
				dropoff[newfloor] = true;
			}
		}
		if( dropoff[floorIndex] ) {
			dropoff[floorIndex] = false;
			reward += 0.25;
		}
	}

	if( MDPIndex == UPTRAFFIC ) {
		for( int f = 1; f < numFloors; f++ ) {
			if( f == floorIndex && action == OPEN )
				continue;
			if(rand_01() < 0.10)
				pickup[f] = true;
		}
		if((rand_01() < 0.20) && !(floorIndex == 0 && action == OPEN))
			pickup[0] = true;
	} else if( MDPIndex == DOWNTRAFFIC ) {
		for( int f = (numFloors-1); f > 0; f-- ) {
			if( f == floorIndex && action == OPEN )
				continue;
			if(rand_01() < 0.20)
				pickup[f] = true;
		}
		if((rand_01() < 0.10) && !(floorIndex == 0 && action == OPEN))
			pickup[0] = true;
	} else if( MDPIndex == BUSYTRAFFIC ) {
		for( int f = 0; f < numFloors; f++ ) {
			if( f == floorIndex && action == OPEN )
				continue;
			if(rand_01() < 0.20)
				pickup[f] = true;
		}
	}

	observation      = ToObservation(floorIndex, pickup, dropoff);
	State.stateIndex = observation;
	NewModeAndTTS(State, timeToStay, MDPIndex);

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return false;
}

double MONO_ELEVATOR::GetTransition(int mdp, int oldObs, int action, int newObs) const {
	int numFloors = GetNumFloors();
	int floorIndex, newfloor;
	vector<bool> pickup, dropoff, newpickup, newdropoff;
	double ret = 1;

	FromObservation(oldObs, floorIndex, pickup, dropoff);
	FromObservation(newObs, newfloor, newpickup, newdropoff);

	switch(action) {
		case OPEN:
			if( floorIndex != newfloor )
				return 0;
			break;
		case UP:
			if( !(newfloor == (floorIndex+1) || (newfloor == floorIndex && floorIndex == (numFloors-1))))
				return 0;
			break;
		case DOWN:
			if( !(newfloor == (floorIndex-1) || (newfloor == floorIndex && floorIndex == 0)))
				return 0;
			break;
	}

	for( int f = 0; f < numFloors; f++ ) {
		if( action == OPEN ) {
			if( f == floorIndex ) {
				if( newpickup[f] )
					return 0;
				if( newdropoff[f] )
					return 0;
				continue;
			}

			if( !dropoff[f] ) {
				if( pickup[floorIndex] ) {
					if( !newdropoff[f] )
						ret *= 1 - 1.0 / (numFloors - 1);
					else
						ret *= 1.0 / (numFloors - 1);
				} else if( newdropoff[f] )
					return 0;
			} else if( !newdropoff[f] )
				return 0;

		} else
			if( dropoff[f] != newdropoff[f] )
				return 0;

		if( !pickup[f] ) {
			if( !newpickup[f] ) {
				if( ((mdp==UPTRAFFIC) && (f==0)) || ((mdp==DOWNTRAFFIC) && (f!=0)) || (mdp==BUSYTRAFFIC))
					ret *= 0.8;
				else
					ret *= 0.9;
			} else {
				if( ((mdp==UPTRAFFIC) && (f==0)) || ((mdp==DOWNTRAFFIC) && (f!=0)) || (mdp==BUSYTRAFFIC))
					ret *= 0.2;
				else
					ret *= 0.1;
			}
		} else
			if( !newpickup[f] )
				return 0;
	}

	return ret;
}

ostream& operator<<( ostream &flux, MONO_ELEVATOR const& elevator ) {
	elevator.toString(flux);
	return flux;
}
