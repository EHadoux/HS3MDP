#include "elevator.h"
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

ELEVATOR::ELEVATOR(int numFloors, int numActions, int numMDP)
: ENVIRONMENT(numActions, numFloors * pow(pow(2, numFloors), 2), numMDP)
{
	_numFloors = numFloors;
	_MDPTransitions = new double* [numMDP];
	_timeToStay = new double** [numMDP];

	for( int m = 0; m < numMDP; m++ ) {
		_MDPTransitions[m] = new double[numMDP];
		_timeToStay[m] = new double* [numMDP];

		for( int mprime = 0; mprime < numMDP; mprime++ ) {
			_MDPTransitions[m][mprime] = 5;
			_timeToStay[m][mprime] = createTimeToStay();
		}

		_MDPTransitions[m][m] = 100 - (numMDP - 1) * 5;
	}

	_startingStateIndex = 0;
}

double* ELEVATOR::createTimeToStay() {
	int maxToStay = GetMaxToStay();
	double *timeToStay = new double[maxToStay];
	double gaussienne[5] = {5, 25, 40, 25, 5};
	int mu = Random(maxToStay);

	for( int i = 0; i < maxToStay; i++ ) {
		if( (mu - 2) <= i && i <= (mu + 2) )
			timeToStay[i] = gaussienne[i - (mu - 2)];
		else
			timeToStay[i] = 0;
	}

	//Pour sommer a 1
	int cumIndex = 0;
	int cumSum = 0;
	while( cumIndex + mu - 2 < 0 ) {
		cumSum += gaussienne[cumIndex];
		cumIndex++;
	}
	timeToStay[0] += cumSum;

	cumIndex = 0;
	cumSum = 0;
	while( mu + 2 - cumIndex > (maxToStay-1) ) {
		cumSum += gaussienne[4 - cumIndex];
		cumIndex++;
	}
	timeToStay[maxToStay-1] += cumSum;
	//Pour sommer a 1

	return timeToStay;
}

ELEVATOR::ELEVATOR(const ELEVATOR& other)
: ENVIRONMENT(other)
{
	_MDPTransitions = other._MDPTransitions;
	_timeToStay = other._timeToStay;
	_numFloors = other._numFloors;
}

ELEVATOR::~ELEVATOR() {
	if(!isCopy()) {
		for( int m = 0; m < GetNumMDP(); m++ ) {
			delete[] _MDPTransitions[m];
			for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
				delete[] _timeToStay[m][mprime];
			delete[] _timeToStay[m];
		}

		delete[] _timeToStay;
		delete[] _MDPTransitions;
	}
}

void ELEVATOR::FromObservation(int observation, int& floorIndex, vector<bool>& pickup, vector<bool>& dropoff) const {
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

int ELEVATOR::ToObservation(int floorIndex, vector<bool> pickup, vector<bool> dropoff) const {
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

bool ELEVATOR::Step(STATE& state, int action, int& observation, double& reward) const
{
	ENVIRONMENT_STATE& env_state = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex = env_state.stateIndex;
	int MDPIndex = env_state.MDPIndex;
	int timeToStay = env_state.timeToStay;
	int numFloors = GetNumFloors();
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
			if(Random(100) < 10)
				pickup[f] = true;
		}
		if((Random(100) < 20) && !(floorIndex == 0 && action == OPEN))
			pickup[0] = true;
	} else if( MDPIndex == DOWNTRAFFIC ) {
		for( int f = (numFloors-1); f > 0; f-- ) {
			if( f == floorIndex && action == OPEN )
				continue;
			if(Random(100) < 20)
				pickup[f] = true;
		}
		if((Random(100) < 10) && !(floorIndex == 0 && action == OPEN))
			pickup[0] = true;
	} else if( MDPIndex == BUSYTRAFFIC ) {
		for( int f = 0; f < numFloors; f++ ) {
			if( f == floorIndex && action == OPEN )
				continue;
			if(Random(100) < 20)
				pickup[f] = true;
		}
	}

	observation = ToObservation(floorIndex, pickup, dropoff);
	env_state.stateIndex = observation;

	int p, cumsum, i;

	if( timeToStay > 0 )
		env_state.timeToStay = timeToStay - 1;
	else {
		p = Random(100) + 1;
		cumsum = _MDPTransitions[MDPIndex][0];
		i = 0;
		while( cumsum < p ) {
			i++;
			cumsum += _MDPTransitions[MDPIndex][i];
		}
		int newMDP = i;
		assert(_MDPTransitions[MDPIndex][newMDP] > 0);
		env_state.MDPIndex = newMDP;

		p = Random(100) + 1;
		cumsum = _timeToStay[MDPIndex][newMDP][0];
		i = 0;
		while( cumsum < p ) {
			i++;
			cumsum += _timeToStay[MDPIndex][newMDP][i];
		}
		env_state.timeToStay = i;
		assert(_timeToStay[MDPIndex][newMDP][i] > 0);
	}

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return false;
}

double ELEVATOR::GetTransition(int mdp, int oldObs, int action, int newObs) const {
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

	return ret * 100;
}

double ELEVATOR::GetMDPTransition(int oldmdp, int newmdp) const {
	return _MDPTransitions[oldmdp][newmdp];
}

double ELEVATOR::GetTimeToStay(int oldmdp, int newmdp, int h) const {
	return _timeToStay[oldmdp][newmdp][h];
}

ostream& ELEVATOR::toString( ostream &flux ) const
{
	flux << "NumStates: " << NumObservations << " NumActions: " << NumActions << " NumMDP: " << GetNumMDP() << endl;

	flux << "MDPTransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << m << ":";
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
			flux << setw(3) << _MDPTransitions[m][mprime] << " ";
		flux << endl;
	} flux << endl;

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
			flux << setw(3) << m << " -> " << setw(2) << mprime << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(3) << _timeToStay[m][mprime][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, ELEVATOR const& elevator ) {
	elevator.toString(flux);
	return flux;
}
