#include "elevator.h"
#include "utils.h"
#include <iomanip>
#include <cmath>
#include <vector>
#include <cassert>
#include <iostream>
#include <map>

#define _unused(x) ((void)x)

#define DOWN 0
#define UP 1
#define OPEN 2
#define UPTRAFFIC 0
#define DOWNTRAFFIC 1
#define BUSYTRAFFIC 2

using namespace std;
using namespace UTILS;

ELEVATOR::ELEVATOR(int numFloors, int numElevators, int maxToStay)
: ENVIRONMENT(pow(3, numElevators), pow(numFloors * pow(2, numFloors), numElevators) * pow(2, numFloors), 3, maxToStay)
{
	_numFloors = numFloors;
	_numElevators = numElevators;
	int numMDP = 3;
	_MDPTransitions = new double* [numMDP];
	_timeToStay = new double** [numMDP];

	for( int m = 0; m < numMDP; m++ ) {
		_MDPTransitions[m] = new double[numMDP];
		_timeToStay[m] = new double* [numMDP];

		for( int mprime = 0; mprime < numMDP; mprime++ ) {
			_MDPTransitions[m][mprime] = 0.45;
			_timeToStay[m][mprime] = createTimeToStay();
		}

		_MDPTransitions[m][m] = 0.10;
	}
}

double* ELEVATOR::createTimeToStay() {
	int maxToStay = GetMaxToStay();
	double *timeToStay = new double[maxToStay];
	double gaussienne[5] = {0.05, 0.25, 0.40, 0.25, 0.05};
	int mu = Random(maxToStay);

	for( int i = 0; i < maxToStay; i++ ) {
		if( (mu - 2) <= i && i <= (mu + 2) )
			timeToStay[i] = gaussienne[i - (mu - 2)];
		else
			timeToStay[i] = 0;
	}

	//Pour sommer a 1
	int cumIndex  = 0;
	double cumSum = 0;
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
	// cppcheck-suppress copyCtorPointerCopying
	_MDPTransitions = other._MDPTransitions;
	// cppcheck-suppress copyCtorPointerCopying
	_timeToStay = other._timeToStay;
	_numFloors = other._numFloors;
	_numElevators = other._numElevators;
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

ENVIRONMENT_STATE* ELEVATOR::CreateStartState() const {
	ENVIRONMENT_STATE* state = ENVIRONMENT::CreateStartState();
	state->stateIndex        = 0;

	return state;
}

void ELEVATOR::FromObservation(int observation, vector<int>& floorIndex, vector<bool>& pickup, vector<vector<bool> >& dropoff) const {
	int numFloors = GetNumFloors();
	int numElevators = GetNumElevators();
	int oldObs = observation;

	floorIndex.assign(numElevators, 0);
	pickup.assign(numFloors, false);

	for( int e = 0; e < numElevators; e++ ) {
		dropoff[e].assign(numFloors, false);

		for( int f = numFloors - 1; f >= 0; f-- ) {
			dropoff[e][f] = (observation & 0x01);
			observation >>= 1;
		}
	}

	for( int f = numFloors - 1; f >= 0; f-- ) {
			pickup[f] = (observation & 0x01);
			observation >>= 1;
	}

	for( int e = 0; e < numElevators; e++ ) {
		//Dependent of numFloors
		floorIndex[e] = (observation & 0x03);
		observation >>= 2;
	}

	assert(ToObservation(floorIndex, pickup, dropoff) == oldObs);
}

int ELEVATOR::ToObservation(const vector<int> &floorIndex, const vector<bool> &pickup, const vector<vector<bool> > &dropoff) const {
	int observation = floorIndex.back();
	//Dependent of numFloors
	for( int e = floorIndex.size() - 2; e >= 0; e-- ) {
		observation <<= 2;
		observation |= floorIndex[e];
	}
	int numFloors = GetNumFloors();

	for( int f = 0; f < numFloors; f++ ) {
		observation <<= 1;
		observation |= pickup[f];
	}

	for( int e = floorIndex.size() - 1; e >= 0; e-- ) {
		for( int f = 0; f < numFloors; f++ ) {
			observation <<= 1;
			observation |= dropoff[e][f];
		}
	}

	return observation;
}

int ELEVATOR::GetAction(int action, int elevatorNumber) const {
	for( int e = 0; e < elevatorNumber; e++ )
		action /= 3;
	return action % 3;
}

bool ELEVATOR::Step(STATE& state, int action, int& observation, double& reward) const
{
	ENVIRONMENT_STATE& State = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex = State.stateIndex;
	int MDPIndex = State.MDPIndex;
	int timeToStay = State.timeToStay;
	int numFloors = GetNumFloors();
	int numElevators = GetNumElevators();
	int currentAction, newfloor;
	vector<int> floorIndex;
	vector<vector<bool> > dropoff;
	vector<bool> templateVector, pickup;

	dropoff.assign(numElevators, templateVector);
	FromObservation(stateIndex, floorIndex, pickup, dropoff);

	reward = 0;

	for( int f = 0; f < numFloors; f++ )
		if( pickup[f] )
			reward -= 0.25;

	for( int e = 0; e < numElevators; e++ ) {
		currentAction = GetAction(action, e);

		for( int f = 0; f < numFloors; f++ )
			if( dropoff[e][f] )
				reward -= 0.25;

		if( currentAction == DOWN ) {
			if( floorIndex[e] > 0 )
				floorIndex[e]--;
		} else if( currentAction == UP ) {
			if( floorIndex[e] < (numFloors - 1))
				floorIndex[e]++;
		} else if( currentAction == OPEN) {
			if( dropoff[e][floorIndex[e]] ) {
				dropoff[e][floorIndex[e]] = false;
				reward += 0.25;
			}

			if( pickup[floorIndex[e]] ) {
				reward += 0.25;
				pickup[floorIndex[e]] = false;

				if( floorIndex[e] == 0 )
					newfloor = Random(numFloors-1) + 1;
				else if( floorIndex[e] == (numFloors - 1) )
					newfloor = Random(numFloors-1);
				else {
					newfloor = Random(numFloors-1);
					if( newfloor >= floorIndex[e] )
						newfloor++;

				}
				dropoff[e][newfloor] = true;
			}
		}
	}

	double r;
	for( int f = 0; f < numFloors; f++ ) {
		bool opened = false;
		for( int e = 0; e < numElevators; e++ ) {
			currentAction = GetAction(action, e);

			if( currentAction == OPEN && f == floorIndex[e] ) {
				pickup[f] = false;
				opened = true;
				continue;
			}
			if( !opened ) {
				r = rand_01();
				if((((MDPIndex == UPTRAFFIC && f == 0 ) || (MDPIndex == DOWNTRAFFIC && f != 0) || MDPIndex == BUSYTRAFFIC) && r < 0.20) || r < 0.10)
						pickup[f] = true;
			}
		}
	}

	observation      = ToObservation(floorIndex, pickup, dropoff);
	State.stateIndex = observation;
	NewModeAndTTS(State, timeToStay, MDPIndex);

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return false;
}

void ELEVATOR::NewModeAndTTS(ENVIRONMENT_STATE& State, int timeToStay, int MDPIndex) const {
	if( timeToStay > 0 )
		State.timeToStay = timeToStay - 1;
	else {
		int newMDP     = discrete_rand(_MDPTransitions[MDPIndex], GetNumMDP());
		State.MDPIndex = newMDP;
		assert(_MDPTransitions[MDPIndex][newMDP] > 0);

		State.timeToStay = discrete_rand(_timeToStay[MDPIndex][newMDP], GetMaxToStay());
		assert(_timeToStay[MDPIndex][newMDP][State.timeToStay] > 0);
	}
}

double ELEVATOR::GetReward(int mdp, int obs, int action) const {
	_unused(mdp);
	_unused(obs);
	_unused(action);
	assert(false);
}

double ELEVATOR::GetTransition(int mdp, int oldObs, int action, int newObs) const {
	int numFloors = GetNumFloors(), currentAction, numElevators = GetNumElevators();
	vector<int> floorIndex, newfloor;
	vector<bool> templateVector, pickup, newpickup;
	map<int, int> openedfloor;
	vector<vector<bool> > dropoff, newdropoff;
	double ret = 1;

	dropoff.assign(numElevators, templateVector);
	newdropoff.assign(numElevators, templateVector);

	FromObservation(oldObs, floorIndex, pickup, dropoff);
	FromObservation(newObs, newfloor, newpickup, newdropoff);

	for( int e = 0; e < numElevators; e++ ) {
		currentAction = GetAction(action, e);

		switch(currentAction) {
			case OPEN:
				if( openedfloor.find(floorIndex[e]) == openedfloor.end() )
					openedfloor[floorIndex[e]] = e;
				if( floorIndex[e] != newfloor[e] )
					return 0;
				break;
			case UP:
				if( !(newfloor[e] == (floorIndex[e]+1) || (newfloor[e] == floorIndex[e] && floorIndex[e] == (numFloors-1))))
					return 0;
				break;
			case DOWN:
				if( !(newfloor[e] == (floorIndex[e]-1) || (newfloor[e] == floorIndex[e] && floorIndex[e] == 0)))
					return 0;
				break;
		}
	}

	map<int,int>::iterator it;
	for( int e = 0; e < numElevators; e++ ) {
		it = openedfloor.find(floorIndex[e]);
		bool otheropenedbefore = (it != openedfloor.end() && it->second != e);
		currentAction = GetAction(action, e);

		if( currentAction == OPEN ) {
			if( newpickup[floorIndex[e]] )
				return 0;
			if( newdropoff[e][floorIndex[e]] )
				return 0;

			for( int f = 0; f < numFloors; f++ ) {
				if( f == floorIndex[e] )
					continue;

				if( otheropenedbefore && !dropoff[e][f] && newdropoff[e][f] )
					return 0;

				if( !(dropoff[e][f]) ) {
					if( pickup[floorIndex[e]] ) {
						if( !otheropenedbefore ) {
							if( !(newdropoff[e][f])  )
								ret *= 1 - 1.0 / (numFloors - 1);
							else
								ret *= 1.0 / (numFloors - 1);
						}
					} else if( newdropoff[e][f] )
						return 0;
				} else if( !(newdropoff[e][f]) )
					return 0;
			}
		} else
			for( int f = 0; f < numFloors; f++ )
				if( dropoff[e][f] != newdropoff[e][f] )
					return 0;
	}

	for( int f = 0; f < numFloors; f++ ) {
		if( !pickup[f] ) {
			if( openedfloor.find(f) != openedfloor.end() )
				continue;

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
			if( !newpickup[f] && openedfloor.find(f) == openedfloor.end() )
				return 0;
	}

	return ret;
}

double ELEVATOR::GetMDPTransition(int oldmdp, int newmdp) const {
	return _MDPTransitions[oldmdp][newmdp];
}

double ELEVATOR::GetTimeToStay(int oldmdp, int newmdp, int h) const {
	return _timeToStay[oldmdp][newmdp][h];
}

ostream& ELEVATOR::toString( ostream &flux ) const
{
	flux << "NumStates: " << NumObservations << " NumActions: " << NumActions << " NumMDP: " << GetNumMDP() << " NumElevators: " << GetNumElevators() << endl;

	flux << "MDPTransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << m << ": ";
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
			flux << setw(4) << _MDPTransitions[m][mprime] << " ";
		flux << endl;
	} flux << endl;

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
			flux << setw(3) << m << " -> " << setw(2) << mprime << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(4) << _timeToStay[m][mprime][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, ELEVATOR const& elevator ) {
	elevator.toString(flux);
	return flux;
}
