#include "controled.h"
#include "utils.h"
#include <set>
#include <iomanip>
#include <cassert>

#define _unused(x) ((void)x)

using namespace std;
using namespace UTILS;

CONTROLED::CONTROLED(int numStates, int numActions, int numMDP, int maxToStay, bool original)
: ENVIRONMENT(numActions, numStates, numMDP, maxToStay, original)
{
	_rewards        = new double** [numMDP];
	_transitions    = new double*** [numMDP];
	_MDPTransitions = new double* [numMDP];
	_timeToStay     = new double** [numMDP];

	for( int m = 0; m < numMDP; m++ ) {
		_rewards[m]        = createRewards();
		_transitions[m]    = createTransitions();
		_MDPTransitions[m] = new double [numMDP];
		_timeToStay[m]     = new double* [numMDP];

		for( int mprime = 0; mprime < numMDP; mprime++ ) {
			_timeToStay[m][mprime]     = createTimeToStay();
			_MDPTransitions[m][mprime] = 0;
		}
		if( numMDP > 2 ) {
			_MDPTransitions[m][(m+1)%numMDP] = 0.5;
			_MDPTransitions[m][(m+2)%numMDP] = 0.25;
			_MDPTransitions[m][(m+3)%numMDP] = 0.25;
		} else
			_MDPTransitions[m][(m+1)%numMDP] = 1;
	}
}

double** CONTROLED::createRewards() {
	double **rewards = new double* [NumObservations];
	set<int> s;
	int toFeed = NumObservations/5, state;
	if(toFeed == 0)
		toFeed = 1;

	for( int o = 0; o < NumObservations; o++ ) {
		rewards[o] = new double[NumActions];
		for( int a = 0; a < NumActions; a++ )
			rewards[o][a] = 0;
	}

	for( int i = 0; i < toFeed; i++ ) {
		do {
			state = Random(NumObservations);
		} while(!s.insert(state).second);

		rewards[state][Random(NumActions)] = 1;
	}

	return rewards;
}

double*** CONTROLED::createTransitions() {
	double*** transitions = new double** [NumObservations];
	set<int> s;
	int toConnect = NumObservations/10, lim, proba, state;
	if(toConnect == 0)
		toConnect = 1;

	for( int o = 0; o < NumObservations; o++ ) {
		transitions[o] = new double* [NumActions];
		for( int a = 0; a < NumActions; a++ ) {
			transitions[o][a] = new double[NumObservations];
			for( int k = 0; k < NumObservations; k++ )
				transitions[o][a][k] = 0;

			lim = 100;
			state = -1;
			while(lim > 0 && s.size() < (unsigned int)(toConnect) ) {
				proba = Random(lim) + 1;
				lim   -= proba;
				do {
					state = Random(NumObservations);
				} while(!s.insert(state).second);
				assert(state != -1);
				transitions[o][a][state] = proba / 100.0;
			}
			if( lim > 0 )
				transitions[o][a][state] += lim / 100.0;
			s.clear();
		}
	}
	return transitions;
}

double* CONTROLED::createTimeToStay() {
	int maxToStay        = GetMaxToStay();
	double *timeToStay   = new double[maxToStay];
	double gaussienne[5] = {0.05, 0.25, 0.40, 0.25, 0.05};
	int mu               = Random(maxToStay);

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
	cumIndex      = 0;
	cumSum        = 0;

	while( mu + 2 - cumIndex > (maxToStay-1) ) {
		cumSum += gaussienne[4 - cumIndex];
		cumIndex++;
	}
	timeToStay[maxToStay-1] += cumSum;
	//Pour sommer a 1

	return timeToStay;
}

CONTROLED::CONTROLED(const CONTROLED& other)
: ENVIRONMENT(other)
{
	// cppcheck-suppress copyCtorPointerCopying
	_rewards        = other._rewards;
	// cppcheck-suppress copyCtorPointerCopying
	_transitions    = other._transitions;
	// cppcheck-suppress copyCtorPointerCopying
	_MDPTransitions = other._MDPTransitions;
	// cppcheck-suppress copyCtorPointerCopying
	_timeToStay     = other._timeToStay;
}

CONTROLED::~CONTROLED() {
	if(!isCopy()) {
		for( int m = 0; m < GetNumMDP(); m++ ) {
			for( int o = 0; o < NumObservations; o++ ) {
				delete[] _rewards[m][o];
				for( int a = 0; a < NumActions; a++ )
					delete[] _transitions[m][o][a];
				delete[] _transitions[m][o];
			}
			delete[] _rewards[m];
			delete[] _transitions[m];
			delete[] _MDPTransitions[m];
			for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
				delete[] _timeToStay[m][mprime];
			delete[] _timeToStay[m];
		}

		delete[] _rewards;
		delete[] _transitions;
		delete[] _timeToStay;
		delete[] _MDPTransitions;
	}
}

bool CONTROLED::Step(STATE& state, int action, int& observation, double& reward) const
{
	ENVIRONMENT_STATE& State = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex           = State.stateIndex;
	int MDPIndex             = State.MDPIndex;
	int timeToStay           = State.timeToStay;

	reward                   = GetReward(MDPIndex, stateIndex, action);
	observation              = discrete_rand(_transitions[MDPIndex][stateIndex][action], NumObservations);
	State.stateIndex         = observation;
	assert(_transitions[MDPIndex][stateIndex][action][observation] > 0);

	if( timeToStay > 0 )
		State.timeToStay = timeToStay - 1;
	else {
		int newMDP     = discrete_rand(_MDPTransitions[MDPIndex], GetNumMDP());
		State.MDPIndex = newMDP;
		assert(_MDPTransitions[MDPIndex][newMDP] > 0);

		State.timeToStay = discrete_rand(_timeToStay[MDPIndex][newMDP], GetMaxToStay());
		assert(_timeToStay[MDPIndex][newMDP][State.timeToStay] > 0);
	}

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return false;
}

double CONTROLED::GetReward(int mdp, int obs, int action) const {
	return _rewards[mdp][obs][action];
}

double CONTROLED::GetTransition(int mdp, int oldObs, int action, int newObs) const {
	return _transitions[mdp][oldObs][action][newObs];
}

double CONTROLED::GetMDPTransition(int oldmdp, int newMDP) const {
	return _MDPTransitions[oldmdp][newMDP];
}

double CONTROLED::GetTimeToStay(int oldmdp, int newMDP, int h) const {
	return _timeToStay[oldmdp][newMDP][h];
}

ostream& CONTROLED::toString( ostream &flux ) const
{
	flux << "NumStates: " << NumObservations << " NumActions: " << NumActions << " NumMDP: " << GetNumMDP() << endl;
	flux << "CostMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << "MDP: " << m << endl;
		for( int i = 0; i < NumObservations; i++ ) {
			flux << " " << setw(2) << i << ": ";
			for( int j = 0; j < NumActions; j++ )
				flux << _rewards[m][i][j] << " ";
			flux << endl;
		}
		flux << endl;
	}

	flux << "TransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << "MDP: " << m << endl;
		for( int i = 0; i < NumObservations; i++ ) {
			flux << " " << setw(2) << i << ": ";
			for( int j = 0; j < NumActions; j++ ) {
				for( int k = 0; k < NumObservations; k++ )
					flux << setw(4) << _transitions[m][i][j][k] << " ";
				flux << " | ";
			}
			flux << endl;
		}
		flux << endl;
	}

	flux << "MDPTransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << m << ": ";
		for( int n = 0; n < GetNumMDP(); n++ )
			flux << setw(4) << _MDPTransitions[m][n] << " ";
		flux << endl;
	} flux << endl;

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int n = 0; n < GetNumMDP(); n++ ) {
			flux << setw(3) << m << " -> " << setw(2) << n << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(4) << _timeToStay[m][n][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, CONTROLED const& controled ) {
	controled.toString(flux);
	return flux;
}
