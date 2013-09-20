#include "traffic.h"
#include "utils.h"
#include <iomanip>
#include <cassert>

#define _unused(x) ((void)x)

using namespace std;
using namespace UTILS;

TRAFFIC::TRAFFIC(int maxToStay)
: ENVIRONMENT(2, 8, 2, maxToStay)
   {
   	int numMDP = GetNumMDP();
	_rewards = new double** [numMDP];

	_transitions = new double*** [numMDP];
	_MDPTransitions = new double* [numMDP];
	_timeToStay = new double** [numMDP];

	for( int m = 0; m < numMDP; m++ ) {
		_MDPTransitions[m] = new double[numMDP];
		_rewards[m] = createRewards();
		_timeToStay[m] = new double* [numMDP];
		_transitions[m] = new double** [NumObservations];

		for( int mprime = 0; mprime < numMDP; mprime++ )
			_timeToStay[m][mprime] = createTimeToStay();

		for( int o = 0; o < NumObservations; o++ ) {
			_transitions[m][o] = new double* [NumActions];
			for( int a = 0; a < NumActions; a++ ) {
				_transitions[m][o][a] = new double[NumObservations];
				for( int oprime = 0; oprime < NumObservations; oprime++ )
					_transitions[m][o][a][oprime] = 0;
			}
		}
	}

	_MDPTransitions[0][0] = _MDPTransitions[1][1] = 0.9;
	_MDPTransitions[1][0] = _MDPTransitions[0][1] = 0.1;

	int offset1 = 0, offset2 = 0, offset3 = 6;
	for( int m = 0; m < 2; m++ ) {
		if( m == 0 ) {
			offset1 = 4;
			offset2 = 2;
		}
		else {
			offset1 = 2;
			offset2 = 4;
		}
		for( int a = 0; a < 2; a++ ) {
			_transitions[m][0][a][a] = _transitions[m][1][a][a] = _transitions[m][3][a][a] = _transitions[m][4][a][a] = 0.63;
			_transitions[m][0][a][a+offset1] = _transitions[m][1][a][a+offset1] = _transitions[m][4][a][a+offset1] = _transitions[m][3][a][a+offset1] = 0.27;
			_transitions[m][0][a][a+offset2] = _transitions[m][1][a][a+offset2] = _transitions[m][4][a][a+offset2] = _transitions[m][3][a][a+offset2] = 0.07;
			_transitions[m][0][a][a+offset3] = _transitions[m][1][a][a+offset3] = _transitions[m][4][a][a+offset3] = _transitions[m][3][a][a+offset3] = 0.03;
		}
	}

	_transitions[0][5][0][4] = _transitions[0][7][0][4] = 0.90;
	_transitions[0][5][1][5] = _transitions[0][7][1][5] = 0.90;
	_transitions[1][2][0][2] = _transitions[1][6][0][2] = 0.90;
	_transitions[1][2][1][3] = _transitions[1][6][1][3] = 0.90;

	_transitions[0][2][0][2] = _transitions[0][6][0][2] = 0.70;
	_transitions[0][2][1][3] = _transitions[0][6][1][3] = 0.70;
	_transitions[1][5][0][4] = _transitions[1][7][0][4] = 0.70;
	_transitions[1][5][1][5] = _transitions[1][7][1][5] = 0.70;

	_transitions[0][2][0][6] = _transitions[0][6][0][6] = 0.30;
	_transitions[0][2][1][7] = _transitions[0][6][1][7] = 0.30;
	_transitions[1][5][0][6] = _transitions[1][7][0][6] = 0.30;
	_transitions[1][5][1][7] = _transitions[1][7][1][7] = 0.30;

	_transitions[0][5][0][6] = _transitions[0][7][0][6] = 0.10;
	_transitions[0][5][1][7] = _transitions[0][7][1][7] = 0.10;
	_transitions[1][2][0][6] = _transitions[1][6][0][6] = 0.10;
	_transitions[1][2][1][7] = _transitions[1][6][1][7] = 0.10;
}

double** TRAFFIC::createRewards() {
	double **rewards = new double* [NumObservations];
	for( int o = 0; o < NumObservations; o++ )
		rewards[o] = new double[NumActions];

	rewards[0][0] = rewards[1][0] = rewards[3][0] = rewards[4][0] = 0;
	rewards[0][1] = rewards[1][1] = rewards[3][1] = rewards[4][1] = 0;

	rewards[2][0] = rewards[5][0] = rewards[6][0] = rewards[7][0] = -1;
	rewards[2][1] = rewards[5][1] = rewards[6][1] = rewards[7][1] = -1;

	return rewards;
}

double* TRAFFIC::createTimeToStay() {
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

TRAFFIC::TRAFFIC(const TRAFFIC& other)
: ENVIRONMENT(other)
{
	// cppcheck-suppress copyCtorPointerCopying
	_rewards = other._rewards;
	// cppcheck-suppress copyCtorPointerCopying
	_transitions = other._transitions;
	// cppcheck-suppress copyCtorPointerCopying
	_MDPTransitions = other._MDPTransitions;
	// cppcheck-suppress copyCtorPointerCopying
	_timeToStay = other._timeToStay;
}

TRAFFIC::~TRAFFIC() {
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

bool TRAFFIC::Step(STATE& state, int action, int& observation, double& reward) const
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
		int newMDP = discrete_rand(_MDPTransitions[MDPIndex], GetNumMDP());
		State.MDPIndex = newMDP;
		assert(_MDPTransitions[MDPIndex][newMDP] > 0);

		State.timeToStay = discrete_rand(_timeToStay[MDPIndex][newMDP], GetMaxToStay());
		assert(_timeToStay[MDPIndex][newMDP][State.timeToStay] > 0);
	}

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return false;
}

double TRAFFIC::GetReward(int mdp, int obs, int action) const {
	return _rewards[mdp][obs][action];
}

double TRAFFIC::GetTransition(int mdp, int oldObs, int action, int newObs) const {
	return _transitions[mdp][oldObs][action][newObs];
}

double TRAFFIC::GetMDPTransition(int oldmdp, int newmdp) const {
	return _MDPTransitions[oldmdp][newmdp];
}

double TRAFFIC::GetTimeToStay(int oldmdp, int newmdp, int h) const {
	return _timeToStay[oldmdp][newmdp][h];
}

ostream& TRAFFIC::toString( ostream &flux ) const
{
	flux << "NumStates: " << NumObservations << " NumActions: " << NumActions << " NumMDP: " << GetNumMDP() << endl;
	flux << "CostMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << "MDP: " << m << endl;
		for( int i = 0; i < NumObservations; i++ ) {
			flux << " " << setw(2) << i << ": ";
			for( int a = 0; a < NumActions; a++ )
				flux << setw(2) << _rewards[m][i][a] << " ";
			flux << endl;
		}
		flux << endl;
	}

	flux << "TransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		for( int a = 0; a < NumActions; a++ ) {
			flux << "MDP: " << m << " action: " << a << endl;
			for( int o = 0; o < NumObservations; o++ ) {
				flux << " " << setw(4) << o << ": ";
				for( int oprime = 0; oprime < NumObservations; oprime++ )
					flux << setw(4) << _transitions[m][o][a][oprime] << " ";
				flux << endl;
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
	}

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int n = 0; n < GetNumMDP(); n++ ) {
			flux << setw(4) << m << " -> " << setw(2) << n << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(4) << _timeToStay[m][n][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, TRAFFIC const& traffic ) {
	traffic.toString(flux);
	return flux;
}
