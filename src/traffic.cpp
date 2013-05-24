#include "traffic.h"
#include "utils.h"
#include <iomanip>
#include <cassert>

#define _unused(x) ((void)x)

using namespace std;
using namespace UTILS;

TRAFFIC::TRAFFIC()
: ENVIRONMENT(2, 8, 2)
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
			for( int a = 0; a < NumActions; a++ )
				_transitions[m][o][a] = new double[NumObservations];
		}
	}

	_MDPTransitions[0][0] = _MDPTransitions[1][1] = 90;
	_MDPTransitions[1][0] = _MDPTransitions[0][1] = 10;

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
			_transitions[m][0][a][a] = _transitions[m][1][a][a] = _transitions[m][3][a][a] = _transitions[m][4][a][a] = 63;
			_transitions[m][0][a][a+offset1] = _transitions[m][1][a][a+offset1] = _transitions[m][4][a][a+offset1] = _transitions[m][3][a][a+offset1] = 27;
			_transitions[m][0][a][a+offset2] = _transitions[m][1][a][a+offset2] = _transitions[m][4][a][a+offset2] = _transitions[m][3][a][a+offset2] = 7;
			_transitions[m][0][a][a+offset3] = _transitions[m][1][a][a+offset3] = _transitions[m][4][a][a+offset3] = _transitions[m][3][a][a+offset3] = 3;
		}
	}

	_transitions[0][5][0][4] = _transitions[0][7][0][4] = 90;
	_transitions[0][5][1][5] = _transitions[0][7][1][5] = 90;
	_transitions[1][2][0][2] = _transitions[1][6][0][2] = 90;
	_transitions[1][2][1][3] = _transitions[1][6][1][3] = 90;

	_transitions[0][2][0][2] = _transitions[0][6][0][2] = 70;
	_transitions[0][2][1][3] = _transitions[0][6][1][3] = 70;
	_transitions[1][5][0][4] = _transitions[1][7][0][4] = 70;
	_transitions[1][5][1][5] = _transitions[1][7][1][5] = 70;

	_transitions[0][2][0][6] = _transitions[0][6][0][6] = 30;
	_transitions[0][2][1][7] = _transitions[0][6][1][7] = 30;
	_transitions[1][5][0][6] = _transitions[1][7][0][6] = 30;
	_transitions[1][5][1][7] = _transitions[1][7][1][7] = 30;

	_transitions[0][5][0][6] = _transitions[0][7][0][6] = 10;
	_transitions[0][5][1][7] = _transitions[0][7][1][7] = 10;
	_transitions[1][2][0][6] = _transitions[1][6][0][6] = 10;
	_transitions[1][2][1][7] = _transitions[1][6][1][7] = 10;
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

TRAFFIC::TRAFFIC(const TRAFFIC& other)
: ENVIRONMENT(other)
{
	_rewards = other._rewards;
	_transitions = other._transitions;
	_MDPTransitions = other._MDPTransitions;
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
	ENVIRONMENT_STATE& env_state = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex = env_state.stateIndex;
	int MDPIndex = env_state.MDPIndex;
	int timeToStay = env_state.timeToStay;

	reward = _rewards[MDPIndex][stateIndex][action];
	int p = Random(100) + 1;
	int cumsum = _transitions[MDPIndex][stateIndex][action][0];
	int i = 0;
	while( cumsum < p ) {
		i++;
		cumsum += _transitions[MDPIndex][stateIndex][action][i];
	}

	env_state.stateIndex = i;
	observation = i;

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
				flux << " " << setw(2) << o << ": ";
				for( int oprime = 0; oprime < NumObservations; oprime++ )
					flux << setw(2) << _transitions[m][o][a][oprime] << " ";
				flux << endl;
			}
			flux << endl;
		}
		flux << endl;
	}

	flux << "MDPTransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << m << ":";
		for( int n = 0; n < GetNumMDP(); n++ )
			flux << setw(3) << _MDPTransitions[m][n] << " ";
		flux << endl;
	}

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int n = 0; n < GetNumMDP(); n++ ) {
			flux << setw(3) << m << " -> " << setw(2) << n << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(3) << _timeToStay[m][n][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, TRAFFIC const& traffic ) {
	traffic.toString(flux);
	return flux;
}