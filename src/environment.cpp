#include "environment.h"
#include "utils.h"
#include <cassert>
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>

#define _unused(x) ((void)x)

using namespace std;
using namespace UTILS;

ENVIRONMENT::ENVIRONMENT(int numActions, int numObservations, int numMDP, int maxToStay)
: SIMULATOR(numActions, numObservations, 0.9) {
	_numMDP             = numMDP;
	_startingStateIndex = Random(numObservations);
	_maxToStay          = maxToStay;
	_copy               = false;
}

ENVIRONMENT::ENVIRONMENT(const ENVIRONMENT& other)
: SIMULATOR(other.NumActions, other.NumObservations, 0.9)
{
	_numMDP             = other._numMDP;
	_startingStateIndex = other._startingStateIndex;
	_maxToStay          = other._maxToStay;
	_copy               = true;
}

ENVIRONMENT::~ENVIRONMENT() {}

ENVIRONMENT_STATE* ENVIRONMENT::Allocate() const {
	return MemoryPool.Allocate();
}

ENVIRONMENT_STATE* ENVIRONMENT::CreateStartState() const {
	ENVIRONMENT_STATE* state = MemoryPool.Allocate();
	state->stateIndex        = _startingStateIndex;
	state->MDPIndex          = Random(GetNumMDP());
	state->timeToStay        = 0;

	return state;
}

ENVIRONMENT_STATE* ENVIRONMENT::Copy(const STATE& state) const
{
	const ENVIRONMENT_STATE& env_state = safe_cast<const ENVIRONMENT_STATE&>(state);
	ENVIRONMENT_STATE* newstate        = MemoryPool.Allocate();
	*newstate                          = env_state;

	return newstate;
}

void ENVIRONMENT::FreeState(STATE* state) const
{
	ENVIRONMENT_STATE* env_state = safe_cast<ENVIRONMENT_STATE*>(state);
	MemoryPool.Free(env_state);
}


bool ENVIRONMENT::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const
{
	_unused(stepObs);
	_unused(status);

	ENVIRONMENT_STATE& env_state = safe_cast<ENVIRONMENT_STATE&>(state);
	if( env_state.stateIndex != history.Back().Observation )
		return false;
	return true;
}

void ENVIRONMENT::DisplayState(const STATE& state, std::ostream& ostr) const
{
	const ENVIRONMENT_STATE& env_state = safe_cast<const ENVIRONMENT_STATE&>(state);

	ostr << "State: " << env_state.stateIndex << " in MDP: " << env_state.MDPIndex  << " "
			<< env_state.timeToStay << " step to stay." << endl;
}

void ENVIRONMENT::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	_unused(state);
	ostr << "State: " << observation << " observed." << endl;
}

void ENVIRONMENT::DisplayAction(int action, std::ostream& ostr) const
{
	ostr << "Action: " << action << " observed." << endl;
}

bool ENVIRONMENT::TestTransitionsSumToOne() const {
	double sum;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int o = 0; o < NumObservations; o++ )
			for( int a = 0; a < NumActions; a++ ) {
				sum = 0;
				for( int oprime = 0; oprime < NumObservations; oprime++ )
					sum += GetTransition(m, o, a, oprime);
				if( abs(sum - 100.0) > 1e-10 )
					return false;
			}
	return true;
}

bool ENVIRONMENT::TestMDPSumToOne() const {
	for( int m = 0; m < GetNumMDP(); m++ ) {
		double sum = 0;
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
			sum += GetMDPTransition(m, mprime);
		if( abs(sum - 100.0) > 1e-10 )
			return false;
	}
	return true;
}

bool ENVIRONMENT::TestTimeToStaySumToOne() const {
	double sum;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
			sum = 0;
			for( int t = 0; t < GetMaxToStay(); t++ )
				sum += GetTimeToStay(m, mprime, t);
			if( abs(sum - 100.0) > 1e-10 )
				return false;
	}
	return true;
}

void ENVIRONMENT::TestConstructor() const {
	cout << "Starting constructor tests." << endl;
	assert(TestTransitionsSumToOne());
	cout << "State transitions sum to one." << endl;
	assert(TestMDPSumToOne());
	cout << "MDP transitions sum to one." << endl;
	assert(TestTimeToStaySumToOne());
	cout << "Time to stay sum to one." << endl;

	cout << "Tests constructeur passés." << endl;
}

void ENVIRONMENT::ToPOMDP( string filename ) const {
	ofstream f(filename.c_str());
	double MDPTransition = 0, timeToStay = 0;
	int numMDP           = GetNumMDP(), maxToStay = GetMaxToStay(), numObs = GetNumObservations();

	f << "discount: " << Discount << endl;
	f << "values: reward" << endl;
	f << "states: " << numMDP * maxToStay * numObs << endl;
	f << "actions: " << GetNumActions() << endl;
	f << "observations: " << numObs << endl << endl;
	for( int a = 0; a < GetNumActions(); a++ ) {
		f << "T: " << a << endl;
		for( int h = 0; h < maxToStay; h++ )
			for( int m = 0; m < numMDP; m++ )
				for( int o = 0; o < numObs; o++ ) {
					for( int hprime = 0; hprime < maxToStay; hprime++ )
						for( int mprime = 0; mprime < numMDP; mprime++ ) {
							MDPTransition = 0, timeToStay = 0;

							if( h == 0 ) {
								MDPTransition = GetMDPTransition(m, mprime) / 100;
								timeToStay = GetTimeToStay(m, mprime, hprime) / 100;
							} else if( hprime == (h-1) && m == mprime ) {
								MDPTransition = 1;
								timeToStay = 1;
							}

							for( int oprime = 0; oprime < numObs; oprime++ ) {
								f << GetTransition(m, o, a, oprime) / 100 * MDPTransition * timeToStay << " ";
							}
						}
					f << endl;
				}
		f << endl;
	}

	for( int i = 0; i < numMDP * maxToStay * numObs; i++ )
		f << "O : * : " << i << " : " << i % numObs << " " << 1 << endl;
	f << endl;

	for( int a = 0; a < GetNumActions(); a++ )
		for( int h = 0; h < GetMaxToStay(); h++ )
			for( int m = 0; m < GetNumMDP(); m++ )
				for( int o = 0; o < GetNumObservations(); o++ )
					if( GetReward(m, o, a) != 0 )
						f << "R: " << a << " : " << h * GetNumMDP() * GetNumObservations() + m * GetNumObservations() + o << " : * : * " << GetReward(m, o, a) << endl;
	f << endl;
	f.close();
}
