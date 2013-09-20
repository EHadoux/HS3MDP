#include "beliefprobastate.h"
#include "simulator.h"
#include "environment.h"
#include "utils.h"
#include <iostream>
#include <cassert>

using namespace std;
using namespace UTILS;

#define _unused(x) ((void)x)

BELIEF_PROBA_STATE::BELIEF_PROBA_STATE() {
	_uniqueState = 0;
	MH = 0;
}

BELIEF_PROBA_STATE::~BELIEF_PROBA_STATE() {
	if( MH )
		delete[] MH;
}

void BELIEF_PROBA_STATE::Free(const SIMULATOR& simulator) {
	if( MH )
		delete[] MH;
	MH = 0;
	if( _uniqueState )
		simulator.FreeState(_uniqueState);
	_uniqueState = 0;
}

ENVIRONMENT_STATE* BELIEF_PROBA_STATE::CreateSample(const SIMULATOR& simulator) const {
	const ENVIRONMENT& env   = safe_cast<const ENVIRONMENT&>(simulator);
	ENVIRONMENT_STATE *State = env.Copy(*_uniqueState);
	int i                    = 0, maxToStay = env.GetMaxToStay(), numMDP = env.GetNumMDP();

	mt19937_64 gen;
	discrete_distribution<> dist(MH, MH + maxToStay * numMDP);
	i = dist(gen);

	assert(MH[i] != 0);
	assert(i < (maxToStay * numMDP));

	State->MDPIndex   = i / maxToStay;
	State->timeToStay = i % maxToStay;
	State->stateIndex = _uniqueState->stateIndex;

	assert(State->MDPIndex >= 0 && State->MDPIndex < numMDP);

	return State;
}

void BELIEF_PROBA_STATE::AddSample(STATE* state) {
	_unused(state);
}

void BELIEF_PROBA_STATE::Copy(const BELIEF_STATE* beliefs, const SIMULATOR& simulator) {
	const BELIEF_PROBA_STATE* bs = safe_cast<const BELIEF_PROBA_STATE*>(beliefs);
	const ENVIRONMENT &Simulator = safe_cast<const ENVIRONMENT&>(simulator);
	_uniqueState                 = Simulator.Copy(*bs->_uniqueState);
}

void BELIEF_PROBA_STATE::Move(BELIEF_STATE* beliefs) {
	BELIEF_PROBA_STATE* bs = safe_cast<BELIEF_PROBA_STATE*>(beliefs);
	_uniqueState           = bs->_uniqueState;
	bs->_uniqueState       = 0;
}

void BELIEF_PROBA_STATE::SetState(ENVIRONMENT_STATE* state) {
	_uniqueState = state;
}

void BELIEF_PROBA_STATE::UnitTest() {
	double probas[] = {0.2, 0.4, 0.05, 0.3, 0.05};
	double count[] = {0, 0, 0, 0, 0};
	mt19937_64 gen;
	discrete_distribution<> dist(probas, probas + 5);

	for( int i = 0; i < 100000; i++ )
		count[dist(gen)]++;
	for( int i = 0; i < 5; i++ )
		assert(abs(probas[i] - (count[i] / 100000.0)) < 0.01);
}