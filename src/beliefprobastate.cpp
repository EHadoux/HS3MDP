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
	if(MH)
		delete[] MH;
}

void BELIEF_PROBA_STATE::Free(const SIMULATOR& simulator) {
	if(MH)
		delete[] MH;
	MH = 0;
	if( _uniqueState )
		simulator.FreeState(_uniqueState);
	_uniqueState = 0;
}

ENVIRONMENT_STATE* BELIEF_PROBA_STATE::CreateSample(const SIMULATOR& simulator) const {
	const ENVIRONMENT& env_sim   = safe_cast<const ENVIRONMENT&>(simulator);
	ENVIRONMENT_STATE *env_state = env_sim.Copy(*_uniqueState);
	double p                     = RandomDouble(0, 100);
	double cumsum                = MH[0];
	int i                        = 0;
	while( (cumsum <= p) && (cumsum < 100) ) {
		i++;
		cumsum += MH[i];
	}

	assert(MH[i] != 0);
	assert(i < (env_sim.GetMaxToStay() * env_sim.GetNumMDP()));

	env_state->MDPIndex   = i / env_sim.GetMaxToStay();
	env_state->timeToStay = i % env_sim.GetMaxToStay();
	env_state->stateIndex = _uniqueState->stateIndex;

	assert(env_state->MDPIndex >= 0 && env_state->MDPIndex < env_sim.GetNumMDP());

	return env_state;
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
