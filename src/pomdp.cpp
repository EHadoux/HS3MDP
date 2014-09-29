#include "pomdp.h"

POMDP::POMDP(int numStates, int numActions, int numObservations, double discount, int seed) :
SIMULATOR(numActions, numObservations, discount),
Gen(seed) {
    NumStates = numStates;
    Copied    = false;
}

POMDP::POMDP(const POMDP& orig) :
SIMULATOR(orig),
Gen(orig.Gen),
TransitionProbabilities(orig.TransitionProbabilities),
ObservationProbabilities(orig.ObservationProbabilities),
Rewards(orig.Rewards),
RewardProbabilities(orig.RewardProbabilities) {
    NumStates = orig.NumStates;
    Copied    = true;
    RMin      = orig.RMin;
    RMax      = orig.RMax;
}

POMDP::~POMDP() {
    if( !IsCopy() ) {
        for( auto s : TransitionProbabilities ) {
            for( auto a : *s )
                delete a;
            delete s;
        }

        for( auto s : ObservationProbabilities ) {
            for( auto a : *s )
                delete a;
            delete s;
        }

        for( auto s : Rewards )
            delete s;

        for( auto s : RewardProbabilities ) {
            for( auto a : *s )
                delete a;
            delete s;
        }
    }
}

bool POMDP::Step(STATE& state, int action, int& observation, double& reward) const {
    POMDP_STATE& pomdpstate = safe_cast<POMDP_STATE&>(state);
    int index               = pomdpstate.POMDPState;

    auto ovector            = ObservationProbabilities.at(index)->at(action);
    discrete_distribution<int> observations(ovector->begin(), ovector->end());
    observation             = observations(Gen);

    reward                  = Rewards.at(index)->at(action);

    auto tvector            = TransitionProbabilities.at(index)->at(action);
    discrete_distribution<int> transitions(tvector->begin(), tvector->end());
    pomdpstate.POMDPState   = transitions(Gen);

    return false;
}

POMDP_STATE* POMDP::CreateStartState() const {
    POMDP_STATE* pomdpstate = MemoryPool.Allocate();
    uniform_int_distribution<> states(0, NumStates - 1);
    pomdpstate->POMDPState  = states(Gen);
    return pomdpstate;
}

void POMDP::FreeState(STATE* state) const {
    POMDP_STATE* pomdpstate = safe_cast<POMDP_STATE*>(state);
    MemoryPool.Free(pomdpstate);
}

POMDP_STATE* POMDP::Copy(const STATE& state) const {
    const POMDP_STATE& pomdpstate = safe_cast<const POMDP_STATE&>(state);
    POMDP_STATE* newstate         = MemoryPool.Allocate();
    newstate->POMDPState          = pomdpstate.POMDPState;

    return newstate;
}