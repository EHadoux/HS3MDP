#include "hs3mdp.h"
#include <random>

int HS3MDP::StartingHS3MDPStateIndex = 0;

HS3MDP::HS3MDP(int numActions, int numObservations, int numModes, int maxDuration,
               double discount, int seed) :
POMDP(numObservations * numModes * maxDuration, numActions, numObservations, discount, seed) {
    NumModes         = numModes;
    MaxDuration      = maxDuration;
    StructureAdapted = false;

    uniform_int_distribution<> states(0, NumObservations - 1);
    HS3MDP::StartingHS3MDPStateIndex = states(Gen);
}

HS3MDP::HS3MDP(const HS3MDP& orig) :
POMDP(orig),
InModeTransitionProbabilites(orig.InModeTransitionProbabilites),
InModeRewards(orig.InModeRewards),
ModeTransitionProbabilities(orig.ModeTransitionProbabilities),
DurationProbabilities(orig.DurationProbabilities),
InModeRewardProbabilities(orig.InModeRewardProbabilities) {
    NumModes         = orig.NumModes;
    MaxDuration      = orig.MaxDuration;
    StructureAdapted = orig.StructureAdapted;
}

HS3MDP::~HS3MDP() {
    if( !IsCopy() ) {
        for( auto m : ModeTransitionProbabilities )
            delete m;

        for( auto m : DurationProbabilities ) {
            for( auto mprime : *m )
                delete mprime;
            delete m;
        }

        for( auto m : InModeTransitionProbabilites ) {
            for( auto s : *m ) {
                for( auto a : *s )
                    delete a;
                delete s;
            }
            delete m;
        }

        for( auto m : InModeRewards ) {
            for( auto s : *m )
                delete s;
            delete m;
        }

        for( auto m : InModeRewardProbabilities ) {
            for( auto s : *m ) {
                for( auto a : *s )
                    delete a;
                delete s;
            }
            delete m;
        }
    }
}

HS3MDP_STATE* HS3MDP::CreateStartState() const {
    HS3MDP_STATE* hs3mdpstate     = MemoryPool.Allocate();
    if( !IsStructureAdapted() ) {
        uniform_int_distribution<>  states(0, NumObservations - 1);
        hs3mdpstate->HS3MDPState  = states(Gen);
    } else
        hs3mdpstate->HS3MDPState  = HS3MDP::GetStartingHS3MDPStateIndex();
    hs3mdpstate->Mode             = 0;
    hs3mdpstate->Duration         = 0;
    hs3mdpstate->POMDPState       = hs3mdpstate->HS3MDPState;

    return hs3mdpstate;
}

HS3MDP_STATE* HS3MDP::Copy(const STATE& state) const {
    const HS3MDP_STATE& hs3mdpstate = safe_cast<const HS3MDP_STATE&>(state);
    HS3MDP_STATE* newstate          = MemoryPool.Allocate();
    newstate->POMDPState            = hs3mdpstate.POMDPState;
    newstate->HS3MDPState           = hs3mdpstate.HS3MDPState;
    newstate->Mode                  = hs3mdpstate.Mode;
    newstate->Duration              = hs3mdpstate.Duration;

    return newstate;
}

bool HS3MDP::Step(STATE& state, int action, int& observation, double& reward) const {
    HS3MDP_STATE& hs3mdpstate = safe_cast<HS3MDP_STATE&>(state);
    int index = hs3mdpstate.HS3MDPState, mode = hs3mdpstate.Mode, duration = hs3mdpstate.Duration;

    auto tvector = InModeTransitionProbabilites.at(mode)->at(index)->at(action);
    discrete_distribution<int> transitions(tvector->begin(), tvector->end());
    hs3mdpstate.HS3MDPState = transitions(Gen);

    if( duration == 0 ) {
        auto mvector         = ModeTransitionProbabilities.at(mode);
        discrete_distribution<int> modetransitions(mvector->begin(), mvector->end());
        hs3mdpstate.Mode     = modetransitions(Gen);

        auto hvector         = DurationProbabilities.at(mode)->at(hs3mdpstate.Mode);
        discrete_distribution<int> durationfun(hvector->begin(), hvector->end());
        hs3mdpstate.Duration = durationfun(Gen);
    }

    hs3mdpstate.POMDPState = (hs3mdpstate.Duration * GetNumModes() + hs3mdpstate.Mode)
            * GetNumObservations() + hs3mdpstate.HS3MDPState;

    observation = hs3mdpstate.HS3MDPState;
    reward      = InModeRewards.at(mode)->at(index)->at(action);

    return false;
}
