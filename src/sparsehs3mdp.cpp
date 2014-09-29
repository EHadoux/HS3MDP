#include "sparsehs3mdp.h"
#include <random>
#include <tuple>

using namespace std;

SPARSE_HS3MDP::SPARSE_HS3MDP(int numActions, int numObservations, int numModes, int maxDuration,
                             double discount, int seed) :
POMDP(numObservations * numModes * maxDuration, numActions, numObservations, discount, seed) {
    NumModes         = numModes;
    MaxDuration      = maxDuration;
    RewardDefault    = 0;
}

SPARSE_HS3MDP::SPARSE_HS3MDP(const SPARSE_HS3MDP& orig) :
POMDP(orig),
InModeTransitionProbabilites(orig.InModeTransitionProbabilites),
InModeRewards(orig.InModeRewards),
ModeTransitionProbabilities(orig.ModeTransitionProbabilities),
DurationProbabilities(orig.DurationProbabilities),
InModeRewardProbabilities(orig.InModeRewardProbabilities) {
    NumModes         = orig.NumModes;
    MaxDuration      = orig.MaxDuration;
    RewardDefault    = orig.RewardDefault;
}

SPARSE_HS3MDP::~SPARSE_HS3MDP() {
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
                for( auto a : *s ) {
                    delete get<0>(*a);
                    delete get<1>(*a);
                    delete get<2>(*a);
                    delete a;
                }
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
                for( auto a : *s ) {
                    delete get<0>(*a);
                    delete get<1>(*a);
                    delete get<2>(*a);
                    delete a;
                }
                delete s;
            }
            delete m;
        }
    }
}

double SPARSE_HS3MDP::GetInModeTransitionProbability(int m, int s, int a, int sprime) const {
    auto sparsemap = InModeTransitionProbabilites.at(m)->at(s)->at(a);
    auto map       = get<2>(*sparsemap);
    auto it        = map->find(sprime);
    if( it == map->end() )
        return 0;
    else
        return get<1>(*sparsemap)->at(it->second);
}

double SPARSE_HS3MDP::GetModeTransitionProbability(int m, int mprime) const {
    return ModeTransitionProbabilities.at(m)->at(mprime);
}

double SPARSE_HS3MDP::GetDurationProbability(int m, int mprime, int h) const {
    return DurationProbabilities.at(m)->at(mprime)->at(h);    
}

HS3MDP_STATE* SPARSE_HS3MDP::CreateStartState() const {
    HS3MDP_STATE* hs3mdpstate = MemoryPool.Allocate();
    uniform_int_distribution<> states(0, NumObservations - 1);
    hs3mdpstate->HS3MDPState  = states(Gen);
    hs3mdpstate->Mode         = 0;
    hs3mdpstate->Duration     = 0;
    hs3mdpstate->POMDPState   = hs3mdpstate->HS3MDPState;

    return hs3mdpstate;
}

HS3MDP_STATE* SPARSE_HS3MDP::Copy(const STATE& state) const {
    const HS3MDP_STATE& hs3mdpstate = safe_cast<const HS3MDP_STATE&>(state);
    HS3MDP_STATE* newstate          = MemoryPool.Allocate();
    newstate->POMDPState            = hs3mdpstate.POMDPState;
    newstate->HS3MDPState           = hs3mdpstate.HS3MDPState;
    newstate->Mode                  = hs3mdpstate.Mode;
    newstate->Duration              = hs3mdpstate.Duration;

    return newstate;
}

bool SPARSE_HS3MDP::Step(STATE& state, int action, int& observation, double& reward) const {
    HS3MDP_STATE& hs3mdpstate = safe_cast<HS3MDP_STATE&>(state);
    int index = hs3mdpstate.HS3MDPState, mode = hs3mdpstate.Mode, duration = hs3mdpstate.Duration;

    auto tmap                = InModeTransitionProbabilites.at(mode)->at(index)->at(action);
    auto tvector             = get<1>(*tmap);
    discrete_distribution<int> transitions(tvector->begin(), tvector->end());
    hs3mdpstate.HS3MDPState  = get<0>(*tmap)->at(transitions(Gen));

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
    
    auto rmap = InModeRewards.at(mode)->at(index);
    auto it   = rmap->find(action);
    if( it == rmap->end() )
        reward = RewardDefault;
    else
        reward = it->second;

    return false;
}
