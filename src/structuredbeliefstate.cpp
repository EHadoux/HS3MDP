#include "structuredbeliefstate.h"
#include "pomdp.h"

int STRUCTURED_BELIEF_STATE::HS3MDPState = 0;

STRUCTURED_BELIEF_STATE::STRUCTURED_BELIEF_STATE() :
BELIEF_STATE(){}

STRUCTURED_BELIEF_STATE::STRUCTURED_BELIEF_STATE(const STRUCTURED_BELIEF_STATE& orig) {
}

STRUCTURED_BELIEF_STATE::~STRUCTURED_BELIEF_STATE() {
}

HS3MDP_STATE* STRUCTURED_BELIEF_STATE::CreateSample(const SIMULATOR& simulator) const {
    std::cout << "test" << std::endl;
    const HS3MDP& Simulator = safe_cast<const HS3MDP&>(simulator);
    HS3MDP_STATE *State     = Simulator.Copy(*_uniqueState);
   
    int maxDuration = Simulator.GetMaxDuration();

    discrete_distribution<> dist(BeliefFunction.begin(), BeliefFunction.end());
    int i = dist(Simulator.GetGen());

    State->modeIndex  = i / maxDuration;
    State->duration   = i % maxDuration;
    State->stateIndex = STRUCTURED_BELIEF_STATE::HS3MDPState;

    return State;
}
