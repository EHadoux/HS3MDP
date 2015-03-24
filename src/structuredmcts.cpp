#include "structuredmcts.h"
#include "hs3mdp.h"
#include "structurednode.h"

STRUCTURED_MCTS::STRUCTURED_MCTS(const SIMULATOR& simulator, const PARAMS& params) :
MCTS(simulator, params) {
}

STRUCTURED_MCTS::STRUCTURED_MCTS(const STRUCTURED_MCTS& orig) :
MCTS(orig) {
}

STRUCTURED_MCTS::~STRUCTURED_MCTS() {
}

void STRUCTURED_MCTS::InitialiseRoot() {
    const HS3MDP& env_sim = safe_cast<const HS3MDP&>(Simulator);
    HS3MDP_STATE* startState = env_sim.CreateStartState();
    Root = ExpandNode(startState);
    STRUCTURED_BELIEF_STATE* PRoot = BeliefState();
    int numModes = env_sim.GetNumModes(), maxDuration = env_sim.GetMaxDuration();

    STRUCTURED_BELIEF_STATE::HS3MDPState = startState->HS3MDPState;
    PRoot->BeliefFunction.assign(numModes * maxDuration, 0);
    PRoot->BeliefFunction.shrink_to_fit();

    for( int i = 0; i < numModes; i++ )
        PRoot->BeliefFunction[i * maxDuration + 0] = 1.0 / numModes;
}

STRUCTURED_NODE* STRUCTURED_MCTS::ExpandNode(const STATE* state) {
    STRUCTURED_NODE* vnode = STRUCTURED_NODE::Create();
    vnode->Value.Set(0, 0);
    Simulator.Prior(state, History, vnode, Status);

    if( Params.Verbose >= 2 ) {
        cout << "Expanding node: ";
        History.Display(cout);
        cout << endl;
    }

    return vnode;
}

bool STRUCTURED_MCTS::Update(int action, int observation) {
    const HS3MDP& simulator = safe_cast<const HS3MDP&>(Simulator);
    int oldObs, numModes = simulator.GetNumModes(), maxDuration = simulator.GetMaxDuration();
    if( History.Size() > 0 )
        oldObs = History.Back().Observation;
    else {
        oldObs = HS3MDP::GetStartingHS3MDPStateIndex();
    }

    History.Add(action, observation);
    STRUCTURED_BELIEF_STATE *beliefs = new STRUCTURED_BELIEF_STATE();
    STRUCTURED_BELIEF_STATE *rootBeliefs = static_cast<STRUCTURED_BELIEF_STATE*> (Root->Beliefs());
    beliefs->BeliefFunction.reserve(numModes * maxDuration);

    STRUCTURED_BELIEF_STATE::HS3MDPState = observation;

    double sum = 0, pmh, pmm, pssam, msum, phmm;
    for( int mprime = 0; mprime < numModes; mprime++ ) {
        double init = simulator.GetInModeTransitionProbability(mprime, oldObs, action, observation);
        for( int hprime = 0; hprime < maxDuration; hprime++ ) {
            if( hprime + 1 < maxDuration )
                msum = init * rootBeliefs->BeliefFunction[mprime * maxDuration + hprime + 1];
            else
                msum = 0;

            for( int m = 0; m < numModes; m++ ) {
                pmm = simulator.GetModeTransitionProbability(m, mprime);
                pssam = simulator.GetInModeTransitionProbability(m, oldObs, action, observation);
                pmh = rootBeliefs->BeliefFunction[m * maxDuration + 0];
                phmm = simulator.GetDurationProbability(m, mprime, hprime);
                msum += pmm * pssam * pmh * phmm;
            }

            beliefs->BeliefFunction[mprime * maxDuration + hprime] = msum;
            sum += msum;
        }
    }

    assert(sum != 0);
    for( double& b : beliefs->BeliefFunction ) { b /= sum; }

    // Delete old tree and create new root
    STRUCTURED_NODE::Free(Root, simulator);
    STRUCTURED_NODE* newRoot = ExpandNode(beliefs->GetState());
    newRoot->Beliefs()->Free(simulator);
    delete newRoot->Beliefs();
    newRoot->Beliefs(beliefs);
    Root = newRoot;
    return true;
}