#include "probamcts.h"
#include "beliefprobastate.h"
#include "environment.h"
#include <iostream>
#include <cassert>
#include <iomanip>

using namespace std;

PROBA_MCTS::PROBA_MCTS(const SIMULATOR& simulator, const PARAMS& params)
: MCTS(simulator, params) {}

void PROBA_MCTS::InitialiseRoot() {
	const ENVIRONMENT& env_sim    = safe_cast<const ENVIRONMENT&>(Simulator);
	ENVIRONMENT_STATE* startState = env_sim.CreateStartState();
	Root                          = ExpandNode(startState);
	BELIEF_PROBA_STATE* PRoot     = safe_cast<BELIEF_PROBA_STATE*>(Root->Beliefs());

	PRoot->SetState(startState);
	int size  = env_sim.GetNumMDP() * env_sim.GetMaxToStay();
	PRoot->MH.resize(size);
	for( int i = 0; i < size; i++ )
		PRoot->MH[i] = 100.0 / size;
}

PROBA_VNODE* PROBA_MCTS::ExpandNode(const STATE* state)
{
	const ENVIRONMENT& simulator = safe_cast<const ENVIRONMENT&>(Simulator);
	PROBA_VNODE* vnode           = PROBA_VNODE::Create();
	vnode->Value.Set(0, 0);
	simulator.Prior(state, History, vnode, Status);

	if (Params.Verbose >= 2)
	{
		cout << "Expanding node: ";
		History.Display(cout);
		cout << endl;
	}

	return vnode;
}

bool PROBA_MCTS::Update(int action, int observation) {
	const ENVIRONMENT& simulator = safe_cast<const ENVIRONMENT&>(Simulator);
	int oldObs, numMDP           = simulator.GetNumMDP(), maxToStay = simulator.GetMaxToStay();
	if( History.Size() > 0 )
		oldObs = History.Back().Observation;
	else {
		oldObs = simulator.GetStartingObservation();
	}

	History.Add(action, observation);
	BELIEF_PROBA_STATE *beliefs     = new BELIEF_PROBA_STATE();
	BELIEF_PROBA_STATE *rootBeliefs = static_cast<BELIEF_PROBA_STATE*>(Root->Beliefs());
	beliefs->MH.resize(numMDP * maxToStay);
	beliefs->Copy(rootBeliefs, simulator);

	// Find matching vnode from the rest of the tree
	QNODE& qnode       = Root->Child(action);
	PROBA_VNODE* vnode = safe_cast<PROBA_VNODE*>(qnode.Child(observation));
	double sum         = 0, pmh, pmm, pssam, msum, phmm;

	for( int mprime = 0; mprime < numMDP; mprime++ ) {
		double init = simulator.GetTransition(mprime, oldObs, action, observation);
		for( int hprime = 0; hprime < maxToStay; hprime++ ) {
			msum = init;
			if( hprime + 1 < maxToStay )
				msum *= rootBeliefs->MH[mprime * maxToStay + hprime + 1];
			else
				msum *= 0;

			for( int m = 0; m < numMDP; m++ ) {
				pmm   = simulator.GetMDPTransition(m,mprime);
				pssam = simulator.GetTransition(m, oldObs, action, observation);
				pmh   = rootBeliefs->MH[m * maxToStay + 0];
				phmm  = simulator.GetTimeToStay(m, mprime, hprime);
				msum  += pmm * pssam * pmh * phmm;
			}

			beliefs->MH[mprime * maxToStay + hprime] = msum;
			sum += msum;
		}
	}

	assert(sum != 0);
	for( int i = 0; i < numMDP * maxToStay; i++ ) {
		beliefs->MH[i] = beliefs->MH[i] / sum * 100.0;
		if( Params.ShowDistribution )
			cout << setw(10) << beliefs->MH[i] << " ";
	}
	if( Params.ShowDistribution ) {
		cout << oldObs << " " << action << " " << observation;
		cout << endl;
	}

	// Find a state to initialise prior (only requires fully observed state)
	ENVIRONMENT_STATE* state = 0;

	if( vnode && vnode->Beliefs()->GetState() != 0 ) {
		state = vnode->Beliefs()->CreateSample(simulator);
	}
	else {
		state = beliefs->CreateSample(simulator);
	}

	// Delete old tree and create new root
	PROBA_VNODE::Free(Root, simulator);
	PROBA_VNODE* newRoot = ExpandNode(state);
	newRoot->Beliefs()->Free(simulator);
	delete newRoot->Beliefs();
	simulator.FreeState(state);
	beliefs->GetState()->stateIndex = observation;
	newRoot->Beliefs(beliefs);
	Root = newRoot;
	return true;
}
