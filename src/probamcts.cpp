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
	int numMDP                    = env_sim.GetNumMDP(), maxToStay = env_sim.GetMaxToStay();

	PRoot->SetState(startState);
	PRoot->MH = new double[numMDP * maxToStay];
	for( int i = 0; i < numMDP * maxToStay; i++ )
		PRoot->MH[i] = 0;
	for( int i = 0; i < numMDP; i++ )
		PRoot->MH[i * maxToStay + 0] = 1.0 / numMDP;
}

PROBA_VNODE* PROBA_MCTS::ExpandNode(const STATE* state)
{
	PROBA_VNODE* vnode = PROBA_VNODE::Create();
	vnode->Value.Set(0, 0);
	Simulator.Prior(state, History, vnode, Status);

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
	beliefs->MH                     = new double[numMDP * maxToStay];
	beliefs->Copy(rootBeliefs, simulator);
	rootBeliefs->GetState()->stateIndex = observation;
	beliefs->GetState()->stateIndex = observation;

	double sum         = 0, pmh, pmm, pssam, msum, phmm;

	for( int mprime = 0; mprime < numMDP; mprime++ ) {
		double init = simulator.GetTransition(mprime, oldObs, action, observation);
		for( int hprime = 0; hprime < maxToStay; hprime++ ) {
			if( hprime + 1 < maxToStay )
				msum = init * rootBeliefs->MH[mprime * maxToStay + hprime + 1];
			else
				msum = 0;

			for( int m = 0; m < numMDP; m++ ) {
				pmm   = simulator.GetMDPTransition(m,mprime);
				cout << "PMM:" << pmm << endl;
				pssam = simulator.GetTransition(m, oldObs, action, observation);
				cout << "PSSAM: " << pssam << endl;
				pmh   = rootBeliefs->MH[m * maxToStay + 0];
				cout << "PMH: " << pmh << endl;
				phmm  = simulator.GetTimeToStay(m, mprime, hprime);
				cout << "PHMM: " << phmm << endl;
				msum  += pmm * pssam * pmh * phmm;
			}

			beliefs->MH[mprime * maxToStay + hprime] = msum;
			sum += msum;
		}
	}

	assert(sum != 0);
	for( int i = 0; i < numMDP * maxToStay; i++ ) {
		beliefs->MH[i] = beliefs->MH[i] / sum;
		if( Params.ShowDistribution )
			cout << setw(10) << beliefs->MH[i] << " ";
	}
	if( Params.ShowDistribution ) {
		cout << oldObs << " " << action << " " << observation;
		cout << endl;
	}

	// Delete old tree and create new root
	PROBA_VNODE::Free(Root, simulator);
	PROBA_VNODE* newRoot = ExpandNode(beliefs->GetState());
	newRoot->Beliefs()->Free(simulator);
	delete newRoot->Beliefs();
	newRoot->Beliefs(beliefs);
	Root = newRoot;
	return true;
}

void PROBA_MCTS::AddSample(VNODE* node, const STATE& state)
{
	_unused(node);
	_unused(state);
}

