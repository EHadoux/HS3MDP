#include "probanode.h"
#include "beliefprobastate.h"
#include <iostream>

using namespace std;

MEMORY_POOL<PROBA_VNODE> PROBA_VNODE::ProbaVNodePool;

PROBA_VNODE* PROBA_VNODE::Create() {
	PROBA_VNODE* vnode = PROBA_VNODE::ProbaVNodePool.Allocate();
	vnode->Initialise();
	vnode->BeliefState = new BELIEF_PROBA_STATE();

	return vnode;
}

void PROBA_VNODE::Free(VNODE* vnode, const SIMULATOR& simulator) {
	PROBA_VNODE* pvnode = safe_cast<PROBA_VNODE*>(vnode);
	pvnode->Beliefs()->Free(simulator);
	delete pvnode->BeliefState;
	ProbaVNodePool.Free(pvnode);
	for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
}

void PROBA_VNODE::FreeAll()
{
	PROBA_VNODE::ProbaVNodePool.DeleteAll();
}

