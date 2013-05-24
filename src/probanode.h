#ifndef PROBA_NODE_H
#define PROBA_NODE_H

#include "node.h"
#include "beliefprobastate.h"
#include <iostream>
#include <vector>

using namespace std;

class PROBA_VNODE : public VNODE {
public:
	static PROBA_VNODE* Create();
	static void Free(VNODE* vnode, const SIMULATOR& simulator);
	static void FreeAll();

	BELIEF_PROBA_STATE* Beliefs() { return safe_cast<BELIEF_PROBA_STATE*>(BeliefState); }
	const BELIEF_PROBA_STATE* Beliefs() const { return safe_cast<const BELIEF_PROBA_STATE*>(BeliefState); }
	void Beliefs(BELIEF_STATE* b) { BeliefState = safe_cast<BELIEF_PROBA_STATE*>(b); }
private:
	static MEMORY_POOL<PROBA_VNODE> ProbaVNodePool;
};

#endif
