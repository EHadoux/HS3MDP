#ifndef PROBA_MCTS_H
#define PROBA_MCTS_H

#include "mcts.h"
#include "probanode.h"
#include "statistic.h"

class PROBA_MCTS : public MCTS
{
public:
	PROBA_MCTS(const SIMULATOR& simulator, const PARAMS& params);

	bool Update(int action, int observation);
	BELIEF_PROBA_STATE* BeliefState() { return safe_cast<BELIEF_PROBA_STATE*>(Root->Beliefs()); }
	void InitialiseRoot();

private:
	PROBA_VNODE* ExpandNode(const STATE* state);
};

#endif
