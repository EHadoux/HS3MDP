#ifndef STRUCTUREDMCTS_H
#define	STRUCTUREDMCTS_H

#include "mcts.h"
#include "structuredbeliefstate.h"

class STRUCTURED_MCTS : public MCTS {
public:
    STRUCTURED_MCTS(const SIMULATOR& simulator, const PARAMS& params);
    STRUCTURED_MCTS(const STRUCTURED_MCTS& orig);
    virtual ~STRUCTURED_MCTS();

    bool Update(int action, int observation);
    STRUCTURED_BELIEF_STATE* BeliefState() {
        return safe_cast<STRUCTURED_BELIEF_STATE*>(Root->Beliefs());
    }
    void InitialiseRoot();    

private:
    PROBA_VNODE* ExpandNode(const STATE* state);
    void AddSample(VNODE*, const STATE&) {}


} ;

#endif	/* STRUCTUREDMCTS_H */

