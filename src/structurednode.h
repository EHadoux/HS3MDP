#ifndef STRUCTUREDNODE_H
#define	STRUCTUREDNODE_H

#include "structuredbeliefstate.h"
#include "node.h"

class STRUCTURED_NODE : public VNODE {
public:
    static STRUCTURED_NODE* Create();
    static void Free(VNODE* vnode, const SIMULATOR& simulator);
    static void FreeAll();

    STRUCTURED_BELIEF_STATE* Beliefs() {
        return safe_cast<STRUCTURED_BELIEF_STATE*>(BeliefState);
    }

    const STRUCTURED_BELIEF_STATE* Beliefs() const {
        return safe_cast<const STRUCTURED_BELIEF_STATE*>(BeliefState);
    }

    void Beliefs(BELIEF_STATE* b) {
        BeliefState = safe_cast<STRUCTURED_BELIEF_STATE*>(b);
    }
private:
    static MEMORY_POOL<STRUCTURED_NODE> StructuredVNodePool;
} ;

#endif	/* STRUCTUREDNODE_H */

