#include "structurednode.h"

MEMORY_POOL<STRUCTURED_NODE> STRUCTURED_NODE::StructuredVNodePool;

STRUCTURED_NODE* STRUCTURED_NODE::Create() {
    STRUCTURED_NODE* vnode = STRUCTURED_NODE::StructuredVNodePool.Allocate();
    vnode->Initialise();
    vnode->BeliefState = new STRUCTURED_BELIEF_STATE();

    return vnode;
}

void STRUCTURED_NODE::Free(VNODE* vnode, const SIMULATOR& simulator) {
    STRUCTURED_NODE* snode = safe_cast<STRUCTURED_NODE*>(vnode);
    delete snode->BeliefState;
    StructuredVNodePool.Free(snode);
    for ( int action = 0; action < VNODE::NumChildren; action++ )
        for ( int observation = 0; observation < QNODE::NumChildren; observation++ )
            if ( vnode->Child(action).Child(observation) )
                Free(vnode->Child(action).Child(observation), simulator);
}

void STRUCTURED_NODE::FreeAll() {
    STRUCTURED_NODE::StructuredVNodePool.DeleteAll();
}