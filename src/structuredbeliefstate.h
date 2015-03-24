#ifndef STRUCTUREDBELIEFSTATE_H
#define	STRUCTUREDBELIEFSTATE_H

#include "beliefstate.h"
#include <vector>

using namespace std;

class STRUCTURED_BELIEF_STATE : public BELIEF_STATE {
public:
    STRUCTURED_BELIEF_STATE();
    STRUCTURED_BELIEF_STATE(const STRUCTURED_BELIEF_STATE& orig);
    virtual ~STRUCTURED_BELIEF_STATE();
    
    HS3MDP_STATE* CreateSample(const SIMULATOR& simulator) const;
    void AddSample(STATE*) {}
    void Copy(const BELIEF_STATE*, const SIMULATOR&) {}
    void Move(BELIEF_STATE*){}
    
    static int HS3MDPState;
    vector<double> BeliefFunction;    
} ;

#endif	/* STRUCTUREDBELIEFSTATE_H */

