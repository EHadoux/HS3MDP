#ifndef SAILBOAT_H
#define	SAILBOAT_H

#include "sparsehs3mdp.h"

class SAILBOAT : public SPARSE_HS3MDP {
public:
    SAILBOAT(int gridSize, int maxDuration, double discount, int seed);
    SAILBOAT(const SAILBOAT& orig);
    virtual ~SAILBOAT();
    
    HS3MDP_STATE* CreateStartState() const;
    bool Step(STATE& state, int action, int& observation, double& reward) const;
    
private:
    double GetTProbability(int mode, int x, int y, int action, int newx, int newy) const;
    
    int SideSize;
} ;

#endif	/* SAILBOAT_H */

