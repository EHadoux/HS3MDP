#ifndef TRAFFICLIGHT_H
#define	TRAFFICLIGHT_H

#include "hs3mdp.h"

class TRAFFICLIGHT : public HS3MDP {
public:
    TRAFFICLIGHT(int maxDuration, double discount, int seed);
} ;

#endif	/* TRAFFICLIGHT_H */

