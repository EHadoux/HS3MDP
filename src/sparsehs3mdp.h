#ifndef SPARSEHS3MDP_H
#define	SPARSEHS3MDP_H

#include "hs3mdp.h"
#include <map>
#include <tuple>

/*
 * SPARSEMAP<0> : old non-sparse index vector
 * SPARSEMAP<1> : value vector
 * SPARSEMAP<2> : mapping old sparse-index -> new consecutive index
*/
typedef tuple<vector<int>*, vector<double>*, map<int, int>*> SPARSEMAP;

class SPARSE_HS3MDP : public HS3MDP {
public:
    SPARSE_HS3MDP(int numActions, int numObservations, int numModes, int maxDuration, 
           double discount, int seed);
    SPARSE_HS3MDP(const SPARSE_HS3MDP& orig);
    virtual ~SPARSE_HS3MDP();
    
    virtual bool Step(STATE& state, int action, int& observation, double& reward) const;
    
    double GetInModeTransitionProbability(int m, int s, int a, int sprime) const;
    double GetModeTransitionProbability(int m, int mprime) const;
    double GetDurationProbability(int m, int mprime, int h) const;
    void SetRewardDefault(double rdefault) { RewardDefault = rdefault; }
    double GetRewardDefault() const { return RewardDefault; }
    
protected:
    double RewardDefault;
    vector<vector<vector<SPARSEMAP*>*>*> InModeTransitionProbabilities;
    vector<vector<map<int,double>*>*>    InModeRewards;
    vector<vector<double>*>              ModeTransitionProbabilities;
    vector<vector<vector<double>*>*>     DurationProbabilities;
    vector<vector<vector<SPARSEMAP*>*>*> InModeRewardProbabilities;
} ;

#endif	/* SPARSEHS3MDP_H */

