#ifndef HS3MDP_H
#define	HS3MDP_H

#include "pomdp.h"
#include <vector>

using namespace std;

class HS3MDP_STATE : public POMDP_STATE {
    public:
        int HS3MDPState, Mode, Duration;
};

class HS3MDP : public POMDP {
public:
    HS3MDP(int numActions, int numObservations, int numModes, int maxDuration, 
           double discount, int seed);
    HS3MDP(const HS3MDP& orig);
    virtual ~HS3MDP();
    
    virtual HS3MDP_STATE* CreateStartState() const;
    virtual HS3MDP_STATE* Copy(const STATE& state) const;
    virtual bool Step(STATE& state, int action, int& observation, double& reward) const;
    
    int GetNumModes() const { return NumModes; }
    int GetMaxDuration() const { return MaxDuration; }
    double GetInModeTransitionProbability(int m, int s, int a, int sprime) const {
        return InModeTransitionProbabilites.at(m)->at(s)->at(a)->at(sprime);
    }
    double GetModeTransitionProbability(int m, int mprime) const { 
        return ModeTransitionProbabilities.at(m)->at(mprime);
    }
    
    double GetDurationProbability(int m, int mprime, int h) const {
        return DurationProbabilities.at(m)->at(mprime)->at(h);
    }
    bool IsStructureAdapted() const { return StructureAdapted; }
    void UseStructure() { StructureAdapted = true; }
    
    static int GetStartingHS3MDPStateIndex() { return HS3MDP::StartingHS3MDPStateIndex; }
    
protected:
    int  NumModes, MaxDuration;
    bool StructureAdapted;
    vector<vector<vector<vector<double>*>*>*> InModeTransitionProbabilites;
    vector<vector<vector<double>*>*>          InModeRewards;
    vector<vector<double>*>                   ModeTransitionProbabilities;
    vector<vector<vector<double>*>*>          DurationProbabilities;
    vector<vector<vector<vector<double>*>*>*> InModeRewardProbabilities;
    mutable MEMORY_POOL<HS3MDP_STATE> MemoryPool;
    
    static int StartingHS3MDPStateIndex;
} ;

#endif	/* HS3MDP_H */

