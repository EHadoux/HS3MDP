#ifndef POMDP_H
#define	POMDP_H

#include "simulator.h"
#include <vector>
#include <random>

using namespace std;

class POMDP_STATE : public STATE {
public:
    int POMDPState;
};

class POMDP : public SIMULATOR {
public:
    POMDP(int numStates, int numActions, int numObservations, double discount, int seed);
    POMDP(const POMDP& orig);
    virtual ~POMDP();
    
    virtual bool Step(STATE& state, int action, int& observation, double& reward) const;
    virtual POMDP_STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual POMDP_STATE* Copy(const STATE& state) const;
    
    int GetNumStates() const { return NumStates; }
    double GetTransitionProbability(int s, int a, int sprime) const { 
        return TransitionProbabilities.at(s)->at(a)->at(sprime);
    }
    void SetTransitionProbability(int s, int a, int sprime, double p) {
        TransitionProbabilities.at(s)->at(a)->at(sprime) = p;
    }
    double GetObservationProbability(int s, int a, int o) const { 
        return ObservationProbabilities.at(s)->at(a)->at(o);
    }
    double GetReward(int s, int a) const {
        return Rewards.at(s)->at(a);
    }
    double GetRewardProbabilities(int s, int a, int r) const {
        return RewardProbabilities.at(s)->at(a)->at(r);
    }
    double GetRMin() const { return RMin; }
    double GetRMax() const { return RMax; }
    void SetRMin(double rmin) { RMin = rmin; }
    void SetRMax(double rmax) { RMax = rmax; }
    bool IsCopy() const { return Copied; }

protected:
    int             NumStates;
    double          RMin, RMax;
    mutable mt19937 Gen;
    bool            Copied;
    
    vector<vector<vector<double>*>*> TransitionProbabilities;
    vector<vector<vector<double>*>*> ObservationProbabilities;
    vector<vector<double>*>          Rewards;
    vector<vector<vector<double>*>*> RewardProbabilities;

private:    
    mutable MEMORY_POOL<POMDP_STATE> MemoryPool;
} ;

#endif	/* POMDP_H */

