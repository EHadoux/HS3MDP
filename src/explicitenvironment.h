#ifndef EXPLICITENVIRONMENT_H
#define EXPLICITENVIRONMENT_H

#include "environment.h"
#include <vector>

using namespace std;

class EXPLICITENVIRONMENT : public ENVIRONMENT {
  public:
    EXPLICITENVIRONMENT(int numActions, int numObservations, int numMDP, int maxToStay, bool original);
    EXPLICITENVIRONMENT(const EXPLICITENVIRONMENT &other);
    ~EXPLICITENVIRONMENT();

    virtual double GetTransition(int mdp, int oldObs, int action, int newObs) const;
    virtual double GetMDPTransition(int oldmdp, int newmdp) const;
    virtual double GetTimeToStay(int oldmdp, int newmdp, int h) const;
    virtual double GetReward(int mdp, int obs, int action, int obsprime) const;

  private:
    vector<double***> _rewards;
    vector<double***> _transitions;
    vector<double*>   _MDPTransitions;
    vector<double**>  _durations;

    virtual double*** createTransitions() = 0;
    virtual double*** createRewards() = 0;
    virtual double*   createDurations() = 0;
}

#endif
