#include "explicitenvironment.h"

#define _unused(x) ((void)x)

using namespace std;

EXPLICITENVIRONMENT::EXPLICITENVIRONMENT(int numActions, int numObservations,
                                         int numMDP, int maxToStay, bool original)
  : ENVIRONMENT(numActions, numObservations, numMDP, maxToStay, original) {}

EXPLICITENVIRONMENT::EXPLICITENVIRONMENT(const EXPLICITENVIRONMENT &other) {
  _rewards        = other._rewards;
  _transitions    = other._transitions;
  _MDPTransitions = other._MDPTransitions;
  _durations      = other._durations;
}

EXPLICITENVIRONMENT::~EXPLICITENVIRONMENT() {
  if(!isCopy()) {
    for( int i = 0; i < GetNumMDP(); i++ ) {
      double*** reward         = _rewards.get(i);
      double*** transitions    = _transitions.get(i);
      double*   MDPTransitions = _MDPTransitions.get(i);
      double**  durations      = _durations.get(i);
      for( int j = 0; j < NumObservations; j++ ) {
        #ATTENTIONICI
        delete[] reward[j];
        for( int k = 0; k < NumActions; k++ )
          delete[] transitions[j][k];
        delete[] transitions[j];
      }
      for( int j = 0; j < GetNumMDP(); j++ )
        delete[] durations[j];
      delete[] reward;
      delete[] transitions;
      delete[] MDPTransitions;
      delete[] durations;
    }
  }
}

double EXPLICITENVIRONMENT::GetReward(int mdp, int obs, int action, int obsprime) const {
  _unused(obsprime);
  return _rewards[mdp][obs][action];
}

double EXPLICITENVIRONMENT::GetTransition(int mdp, int oldObs, int action, int newObs) const {
  return _transitions[mdp][oldObs][action][newObs];
}

double EXPLICITENVIRONMENT::GetMDPTransition(int oldmdp, int newmdp) const {
  return _MDPTransitions[oldmdp][newmdp];
}

double EXPLICITENVIRONMENT::GetTimeToStay(int oldmdp, int newmdp, int h) const {
  return _timeToStay[oldmdp][newmdp][h];
}
