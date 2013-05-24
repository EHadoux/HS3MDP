#ifndef CONTROLED_H
#define CONTROLED_H

#include "environment.h"
#include <iostream>

using namespace std;

class CONTROLED : public ENVIRONMENT {
public:
	CONTROLED(int numStates, int numActions, int numMDP);
	CONTROLED(const CONTROLED& other);
	~CONTROLED();

	bool Step(STATE& state, int action, int& observation, double& reward) const;

	double GetTransition(int mdp, int oldObs, int action, int newObs) const;
	double GetMDPTransition(int oldmdp, int newmdp) const;
	double GetTimeToStay(int oldmdp, int newmdp, int h) const;

	ostream& toString( ostream &flux ) const;

private:
	double*** _rewards;
	double**** _transitions;
	double** _MDPTransitions;
	double*** _timeToStay;

	double*** createTransitions();
	double** createRewards();
	double* createTimeToStay();
};

ostream& operator<<( ostream &flux, CONTROLED const& controled );
#endif
