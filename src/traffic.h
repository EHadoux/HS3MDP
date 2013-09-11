#ifndef TRAFFIC_H
#define TRAFFIC_H

#include "environment.h"
#include <iostream>

using namespace std;

class TRAFFIC : public ENVIRONMENT {
public:
	TRAFFIC(int maxToStay = 5);
	TRAFFIC(const TRAFFIC& other);
	~TRAFFIC();

	bool Step(STATE& state, int action, int& observation, double& reward) const;

	double GetTransition(int mdp, int oldObs, int action, int newObs) const;
	double GetMDPTransition(int oldmdp, int newmdp) const;
	double GetTimeToStay(int oldmdp, int newmdp, int h) const;
	double GetReward(int mdp, int obs, int action) const;

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

ostream& operator<<( ostream &flux, TRAFFIC const& traffic );
#endif
