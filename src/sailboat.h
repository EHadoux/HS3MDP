#ifndef SAILBOAT_H
#define SAILBOAT_H

#include "environment.h"
#include <iostream>

using namespace std;

class SAILBOAT : public ENVIRONMENT {
public:
	SAILBOAT(int numStates, int maxToStay = 5);
	SAILBOAT(const SAILBOAT& other);
	~SAILBOAT();

	bool Step(STATE& state, int action, int& observation, double& reward) const;

	ENVIRONMENT_STATE* CreateStartState() const;
	double GetTransition(int mdp, int oldObs, int action, int newObs) const;
	double GetMDPTransition(int oldmdp, int newmdp) const;
	double GetTimeToStay(int oldmdp, int newmdp, int h) const;
	double GetReward(int mdp, int obs, int action) const;
	int GetCote() const { return _cote; }

	void DisplayState(const STATE& state, std::ostream& ostr) const;
	void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
	void DisplayAction(int action, std::ostream& ostr) const;

	ostream& toString( ostream &flux ) const;

private:
	double** _MDPTransitions;
	double*** _timeToStay;

	double* createTimeToStay();

	int _cote;
};

ostream& operator<<( ostream &flux, SAILBOAT const& sailboat );
#endif
