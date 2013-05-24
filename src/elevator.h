#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "environment.h"
#include <iostream>

using namespace std;

class ELEVATOR : public ENVIRONMENT {
public:
	ELEVATOR(int numFloors, int numActions, int numMDP);
	ELEVATOR(const ELEVATOR& other);
	~ELEVATOR();

	bool Step(STATE& state, int action, int& observation, double& reward) const;

	double GetTransition(int mdp, int oldObs, int action, int newObs) const;
	double GetMDPTransition(int oldmdp, int newmdp) const;
	double GetTimeToStay(int oldmdp, int newmdp, int h) const;

	int GetNumFloors() const { return _numFloors; }
	void FromObservation(int observation, int& floorIndex, vector<bool>& pickup, vector<bool>& dropoff) const;
	int ToObservation(int floorIndex, vector<bool> pickup, vector<bool> dropoff) const;

	ostream& toString( ostream &flux ) const;

private:
	int _numFloors;
	double** _MDPTransitions;
	double*** _timeToStay;

	double* createTimeToStay();
};

ostream& operator<<( ostream &flux, ELEVATOR const& elevator );
#endif
