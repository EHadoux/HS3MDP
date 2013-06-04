#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "environment.h"
#include <iostream>
#include <vector>

using namespace std;

class ELEVATOR : public ENVIRONMENT {
public:
	ELEVATOR(int numFloors, int numElevator);
	ELEVATOR(const ELEVATOR& other);
	~ELEVATOR();

	bool Step(STATE& state, int action, int& observation, double& reward) const;

	double GetTransition(int mdp, int oldObs, int action, int newObs) const;
	double GetMDPTransition(int oldmdp, int newmdp) const;
	double GetTimeToStay(int oldmdp, int newmdp, int h) const;

	int GetNumFloors() const { return _numFloors; }
	int GetNumElevators() const { return _numElevators; }
	int GetAction(int observation, int elevatorNumber) const;
	void FromObservation(int observation, vector<int>& floorIndex, vector<bool>& pickup, vector<vector<bool> >& dropoff) const;
	int ToObservation(const vector<int> &floorIndex, const vector<bool> &pickup, const vector<vector<bool> > &dropoff) const;

	ostream& toString( ostream &flux ) const;

private:
	int _numFloors;
	int _numElevators;
	double** _MDPTransitions;
	double*** _timeToStay;

	double* createTimeToStay();
};

ostream& operator<<( ostream &flux, ELEVATOR const& elevator );
#endif
