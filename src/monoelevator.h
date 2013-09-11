#ifndef MELEVATOR_H
#define MELEVATOR_H

#include "elevator.h"
#include <iostream>

using namespace std;

class MONO_ELEVATOR : public ELEVATOR {
public:
	MONO_ELEVATOR(int numFloors, int maxToStay = 5);
	MONO_ELEVATOR(const MONO_ELEVATOR& other);

	bool Step(STATE& state, int action, int& observation, double& reward) const;
	double GetTransition(int mdp, int oldObs, int action, int newObs) const;

	void FromObservation(int observation, int& floorIndex, vector<bool>& pickup, vector<bool>& dropoff) const;
	int ToObservation(int floorIndex, vector<bool> pickup, vector<bool> dropoff) const;
};

ostream& operator<<( ostream &flux, MONO_ELEVATOR const& elevator );
#endif
