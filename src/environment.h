#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "simulator.h"
#include <string>
#include <iostream>
#include <random>

using namespace std;

class ENVIRONMENT_STATE : public STATE {
public:
	int stateIndex, MDPIndex, timeToStay;
};

class ENVIRONMENT : public SIMULATOR {

public:
	ENVIRONMENT(int numActions, int numObservations, int numMDP, int maxToStay, bool original);
	ENVIRONMENT(const ENVIRONMENT& other);
	virtual ~ENVIRONMENT();

	ENVIRONMENT_STATE* Copy(const STATE& state) const;
	virtual ENVIRONMENT_STATE* CreateStartState() const;
	ENVIRONMENT_STATE* Allocate() const;
	void FreeState(STATE* state) const;

	virtual bool Step(STATE& state, int action, int& observation, double& reward) const = 0;
	virtual bool LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const;

	virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
	virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
	virtual void DisplayAction(int action, std::ostream& ostr) const;

	int GetStartingObservation() const { return *_startingStateIndex; }
	int GetNumMDP() const { return _numMDP; }
	int GetMaxToStay() const { return _maxToStay; }

	virtual double GetTransition(int mdp, int oldObs, int action, int newObs) const = 0;
	virtual double GetMDPTransition(int oldmdp, int newmdp) const = 0;
	virtual double GetTimeToStay(int oldmdp, int newmdp, int h) const = 0;
	virtual double GetReward(int mdp, int obs, int action) const = 0;

	void ToPOMDP(string filename) const;
	void ToPOMDPX(string filename) const;

	void TestConstructor() const;

protected:
	bool isCopy() const { return _copy; }
	bool useStructure() const { return !_original; }
	int* _startingStateIndex;
	bool TestTransitionsSumToOne() const;
	bool TestMDPSumToOne() const;
	bool TestTimeToStaySumToOne() const;
	int discrete_rand(const double* array, const int size) const;
	double rand_01() const;

private:
	mutable MEMORY_POOL<ENVIRONMENT_STATE> MemoryPool;
	int _maxToStay;
	int _numMDP;
	bool _copy;
	bool _original;
	mutable mt19937_64 _gen;
	mutable uniform_real_distribution<> _dis;
};
#endif
