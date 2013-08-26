#ifndef BELIEF_PROBA_STATE_H
#define BELIEF_PROBA_STATE_H

#include "beliefstate.h"
#include "simulator.h"
#include "environment.h"
#include <vector>
#include <random>

#define _unused(x) ((void)x)

using namespace std;

class BELIEF_PROBA_STATE : public BELIEF_STATE {
public:
	BELIEF_PROBA_STATE();
	~BELIEF_PROBA_STATE();

	const STATE* GetSample(int index) const { _unused(index); return _uniqueState; }
	void AddSample(STATE* state);
	void Copy(const BELIEF_STATE* beliefs, const SIMULATOR& simulator);
	void Free(const SIMULATOR& simulator);
	ENVIRONMENT_STATE* CreateSample(const SIMULATOR& simulator) const;
	bool Empty() const { return _uniqueState == 0; }
	void Move(BELIEF_STATE* beliefs);
	void SetState(ENVIRONMENT_STATE* state);
	ENVIRONMENT_STATE* GetState() {return _uniqueState;}

	vector<double> MH;

private:
	ENVIRONMENT_STATE* _uniqueState;
};

#endif
