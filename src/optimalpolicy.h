#ifndef OPTIMAL_POLICY_H
#define OPTIMAL_POLICY_H

#include "mcts.h"
#include <string>
#include <vector>

using namespace std;

class OptimalPolicy : public MCTS {
public:
	OptimalPolicy(string policyfile, STATE* policyStartingState, const SIMULATOR& simulator, const PARAMS& params);
	~OptimalPolicy();

	int SelectAction();
	bool Update(int action, int observation);

private:
	int _policyState;
	vector<vector<int>*> _graph;
	vector<int> _actions;
};

#endif
