#include "optimalpolicy.h"
#include "environment.h"
#include <fstream>
#include <vector>
#include <cassert>
#include <iostream>

using namespace std;

OptimalPolicy::OptimalPolicy(string policyFile, STATE* startingState, const SIMULATOR& simulator, const PARAMS& params)
	: MCTS(simulator, params) {
	ifstream f(policyFile.c_str());
	int n;
	f >> n;
	while( !f.eof() ) {
		f >> n;
		_actions.push_back(n);
		vector<int>* node = new vector<int>;
		for( int i = 0; i < simulator.GetNumObservations(); i++ ) {
			f >> n;
			node->push_back(n);
		}
		_graph.push_back(node);
		f >> n;
	}
	f.close();
	ENVIRONMENT_STATE* s = safe_cast<ENVIRONMENT_STATE*>(startingState);
	for( unsigned int i = 0; i < _graph.size(); i++ ) {
		if( _graph.at(i)->at(s->stateIndex) != -1 ) {
			_policyState = _graph.at(i)->at(s->stateIndex);
			break;
		}
	}
}

OptimalPolicy::~OptimalPolicy() {
	for( unsigned int i = 0; i < _graph.size(); i++ )
		delete _graph.at(i);
}

int OptimalPolicy::SelectAction() {
	return _actions.at(_policyState);
}

bool OptimalPolicy::Update(int action, int observation) {
	assert(action == _actions.at(_policyState));
	_policyState = _graph.at(_policyState)->at(observation);
	assert(_policyState != -1);
	return true;
}
