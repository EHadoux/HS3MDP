#include "sailboat.h"
#include "utils.h"
#include <iomanip>
#include <iostream>
#include <cassert>
#include <cmath>

#define _unused(x) ((void)x)
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define N_S 0
#define W_E 1

using namespace std;
using namespace UTILS;

SAILBOAT::SAILBOAT(int numStates)
: ENVIRONMENT(2, numStates, 4)
{
	_cote = sqrt(numStates);

	int numMDP = GetNumMDP();

	_MDPTransitions = new double* [numMDP];
	_timeToStay = new double** [numMDP];

	for( int m = 0; m < numMDP; m++ ) {
		_MDPTransitions[m] = new double[numMDP];
		_timeToStay[m] = new double* [numMDP];
		for( int mprime = 0; mprime < numMDP; mprime++ ) {
			_MDPTransitions[m][mprime] = 0;
			_timeToStay[m][mprime] = createTimeToStay();
		}

		_MDPTransitions[m][(((m-1) < 0) ? (numMDP-1) : (m-1))] = 10;
		_MDPTransitions[m][m] = 80;
		_MDPTransitions[m][(m+1)%numMDP] = 10;

	}

	_startingStateIndex = 0;
}

double* SAILBOAT::createTimeToStay() {
	int maxToStay = GetMaxToStay();
	double *timeToStay = new double[maxToStay];
	double gaussienne[5] = {5, 25, 40, 25, 5};
	int mu = Random(maxToStay / 2);

	for( int i = 0; i < maxToStay; i++ ) {
		if( (mu - 2) <= i && i <= (mu + 2) )
			timeToStay[i] = gaussienne[i - (mu - 2)];
		else
			timeToStay[i] = 0;
	}

	//Pour sommer a 1
	int cumIndex = 0;
	int cumSum = 0;
	while( cumIndex + mu - 2 < 0 ) {
		cumSum += gaussienne[cumIndex];
		cumIndex++;
	}
	timeToStay[0] += cumSum;

	cumIndex = 0;
	cumSum = 0;
	while( mu + 2 - cumIndex > (maxToStay-1) ) {
		cumSum += gaussienne[4 - cumIndex];
		cumIndex++;
	}
	timeToStay[maxToStay-1] += cumSum;
	//Pour sommer a 1

	return timeToStay;
}

SAILBOAT::SAILBOAT(const SAILBOAT& other)
: ENVIRONMENT(other)
{
	_cote = other._cote;
	_MDPTransitions = other._MDPTransitions;
	_timeToStay = other._timeToStay;
}

SAILBOAT::~SAILBOAT() {
	if(!isCopy()) {
		for( int m = 0; m < GetNumMDP(); m++ ) {
			delete[] _MDPTransitions[m];
			for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
				delete[] _timeToStay[m][mprime];
			delete[] _timeToStay[m];
		}

		delete[] _timeToStay;
		delete[] _MDPTransitions;
	}
}

bool SAILBOAT::Step(STATE& state, int action, int& observation, double& reward) const
{
	ENVIRONMENT_STATE& sailboatState = safe_cast<ENVIRONMENT_STATE&>(state);
	int stateIndex = sailboatState.stateIndex;
	int MDPIndex = sailboatState.MDPIndex;
	int timeToStay = sailboatState.timeToStay;
	int cote = GetCote();

	int p = Random(100);
	int i = stateIndex;
	int cumsum;
	bool ret = false;

	switch(MDPIndex) {
		case NORTH:
			if( action == W_E ) {
				if( p < 80 && i < ((cote - 1) * cote))
					i += cote;
				else if( p < 90 && i < ((cote - 1) * cote) && (i % cote) < (cote - 1) )
					i += (cote + 1);
				else if( i < ((cote - 1) * cote) && (i % cote) > 0 )
					i += (cote - 1);

			} break;
		case SOUTH:
			if( action == W_E ) {
				if( p < 80 && i > (cote - 1))
					i -= cote;
				else if( p < 90 && i > (cote - 1) && (i % cote) > 0 )
					i -= (cote + 1);
				else if( i > (cote - 1) && (i % cote) < (cote - 1) )
					i -= (cote - 1);

			} break;
		case WEST:
			if( action == N_S ) {
				if( p < 80 && (i % cote) < (cote - 1))
					i += 1;
				else if( p < 90 && i < ((cote - 1) * cote) && (i % cote) < (cote - 1) )
					i += (cote + 1);
				else if( i > (cote - 1) && (i % cote) < (cote - 1) )
					i += (-cote + 1);

			} break;
		case EAST:
			if( action == N_S ) {
				if( p < 80 && (i % cote) > 0)
					i -= 1;
				else if( p < 90 && i > (cote - 1) && (i % cote) > 0 )
					i -= (cote + 1);
				else if( i < ((cote - 1) * cote) && (i % cote) > 0 )
					i -= (-cote + 1);

			} break;
	}
	assert(i < cote * cote);
	reward = ((i / cote) + 1) * ((i % cote) + 1);

	if( i == (NumObservations - 1)) {
		reward = cote * cote;
		ret = true;
	}

	sailboatState.stateIndex = i;
	observation = i;

	if( timeToStay > 0 )
		sailboatState.timeToStay = timeToStay - 1;
	else {
		p = Random(100) + 1;
		cumsum = _MDPTransitions[MDPIndex][0];
		i = 0;
		while( cumsum < p ) {
			i++;
			cumsum += _MDPTransitions[MDPIndex][i];
		}
		int newMDP = i;
		assert(_MDPTransitions[MDPIndex][newMDP] > 0);
		sailboatState.MDPIndex = newMDP;

		p = Random(100) + 1;
		cumsum = _timeToStay[MDPIndex][newMDP][0];
		i = 0;
		while( cumsum < p ) {
			i++;
			cumsum += _timeToStay[MDPIndex][newMDP][i];
		}
		sailboatState.timeToStay = i;
		assert(_timeToStay[MDPIndex][newMDP][i] > 0);
	}

	assert(GetTransition(MDPIndex, stateIndex, action, observation) > 0);

	return ret;
}

double SAILBOAT::GetTransition(int mdp, int oldObs, int action, int newObs) const {
	int cote = GetCote();
	int x = oldObs % cote, y = oldObs / cote, newx = newObs % cote, newy = newObs / cote;

	if( abs(x-newx) > 1 || abs(y-newy) > 1 )
		return 0;

	switch(mdp) {
	case NORTH:
		if( action == W_E ) {
			if( (newy - y) == 1 ) {
				if( abs(newx - x) == 1 )
					return 10;
				else
					return 80;
			} else if( newy == y && x == newx ) {
				if( y == (cote - 1) )
					return 100;
				else if( x == 0 || x == (cote - 1) )
					return 10;
			}
		} else if( oldObs == newObs )
			return 100;
		break;

	case SOUTH:
		if( action == W_E ) {
			if( (newy - y) == -1 ) {
				if( abs(newx - x) == 1 )
					return 10;
				else
					return 80;
			} else if( newy == y && x == newx ) {
				if( y == 0 )
					return 100;
				else if( x == 0 || x == (cote - 1) )
					return 10;
			}
		} else if( oldObs == newObs )
			return 100;
		break;

	case WEST:
		if( action == N_S ) {
			if( (newx - x) == 1 ) {
				if( abs(newy - y) == 1 )
					return 10;
				else
					return 80;
			} else if( newx == x && y == newy ) {
				if( x == (cote - 1) )
					return 100;
				else if( y == 0 || y == (cote - 1) )
					return 10;
			}
		} else if( oldObs == newObs )
			return 100;
		break;

	case EAST:
		if( action == N_S ) {
			if( (newx - x) == -1 ) {
				if( abs(newy - y) == 1 )
					return 10;
				else
					return 80;
			} else if( newx == x && y == newy ) {
				if( x == 0 )
					return 100;
				else if( y == 0 || y == (cote - 1) )
					return 10;
			}
		} else if( oldObs == newObs )
			return 100;
		break;
	}

	return 0;
}

double SAILBOAT::GetMDPTransition(int oldmdp, int newmdp) const {
	return _MDPTransitions[oldmdp][newmdp];
}

double SAILBOAT::GetTimeToStay(int oldmdp, int newmdp, int h) const {
	return _timeToStay[oldmdp][newmdp][h];
}

void SAILBOAT::DisplayState(const STATE& state, std::ostream& ostr) const
{
	const ENVIRONMENT_STATE& sailboatState = safe_cast<const ENVIRONMENT_STATE&>(state);

	ostr << "State: ";
	ostr << sailboatState.stateIndex / GetCote() << " " << sailboatState.stateIndex % GetCote();
	ostr << " in MDP: ";
	switch( sailboatState.MDPIndex ) {
		case NORTH: ostr << "NORTH"; break;
		case SOUTH: ostr << "SOUTH"; break;
		case WEST: ostr << "WEST"; break;
		case EAST: ostr << "EAST"; break;
	}
	ostr << " " << sailboatState.timeToStay << " step to stay." << endl;
}

void SAILBOAT::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	_unused(state);
	ostr << "State: " << observation / GetCote() << " " << observation % GetCote() << " observed." << endl;
}

void SAILBOAT::DisplayAction(int action, std::ostream& ostr) const
{
	ostr << "Action: " << (action == N_S ? "N_S" : "W_E") << " observed." << endl;
}

ostream& SAILBOAT::toString( ostream &flux ) const
{
	flux << "NumStates: " << NumObservations << " NumActions: " << NumActions << " NumMDP: " << GetNumMDP() << endl;

	flux << "MDPTransitionsMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		flux << m << ":";
		for( int n = 0; n < GetNumMDP(); n++ )
			flux << setw(3) << _MDPTransitions[m][n] << " ";
		flux << endl;
	} flux << endl;

	flux << "TimeToStayMatrix:" << endl;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int n = 0; n < GetNumMDP(); n++ ) {
			flux << setw(3) << m << " -> " << setw(2) << n << ": ";
			for( int i = 0; i < GetMaxToStay(); i++ )
				flux << setw(3) << _timeToStay[m][n][i] << " ";
			flux << endl;
	}

	return flux;
}

ostream& operator<<( ostream &flux, SAILBOAT const& sailboat ) {
	sailboat.toString(flux);
	return flux;
}