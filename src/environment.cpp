#include "environment.h"
#include "utils.h"
#include <cassert>
#include <iostream>
#include <cmath>
#include <string>
#include <fstream>

#define _unused(x) ((void)x)

using namespace std;
using namespace UTILS;

ENVIRONMENT::ENVIRONMENT(int numActions, int numObservations, int numMDP, int maxToStay, bool original)
: SIMULATOR(numActions, numObservations, 0.9) {
	_numMDP             = numMDP;
	_startingStateIndex = new int(Random(numObservations));
	_maxToStay          = maxToStay;
	_copy               = false;
	_gen 				= mt19937_64(rand());
	_dis				= uniform_real_distribution<>();
	_original			= original;
}

ENVIRONMENT::ENVIRONMENT(const ENVIRONMENT& other)
: SIMULATOR(other.NumActions, other.NumObservations, 0.9)
{
	_numMDP             = other._numMDP;
	_startingStateIndex = other._startingStateIndex;
	_maxToStay          = other._maxToStay;
	_copy               = true;
	_gen 				= other._gen;
	_dis				= other._dis;
	_original			= other._original;
}

ENVIRONMENT::~ENVIRONMENT() {
	if( !isCopy() )
		delete _startingStateIndex;
}

ENVIRONMENT_STATE* ENVIRONMENT::Allocate() const {
	return MemoryPool.Allocate();
}

ENVIRONMENT_STATE* ENVIRONMENT::CreateStartState() const {
	ENVIRONMENT_STATE* state = MemoryPool.Allocate();
	if( !isCopy() || !useStructure() )
		*_startingStateIndex = Random(GetNumObservations());
	state->stateIndex        = *_startingStateIndex;
	state->MDPIndex          = Random(GetNumMDP());
	state->timeToStay        = 0;

	return state;
}

ENVIRONMENT_STATE* ENVIRONMENT::Copy(const STATE& state) const
{
	const ENVIRONMENT_STATE& env_state = safe_cast<const ENVIRONMENT_STATE&>(state);
	ENVIRONMENT_STATE* newstate        = MemoryPool.Allocate();
	*newstate                          = env_state;

	return newstate;
}

void ENVIRONMENT::FreeState(STATE* state) const
{
	ENVIRONMENT_STATE* env_state = safe_cast<ENVIRONMENT_STATE*>(state);
	MemoryPool.Free(env_state);
}


bool ENVIRONMENT::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const
{
	_unused(stepObs);
	_unused(status);

	ENVIRONMENT_STATE& env_state = safe_cast<ENVIRONMENT_STATE&>(state);
	if( env_state.stateIndex != history.Back().Observation )
		return false;
	return true;
}

void ENVIRONMENT::DisplayState(const STATE& state, std::ostream& ostr) const
{
	const ENVIRONMENT_STATE& env_state = safe_cast<const ENVIRONMENT_STATE&>(state);

	ostr << "State: " << env_state.stateIndex << " in MDP: " << env_state.MDPIndex  << " "
			<< env_state.timeToStay << " step to stay." << endl;
}

void ENVIRONMENT::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
	_unused(state);
	ostr << "State: " << observation << " observed." << endl;
}

void ENVIRONMENT::DisplayAction(int action, std::ostream& ostr) const
{
	ostr << "Action: " << action << " observed." << endl;
}

bool ENVIRONMENT::TestTransitionsSumToOne() const {
	double sum;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int o = 0; o < NumObservations; o++ )
			for( int a = 0; a < NumActions; a++ ) {
				sum = 0;
				for( int oprime = 0; oprime < NumObservations; oprime++ )
					sum += GetTransition(m, o, a, oprime);
				if( abs(sum - 1.0) > 1e-10 )
					return false;
			}
	return true;
}

bool ENVIRONMENT::TestMDPSumToOne() const {
	for( int m = 0; m < GetNumMDP(); m++ ) {
		double sum = 0;
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
			sum += GetMDPTransition(m, mprime);
		if( abs(sum - 1.0) > 1e-10 )
			return false;
	}
	return true;
}

bool ENVIRONMENT::TestTimeToStaySumToOne() const {
	double sum;
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
			sum = 0;
			for( int t = 0; t < GetMaxToStay(); t++ )
				sum += GetTimeToStay(m, mprime, t);
			if( abs(sum - 1.0) > 1e-10 )
				return false;
	}
	return true;
}

void ENVIRONMENT::TestConstructor() const {
	cout << "Starting constructor tests." << endl;
	assert(TestTransitionsSumToOne());
	cout << "State transitions sum to one." << endl;
	assert(TestMDPSumToOne());
	cout << "MDP transitions sum to one." << endl;
	assert(TestTimeToStaySumToOne());
	cout << "Time to stay sum to one." << endl;

	cout << "Tests constructeur passés." << endl;
}

void ENVIRONMENT::ToPOMDP( string filename ) const {
	ofstream f(filename.c_str());
	double MDPTransition = 0, timeToStay = 0, sum = 0;
	int numMDP           = GetNumMDP(), maxToStay = GetMaxToStay(), numObs = GetNumObservations();

	f << "discount: " << Discount << endl;
	f << "values: reward" << endl;
	f << "states: " << numMDP * maxToStay * numObs << endl;
	f << "actions: " << GetNumActions() << endl;
	f << "observations: " << numObs << endl << endl;
	for( int a = 0; a < GetNumActions(); a++ ) {
		f << "T: " << a << endl;
		for( int h = 0; h < maxToStay; h++ )
			for( int m = 0; m < numMDP; m++ )
				for( int o = 0; o < numObs; o++ ) {
					sum = 0;
					for( int hprime = 0; hprime < maxToStay; hprime++ )
						for( int mprime = 0; mprime < numMDP; mprime++ ) {
							MDPTransition = 0, timeToStay = 0;

							if( h == 0 ) {
								MDPTransition = GetMDPTransition(m, mprime);
								timeToStay = GetTimeToStay(m, mprime, hprime);
							} else if( hprime == (h-1) && m == mprime ) {
								MDPTransition = 1;
								timeToStay = 1;
							}

							for( int oprime = 0; oprime < numObs; oprime++ ) {
								sum += GetTransition(m, o, a, oprime) * MDPTransition * timeToStay;
								f << GetTransition(m, o, a, oprime) * MDPTransition * timeToStay << " ";
							}
						}
					f << endl;
					assert(abs(sum - 1.0) < 1e-10);
				}
		f << endl;
	}

	for( int i = 0; i < numMDP * maxToStay * numObs; i++ )
		f << "O : * : " << i << " : " << i % numObs << " " << 1 << endl;
	f << endl;

	for( int a = 0; a < GetNumActions(); a++ )
		for( int h = 0; h < GetMaxToStay(); h++ )
			for( int m = 0; m < GetNumMDP(); m++ )
				for( int o = 0; o < GetNumObservations(); o++ )
					if( GetReward(m, o, a) != 0 )
						f << "R: " << a << " : " << h * GetNumMDP() * GetNumObservations() + m * GetNumObservations() + o << " : * : * " << GetReward(m, o, a) << endl;
	f << endl;
	f.close();
}

void ENVIRONMENT::ToPOMDPX( string filename ) const {
	ofstream f(filename.c_str());

	f << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << endl;
	f << "<pomdpx version=\"0.1\" id=\"autogenerated\" ";
	f << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"";
	f << "xsi:noNamespaceSchemaLocation=\"pomdpx.xsd\">" << endl;
	f << "\t<Discount> " << GetDiscount() << " </Discount>" << endl;
	f << "\t<Variable>" << endl;
	f << "\t\t<StateVar vnamePrev=\"s_0\" vnameCurr=\"s_1\" fullyObs=\"true\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumObservations() << "</NumValues>" << endl << "\t\t</StateVar>" << endl;
	f << "\t\t<StateVar vnamePrev=\"m_0\" vnameCurr=\"m_1\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumMDP() << "</NumValues>" << endl << "\t\t</StateVar>" << endl;
	f << "\t\t<StateVar vnamePrev=\"h_0\" vnameCurr=\"h_1\">" << endl;
	f << "\t\t\t<NumValues>" << GetMaxToStay() << "</NumValues>" << endl << "\t\t</StateVar>" << endl;
	f << "\t\t<ObsVar vname=\"obs\">" << endl;
	f << "\t\t\t<NumValues> " << GetNumObservations() << " </NumValues>" << endl << "\t\t</ObsVar>" << endl;
	f << "\t\t<ActionVar vname=\"action\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumActions() << "</NumValues>" << endl << "\t\t</ActionVar>" << endl;
	f << "\t\t<RewardVar vname=\"reward\"/>" << endl;
	f << "\t</Variable>" << endl;
	f << "\t<InitialStateBelief>" << endl;
	f << "\t\t<CondProb>" << endl;
	f << "\t\t\t<Var>s_0</Var>" << endl << "\t\t\t<Parent>null</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl << "\t\t\t\t<Entry>" << endl;
	f << "\t\t\t\t\t<Instance> s0 </Instance>" << endl;
	f << "\t\t\t\t\t<ProbTable> 1.0 </ProbTable>" << endl << "\t\t\t\t</Entry>";
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t\t<CondProb>" << endl;
	f << "\t\t\t<Var>m_0</Var>" << endl << "\t\t\t<Parent>null</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl << "\t\t\t\t<Entry>" << endl;
	f << "\t\t\t\t\t<Instance> - </Instance>" << endl;
	f << "\t\t\t\t\t<ProbTable> uniform </ProbTable>" << endl << "\t\t\t\t</Entry>";
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t\t<CondProb>" << endl;
	f << "\t\t\t<Var>h_0</Var>" << endl << "\t\t\t<Parent>null</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl << "\t\t\t\t<Entry>" << endl;
	f << "\t\t\t\t\t<Instance> s0 </Instance>" << endl;
	f << "\t\t\t\t\t<ProbTable> 1.0 </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</InitialStateBelief>" << endl;
	f << "\t<StateTransitionFunction>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>s_1</Var>" << endl;
	f << "\t\t\t<Parent>action s_0 m_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int a = 0; a < GetNumActions(); a++ )
		for( int m = 0; m < GetNumMDP(); m++ )
			for( int s = 0; s < GetNumObservations(); s++ ) {
				f << "\t\t\t\t<Entry>" << endl;
				f << "\t\t\t\t\t<Instance> a" << a << " s" << s << " s" << m << " - </Instance>" << endl;
				f << "\t\t\t\t\t<ProbTable> ";
				for( int sprime = 0; sprime < GetNumObservations(); sprime++ )
					f << GetTransition(m, s, a, sprime) << " ";
				f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
			}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>m_1</Var>" << endl;
	f << "\t\t\t<Parent>action m_0 h_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int h = 1; h < GetMaxToStay(); h++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> * - s" << h << " - </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> identity </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	for( int m = 0; m < GetNumMDP(); m++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> * s" << m << " s0 - </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> ";
		for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
			f << GetMDPTransition(m, mprime) << " ";
		f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>h_1</Var>" << endl;
	f << "\t\t\t<Parent>action m_0 h_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int hprime = 0; hprime < GetMaxToStay() - 1; hprime++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> * * s" << hprime+1 << " s" << hprime << " </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> 1.0 </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	for( int m = 0; m < GetNumMDP(); m++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> * s" << m << " s0 - </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> ";
		for( int hprime = 0; hprime < GetMaxToStay(); hprime++ ) {
			float sum = 0;
			for( int mprime = 0; mprime < GetNumMDP(); mprime++ )
				sum += GetTimeToStay(m, mprime, hprime);
			f << sum / GetNumMDP() << " ";
		}
		f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</StateTransitionFunction>" << endl;
	f << "\t<ObsFunction>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>obs</Var>" << endl;
	f << "\t\t\t<Parent>action s_1</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int i = 0; i < GetNumObservations(); i++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> * s" << i << " o" << i << " </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> identity </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</ObsFunction>" << endl;
	f << "\t<RewardFunction>" << endl;
	f << "\t\t<Func>" << endl << "\t\t\t<Var>reward</Var>" << endl;
	f << "\t\t\t<Parent>action s_0 m_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int a = 0; a < GetNumActions(); a++ )
		for( int m = 0; m < GetNumMDP(); m++ )
			for( int o = 0; o < GetNumObservations(); o++ ) {
				f << "\t\t\t\t<Entry>" << endl;
				f << "\t\t\t\t\t<Instance> a" << a << " s" << o << " s" << m << " </Instance>" << endl;
				f << "\t\t\t\t\t<ValueTable> " << GetReward(m, o, a) << " ";
				f << "</ValueTable>" << endl << "\t\t\t\t</Entry>" << endl;
			}
	f << "\t\t\t</Parameter>" << endl << "\t\t</Func>" << endl;
	f << "\t</RewardFunction>" << endl << "</pomdpx>" << endl;

	f.close();
}

int ENVIRONMENT::discrete_rand(const double* array, const int size) const {
	discrete_distribution<> d(array, array + size);
	return d(_gen);
}

double ENVIRONMENT::rand_01() const {
	return _dis(_gen);
}
