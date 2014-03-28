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

void ENVIRONMENT::discountPOMDP(ostream &f) const {
	f << "discount: " << Discount << endl;
}

void ENVIRONMENT::headerToPOMDP(ostream &f) const {
	discountPOMDP(f);
	f << "values: reward" << endl;
	f << "states: " << GetNumMDP() * GetMaxToStay() * GetNumObservations() << endl;
	f << "actions: " << GetNumActions() << endl;
	f << "observations: " << GetNumObservations() << endl << endl;
}

void ENVIRONMENT::stateTransitionPOMDP(ostream &f, bool sparse) const {
	double sum = 0, MDPTransition, duration;
	for( int a = 0; a < GetNumActions(); a++ ) {
		if( !sparse ) f << "T: " << a << endl;
		for( int o = 0; o < GetNumObservations(); o++ )
			for( int h = 0; h < GetMaxToStay(); h++ )
				for( int m = 0; m < GetNumMDP(); m++ ) {
					sum = 0;
					for( int oprime = 0; oprime < GetNumObservations(); oprime++ )
						for( int hprime = 0; hprime < GetMaxToStay(); hprime++ )
							for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
								MDPTransition = 0, duration = 0;
								if( h == 0 ) {
									MDPTransition = GetMDPTransition(m, mprime);
									duration = GetTimeToStay(m, mprime, hprime);
								} else if( hprime == (h-1) && m == mprime ) {
									MDPTransition = 1;
									duration = 1;
								}

								float t = GetTransition(m, o, a, oprime) * MDPTransition * duration;
								sum += t;
								if( sparse ) {
									if( t == 0 ) continue;
									f << "T: " << a << ": " << m + GetNumMDP() * (h + o * GetMaxToStay());
									f << " : " << mprime + GetNumMDP() * (hprime + oprime * GetMaxToStay());
									f << " " << t << endl;
								}
								else
									f << t << " ";
							}
					f << endl;
	      				assert(abs(sum - 1.0) < 1e-7);
				}
		if( !sparse ) f << endl;
	}
	f << endl;
}

void ENVIRONMENT::observationFunctionPOMDP(ostream &f) const {
	for( int i = 0; i < GetNumMDP() * GetMaxToStay() * GetNumObservations(); i++ )
		f << "O : * : " << i << " : " << i / (GetMaxToStay() * GetNumMDP()) << " " << 1 << endl;
	f << endl;
}

void ENVIRONMENT::rewardFunctionPOMDP(ostream &f) const {
	for( int a = 0; a < GetNumActions(); a++ )
		for( int o = 0; o < GetNumObservations(); o++ )
			for( int h = 0; h < GetMaxToStay(); h++ )
				for( int m = 0; m < GetNumMDP(); m++ ) {
					float r = GetReward(m, o, a, 0);
					if( r != 0 ) {
						f << "R: " << a << " : " << m + GetNumMDP() * (h + o * GetMaxToStay());
						f << " : * : * " << r << endl;
					}
				}
	f << endl;
}

void ENVIRONMENT::ToPOMDP( string filename, bool sparse ) const {
	ofstream f(filename.c_str());
	headerToPOMDP(f);
	stateTransitionPOMDP(f, sparse);
	observationFunctionPOMDP(f);
	rewardFunctionPOMDP(f);

	f.close();
}

void ENVIRONMENT::headerToPOMDPX(ostream &f) const {
	f << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << endl;
	f << "<pomdpx version=\"0.1\" id=\"autogenerated\" ";
	f << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"";
	f << "xsi:noNamespaceSchemaLocation=\"pomdpx.xsd\">" << endl;
	discountPOMDPX(f);
	f << "\t<Variable>" << endl;
	f << "\t\t<StateVar vnamePrev=\"s_0\" vnameCurr=\"s_1\" fullyObs=\"true\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumObservations() << "</NumValues>" << endl ;
	f << "\t\t</StateVar>" << endl;
	f << "\t\t<StateVar vnamePrev=\"mh_0\" vnameCurr=\"mh_1\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumMDP() * GetMaxToStay() << "</NumValues>" << endl;
	f << "\t\t</StateVar>" << endl;
	f << "\t\t<ObsVar vname=\"obs\">" << endl;
	f << "\t\t\t<NumValues> " << GetNumObservations() << " </NumValues>" << endl;
	f << "\t\t</ObsVar>" << endl;
	f << "\t\t<ActionVar vname=\"action\">" << endl;
	f << "\t\t\t<NumValues>" << GetNumActions() << "</NumValues>" << endl << "\t\t</ActionVar>" << endl;
	f << "\t\t<RewardVar vname=\"reward\"/>" << endl;
	f << "\t</Variable>" << endl;
}

void ENVIRONMENT::discountPOMDPX(ostream &f) const {
	f << "\t<Discount> " << GetDiscount() << " </Discount>" << endl;
}

void ENVIRONMENT::initialStatePOMDPX(ostream &f) const {
	f << "\t\t\t<Var>s_0</Var>" << endl << "\t\t\t<Parent>null</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl << "\t\t\t\t<Entry>" << endl;
	f << "\t\t\t\t\t<Instance> - </Instance>" << endl;
	f << "\t\t\t\t\t<ProbTable> uniform </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
}

void ENVIRONMENT::initialBeliefStatePOMDPX(ostream &f) const {
	f << "\t<InitialStateBelief>" << endl;
	f << "\t\t<CondProb>" << endl;
	initialStatePOMDPX(f);
	f << "\t\t<CondProb>" << endl;
	f << "\t\t\t<Var>mh_0</Var>" << endl << "\t\t\t<Parent>null</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int m = 0; m < GetNumMDP(); m++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> s" << m << " </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> " << 1.0/GetNumMDP() << " </ProbTable>" << endl << "\t\t\t\t</Entry>";
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</InitialStateBelief>" << endl;
}

void ENVIRONMENT::stateTransitionPOMDPX(ostream &f, bool sparse) const {
	f << "\t<StateTransitionFunction>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>s_1</Var>" << endl;
	f << "\t\t\t<Parent>action s_0 mh_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int a = 0; a < GetNumActions(); a++ )
		for( int h = 0; h < GetMaxToStay(); h++ )
			for( int m = 0; m < GetNumMDP(); m++ )
				for( int s = 0; s < GetNumObservations(); s++ ) {
					if( sparse )
						for( int sprime = 0; sprime < GetNumObservations(); sprime++ ) {
							float t = GetTransition(m, s, a, sprime);
							if( t == 0 ) continue;
							f << "\t\t\t\t<Entry>" << endl;
							f << "\t\t\t\t\t<Instance> a" << a << " s" << s;
							f << " s" << m + h * GetNumMDP() << " s" << sprime << " </Instance>" << endl;
							f << "\t\t\t\t\t<ProbTable> " << t << " ";
							f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
						}
					else {
						f << "\t\t\t\t<Entry>" << endl;
						f << "\t\t\t\t\t<Instance> a" << a << " s" << s;
						f << " s" << m + h * GetNumMDP() << " - </Instance>" << endl;
						f << "\t\t\t\t\t<ProbTable> ";
						for( int sprime = 0; sprime < GetNumObservations(); sprime++ )
							f << GetTransition(m, s, a, sprime) << " ";
						f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
					}
				}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>mh_1</Var>" << endl;
	f << "\t\t\t<Parent>action mh_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int h = 1; h < GetMaxToStay(); h++ )
		for( int m = 0; m < GetNumMDP(); m++ ) {
			f << "\t\t\t\t<Entry>" << endl;
			f << "\t\t\t\t\t<Instance> * s" << m + h * GetNumMDP();
			f << " s" << m + (h-1) * GetNumMDP() <<" </Instance>" << endl;
			f << "\t\t\t\t\t<ProbTable> 1.0 </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
		}
	for( int m = 0; m < GetNumMDP(); m++ )
		for( int hprime = 0; hprime < GetMaxToStay(); hprime++ )
			for( int mprime = 0; mprime < GetNumMDP(); mprime++ ) {
				float t = GetMDPTransition(m, mprime) * GetTimeToStay(m, mprime, hprime);
				if( t == 0 ) continue;
				f << "\t\t\t\t<Entry>" << endl;
				f << "\t\t\t\t\t<Instance> * s" << m << " s" << mprime + hprime * GetNumMDP();
				f << " </Instance>" << endl << "\t\t\t\t\t<ProbTable> " << t << " ";
				f << "</ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</StateTransitionFunction>" << endl;
}

void ENVIRONMENT::observationFunctionPOMDPX(ostream &f) const {
	f << "\t<ObsFunction>" << endl;
	f << "\t\t<CondProb>" << endl << "\t\t\t<Var>obs</Var>" << endl;
	f << "\t\t\t<Parent>s_1</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int i = 0; i < GetNumObservations(); i++ ) {
		f << "\t\t\t\t<Entry>" << endl;
		f << "\t\t\t\t\t<Instance> s" << i << " o" << i << " </Instance>" << endl;
		f << "\t\t\t\t\t<ProbTable> 1.0 </ProbTable>" << endl << "\t\t\t\t</Entry>" << endl;
	}
	f << "\t\t\t</Parameter>" << endl << "\t\t</CondProb>" << endl;
	f << "\t</ObsFunction>" << endl;
}

void ENVIRONMENT::rewardFunctionPOMDPX(ostream &f) const {
	f << "\t<RewardFunction>" << endl;
	f << "\t\t<Func>" << endl << "\t\t\t<Var>reward</Var>" << endl;
	f << "\t\t\t<Parent>action s_0 mh_0</Parent>" << endl;
	f << "\t\t\t<Parameter type=\"TBL\">" << endl;
	for( int a = 0; a < GetNumActions(); a++ )
		for( int h = 0; h < GetMaxToStay(); h++ )
			for( int m = 0; m < GetNumMDP(); m++ )
				for( int o = 0; o < GetNumObservations(); o++ ) {
					float r = GetReward(m, o, a, 0);
					if( r == 0 ) continue;
					f << "\t\t\t\t<Entry>" << endl;
					f << "\t\t\t\t\t<Instance> a" << a << " s" << o;
					f << " s" << m + h * GetNumMDP() << " </Instance>" << endl;
					f << "\t\t\t\t\t<ValueTable> " << r << " ";
					f << "</ValueTable>" << endl << "\t\t\t\t</Entry>" << endl;
				}
	f << "\t\t\t</Parameter>" << endl << "\t\t</Func>" << endl;
	f << "\t</RewardFunction>" << endl;
}

void ENVIRONMENT::ToPOMDPX( string filename, bool sparse ) const {
	ofstream f(filename.c_str());
	headerToPOMDPX(f);
	initialBeliefStatePOMDPX(f);
	stateTransitionPOMDPX(f, sparse);
	observationFunctionPOMDPX(f);
	rewardFunctionPOMDPX(f);

	f << "</pomdpx>" << endl;

	f.close();
}

int ENVIRONMENT::discrete_rand(const double* array, const int size) const {
	discrete_distribution<> d(array, array + size);
	return d(_gen);
}

double ENVIRONMENT::rand_01() const {
	return _dis(_gen);
}
