#include "experiment.h"
#include "probamcts.h"
#include "boost/timer.hpp"
#include "optimalpolicy.h"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
	NumSteps(100000),
	SimSteps(1000),
	TimeOut(3600),
	MinDoubles(0),
	MaxDoubles(20),
	TransformDoubles(-4),
	TransformAttempts(1000),
	Accuracy(0.01),
	UndiscountedHorizon(1000),
	AutoExploration(true),
	ProbaMCTS(false),
	Optimal(false),
	OptimalFile("")
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
	const SIMULATOR& simulator, const string& outputFile,
	EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
	Simulator(simulator),
	ExpParams(expParams),
	SearchParams(searchParams),
	OutputFile(outputFile.c_str())
{
	if (ExpParams.AutoExploration)
	{
		if (SearchParams.UseRave)
			SearchParams.ExplorationConstant = 0;
		else
			SearchParams.ExplorationConstant = simulator.GetRewardRange();
	}
	MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::Run()
{
	boost::timer timer;

	STATE* state = Real.CreateStartState();
	if (SearchParams.Verbose >= 1)
		Real.DisplayState(*state, cout);

	MCTS *mcts;
	if( ExpParams.Optimal )
		mcts = new OptimalPolicy(ExpParams.OptimalFile, state, Simulator, SearchParams);
	else {
		if( ExpParams.ProbaMCTS )
		mcts = new PROBA_MCTS(Simulator, SearchParams);
	else
		mcts = new MCTS(Simulator, SearchParams);
	}
	mcts->InitialiseRoot();

	double undiscountedReturn = 0.0;
	double discountedReturn = 0.0;
	double discount = 1.0;
	bool terminal = false;
	bool outOfParticles = false;
	int t;

	for (t = 0; t < ExpParams.NumSteps; t++)
	{
		int observation;
		double reward;
		int action = mcts->SelectAction();
		terminal = Real.Step(*state, action, observation, reward);

		Results.Reward.Add(reward);
		undiscountedReturn += reward;
		discountedReturn += reward * discount;
		discount *= Real.GetDiscount();

		if (SearchParams.Verbose >= 1)
		{
			Real.DisplayAction(action, cout);
			Real.DisplayState(*state, cout);
			Real.DisplayObservation(*state, observation, cout);
			Real.DisplayReward(reward, cout);
		}

		if (terminal)
		{
			cout << "Terminated" << endl;
			break;
		}

		outOfParticles = !mcts->Update(action, observation);
		if (outOfParticles)
			break;

		if (timer.elapsed() > ExpParams.TimeOut)
		{
			cout << "Timed out after " << t << " steps in "
				<< Results.Time.GetTotal() << "seconds" << endl;
			break;
		}
	}

	if (outOfParticles)
	{
		cout << "Out of particles, finishing episode with SelectRandom" << endl;
		HISTORY history = mcts->GetHistory();
		while (++t < ExpParams.NumSteps)
		{
			int observation;
			double reward;

			// This passes real state into simulator!
			// SelectRandom must only use fully observable state
			// to avoid "cheating"
			int action = Simulator.SelectRandom(*state, history, mcts->GetStatus());
			terminal = Real.Step(*state, action, observation, reward);

			Results.Reward.Add(reward);
			undiscountedReturn += reward;
			discountedReturn += reward * discount;
			discount *= Real.GetDiscount();

			if (SearchParams.Verbose >= 1)
			{
				Real.DisplayAction(action, cout);
				Real.DisplayState(*state, cout);
				Real.DisplayObservation(*state, observation, cout);
				Real.DisplayReward(reward, cout);
			}

			if (terminal)
			{
				cout << "Terminated" << endl;
				break;
			}

			history.Add(action, observation);
		}
	}

	Results.Time.Add(timer.elapsed());
	Results.UndiscountedReturn.Add(undiscountedReturn);
	Results.DiscountedReturn.Add(discountedReturn);
	cout << "Discounted return = " << discountedReturn
		<< ", average = " << Results.DiscountedReturn.GetMean() << endl;
	cout << "Undiscounted return = " << undiscountedReturn
		<< ", average = " << Results.UndiscountedReturn.GetMean() << endl;

	delete mcts;
}

bool EXPERIMENT::MultiRun()
{
	bool timedout = false;
	for (int n = 0; n < ExpParams.NumRuns; n++)
	{
		cout << "Starting run " << n + 1 << " with "
			<< SearchParams.NumSimulations << " simulations... " << endl;
		Run();
		if (Results.Time.GetTotal() > ExpParams.TimeOut)
		{
			timedout = true;
			cout << "Timed out after " << n << " runs in "
				<< Results.Time.GetTotal() << "seconds" << endl;
			break;
		}
	}
	return timedout;
}

void EXPERIMENT::DiscountedReturn()
{
	cout << "Main runs" << endl;
	OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

	SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

	bool timedout;
	for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
	{
		SearchParams.NumSimulations = 1 << i;
		SearchParams.NumStartStates = 1 << i;
		if (i + ExpParams.TransformDoubles >= 0)
			SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
		else
			SearchParams.NumTransforms = 1;
		SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

		Results.Clear();
		timedout = MultiRun();

		cout << "Simulations = " << SearchParams.NumSimulations << endl
			<< "Runs = " << Results.Time.GetCount() << endl
			<< "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
			<< " +- " << Results.UndiscountedReturn.GetStdErr() << endl
			<< "Discounted return = " << Results.DiscountedReturn.GetMean()
			<< " +- " << Results.DiscountedReturn.GetStdErr() << endl
			<< "Time = " << Results.Time.GetMean() << endl;
		OutputFile << SearchParams.NumSimulations << "\t\t"
			<< Results.Time.GetCount() << "\t"
			<< Results.UndiscountedReturn.GetMean() << "\t\t\t"
			<< Results.UndiscountedReturn.GetStdErr() << "\t\t"
			<< Results.DiscountedReturn.GetMean() << "\t\t"
			<< Results.DiscountedReturn.GetStdErr() << "\t\t"
			<< Results.Time.GetMean() << endl;

		if( timedout )
			break;
	}
}

void EXPERIMENT::AverageReward()
{
	cout << "Main runs" << endl;
	OutputFile << "Simulations\tSteps\tAverage reward\tAverage time\n";

	SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

	for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
	{
		SearchParams.NumSimulations = 1 << i;
		SearchParams.NumStartStates = 1 << i;
		if (i + ExpParams.TransformDoubles >= 0)
			SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
		else
			SearchParams.NumTransforms = 1;
		SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

		Results.Clear();
		Run();

		cout << "Simulations = " << SearchParams.NumSimulations << endl
			<< "Steps = " << Results.Reward.GetCount() << endl
			<< "Average reward = " << Results.Reward.GetMean()
			<< " +- " << Results.Reward.GetStdErr() << endl
			<< "Average time = " << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
		OutputFile << SearchParams.NumSimulations << "\t"
			<< Results.Reward.GetCount() << "\t"
			<< Results.Reward.GetMean() << "\t"
			<< Results.Reward.GetStdErr() << "\t"
			<< Results.Time.GetMean() / Results.Reward.GetCount() << endl;
	}
}

//----------------------------------------------------------------------------
