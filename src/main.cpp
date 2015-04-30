#include "consensus.h"
#include "sailboat.h"
#include "trafficlight.h"
#include "battleship.h"
#include "mcts.h"
#include "network.h"
#include "pocman.h"
#include "rocksample.h"
#include "tag.h"
#include "experiment.h"
#include <boost/program_options.hpp>
#include <ctime>

using namespace std;
using namespace boost::program_options;

void UnitTests() {
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing COORD" << endl;
    COORD::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest();
}

void disableBufferedIO(void) {
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main(int argc, char* argv[]) {
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    string problem, outputfile, policy;
    int size, number, maxDuration, seed;
    double discount, bernoulli;
    bool meanmodel, show, shortest;

    options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("test", "run unit tests")
            ("problem", value<string>(&problem), "problem to run")
            ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
            ("policy", value<string>(&policy), "policy file (explicit POMDPs only)")
            ("size", value<int>(&size), "size of problem (problem specific)")
            ("number", value<int>(&number), "number of elements in problem (problem specific)")
            ("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
            ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
            ("maxdoubles", value<int>(&expParams.MaxDoubles), "maximum power of two simulations")
            ("runs", value<int>(&expParams.NumRuns), "number of runs")
            ("accuracy", value<double>(&expParams.Accuracy), "accuracy level used to determine horizon")
            ("horizon", value<int>(&expParams.UndiscountedHorizon), "horizon to use when not discounting")
            ("num steps", value<int>(&expParams.NumSteps), "number of steps to run when using average reward")
            ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
            ("autoexploration", value<bool>(&expParams.AutoExploration), "Automatically assign UCB exploration constant")
            ("exploration", value<double>(&searchParams.ExplorationConstant), "Manual value for UCB exploration constant")
            ("usetransforms", value<bool>(&searchParams.UseTransforms), "Use transforms")
            ("transformdoubles", value<int>(&expParams.TransformDoubles), "Relative power of two for transforms compared to simulations")
            ("transformattempts", value<int>(&expParams.TransformAttempts), "Number of attempts for each transform")
            ("userave", value<bool>(&searchParams.UseRave), "RAVE")
            ("ravediscount", value<double>(&searchParams.RaveDiscount), "RAVE discount factor")
            ("raveconstant", value<double>(&searchParams.RaveConstant), "RAVE bias constant")
            ("treeknowledge", value<int>(&knowledge.TreeLevel), "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
            ("rolloutknowledge", value<int>(&knowledge.RolloutLevel), "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
            ("smarttreecount", value<int>(&knowledge.SmartTreeCount), "Prior count for preferred actions during smart tree search")
            ("smarttreevalue", value<double>(&knowledge.SmartTreeValue), "Prior value for preferred actions during smart tree search")
            ("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
            ("discount", value<double>(&discount)->default_value(0.9), "Discount factor")
            ("maxDuration", value<int>(&maxDuration)->default_value(1), "Duration maximum (default 1 = HM-MDP)")
            ("seed", value<int>(&seed)->default_value(time(0)), "Seed of the mt19973 generator")
            ("bernoulli", value<double>(&bernoulli)->default_value(0.5), "Bernoulli value for consensus")
            ("meanmodel", value<bool>(&meanmodel)->default_value(false), "Mean model for simulator for consensus")
            ("show", value<bool>(&show)->default_value(false), "Shows the model and quit")
            ("shortest", value<bool>(&shortest)->default_value(false), "Shortest sequence for consensus")
            ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if ( vm.count("help") ) {
        cout << desc << "\n";
        return 1;
    }

    if ( vm.count("problem") == 0 ) {
        cout << "No problem specified" << endl;
        return 1;
    }

    if ( vm.count("test") ) {
        cout << "Running unit tests" << endl;
        UnitTests();
        return 0;
    }

    SIMULATOR* real = 0;
    SIMULATOR* simulator = 0;

    if ( problem == "battleship" ) {
        real      = new BATTLESHIP(size, size, number);
        simulator = new BATTLESHIP(safe_cast<const BATTLESHIP&>(*real));
    } else if ( problem == "pocman" ) {
        real      = new FULL_POCMAN;
        simulator = new FULL_POCMAN;
    } else if ( problem == "network" ) {
        real      = new NETWORK(size, number);
        simulator = new NETWORK(safe_cast<const NETWORK&>(*real));
    } else if ( problem == "rocksample" ) {
        real      = new ROCKSAMPLE(size, number);
        simulator = new ROCKSAMPLE(safe_cast<const ROCKSAMPLE&>(*real));
    } else if ( problem == "tag" ) {
        real      = new TAG(number);
        simulator = new TAG(safe_cast<const TAG&>(*real));
    } else if( problem == "trafficlight" ) {
        real      = new TRAFFICLIGHT(maxDuration, discount, seed);
        simulator = new TRAFFICLIGHT(safe_cast<const TRAFFICLIGHT&>(*real));
    } else if( problem == "sailboat" ) {
        real      = new SAILBOAT(size, maxDuration, discount, seed);
        simulator = new SAILBOAT(safe_cast<const SAILBOAT&>(*real));
    } else if( problem == "consensus" ) {
        auto r1  = new Rule;
        r1->premises = {UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, POS, UNDEF, UNDEF};       
        auto alt1 = new vector<PRED_MODIF>{ADD, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE};
        auto alt2 = new vector<PRED_MODIF>(9, NONE);
        r1->acts.push_back(alt1);
        r1->acts.push_back(alt2);
        
        auto r2  = new Rule;
        r2->premises = {UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, POS, UNDEF};       
        alt1 = new vector<PRED_MODIF>{NONE, NONE, NONE, NONE, NONE, NONE, ADD, NONE, NONE};
        alt2 = new vector<PRED_MODIF>(9, NONE);
        r2->acts.push_back(alt1);
        r2->acts.push_back(alt2);
        
        auto r3  = new Rule;
        r3->premises = vector<PRED_STATE>(9, UNDEF);       
        alt1 = new vector<PRED_MODIF>{NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, ADD};
        alt2 = new vector<PRED_MODIF>{NONE, NONE, NONE, NONE, ADD, NONE, NONE, NONE, NONE};
        r3->acts.push_back(alt1);
        r3->acts.push_back(alt2);
        
        auto r4  = new Rule;
        r4->premises = {UNDEF, UNDEF, UNDEF, POS, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF};
        alt1 = new vector<PRED_MODIF>{ADD, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE};
        alt2 = new vector<PRED_MODIF>{NONE, NONE, ADD, NONE, NONE, NONE, NONE, NONE, NONE};
        r4->acts.push_back(alt1);
        r4->acts.push_back(alt2);
        
        auto t1 = new vector<Rule*>();
        t1->push_back(r1);
        t1->push_back(r2);
        t1->push_back(r3);
        t1->push_back(r4);
                          
        r1  = new Rule;
        r1->premises = {POS, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF, UNDEF};       
        alt1 = new vector<PRED_MODIF>{NONE, ADD, NONE, NONE, NONE, NONE, NONE, NONE, NONE};
        alt2 = new vector<PRED_MODIF>(9, NONE);
        r1->acts.push_back(alt1);
        r1->acts.push_back(alt2);
        
        r2  = new Rule;
        r2->premises = vector<PRED_STATE>(9, UNDEF);       
        alt1 = new vector<PRED_MODIF>{NONE, NONE, NONE, ADD, NONE, NONE, NONE, NONE, NONE};
        alt2 = new vector<PRED_MODIF>{NONE, NONE, NONE, NONE, NONE, NONE, NONE, ADD, NONE};
        r2->acts.push_back(alt1);
        r2->acts.push_back(alt2);
        
        r3  = new Rule;
        r3->premises = {UNDEF, UNDEF, UNDEF, UNDEF, POS, UNDEF, UNDEF, UNDEF, UNDEF};
        alt1 = new vector<PRED_MODIF>{NONE, NONE, NONE, NONE, NONE, ADD, NONE, NONE, NONE};
        alt2 = new vector<PRED_MODIF>(9, NONE);
        r3->acts.push_back(alt2);
        r3->acts.push_back(alt1);
        
        auto t2 = new vector<Rule*>();
        t2->push_back(r1);
        t2->push_back(r2);
        t2->push_back(r3);
        
        auto atks = multimap<int, int>();
        atks.emplace(0, 1);
        atks.emplace(3, 4);
        atks.emplace(4, 5);
        atks.emplace(6, 5);
        atks.emplace(7, 6);
        atks.emplace(8, 7);
        
        real      = new CONSENSUS(9, atks, vector<int>{3,2}, vector<vector<Rule*>*>{t1,t2}, 
                maxDuration, discount, seed, bernoulli, meanmodel, shortest);
        simulator = new CONSENSUS(safe_cast<const CONSENSUS&>(*real));
    } else {
        cout << "Unknown problem" << endl;
        exit(1);
    }

    if(!show) {
        cout << "Used seed: " << seed << endl;        
        simulator->SetKnowledge(knowledge);
        EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams);
        experiment.DiscountedReturn();
    }
    delete real;
    delete simulator;
    return 0;
}
