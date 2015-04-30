#include "consensus.h"
#include <cmath>
#include "utils.h"
#include <random>

using namespace UTILS;

CONSENSUS::CONSENSUS(int numOfArgs, multimap<int, int> atks, vector<int> teams,
                     vector<vector<Rule*>*> rules, int maxDur, double discount, int seed, 
                     double bernoulli, bool meanModel, bool shortest) :
//HS3MDP(numOfAgents, teams.size() * pow(2, numOfArgs) * pow(2, numOfAtks), teams.size(), maxDur, discount, seed),
HS3MDP(teams[0] + teams[1], teams.size() * pow(2, numOfArgs), pow(2, teams[0] + teams[1]), maxDur, discount, seed),
AllModesProbabilities(2),        
Rules(rules),
Atks(atks){
    Shortest    = shortest;
    MeanModel   = meanModel;
    _Bernoulli  = bernoulli;
    NumOfArgs   = numOfArgs;
    //NumOfAtks   = numOfAtks;
    NumOfAgents = teams[0] + teams[1];
    RandomSeed(seed);
    Goals.insert(Random(NumOfArgs));
    AllModesProbabilities[0] = new ProbaVectors();
    AllModesProbabilities[1] = new ProbaVectors();
    for( int i = 0; i < teams[0]; i++ ) {
        auto v01 = new vector<vector<double>*>();
        auto v02 = new vector<vector<double>*>();
        for( unsigned int j = 0; j < Rules[0]->size() + 1; j++ ) {
            double val1 = abs(RandomDouble(0,1) - 0.5);
            double val2 = abs(RandomDouble(0,1) - 0.5);
            auto v1 = new vector<double>{ 1 - val1, val1 };
            auto v2 = new vector<double>{ val2, 1 - val2 };
            v01->push_back(v1);
            v02->push_back(v2);
        }
        AllModesProbabilities[0]->push_back(v01);
        AllModesProbabilities[1]->push_back(v02);
    }
    for( int i = 0; i < teams[1]; i++ ) {
        auto v11 = new vector<vector<double>*>();
        auto v12 = new vector<vector<double>*>();
        for( unsigned int j = 0; j < Rules[1]->size() + 1; j++ ) {
            double val1 = abs(RandomDouble(0,1) - 0.5);
            double val2 = abs(RandomDouble(0,1) - 0.5);
            auto v1 = new vector<double>{ 1 - val1, val1 };
            auto v2 = new vector<double>{ val2, 1 - val2 };
            v11->push_back(v1);
            v12->push_back(v2);
        }
        AllModesProbabilities[0]->push_back(v11);
        AllModesProbabilities[1]->push_back(v12);
    }
    int count = 0;
    for( unsigned int i = 0; i < teams.size(); i++ )
        for( int j = 0; j < teams[i]; j++ ) {
            Teams[count] = i;
            count++;
        }   
    
    cout << "Real" << endl;
    for(auto m : AllModesProbabilities) {
        for(auto a : *m) {
            for(auto r : *a) {
                for(auto alt : *r)
                    cout << alt << " ";
                cout << endl;
            }
            cout << endl;
        }
        cout << endl;
    }
}

CONSENSUS::CONSENSUS(const CONSENSUS& orig) :
HS3MDP(orig),
Teams(orig.Teams),
Goals(orig.Goals),
Atks(orig.Atks) {
    NumOfArgs   = orig.NumOfArgs;
    //NumOfAtks   = orig.NumOfAtks;
    NumOfAgents = orig.NumOfAgents;
    _Bernoulli  = orig._Bernoulli;
    Shortest    = orig.Shortest;
    if(orig.MeanModel) {
        auto newmode1 = new ProbaVectors();
        auto newmode2 = new ProbaVectors();
        for(int a = 0; a < NumOfAgents; a++) {
            auto newv1 = new vector<vector<double>*>();
            auto newv2 = new vector<vector<double>*>();
            auto rules = orig.AllModesProbabilities[0]->at(a);
            for(unsigned int r = 0; r < rules->size(); r++ ) {
                auto rvect = new vector<double>();
                for(unsigned int alt = 0; alt < rules->at(r)->size(); alt++) {
                    double value = (rules->at(r)->at(alt) + orig.AllModesProbabilities[1]->at(a)->at(r)->at(alt))/2.0;                    
                    rvect->push_back(value);
                }
                newv1->push_back(rvect);
                newv2->push_back(new vector<double>(*rvect));
            }
            newmode1->push_back(newv1);
            newmode2->push_back(newv2);
        }
        AllModesProbabilities.push_back(newmode1);
        AllModesProbabilities.push_back(newmode2);
    } else {
        for( auto mode : orig.AllModesProbabilities ) {
            auto newmode = new ProbaVectors();
            for( auto agent : *mode ) {
                auto newv = new vector<vector<double>*>();
                for( auto r : *agent )
                    newv->push_back(new vector<double>(*r));
                newmode->push_back(newv);
            }
            AllModesProbabilities.push_back(newmode);
        }
    }
    for( auto team_rules : orig.Rules ) {
        auto v = new vector<Rule*>();
        for( auto r : *team_rules )
            v->push_back(new Rule(*r));
        Rules.push_back(v);
    }    
    
    cout << "Copy" << endl;
    for(auto m : AllModesProbabilities) {
        for(auto a : *m) {
            for(auto r : *a) {
                for(auto alt : *r)
                    cout << alt << " ";
                cout << endl;
            }
            cout << endl;
        }
        cout << endl;
    }
}

CONSENSUS::~CONSENSUS() {
    for( auto m : AllModesProbabilities ) {
        for( auto a : *m ) {
            for( auto r : *a ) delete r;
            delete a;
        }
        delete m;
    }
    for( auto t : Rules ) {
        for( auto r : *t ) delete r;
        delete t;
    }
    for( auto p : Trace ) {
        delete get<0>(p);
        delete get<3>(p);
    }
}

CONSENSUS_STATE* CONSENSUS::CreateStartState() const {
    CONSENSUS_STATE *state = MemoryPool.Allocate();
    state->Public.assign(NumOfArgs, false);
    //state->Internals       = new vector<vector<bool>*>();
    //state->Internals->push_back(new vector<bool>{true, true, true, true, true, false, false, false});
    //state->Internals->push_back(new vector<bool>{false, false, false, false, false, true, true, true});
    state->LastTeam        = -1;
    state->SoloModes.assign(NumOfAgents, 1);
    state->Mode            = 0;
    state->Duration        = MaxDuration;
    state->HS3MDPState     = 0;
    state->POMDPState      = 0;

    return state;
}

CONSENSUS_STATE* CONSENSUS::Copy(const STATE& state) const {
    const CONSENSUS_STATE& cstate = safe_cast<const CONSENSUS_STATE&>(state);
    CONSENSUS_STATE* newstate     = MemoryPool.Allocate();
    newstate->POMDPState          = cstate.POMDPState;
    newstate->HS3MDPState         = cstate.HS3MDPState;
    newstate->Mode                = cstate.Mode;
    newstate->Duration            = cstate.Duration;
    newstate->LastTeam            = cstate.LastTeam;
    newstate->Public.assign(cstate.Public.begin(), cstate.Public.end());
    newstate->SoloModes.assign(cstate.SoloModes.begin(), cstate.SoloModes.end());

    //newstate->Internals                   = new vector<vector<bool>*>();
    //for( auto v : *(consensusstate.Internals) )
    //    newstate->Internals->push_back(new vector<bool>(*v));

    return newstate;
}

bool CONSENSUS::Step(STATE& state, int action, int& observation, double& reward) const {
    CONSENSUS_STATE &s = safe_cast<CONSENSUS_STATE&>(state), *starting, *ending;
    int team = Teams.at(action);
    vector<Rule*> applicable;
    if( !IsCopy() ) starting = new CONSENSUS_STATE(s);
    bool modified = false;
    /*for( auto r : *(Rules[team]) ) {
        bool compatible = true;
        for( int i = 0; i < NumOfArgs; i++ ) {
            PRED_STATE prem = r->premises[i];
            bool pred       = s.Internals->at(team)->at(i);
            if(prem == UNDEF)
                continue;
            if((pred && prem == POS) || (!pred && prem == NEG))
                continue;
            else {
                compatible = false;
                break;
            }          
        }
        if(compatible) {
            for(int i = NumOfArgs; i < NumOfArgs * 2; i++) {
                PRED_STATE prem = r->premises[i];
                bool pred       = s.Public->at(i);
                if(prem == UNDEF)
                    continue;
                if((pred && prem == POS) || (!pred && prem == NEG))
                    continue;
                else {
                    compatible = false;
                    break;
                }            
            }
            if(compatible) 
                applicable.push_back(r);
        }                                                          
    }*/

    for( auto r : *(Rules[team]) ) {
        bool compatible = true;
        for( int i = 0; i < NumOfArgs; i++ ) {
            PRED_STATE prem = r->premises[i];
            bool pred       = s.Public[i];
            auto status     = vector<bool>(s.Public);
            pred            &= !checkAttacked(i, status);
            if( (pred && prem == NEG) || (!pred && prem == POS) ) {
                compatible = false;
                break;
            }
        }
        if( compatible ) {
            //cout << r->to_s() << endl;
            applicable.push_back(r);
        }
    }

    unsigned int r = Random(applicable.size() + 1);
    auto probavect = AllModesProbabilities[1 - s.SoloModes[action]]->at(action)->at(r);
    discrete_distribution<int> act(probavect->begin(), probavect->end());
    int alt = act(Gen);
    if( r != applicable.size() ) {
        auto modif = applicable[r]->acts[alt];
        /*for( int i = 0; i < NumOfArgs; i++ ) {
            switch(modif->at(i)) {
                case ADD:
                    s.Internals->at(team)->at(i) = true;
                    break;
                case REMOVE:
                    s.Internals->at(team)->at(i) = false;
                    break;
                default: break;
            }
        }*/
        for( int i = 0; i < NumOfArgs ; i++ ) {
            switch(modif->at(i)) {
                case ADD:
                    if(!s.Public[i]) {
                        s.Public[i] = true;
                        modified = true;
                    }
                    break;
                case REMOVE:
                    if(s.Public[i]) {
                        modified = true;
                        s.Public[i] = false;
                    }
                    break;
                default: break;
            }
        }
    }

    if( team == s.LastTeam )
        reward = -100;
    else {
        reward = 0;
        auto status = vector<bool>(s.Public);
        for( int i : Goals ) {
            auto goal = s.Public[i] && !checkAttacked(i, status);
            if( goal )
                reward += 10;
        }
    }
    if(!modified && Shortest) 
        reward -= 20;
    s.LastTeam = Teams.at(action);

    auto shift  = [](int x, int y) {
        return (x << 1) + y;
    };
    observation = accumulate(s.Public.begin(), s.Public.end(), 0, shift);
    observation = (observation << 1) + s.LastTeam;
    
    s.Duration--;
    if( s.Duration == 0 ) {
        if( Bernoulli(0.3) )
            s.Duration = MaxDuration / 3;
        else {
            if( Bernoulli(0) )
                s.Duration = MaxDuration / 2;
            else
                s.Duration = MaxDuration / 3.0 * 2;
        }
        discrete_distribution<int> mode(s.SoloModes.begin(), s.SoloModes.end());
        int changingAgent = mode(Gen);
        s.SoloModes[changingAgent] = 0;        
    }

    if( !IsCopy() ) {
        //cout << applicable.size() << " " << r << " " << alt << endl;
        ending = new CONSENSUS_STATE(s);
        Trace.push_back(make_tuple(starting, action, (r==applicable.size() ? NULL : applicable[r]), ending));
    }
        
    return (r == applicable.size() && alt == 1 && (Bernoulli(_Bernoulli) || s.SoloModes[action] == 0));
}

bool CONSENSUS::checkAttacked(int i, vector<bool> &status) const {
    bool attacked = false;
    auto range    = Atks.equal_range(i);
    for( auto atk = range.first; atk != range.second; ++atk )
        if( status[atk->second] && !checkAttacked(atk->second, status) ) {
            attacked = true;
            break;
        }

    if( attacked )
        status[i] = false;
    return attacked;
}

bool CONSENSUS::LocalMove(STATE&, const HISTORY& history, int stepObs, const STATUS&) const {    
    return stepObs == history.Back().Observation;
}