#ifndef CONSENSUS_H
#define	CONSENSUS_H

#include "hs3mdp.h"
#include <vector>
#include <set>
#include <map>
#include <tuple>
#include <string>
#include <sstream>

using namespace std;

typedef vector<vector<vector<double>*>*> ProbaVectors; //forall agent, forall rule, forall alt

enum PRED_STATE {
    POS, NEG, UNDEF
} ;

enum PRED_MODIF {
    ADD, REMOVE, NONE
} ;

class CONSENSUS_STATE : public HS3MDP_STATE {
public:
    //vector<vector<bool>*> *Internals = NULL;
    vector<bool> Public;
    vector<int>  SoloModes;
    int LastTeam;

    string to_s() const {
        stringstream s;
        s << "[";
        for( bool p : Public )
            s << p << ", ";
        s << "], [";
        for( int p : SoloModes )
            s << p << ", ";
        s << "] " << LastTeam;
        return s.str();
    }
} ;

struct Rule {
    vector<PRED_STATE> premises;
    vector<vector<PRED_MODIF>*> acts;

    Rule() {
    }

    Rule(const Rule &orig) : premises(orig.premises) {
        for( auto a : orig.acts )
            acts.push_back(new vector<PRED_MODIF>(*a));
    }

    ~Rule() {
        for( auto a : acts ) delete a;
    }
    
    string to_s() const {
        stringstream s;
        s << "[";
        for(auto p : premises)
            s << p << ", ";
        s << "] => ";
        for(auto a : acts) {
            s << "[";
            for(auto p : *a)
                s << p << ", ";
            s << "] & ";
        }
        return s.str();
    }
} ;

class CONSENSUS : public HS3MDP {
public:
    CONSENSUS(int numOfArgs, multimap<int, int> atks, vector<int> teams,
              vector<vector<Rule*>*> rules, int maxDur, double discount, int seed, 
              double bernoulli, bool meanModel, bool shortest);
    CONSENSUS(const CONSENSUS& orig);
    virtual ~CONSENSUS();

    virtual CONSENSUS_STATE* CreateStartState() const;
    virtual CONSENSUS_STATE* Copy(const STATE& state) const;
    virtual bool Step(STATE& state, int action, int& observation, double& reward) const;

    vector<tuple<CONSENSUS_STATE*, int, Rule*, CONSENSUS_STATE*> > getTrace() const {
        return Trace;
    }

    void clearTrace() const {
        for( auto p : Trace ) {
            delete get<0>(p);
            delete get<3>(p);
        }
        Trace.clear();
    }
    
    bool LocalMove(STATE&, const HISTORY& history, int stepObs, const STATUS&) const;
    
    CONSENSUS_STATE& getCurrent() const { return _current; }
    void assignCurrent(STATE* s) const {
        CONSENSUS_STATE *cs = safe_cast<CONSENSUS_STATE*>(s);
        cs->Public.assign(_current.Public.begin(), _current.Public.end());
        cs->LastTeam = _current.LastTeam;
    }
    
    void GenerateLegal(const STATE&, const HISTORY&, std::vector<int>& actions, const STATUS&) const;

private:
    bool MeanModel, Shortest;
    double _Bernoulli;
    map<int, int> Teams;
    vector<ProbaVectors*> AllModesProbabilities;
    
    vector<vector<Rule*>*> Rules;
    set<int> Goals;
    int NumOfArgs, NumOfAgents;
    multimap<int, int> Atks;
    mutable vector<tuple<CONSENSUS_STATE*, int, Rule*, CONSENSUS_STATE*> > Trace;

    mutable MEMORY_POOL<CONSENSUS_STATE> MemoryPool;

    bool checkAttacked(int i, vector<bool> &status) const;
    
    mutable CONSENSUS_STATE _current;
} ;

#endif	/* CONSENSUS_H */

