#include "sailboat.h"
#include <cmath>

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define N_S 0
#define W_E 1

SAILBOAT::SAILBOAT(int gridSize, int maxDuration, double discount, int seed) :
SPARSE_HS3MDP(2, gridSize, 4, maxDuration, discount, seed) {
    SideSize      = sqrt(gridSize);
    RewardDefault = 0;

    InModeTransitionProbabilites.reserve(NumModes);
    ModeTransitionProbabilities.reserve(NumModes);
    InModeRewards.reserve(NumModes);
    DurationProbabilities.reserve(NumModes);

    for( int m = 0; m < NumModes; m++ ) {
        DurationProbabilities.push_back(new vector<vector<double>*>(NumModes, NULL));
        ModeTransitionProbabilities.push_back(new vector<double>(NumModes, 0));
        InModeTransitionProbabilites.push_back(new vector<vector<SPARSEMAP*>*>(NumObservations, NULL));
        InModeRewards.push_back(new vector<map<int, double>*>(NumObservations, NULL));
        for( int o = 0; o < NumObservations; o++ ) {
            InModeRewards.at(m)->at(o) = new map<int, double>();
            InModeTransitionProbabilites.at(m)->at(o) = new vector<SPARSEMAP*>(NumActions, NULL);

            for( int a = 0; a < NumActions; a++ ) {
                auto indexv = new vector<int>();
                auto probav = new vector<double>();
                auto indexm = new map<int, int>();
                SPARSEMAP* sparsemap = new SPARSEMAP(indexv, probav, indexm);
                InModeTransitionProbabilites.at(m)->at(o)->at(a) = sparsemap;

                int x = o % SideSize, y = o / SideSize;

                for( int dx = -1; dx <= 1; dx++ ) {
                    if( (x + dx) < 0 ) continue;
                    if( (x + dx) >= SideSize ) continue;
                    for( int dy = -1; dy <= 1; dy++ ) {
                        if( (y + dy) < 0 ) continue;
                        if( (y + dy) >= SideSize ) continue;
                        
                        double p = GetTProbability(m, x, y, a, x + dx, y + dy);
                        if( p > 0 ) {
                            int newobs = (y+dy)*SideSize + (x+dx);
                            indexm->emplace(newobs, probav->size());
                            probav->push_back(p);
                            indexv->push_back(newobs);     
                            
                            if( newobs == (NumObservations - 1))
                                InModeRewards.at(m)->at(o)->emplace(a, 1);
                        }
                    }
                }
                
                indexv->shrink_to_fit();
                probav->shrink_to_fit();
            }
        }

        ModeTransitionProbabilities.at(m)->at((m + 2) % 4) = 0.1;
        ModeTransitionProbabilities.at(m)->at((m + 3) % 4) = 0.2;
        ModeTransitionProbabilities.at(m)->at(m)           = 0.5;
        ModeTransitionProbabilities.at(m)->at((m + 1) % 4) = 0.2;

        for( int mprime = 0; mprime < NumModes; mprime++ )
            DurationProbabilities.at(m)->at(mprime) = UTILS::GaussianWeights(MaxDuration, Gen);
    }
}

SAILBOAT::SAILBOAT(const SAILBOAT & orig) :
SPARSE_HS3MDP(orig) {
    SideSize = orig.SideSize;
}

SAILBOAT::~SAILBOAT() {
}

HS3MDP_STATE* SAILBOAT::CreateStartState() const {
    HS3MDP_STATE* state = SPARSE_HS3MDP::CreateStartState();
    state->HS3MDPState  = 0;
    state->POMDPState   = 0;
    
    return state;
}

double SAILBOAT::GetTProbability(int mode, int x, int y, int action, int newx, int newy) const {
    int cote = SideSize;
    switch (mode) {
        case NORTH:
            if ( action == W_E ) {
                if ( (newy - y) == 1 ) {
                    if ( abs(newx - x) == 1 )
                        return 0.1;
                    else
                        return 0.8;
                } else if ( newy == y && x == newx ) {
                    if ( y == (cote - 1) )
                        return 1;
                    else if ( x == 0 || x == (cote - 1) )
                        return 0.1;
                }
            } else if ( (x == newx) && (y == newy) )
                return 1;
            break;

        case SOUTH:
            if ( action == W_E ) {
                if ( (newy - y) == -1 ) {
                    if ( abs(newx - x) == 1 )
                        return 0.1;
                    else
                        return 0.8;
                } else if ( newy == y && x == newx ) {
                    if ( y == 0 )
                        return 1;
                    else if ( x == 0 || x == (cote - 1) )
                        return 0.1;
                }
            } else if ( (x == newx) && (y == newy) )
                return 1;
            break;

        case WEST:
            if ( action == N_S ) {
                if ( (newx - x) == 1 ) {
                    if ( abs(newy - y) == 1 )
                        return 0.1;
                    else
                        return 0.8;
                } else if ( newx == x && y == newy ) {
                    if ( x == (cote - 1) )
                        return 1;
                    else if ( y == 0 || y == (cote - 1) )
                        return 0.1;
                }
            } else if ( (x == newx) && (y == newy) )
                return 1;
            break;

        case EAST:
            if ( action == N_S ) {
                if ( (newx - x) == -1 ) {
                    if ( abs(newy - y) == 1 )
                        return 0.1;
                    else
                        return 0.8;
                } else if ( newx == x && y == newy ) {
                    if ( x == 0 )
                        return 1;
                    else if ( y == 0 || y == (cote - 1) )
                        return 0.1;
                }
            } else if ( (x == newx) && (y == newy) )
                return 1;
            break;
    }
    return 0;
}

bool SAILBOAT::Step(STATE& state, int action, int& observation, double& reward) const {
    SPARSE_HS3MDP::Step(state, action, observation, reward);
    if( observation == (NumObservations - 1))
        return true;
    return false;
}
