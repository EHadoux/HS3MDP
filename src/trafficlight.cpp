#include "trafficlight.h"
#include "utils.h"

TRAFFICLIGHT::TRAFFICLIGHT(int maxDuration, double discount, int seed) :
HS3MDP(2, 8, 2, maxDuration, discount, seed) {
    InModeTransitionProbabilites.reserve(NumModes);
    ModeTransitionProbabilities.reserve(NumModes);
    InModeRewards.reserve(NumModes);
    DurationProbabilities.reserve(NumModes);
    
    int offset1 = 0, offset2 = 0, offset3 = 6;    
    for( int m = 0; m < NumModes; m++ ) {
        DurationProbabilities.push_back(new vector<vector<double>*>(NumModes, NULL));
        ModeTransitionProbabilities.push_back(new vector<double>(NumModes, 0));
        InModeTransitionProbabilites.push_back(
            new vector<vector<vector<double>*>*>(NumObservations, NULL));
        InModeRewards.push_back(new vector<vector<double>*>(NumObservations, NULL)); 
        for( int o = 0; o < NumObservations; o++ ) {
            InModeRewards.at(m)->at(o) = new vector<double>(NumActions, 0);
            InModeTransitionProbabilites.at(m)->at(o) = new vector<vector<double>*>(NumActions, NULL);
            
            for( int a = 0; a < NumActions; a++ ) {
                InModeTransitionProbabilites.at(m)->at(o)->at(a) = new vector<double>(NumObservations, 0);
            }            
        }
        
        InModeRewards.at(m)->at(2)->at(0) = InModeRewards.at(m)->at(5)->at(0) = 
        InModeRewards.at(m)->at(6)->at(0) = InModeRewards.at(m)->at(7)->at(0) =
        InModeRewards.at(m)->at(2)->at(1) = InModeRewards.at(m)->at(5)->at(1) = 
        InModeRewards.at(m)->at(6)->at(1) = InModeRewards.at(m)->at(7)->at(1) = -1;
        
        if ( m == 0 ) {
            offset1 = 4;
            offset2 = 2;
        } else {
            offset1 = 2;
            offset2 = 4;
        }
        for ( int a = 0; a < NumActions; a++ ) {
            InModeTransitionProbabilites.at(m)->at(0)->at(a)->at(a) = 
            InModeTransitionProbabilites.at(m)->at(1)->at(a)->at(a) = 
            InModeTransitionProbabilites.at(m)->at(3)->at(a)->at(a) = 
            InModeTransitionProbabilites.at(m)->at(4)->at(a)->at(a) = 0.63;
            
            InModeTransitionProbabilites.at(m)->at(0)->at(a)->at(a + offset1) = 
            InModeTransitionProbabilites.at(m)->at(1)->at(a)->at(a + offset1) = 
            InModeTransitionProbabilites.at(m)->at(4)->at(a)->at(a + offset1) = 
            InModeTransitionProbabilites.at(m)->at(3)->at(a)->at(a + offset1) = 0.27;
            
            InModeTransitionProbabilites.at(m)->at(0)->at(a)->at(a + offset2) = 
            InModeTransitionProbabilites.at(m)->at(1)->at(a)->at(a + offset2) = 
            InModeTransitionProbabilites.at(m)->at(4)->at(a)->at(a + offset2) = 
            InModeTransitionProbabilites.at(m)->at(3)->at(a)->at(a + offset2) = 0.07;
            
            InModeTransitionProbabilites.at(m)->at(0)->at(a)->at(a + offset3) = 
            InModeTransitionProbabilites.at(m)->at(1)->at(a)->at(a + offset3) = 
            InModeTransitionProbabilites.at(m)->at(4)->at(a)->at(a + offset3) = 
            InModeTransitionProbabilites.at(m)->at(3)->at(a)->at(a + offset3) = 0.03;
        }
        
        for( int mprime = 0; mprime < NumModes; mprime++ )
            DurationProbabilities.at(m)->at(mprime) = UTILS::GaussianWeights(MaxDuration, Gen);
    }
    
    ModeTransitionProbabilities.at(0)->at(0) = ModeTransitionProbabilities.at(1)->at(1) = 0.9;
    ModeTransitionProbabilities.at(1)->at(0) = ModeTransitionProbabilities.at(0)->at(1) = 0.1;    
    
    InModeTransitionProbabilites.at(0)->at(5)->at(0)->at(4) = 
    InModeTransitionProbabilites.at(0)->at(7)->at(0)->at(4) = 
    InModeTransitionProbabilites.at(0)->at(5)->at(1)->at(5) = 
    InModeTransitionProbabilites.at(0)->at(7)->at(1)->at(5) = 
    InModeTransitionProbabilites.at(1)->at(2)->at(0)->at(2) = 
    InModeTransitionProbabilites.at(1)->at(6)->at(0)->at(2) = 
    InModeTransitionProbabilites.at(1)->at(2)->at(1)->at(3) = 
    InModeTransitionProbabilites.at(1)->at(6)->at(1)->at(3) = 0.90;

    InModeTransitionProbabilites.at(0)->at(2)->at(0)->at(2) = 
    InModeTransitionProbabilites.at(0)->at(6)->at(0)->at(2) = 
    InModeTransitionProbabilites.at(0)->at(2)->at(1)->at(3) = 
    InModeTransitionProbabilites.at(0)->at(6)->at(1)->at(3) =
    InModeTransitionProbabilites.at(1)->at(5)->at(0)->at(4) = 
    InModeTransitionProbabilites.at(1)->at(7)->at(0)->at(4) = 
    InModeTransitionProbabilites.at(1)->at(5)->at(1)->at(5) = 
    InModeTransitionProbabilites.at(1)->at(7)->at(1)->at(5) = 0.70;

    InModeTransitionProbabilites.at(0)->at(2)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(0)->at(6)->at(0)->at(6) =
    InModeTransitionProbabilites.at(0)->at(2)->at(1)->at(7) = 
    InModeTransitionProbabilites.at(0)->at(6)->at(1)->at(7) =
    InModeTransitionProbabilites.at(1)->at(5)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(1)->at(7)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(1)->at(5)->at(1)->at(7) = 
    InModeTransitionProbabilites.at(1)->at(7)->at(1)->at(7) = 0.30;

    InModeTransitionProbabilites.at(0)->at(5)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(0)->at(7)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(0)->at(5)->at(1)->at(7) = 
    InModeTransitionProbabilites.at(0)->at(7)->at(1)->at(7) = 
    InModeTransitionProbabilites.at(1)->at(2)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(1)->at(6)->at(0)->at(6) = 
    InModeTransitionProbabilites.at(1)->at(2)->at(1)->at(7) = 
    InModeTransitionProbabilites.at(1)->at(6)->at(1)->at(7) = 0.10;
}
