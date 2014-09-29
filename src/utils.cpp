#include "utils.h"

namespace UTILS {

    void UnitTest() {
        assert(Sign(+10) == +1);
        assert(Sign(-10) == -1);
        assert(Sign(0) == 0);

        int n[6] = { 0 };
        for ( int i = 0; i < 10000; i++ )
            for ( int j = 1; j < 6; j++ )
                n[j] += (Random(j) == 0);
        assert(Near(n[1], 10000, 0));
        assert(Near(n[2], 5000, 250));
        assert(Near(n[3], 3333, 250));
        assert(Near(n[4], 2500, 250));
        assert(Near(n[5], 2000, 250));

        int c = 0;
        for ( int i = 0; i < 10000; i++ )
            c += Bernoulli(0.5);
        assert(Near(c, 5000, 250));
        assert(CheckFlag(5, 0));
        assert(!CheckFlag(5, 1));
        assert(CheckFlag(5, 2));
        assert(!CheckFlag(5, 3));
        int flag = 1;
        SetFlag(flag, 2);
        SetFlag(flag, 4);
        assert(flag == 21);
    }

    std::vector<double>* GaussianWeights(int maximum, std::mt19937 gen) {
        std::vector<double>* weights = new std::vector<double>(maximum, 0);

        double gaussienne[5] = { 0.05, 0.25, 0.40, 0.25, 0.05 };
        std::uniform_int_distribution<> dist(0, maximum - 1);
        int mu = dist(gen);

        for( int i = mu - 2; i <= mu + 2; i++ ) {
            if( i < 0 || (unsigned int)i >= weights->size() ) 
                continue;
            weights->at(i) = gaussienne[i - (mu - 2)];
        }

        //Pour sommer a 1
        int cumIndex = 0;
        double cumSum = 0;
        while ( cumIndex + mu - 2 < 0 ) {
            cumSum += gaussienne[cumIndex];
            cumIndex++;
        }
        weights->at(0) += cumSum;

        cumIndex = 0;
        cumSum = 0;
        while( mu + 2 - cumIndex > (maximum - 1) ) {
            cumSum += gaussienne[4 - cumIndex];
            cumIndex++;
        }
        weights->at(maximum - 1) += cumSum;
        //Pour sommer a 1

        return weights;
    }

}
