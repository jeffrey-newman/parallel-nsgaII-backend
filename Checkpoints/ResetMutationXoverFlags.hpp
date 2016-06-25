#ifndef RESETMUTATIONCROSSOVERFLAGSININDIVIDUALS_HPP
#define RESETMUTATIONCROSSOVERFLAGSININDIVIDUALS_HPP

#include "../Checkpoint.hpp"
#include "../Individual.hpp"
#include <boost/foreach.hpp>

class ResetMutXvrDebugFlags : public CheckpointBase
{


public:

    bool
    operator()(PopulationSPtr population)
    {
//        int n_mutated = 0;
//        int n_xovered = 0;
//        int pop_size = 0;

//        BOOST_FOREACH(IndividualSPtr ind, *population)
//        {
//            if (ind->mutated) {
//                ++n_mutated;
//            }
//            if (ind->crossovered)
//            {
//                ++n_xovered;
//            }
//            ind->mutated = false;
//            ind->crossovered = false;
//            ind->child = false;
//            ind->parent = false;
//            ++pop_size;
//        }
//        std::cout << "Mutation: " << n_mutated << " of " << pop_size << " underwent: " << 100 * double(n_mutated) / pop_size << "%\n";
//        std::cout << "Crossover: " << n_xovered << " of " << pop_size << " underwent: " << 100 * double(n_xovered) / pop_size << "%\n";
        return true;
    }

};

#endif // RESETMUTATIONCROSSOVERFLAGSININDIVIDUALS_HPP
