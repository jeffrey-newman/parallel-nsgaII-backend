//
//  TestParallelEvaluator.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 13/12/2015.
//
//

#include <stdio.h>
#include <boost/mpi.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include "../ParallelEvaluator.hpp"
#include "../TestFunctions.hpp"



int main(int argc, char* argv[])
{
    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator world;
    SUM eval;
    
    
    if (world.rank() == 0)
    {
        //create evaluator server
        ParallelEvaluatePopServer eval_server(env, world, eval.getProblemDefinitions());
        
        // The random number generator
        typedef std::mt19937 RNG;
        unsigned seed_mutation = std::chrono::system_clock::now().time_since_epoch().count();
        RNG rng(seed_mutation);
        
        // Initialise population
        int pop_size = 100;
        PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, eval.getProblemDefinitions(), rng);
        
        //send population to evaluator server
        eval_server(pop);
        
        BOOST_FOREACH(Individual & ind, *pop)
        {
            std::cout << ind.getRealDV(0) << "\t" << ind.getRealDV(1) << "\t" << ind.getRealDV(2) << "\t" << ind.getObjective(0)  << "\t" << ind.getObjective(1) << std::endl;
        }
        
    }
    else
    {
        // create evaluator client
        ParallelEvaluatePopClient eval_client(env, world, eval.getProblemDefinitions(), eval);
        eval_client();
    }
    
}
