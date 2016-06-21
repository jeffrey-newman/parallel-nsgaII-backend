//
//  TestFonParallel.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 8/12/2015.
//
//

#include <stdio.h>
#include <boost/mpi/environment.hpp>
#include <boost/mpi/communicator.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <chrono>
#include <random>
#include "../ParallelEvaluator.hpp"
#include "../TestFunctions.hpp"
#include "../NSGAII.hpp"
#include "../Checkpoints/MaxGenCheckpoint.hpp"



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
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        RNG rng(seed);
        
        // The optimiser
        int max_gen = 10000;
        NSGAII<RNG> optimiser(rng, eval_server);
        MaxGenCheckpoint max_gen_stop(max_gen);
        optimiser.add_checkpoint(max_gen_stop);
        optimiser.visualise();
        
        // Initialise population
        int pop_size = 100;
        PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, eval.getProblemDefinitions(), rng);
        
        // Run the optimisation
        optimiser(pop);
        
    }
    else
    {
        // create evaluator client
        ParallelEvaluatePopClient eval_client(env, world, eval.getProblemDefinitions(), eval);
        eval_client();
    }
    
}
