//
//  TestParallelSpeedup.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 8/12/2015.
//
//

#include <stdio.h>
#include <boost/mpi.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>

#include "../ParallelEvaluator.hpp"
#include "../TestFunctions.hpp"
#include "../NSGAII.hpp"



int main(int argc, char* argv[])
{
    boost::mpi::environment env;
    boost::mpi::communicator world;

    int delay;
    std::string time_fname;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help,h", "produce help message")
    ("delay,d", po::value<int>(&delay)->default_value(10), "delay in function eval to test speedup of parallelisation for objective functions of different computational cost (OF eval time in seconds")
    ("time,t", po::value<std::string>(&time_fname)->default_value("timer.txt"), "File to write elapsed time for optimiser too");
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(desc).run(), vm);
    po::notify(vm);

    DelayFON eval(delay);
    
    std::stringstream timer_info;
    boost::scoped_ptr<boost::timer::auto_cpu_timer> t((boost::timer::auto_cpu_timer *) nullptr);
    
    if (world.rank() == 0)
    {
        std::ofstream ofs(time_fname.c_str());
        if (ofs.is_open())
        {
            t.reset(new boost::timer::auto_cpu_timer(timer_info, 3));
        }
        else
        {
            std::cerr << "Error: Could not open file for writing time elapsed for search, using std::cout";
            t.reset(new boost::timer::auto_cpu_timer(3));
        }
        
        
        //create evaluator server
        ParallelEvaluatePopServer eval_server(env, world, eval.getProblemDefinitions());
        
        // The random number generator
        typedef std::mt19937 RNG;
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        RNG rng(seed);
        
        // The optimiser
        int max_gen = 10;
        NSGAII<RNG> optimiser(rng, eval_server, max_gen);
//        optimiser.visualise();
        
        // Initialise population
        int pop_size = 128	;
        PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, eval.getProblemDefinitions(), rng);
        
        // Run the optimisation
        optimiser(pop);
        
        
        t.reset((boost::timer::auto_cpu_timer *) nullptr);
        if (ofs.is_open())
        {
            ofs << timer_info.str();
            ofs.close();
        }
        std::cout << timer_info.str() << std::endl;
        
    }
    else
    {
        // create evaluator client
        ParallelEvaluatePopClient eval_client(env, world, eval.getProblemDefinitions(), eval);
        eval_client();
    }
    
}
