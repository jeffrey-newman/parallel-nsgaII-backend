//
//  TestNSGAII_FON.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 26/11/2015.
//
//

#include <stdio.h>
#include <random>

#include "../Types.hpp"
#include "../TestFunctions.hpp"
#include "../NSGAII.hpp"
#include <boost/timer/timer.hpp>


int main(int argc, char* argv[])
{
    // The optimisation problem
//    FON test_problem;
    int delay = 1;
    DelayFON test_problem(delay);
    
    std::stringstream timer_info;
    boost::scoped_ptr<boost::timer::auto_cpu_timer> t((boost::timer::auto_cpu_timer *) nullptr);
    std::string time_fname("run_time_fon.txt");
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
    
    // The random number generator
    typedef std::mt19937 RNG;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    RNG rng(seed);
    
    // The optimiser
    int max_gen = 10;
    NSGAII<RNG> optimiser(rng, test_problem, max_gen);
    //        optimiser.visualise();
    
    // Initialise population
    int pop_size = 128	;
    PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, test_problem.getProblemDefinitions(), rng);
    
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