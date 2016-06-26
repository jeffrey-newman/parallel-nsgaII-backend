//
//  TestNSGAII_FON.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 26/11/2015.
//
//

#include <stdio.h>
#include <random>
#include <sstream>

#include "../Types.hpp"
#include "../SampleProblems/TestFunctions.hpp"
#include "../NSGAII.hpp"
//#include "../Checkpoints/SerialiseCheckpoint.hpp"
#include "../Checkpoints/SavePopCheckpoint.hpp"
#include "../Checkpoints/MaxGenCheckpoint.hpp"
#include "../Checkpoints/PlotFronts.hpp"
#include "../Metrics/Hypervolume.hpp"
#include "../Checkpoints/ResetMutationXoverFlags.hpp"
#include "../Checkpoints/MetricLinePlot.hpp"
#include "../Checkpoints/MailCheckpoint.hpp"
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>


int main(int argc, char* argv[])
{
    boost::filesystem::path working_dir = boost::filesystem::initial_path();

    // The optimisation problem
//    FON test_problem;
    int delay = 0;
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
    int max_gen = 1000;
    NSGAII<RNG> optimiser(rng, test_problem);
    MaxGenCheckpoint max_gen_terminate(max_gen);
    SavePopCheckpoint save_pop(1, working_dir);
    std::vector<double> ref_point = {1, 1};
    Hypervolume hvol(ref_point, working_dir, 1, Hypervolume::TERMINATION, 50);
    std::string mail_subj("Hypervolume of front from FON optimiser ");
    MailCheckpoint mail(10, hvol, mail_subj);
    std::string my_address("jeff@jeffandkat.id.au");
    mail.addAddress(my_address);
    MetricLinePlot hvol_plot(hvol);
    PlotFrontVTK plotfront;
//    ResetMutXvrDebugFlags reset_flags;
//    SerialiseCheckpoint<NSGAII<RNG> > save_state(1, optimiser, working_dir);
    optimiser.add_checkpoint(max_gen_terminate);
//    optimiser.add_checkpoint(save_state);
    optimiser.add_checkpoint(save_pop);
    optimiser.add_checkpoint(hvol);
    optimiser.add_checkpoint(hvol_plot);
    optimiser.add_checkpoint(plotfront);
    optimiser.add_checkpoint(mail);
//    optimiser.add_checkpoint(reset_flags);
//    optimiser.visualise();
    
    // Initialise population
    int pop_size = 1000	;
    PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, test_problem.getProblemDefinitions(), rng);
    SetMutationInverseDVSize(pop->at(0), optimiser.getRealMutationOperator());

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
