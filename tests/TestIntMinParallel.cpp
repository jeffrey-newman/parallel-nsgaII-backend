//
//  TestFonParallel.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 8/12/2015.
//
//

#include <stdio.h>
#include <random>
#include <sstream>
#include <chrono>

#include <boost/mpi/environment.hpp>
#include <boost/mpi/communicator.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>

#include "../ParallelEvaluator.hpp"
#include "../Types.hpp"
#include "../SampleProblems/TestFunctions.hpp"
#include "../NSGAII.hpp"
#include "../Checkpoints/SavePopCheckpoint.hpp"
#include "../Checkpoints/MaxGenCheckpoint.hpp"
#include "../Checkpoints/PlotFronts.hpp"
#include "../Metrics/Hypervolume.hpp"
#include "../Checkpoints/ResetMutationXoverFlags.hpp"
#include "../Checkpoints/MetricLinePlot.hpp"
#include "../Checkpoints/MailCheckpoint.hpp"




int main(int argc, char* argv[])
{
    boost::filesystem::path working_dir = boost::filesystem::initial_path();
    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator world;

    IntSum test_problem;
    
    if (world.rank() == 0)
    {
        //create evaluator server

        //create evaluator server
        std::string eval_log_fname("mpi_msgs.txt");
        std::ofstream eval_strm(eval_log_fname.c_str());
        ParallelEvaluatePopServerNonBlocking eval_server(env, world, test_problem.getProblemDefinitions());
        if (eval_strm.is_open())
        {
            eval_server.log(ParallelEvaluatorBase::LVL1, eval_strm);
        }



        
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
        int max_gen = 100;
        NSGAII<RNG> optimiser(rng, eval_server);
        boost::shared_ptr<MaxGenCheckpoint> max_gen_terminate(new MaxGenCheckpoint(max_gen));
    //    SavePopCheckpoint save_pop(1, working_dir);
        std::vector<double> ref_point = {1, -1};
        boost::shared_ptr<Hypervolume> hvol(new Hypervolume(1, working_dir, Hypervolume::TERMINATION, 50, ref_point));
    //    hvol.log();
        std::string mail_subj("Hypervolume of front from FON optimiser ");
    //    MailCheckpoint mail(10, hvol, mail_subj);
    //    std::string my_address("jeff@jeffandkat.id.au");
    //    mail.addAddress(my_address);
        boost::shared_ptr<MetricLinePlot> hvol_plot(new MetricLinePlot(hvol));
        boost::shared_ptr<PlotFrontVTK> plotfront(new PlotFrontVTK);
    //    ResetMutXvrDebugFlags reset_flags;
    //    SerialiseCheckpoint<NSGAII<RNG> > save_state(1, optimiser, working_dir);
        optimiser.add_checkpoint(max_gen_terminate);
    //    optimiser.add_checkpoint(save_state);
    //    optimiser.add_checkpoint(save_pop);
        optimiser.add_checkpoint(hvol);
        optimiser.add_checkpoint(hvol_plot);
        optimiser.add_checkpoint(plotfront);
    //    optimiser.add_checkpoint(mail);
    //    optimiser.add_checkpoint(reset_flags);
    //    optimiser.visualise();

        // Initialise population
        int pop_size = 1000;
        PopulationSPtr pop = intialisePopulationRandomDVAssignment(pop_size, test_problem.getProblemDefinitions(), rng);
        optimiser.getRealMutationOperator().setMutationInverseDVSize(pop->at(0));

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
        ParallelEvaluatePopClientNonBlocking eval_client(env, world, test_problem.getProblemDefinitions(), test_problem);
        //logging eval_client
        std::string log_filename = "evaluation_timing_worker" + std::to_string(world.rank()) + ".log";
        std::ofstream eval_strm(log_filename.c_str());
        if (eval_strm.is_open())
        {
            eval_client.log(ParallelEvaluatorBase::LVL1, eval_strm);
        }
        eval_client();
    }

    return EXIT_SUCCESS;
    
}
