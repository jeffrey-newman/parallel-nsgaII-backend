//
//  NSGAIICE.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef NSGAIICE_h
#define NSGAIICE_h

#include "NSGAII-base.hpp"


//enum Visualise{ON, OFF};


template <typename RNG = std::mt19937>
class NSGAIICE : public NSGAIIBase<RNG>
{

private:

    ParallelEvaluatePopServerNonBlockingContinuousEvolution pop_eval;

public:


    NSGAIICE(RNG & _random_number_generator,
             boost::mpi::environment & _mpi_env,
             boost::mpi::communicator & _world,
             ProblemDefinitionsSPtr _problem_defs)
    : pop_eval(_mpi_env, _world, _problem_defs, selection, crossover, mutation),
      selection(_random_number_generator)
            /*do_visualise(OFF),*/
    {

    }
    
public:



private:

    void
    initialise(PopulationSPtr _parents)
    {
        if (do_log > OFF)  log_stream.get() << "Initialising GA: \n";
        gen_num = 0;
        is_finished = false;
        parents = _parents;
    }

    bool
    step_impl()
    {
        if (is_fstream == true)
        {
            std::ofstream * fstreamPtr = dynamic_cast<std::ofstream *>(&(log_stream.get()));
            fstreamPtr->close();
            fstreamPtr->open(f_path.c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!fstreamPtr->is_open()) do_log = OFF;
        }
        if (do_log > OFF)  log_stream.get() << "Generation: " << ++gen_num << "\n";
        if (do_log > OFF)  log_stream.get() << "parents: \n" << parents;

        children = pop_eval(parents);
        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter generational evolution operators: \n" << children;

        children = merge_calc_front_and_dist(parents, children);

        if (do_log > OFF)  log_stream.get() << "After merge: \n" << children;

        parents = children;

        return (my_checkpoints(parents));
    }
};


#endif /* NSGAIICE_h */
