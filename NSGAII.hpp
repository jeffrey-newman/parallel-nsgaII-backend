//
//  NSGAII.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef NSGAII_h
#define NSGAII_h

#include <iostream>
#include <functional>

#include <boost/scoped_ptr.hpp>
#include "Selection.hpp"
#include "Crossover.hpp"
#include "Mutation.hpp"
#include "DebsCrowdingDistance.hpp"
#include "DebsNondominatedSorting.hpp"
#include "Checkpoint.hpp"
#include "Evaluation.hpp"
#include "Merge.hpp"
#include "Population.hpp"


#ifdef WITH_VTK
#include "Checkpoints/PlotFronts.hpp"
#endif

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/filesystem.hpp>


//enum Visualise{ON, OFF};


template <typename RNG = std::mt19937>
class NSGAII : public NSGAIIBase<RNG>
{
private:
        DummyObjectivesAndConstraints dummy_eval;
    EvaluatePopulation default_evaluator;
    EvaluatePopulationBase & pop_eval;

public:

    NSGAII(RNG & _random_number_generator, ObjectivesAndConstraintsBase & eval)
        : NSGAIIBase<RNG>(_random_number_generator),
          default_evaluator(eval),
          pop_eval(default_evaluator)
    {

    }
    
    NSGAII(RNG & _random_number_generator, EvaluatePopulationBase & _pop_eval)
    : NSGAIIBase<RNG>(_random_number_generator),
      default_evaluator(dummy_eval),
      pop_eval(_pop_eval)

    {

    }
    

private:

    bool
    step_impl()
    {
        ++gen_num;

        if (is_fstream == true)
        {
            std::ofstream * fstreamPtr = dynamic_cast<std::ofstream *>(&(log_stream.get()));
            fstreamPtr->close();
            boost::filesystem::path log_path = this->f_path.parent_path() / (this->f_path.stem().string() + std::string("_gen_") + std::to_string(gen_num) + this->f_path.extension().string());
            boost::filesystem::path old_path = this->f_path.parent_path() / (this->f_path.stem().string() + std::string("_gen_") + std::to_string(gen_num - 2) + this->f_path.extension().string());
            if (boost::filesystem::exists(old_path))
            {
                if (boost::filesystem::is_regular_file(old_path)) boost::filesystem::remove(old_path);
            }
            fstreamPtr->open(log_path.c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!fstreamPtr->is_open()) do_log = OFF;
        }

        if (do_log > OFF)  log_stream.get() << "Generation: " << gen_num << "\n";
        if (do_log > OFF)  log_stream.get() << "parents: \n" << parents;

        children = selection(parents);

        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter selection: \n" << children;

        crossover(children);

        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter crossover: \n" << children;

        mutation(children);

        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter mutation: \n" << children;

        pop_eval(children);

        children = merge_calc_front_and_dist(parents, children);

        if (do_log > OFF)  log_stream.get() << "After merge: \n" << children;

        parents = children;

        return (my_checkpoints(parents));
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path & save_dir)
    {
        pop_eval(population, save_dir);
    }
};


#endif /* NSGAII_h */
