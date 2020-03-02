//
//  NSGAII.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef NSGAII_h
#define NSGAII_h


#include "NSGAII-base.hpp"



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

    virtual void
    initialise(PopulationSPtr _parents, boost::filesystem::path save_dir = "", std::string file_name_prefix = "initial_pop")
    {

        std::ofstream logging_file;
        if (do_log)
        {
            std::string file_name = "nsgaii_initialise.log";
            boost::filesystem::path log_file = log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) do_log = OFF;
        }

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Initialising GA: \n";
        gen_num = 0;
        is_finished = false;
        parents = _parents;
        if (save_dir.string() == "")
        {
            pop_eval(parents);
        }
        else
        {
            savePop(parents, save_dir, file_name_prefix);
        }

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Initial population: \n" << parents;
        parents->calcFronts();

    }

    virtual bool
    step_impl()
    {
        ++gen_num;

        std::ofstream logging_file;
        if (do_log)
        {
            std::string file_name = "nsgaii_gen" + std::to_string(gen_num) + ".log";
            boost::filesystem::path log_file = log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) do_log = OFF;
        }

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Generation: " << gen_num << "\n";
        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "parents: \n" << parents;

        children = selection(parents);

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter selection: \n" << children;

        crossover(children);

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter crossover: \n" << children;

        mutation(children);

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter mutation: \n" << children;

        pop_eval(children);

        children = merge_calc_front_and_dist(parents, children);

        if (do_log > NSGAIIBase<RNG>::OFF)  logging_file << "After merge: \n" << children;

        parents = children;

        if (do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

        return (my_checkpoints(parents));
    }

    virtual void
    evalPop(PopulationSPtr population, boost::filesystem::path & save_dir)
    {
        pop_eval(population, save_dir);
    }
};


#endif /* NSGAII_h */
