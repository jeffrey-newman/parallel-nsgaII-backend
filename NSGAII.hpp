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
        if (this->do_log)
        {
            std::string file_name = "nsgaii_initialise.log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Initialising GA: \n";
        this->gen_num = 0;
        this->is_finished = false;
        this->parents = _parents;
        if (save_dir.string() == "")
        {
            this->pop_eval(this->parents);
        }
        else
        {
            this->savePop(this->parents, save_dir, file_name_prefix);
        }

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Initial population: \n" << this->parents;
        this->parents->calcFronts();

    }

    virtual bool
    step_impl()
    {
        ++this->gen_num;

        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "nsgaii_gen" + std::to_string(this->gen_num) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Generation: " << this->gen_num << "\n";
        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "parents: \n" << this->parents;

        this->children = this->selection(this->parents);

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter selection: \n" << this->children;

        this->crossover(this->children);

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter crossover: \n" << this->children;

        this->mutation(this->children);

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter mutation: \n" << this->children;

        this->pop_eval(this->children);

        this->children = this->merge_calc_front_and_dist(this->parents, this->children);

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "After merge: \n" << this->children;

        this->parents = this->children;

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

        return (this->my_checkpoints(this->parents));
    }

    virtual void
    evalPop(PopulationSPtr population, boost::filesystem::path & save_dir)
    {
        this->pop_eval(population, save_dir);
    }
};


#endif /* NSGAII_h */
