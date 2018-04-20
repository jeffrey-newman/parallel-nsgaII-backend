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
#include "ParallelEvaluator.hpp"


//enum Visualise{ON, OFF};


template <typename RNG = std::mt19937>
class NSGAIICE : public NSGAIIBase<RNG>
{

private:

    TournamentSelectionContinuousEvolution<RNG> selection;
    ParallelEvaluatePopServerNonBlockingContinuousEvolution<RNG> pop_eval;

public:


    NSGAIICE(RNG & _random_number_generator,
             boost::mpi::environment & _mpi_env,
             boost::mpi::communicator & _world,
             ProblemDefinitionsSPtr _problem_defs)
    : NSGAIIBase<RNG>(_random_number_generator),
      pop_eval(_mpi_env, _world, _problem_defs, selection, this->crossover, this->mutation)
            /*do_visualise(OFF),*/
    {

    }
    
public:

//    virtual void
//    log(std::ostream & _stream = std::cout, int _val = NSGAIIBase<RNG>::LVL1)
//    {
//        this->do_log = _val;
//        if (this->do_log > NSGAIIBase<RNG>::OFF)
//        {
//            this->is_fstream = false;
//            this->log_stream = _stream;
//        }
//        this->pop_eval.log(_val, _stream);
//    }

    virtual void
    log(boost::filesystem::path _log_directory, int _val = NSGAIIBase<RNG>::LVL1)
    {
        this->do_log = _val;
        this->log_directory = _log_directory;
        this->pop_eval.log(_log_directory, _val);
    }

//    virtual void
//    log(std::ofstream & _stream, boost::filesystem::path _f_path, int _val = NSGAIIBase<RNG>::LVL1)
//    {
//        this->do_log = _val;
//        if (this->do_log > NSGAIIBase<RNG>::OFF)
//        {
//            this->is_fstream = true;
//            this->f_path = _f_path;
//            this->log_stream = _stream;
//        }
//        this->pop_eval.log(_val, _stream);
//    }


private:

    virtual void
    initialise(PopulationSPtr _parents, boost::filesystem::path save_dir = "", std::string file_name_prefix = "initial_pop")
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "nsgaii_ce_initialise.log";
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
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "nsgaii_ce_gen" + std::to_string(this->gen_num + 1) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "Generation: " << ++this->gen_num << std::endl;
        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "parents: \n" << this->parents << std::endl;

        this->children = this->pop_eval.breedAndEvalPop(this->parents);
        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "\n\n\nAfter generational evolution operators: \n" << this->children << std::endl;

        this->children = this->merge_calc_front_and_dist(this->parents, this->children);

        if (this->do_log > NSGAIIBase<RNG>::OFF)  logging_file << "After merge: \n" << this->children << std::endl;

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
        this->pop_eval.evalAndSavePop(population, save_dir);
    }
};


#endif /* NSGAIICE_h */
