//
// Created by a1091793 on 17/04/18.
//
//
//  NSGAII.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef NSGAII_BASE_h
#define NSGAII_BASE_h

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
class NSGAIIBase {
public:
    enum Log{OFF, LVL1, LVL2, LVL3};
protected:
    RNG & random_number_generator;
//    DummyObjectivesAndConstraints dummy_eval;
//    EvaluatePopulation default_evaluator;
//    EvaluatePopulationBase & pop_eval;
    TournamentSelection<RNG> selection;
    CombinedRealIntCrossover<RNG> crossover;
    CombinedRealIntMutation<RNG> mutation;
    Checkpoints my_checkpoints;
    int max_gen;
//    PlotFrontVTK plot_front1;
//    PlotFrontVTK plot_front2;
    DebsRankingAndCrowdingSelector merge_calc_front_and_dist;
//    Visualise do_visualise;
    PopulationSPtr parents;
    PopulationSPtr children;
    int do_log;
//    bool is_fstream = false;
//    std::ofstream log_stream;
//    boost::filesystem::path f_path;
    int gen_num;
    bool is_finished;
    bool save_initial_pop;
    boost::filesystem::path log_directory;

public:

    NSGAIIBase(RNG & _random_number_generator)
        : random_number_generator(_random_number_generator),
//          default_evaluator(eval),
//          pop_eval(default_evaluator),
          selection(_random_number_generator),
        /*do_visualise(OFF),*/
          parents( (Population *) NULL),
          children( (Population *) NULL),
          do_log(OFF),
//          log_stream(std::cout),
          gen_num(0),
          is_finished(false),
          save_initial_pop(true)
    {

    }

public:

//    void
//    visualise(Visualise _val = ON)
//    {
//        do_visualise = _val;
//    }

    void
    add_checkpoint(CheckpointBaseSPtr chkpnt_2_add)
    {
        my_checkpoints.addCheckpoint(chkpnt_2_add);
    }

//    virtual void
//    log(std::ostream & _stream = std::cout, Log _val = LVL1)
//    {
//        do_log = _val;
//        if (do_log > OFF)
//        {
//            is_fstream = false;
//            log_stream = _stream;
//        }
//    }

    virtual void
    log(boost::filesystem::path _log_directory, int _val = LVL1)
    {
        do_log = _val;
        this->log_directory = _log_directory;
    }

    MutationBase<RNG> &
    getRealMutationOperator()
    {
        return (mutation.getRealMutationOperator());
    }

    MutationBase<RNG> &
    getUnorderedMutationOperator()
    {
        return (mutation.getUnorderedMutationOperator());
    }

	MutationBase<RNG>&
	getOrderedMutationOperator()
	{
		return (mutation.getOrderedMutationOperator());
	}

    PopulationSPtr
    step()
    {
        bool not_finished = this->step_impl();
        if (!not_finished) this->is_finished = true;
        return  (parents);
    }

//    PopulationSPtr
//    step(PopulationSPtr _initial_pop)
//    {
//        this->initialise(_initial_pop);
//        return(this->step());
//    }

    PopulationSPtr
    run()
    {
        do {

        } while (this->step_impl());
        this->is_finished = true;
        return (parents);
    }

    PopulationSPtr
    run(PopulationSPtr _initial_pop)
    {
        this->initialise(_initial_pop);
        return(this->run());

    }

    void
    initialiseWithPop(PopulationSPtr _initial_pop, boost::filesystem::path save_dir = "", std::string file_name_prefix = "initial_pop")
    {
        this->initialise(_initial_pop, save_dir, file_name_prefix);
    }

    bool
    isFinished()
    {
        return (is_finished);
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(crossover);
        ar & BOOST_SERIALIZATION_NVP(mutation);
//            ar & BOOST_SERIALIZATION_NVP(my_checkpoints);
        ar & BOOST_SERIALIZATION_NVP(max_gen);
//            ar & BOOST_SERIALIZATION_NVP(do_visualise);
        ar & BOOST_SERIALIZATION_NVP(parents);

    }

    void
    savePop(PopulationSPtr pop_2_process, boost::filesystem::path save_dir, std::string file_name_prefix)
    {

        boost::filesystem::path save_file2 = save_dir /  file_name_prefix ;
        print(pop_2_process, save_file2);

                // sort as population may already be evaluated...
        ObjectiveValueCompator obj_comparator(0);
        std::sort(pop_2_process->begin(), pop_2_process->end(), obj_comparator);

        // run pop_eval with save directory argument so that request to save each indivudal is done as part of eval.
        evalPop(pop_2_process, save_dir);
        //Do not sort again here so that order in population file aligns with saved results from each individual.

        print(pop_2_process, save_file2);

    }

private:

    virtual void
    initialise(PopulationSPtr _parents, boost::filesystem::path save_dir = "", std::string file_name_prefix = "") = 0;
//    {
//        if (do_log > OFF)  log_stream.get() << "Initialising GA: \n";
//        gen_num = 0;
//        is_finished = false;
//        parents = _parents;
//        pop_eval(parents);
//        if (do_log > OFF)  log_stream.get() << "Initial population: \n" << parents;
//        parents->calcFronts();
//
//    }

    virtual bool
    step_impl() = 0;

    virtual void
    evalPop(PopulationSPtr population, boost::filesystem::path & save_dir) = 0;

};


#endif /* NSGAII_BASE_h */



