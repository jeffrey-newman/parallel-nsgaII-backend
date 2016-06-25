//
//  NSGAII.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef NSGAII_h
#define NSGAII_h

#include <boost/scoped_ptr.hpp>
#include "Types.hpp"
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
#include "PlotFronts.hpp"
#endif

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>


enum Visualise{ON, OFF};

template <typename RNG>
class NSGAII {
private:
    RNG & random_number_generator;
    DummyObjectivesAndConstraints dummy_eval;
    EvaluatePopulation default_evaluator;
    EvaluatePopulationBase & pop_eval;
    TournamentSelection<RNG> selection;
    DebsSBXCrossover<RNG> crossover;
    DebsPolynomialMutation<RNG> mutation;
    Checkpoints my_checkpoints;
    int max_gen;
//    PlotFrontVTK plot_front1;
//    PlotFrontVTK plot_front2;
    DebsRankingAndCrowdingSelector merge_calc_front_and_dist;
    Visualise do_visualise;
    PopulationSPtr pop;
    
public:
    NSGAII(RNG & _random_number_generator, ObjectivesAndConstraintsBase & eval)
    : random_number_generator(_random_number_generator), default_evaluator(eval), pop_eval(default_evaluator), selection(_random_number_generator), do_visualise(OFF)
    {
        
    }
    
    NSGAII(RNG & _random_number_generator, EvaluatePopulationBase & _pop_eval)
    : random_number_generator(_random_number_generator), default_evaluator(dummy_eval), pop_eval(_pop_eval), selection(_random_number_generator), do_visualise(OFF)
    {
        
    }
    
public:
    
    void
    visualise(Visualise _val = ON)
    {
        do_visualise = _val;
    }

    void
    add_checkpoint(CheckpointBase & chkpnt_2_add)
    {
        my_checkpoints.addCheckpoint(&chkpnt_2_add);
    }

    DebsPolynomialMutation<RNG> &
    getRealMutationOperator()
    {
        return mutation;
    }
    
    PopulationSPtr
    operator()(PopulationSPtr parents)
    {

#ifdef WITH_VTK
        PlotFrontVTK plot_front1;
#endif
        PopulationSPtr children( (Population *) NULL);
        pop_eval(parents);



        do {
//            std::cout << "parents: \n" << parents;
            children = selection(parents);
//            std::cout << "\n\n\nAfter selection: \n" << children;
            crossover(children);
//            std::cout << "\n\n\nAfter crossover: \n" << children;
            mutation(children);
            pop_eval(children);
//            std::cout << "\n\n\nAfter mutation: \n" << children;
            children = merge_calc_front_and_dist(parents, children);
//            std::cout << "After merge: \n" << children;
            parents = children;
        } while (my_checkpoints(children));
        
        return (children);
        
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(crossover);
            ar & BOOST_SERIALIZATION_NVP(mutation);
//            ar & BOOST_SERIALIZATION_NVP(my_checkpoints);
            ar & BOOST_SERIALIZATION_NVP(max_gen);
            ar & BOOST_SERIALIZATION_NVP(do_visualise);
            ar & BOOST_SERIALIZATION_NVP(pop);

    }
};


#endif /* NSGAII_h */
