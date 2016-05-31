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


enum Visualise{ON, OFF};

template <typename RNG>
class NSGAII {
private:
    RNG & random_number_generator;
    DummyEvaluator dummy_eval;
    EvaluatePopulation default_evaluator;
    EvaluatePopulationBase & pop_eval;
    TournamentSelection<RNG> selection;
    DebsSBXCrossover<RNG> crossover;
    DebsPolynomialMutation<RNG> mutation;
    Checkpoint checkpoint;
    int max_gen;
//    PlotFrontVTK plot_front1;
//    PlotFrontVTK plot_front2;
    DebsRankingAndCrowdingSelector merge_calc_front_and_dist;
    Visualise do_visualise;
    
public:
    NSGAII(RNG & _random_number_generator, EvaluatorBase & eval, int max_gen)
    : random_number_generator(_random_number_generator), default_evaluator(eval), pop_eval(default_evaluator), selection(_random_number_generator), checkpoint(max_gen), do_visualise(OFF)
    {
        
    }
    
    NSGAII(RNG & _random_number_generator, EvaluatePopulationBase & _pop_eval, int max_gen)
    : random_number_generator(_random_number_generator), default_evaluator(dummy_eval), pop_eval(_pop_eval), selection(_random_number_generator), checkpoint(max_gen), do_visualise(OFF)
    {
        
    }
    
public:
    
    void
    visualise(Visualise _val = ON)
    {
        do_visualise = _val;
    }
    
    PopulationSPtr
    operator()(PopulationSPtr parents)
    {
        // previous_gen is the initialised population.
        pop_eval(parents);
        std::vector<std::vector<IndividualPtr> > fronts = DebsNonDominatesSorting::sort(parents);
        
        BOOST_FOREACH(std::vector<IndividualPtr> front, fronts)
        {
            DebsCrowdingDistance::calculate(front);
        }
//        plot_front1(fronts);
        
        // Make the breeding population using selection, crossover and mutation.
        PopulationSPtr children = selection(parents);
        crossover(children);
        mutation(children);
        pop_eval(children);
        PopulationSPtr previous_gen( (Population *) NULL);

#ifdef WITH_VTK
        boost::scoped_ptr<PlotFrontVTK> plot((PlotFrontVTK *) NULL);
        if (do_visualise == ON) plot.reset(new PlotFrontVTK);
#endif
        
        
        do {
            previous_gen = parents;
            parents = children;
            children = selection(merge_calc_front_and_dist(previous_gen, parents));

#ifdef WITH_VTK
            if (do_visualise == ON) (*plot)(merge_calc_front_and_dist.getFronts());
#endif
            
            crossover(children);
            mutation(children);
            pop_eval(children);
//            fronts = DebsNonDominatesSorting::sort(children);
//            PlotFrontVTK plot2;
//            plot2(fronts);
//            std::cout << "Population size: " << children->populationSize() << std::endl;
        } while (checkpoint(children));
       
        
        previous_gen = parents;
        parents = children;
        children = merge_calc_front_and_dist(previous_gen, parents);
        pop_eval(children);
        fronts = DebsNonDominatesSorting::sort(children);
        
        BOOST_FOREACH(std::vector<IndividualPtr> front, fronts)
        {
            DebsCrowdingDistance::calculate(front);
        }
        
#ifdef WITH_VTK
        if (do_visualise == ON) (*plot)(fronts);
#endif
        
        return (children);
        
    }
};

#endif /* NSGAII_h */
