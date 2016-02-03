//
//  Merge.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 23/11/2015.
//
//

#ifndef Merge_h
#define Merge_h

#include "Types.hpp"
                

class DistanceComparator
{
            
public:
            
     inline bool operator()(std::pair<Individual *, double> & first, std::pair<Individual *, double> & second)
     {
         return (first.second > second.second);
     }
};


class DebsRankingAndCrowdingSelector
{
private:
    
    std::vector<std::vector<IndividualPtr> > front_sets;
    
public:
    
    DebsRankingAndCrowdingSelector() :
    front_sets(0)
    {
        
    }
    
    std::vector<std::vector<IndividualPtr> > &
    getFronts()
    {
        return (front_sets);
    }
    
    PopulationSPtr
    operator()(PopulationSPtr previous_parent_pop, PopulationSPtr previous_child_pop)
    {
        // Combine parent and offspring population
        std::vector<IndividualPtr> combined_set;
        for (int i = 0; i < previous_parent_pop->populationSize(); ++i)
        {
            combined_set.push_back(previous_parent_pop->getPointer2Member(i));
        }
        for (int i = 0; i < previous_child_pop->populationSize(); ++i)
        {
            combined_set.push_back(previous_child_pop->getPointer2Member(i));
        }
        
        // Sort the solutions into front sets
        front_sets = DebsNonDominatesSorting::sort(combined_set);
        
        /***********************************************************************
         *                      New Population                                 *
         **********************************************************************/
        PopulationSPtr new_child_pop(new Population);
        int i = 0;
        // until the parent population is filled...
        while (new_child_pop->populationSize() + front_sets[i].size() <= previous_child_pop->populationSize())
        {
            // assign distances (needed for tournament selection)
            DebsCrowdingDistance::calculate(front_sets[i]);
            BOOST_FOREACH(IndividualPtr ind, front_sets[i])
            {
                new_child_pop->push_back(*ind);
            }
            ++i;
        }
        
        int more_ind_need = previous_child_pop->populationSize() - new_child_pop->populationSize();
        
        if (more_ind_need > 0)
        {
            std::vector<std::pair<IndividualPtr, double> > distances = DebsCrowdingDistance::calculate(front_sets[i]);
            
            //sort by crowding distance in the next dominated set (sorts descendingly)
            DistanceComparator dist_comparator;
            std::sort(distances.begin(), distances.end(), dist_comparator);
            //fill up the new population with solutions with the greatest distance
            for (int l = 0; l < more_ind_need; ++l)
            {
                new_child_pop->push_back(*((distances[l]).first));
            }
        }
        
        return (new_child_pop);
    }
};





#endif /* Merge_h */
