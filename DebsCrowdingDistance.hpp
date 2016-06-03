//
//  DebsCrowdingDistance.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef DebsCrowdingDistance_h
#define DebsCrowdingDistance_h

#include <vector>
#include <boost/foreach.hpp>
#include "Types.hpp"
#include "Comparator.hpp"
#include "DebsNondominatedSorting.hpp"

class ObjectiveValueCompator
{
    int objective_index;
    
public:
    ObjectiveValueCompator(int _objective_index)
    : objective_index(_objective_index)
    {
        
    }
    
    inline bool operator()(const std::pair<IndividualPtr, double> & first, const std::pair<IndividualPtr, double> & second)
    {
        return (Comparator::compareObjective(*(first.first), *(second.first), objective_index));
    }
};




class DebsCrowdingDistance
{
    
public:
    
    static
    std::vector< std::vector<std::pair<IndividualPtr, double > > >
    calculate(const PopulationSPtr pop)
    {
        std::vector< std::vector<std::pair<IndividualPtr, double > > > crowd_dist_by_front;
        std::vector<std::vector<IndividualPtr> > fronts = DebsNonDominatesSorting::sort(pop);
        BOOST_FOREACH(std::vector<IndividualPtr> front, fronts)
        {
            crowd_dist_by_front.push_back(calculate(front));
        }
        return (crowd_dist_by_front);
    }

    static
    std::vector<std::pair<IndividualPtr, double > >
    calculate(const std::vector<IndividualPtr> & front_set)
    {
        
        //Fill up remainder of population from the next dominated set, based on crowding distance
        typedef std::pair<IndividualPtr, double> IndDistPair;
        std::vector<IndDistPair> distances;
        BOOST_FOREACH(IndividualPtr ind, front_set)
        {
            distances.push_back(std::make_pair(ind, 0));
        }
        
        //calculate Crowding distance for solutions in the next dominated set.
        for (int j = 0; j < (distances[0]).first->numberOfObjectives(); ++j)
        {
            ObjectiveValueCompator obj_comparator(j);
            std::sort(distances.begin(), distances.end(), obj_comparator);
            distances.front().second = std::numeric_limits<double>::max();
            distances.front().first->setCrowdingScore(std::numeric_limits<double>::max());
            double min_obj_val = distances.front().first->getObjective(j);
            distances.back().second = std::numeric_limits<double>::max();
            distances.back().first->setCrowdingScore(std::numeric_limits<double>::max());
            double max_obj_val = distances.back().first->getObjective(j);
            for (int k = 1; k < distances.size() -1; ++k)
            {
                distances[k].second += (distances[k+1].first->getObjective(j) - distances[k-1].first->getObjective(j)) / (max_obj_val - min_obj_val);
            }
        }
        
        
        
        BOOST_FOREACH(IndDistPair & ind, distances)
        {
            ind.first->setCrowdingScore(ind.second);
        }
        
        return (distances);
    }
};



#endif /* DebsCrowdingDistance_h */
