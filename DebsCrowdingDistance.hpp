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
#include "Population.hpp"
#include "Comparator.hpp"
//#include "DebsNondominatedSorting.hpp"

inline
std::vector<std::pair<IndividualSPtr, double > >
calculateDebsCrowdingDistance(const Population & front_set)
{


    typedef std::pair<IndividualSPtr, double> IndDistPair;
    std::vector<IndDistPair> distances;
    BOOST_FOREACH(IndividualSPtr ind, front_set)
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

inline
std::vector< std::vector<std::pair<IndividualSPtr, double > > >
calculateDebsCrowdingDistance(const PopulationSPtr pop)
{
    std::vector< std::vector<std::pair<IndividualSPtr, double > > > crowd_dist_by_front;
    FrontsSPtr fronts = pop->getFronts();
    BOOST_FOREACH(Front front, *fronts)
                {
                    crowd_dist_by_front.push_back(calculateDebsCrowdingDistance(front));
                }
    return (crowd_dist_by_front);
}





#endif /* DebsCrowdingDistance_h */
