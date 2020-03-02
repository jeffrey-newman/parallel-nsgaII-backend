//
//  DebsNondominatedSorting.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 24/11/2015.
//
//

#ifndef DebsNondominatedSorting_h
#define DebsNondominatedSorting_h


//class DebsNonDominatesSorting
//{

//public:

//    static
//    std::vector<std::vector<IndividualPtr> >
//    sort(PopulationSPtr pop);

//    static
//    std::vector<Population>
//    sort(Population population_ptrs);
//};

#include <iostream>
#include <vector>
#include "Comparator.hpp"
#include "Population.hpp"


class DominationInfo
{
public:
    IndividualSPtr ind;
    std::vector<DominationInfo *> dominates;
    int dominated;

    DominationInfo()
            :
            dominated(0)
    {

    }
};

inline FrontsSPtr
debNonDominatedSort(const Population & population_ref)
{
    /***********************************************************************
         *              NON-DOMINATED-SORTING                                  *
         **********************************************************************/

    if (population_ref.size() == 0)
    {
        FrontsSPtr fronts(new std::vector<Front>(0));
        return fronts;
    }


    std::vector<DominationInfo> population_set(population_ref.size());
    for (std::vector<DominationInfo>::size_type i = 0; i < population_ref.size(); ++i)
    {
        population_set[i].ind =  population_ref[i];
    }


    FrontsSPtr fronts(new std::vector<Front>(1));
    std::vector<std::vector<DominationInfo * > > front_sets(1);


//        std::map<IndivudalPtr, std::vector<IndivudalPtr> > dominates_sets;
//        std::map<IndivudalPtr, int> number_dominated_by;

    const int P_DOMINATES = 1;
    const int Q_DOMINATES = 2;

    for (std::vector<DominationInfo>::size_type j = 0; j < population_set.size(); ++j)
    {
        DominationInfo & p = population_set[j];
        IndividualSPtr p_ptr = p.ind;
        int & p_dominated_by = p.dominated;
        std::vector<DominationInfo *> & p_dominates = p.dominates;

        for (std::vector<DominationInfo>::size_type k = j+1; k < population_set.size(); ++k)
        {

            DominationInfo & q = population_set[k];
            IndividualSPtr q_ptr = q.ind;
            int & q_dominated_by = q.dominated;
            std::vector<DominationInfo *> & q_dominates = q.dominates;

            if (p_ptr != q_ptr)
            {
                //If p dominates q
                int compare = Comparator::whichDominates(p_ptr,q_ptr);
                if (compare == P_DOMINATES)
                {
                    p_dominates.push_back(&q);
                    ++q_dominated_by;
                }
                else if (compare == Q_DOMINATES)
                {
                    q_dominates.push_back(&p);
                    ++p_dominated_by;
                }
            }

        }

    }

    int front_num = 0;
    for(DominationInfo & p: population_set)
                {
                    IndividualSPtr p_ptr = p.ind;
                    const int & p_dominated_by = p.dominated;
                    if (p_dominated_by == 0)
                    {
                        front_sets[front_num].push_back(&p);
                        (*fronts)[front_num].push_back(p_ptr);
                        p_ptr->setRank(front_num+1);
                    }
                }

    std::vector<DominationInfo * > dummy_front1;
    Population dummy_front2;

    //Could truncate this loop once the top pop_size are sorted.
    while (front_sets[front_num].size() != 0)
    {
        front_sets.push_back(dummy_front1);
        fronts->push_back(dummy_front2);
        for (DominationInfo * p: front_sets[front_num])
                    {
                        for (DominationInfo * q: p->dominates)
                                    {
//                    std::cout << "from " << q->dominated;
                                        q->dominated = --(q->dominated);
//                    std::cout << "to " << q->dominated << std::endl;
                                        if (q->dominated == 0)
                                        {
                                            front_sets[front_num+1].push_back(q);
                                            (*fronts)[front_num+1].push_back(q->ind);
                                            q->ind->setRank(front_num+2);
                                        }
                                    }
                    }
        ++front_num;
    }

//    BOOST_FOREACH(DominationInfo & p, population_set)
//                {
//                    IndividualSPtr p_ptr = p.ind;
//                    int & p_dominated_by = p.dominated;
//                    p_dominated_by = 0;
//                    std::vector<DominationInfo *> & p_dominates = p.dominates;
//
//                    BOOST_FOREACH(DominationInfo & q, population_set)
//                                {
//                                    IndividualSPtr q_ptr = q.ind;
//                                    if (p_ptr != q_ptr)
//                                    {
//                                        //If p dominates q
//                                        int compare = Comparator::whichDominates(p_ptr,q_ptr);
//                                        if (compare == P_DOMINATES)
//                                        {
//                                            p_dominates.push_back(&q);
//                                        }
//                                        else if (compare == Q_DOMINATES)
//                                        {
//                                            ++p_dominated_by;
//                                        }
//                                    }
//                                }
//                    if (p_dominated_by == 0)
//                    {
//                        front_sets[0].push_back(&p);
//                        (*fronts)[0].push_back(p_ptr);
//                        p_ptr->setRank(1);
//
//                    }
//
//                }
//
//    int i = 0;

    if (fronts->back().size() == 0)
    {
        fronts->pop_back();
    }

    return (fronts);
}


//inline FrontsSPtr
//debNonDominatedSort(PopulationSPtr pop)
//{
//    return(debNonDominatedSort(*pop));
//}






#endif /* DebsNondominatedSorting_h */
