////
//// Created by a1091793 on 12/6/17.
////
//
////
////  DebsNondominatedSorting.hpp
////  NSGA-Parallel-Backend
////
////  Created by a1091793 on 24/11/2015.
////
////
//
//
//#include <iostream>
//#include "DebsNondominatedSorting.hpp"
//#include "Comparator.hpp"
//
//
//
//
//const int P_DOMINATES = 1;
//const int Q_DOMINATES = 2;
//
//FrontsSPtr
//debNonDominatedSort(const Population & population_ref)
//{
//    /***********************************************************************
//         *              NON-DOMINATED-SORTING                                  *
//         **********************************************************************/
//
//    if (population_ref.size() == 0)
//    {
//        FrontsSPtr fronts(new std::vector<Front>(0));
//        return fronts;
//    }
//
//
//    std::vector<DominationInfo> population_set(population_ref.size());
//    for (int i = 0; i < population_ref.size(); ++i)
//    {
//        population_set[i].ind =  population_ref[i];
//    }
//
//
//    FrontsSPtr fronts(new std::vector<Front>(1));
//    std::vector<std::vector<DominationInfo * > > front_sets(1);
//
//
////        std::map<IndivudalPtr, std::vector<IndivudalPtr> > dominates_sets;
////        std::map<IndivudalPtr, int> number_dominated_by;
//
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
//
//    std::vector<DominationInfo * > dummy_front1;
//    Population dummy_front2;
//
//    //Could truncate this loop once the top pop_size are sorted.
//    while (front_sets[i].size() != 0)
//    {
//        front_sets.push_back(dummy_front1);
//        fronts->push_back(dummy_front2);
//        BOOST_FOREACH(DominationInfo * p, front_sets[i])
//                    {
//                        BOOST_FOREACH(DominationInfo * q, p->dominates)
//                                    {
////                    std::cout << "from " << q->dominated;
//                                        q->dominated = (q->dominated - 1);
////                    std::cout << "to " << q->dominated << std::endl;
//                                        if (q->dominated == 0)
//                                        {
//                                            front_sets[i+1].push_back(q);
//                                            (*fronts)[i+1].push_back(q->ind);
//                                            q->ind->setRank(i+2);
//                                        }
//                                    }
//                    }
//        ++i;
//    }
//
//    if (fronts->back().size() == 0)
//    {
//        fronts->pop_back();
//    }
//
//    return (fronts);
//}
//
//
//FrontsSPtr
//debNonDominatedSort(PopulationSPtr pop)
//{
//    return(debNonDominatedSort(*pop));
//}
//
