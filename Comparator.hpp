/*
//
//  Comparator.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 23/11/2015.
//
//
 */

#ifndef Comparator_h
#define Comparator_h

#include "Individual.hpp"
#include <tuple>

class Comparator
{
public:

    static
    bool
    compareObjective(const Individual& ind1, const Individual& ind2, int index)
    {
        if (ind1.isMinimiseOrMaximise(index) == MINIMISATION)
        {
            return (ind1.getObjective(index) < ind2.getObjective(index));
        }
        return (ind1.getObjective(index) > ind2.getObjective(index));
    }

    static
    bool
    compareObjective(const IndividualSPtr ind1, const IndividualSPtr ind2, int index)
    {
        return (compareObjective(*ind1, *ind2, index));
    }

    static
    int
    compareObjective2(const Individual& ind1, const Individual& ind2, int index)
    {
        if (ind1.isMinimiseOrMaximise(index) == MINIMISATION)
        {
            if (ind1.getObjective(index) < ind2.getObjective(index)) return 1;
            if (ind1.getObjective(index) == ind2.getObjective(index)) return 0;
            return 2;
        }
        else
        {
            if (ind1.getObjective(index) < ind2.getObjective(index)) return 2;
            if (ind1.getObjective(index) == ind2.getObjective(index)) return 0;
            return 1;
        }
        return (3);
    }

    static
    int
    compareObjective2(const IndividualSPtr ind1, const IndividualSPtr ind2, int index)
    {
        return (compareObjective2(*ind1, *ind2, index));
    }

    static int
    whichDominates(const IndividualSPtr ind1, const IndividualSPtr ind2)
    {
        return whichDominates(*ind1, *ind2);
    }

    static int
    whichDominates(const Individual & ind1, const Individual & ind2)
    {
        // Calculate the number of constraints violated, and the relative amount of violation.
        int ind1_num_constr_violatn = 0;
        int ind2_num_constr_violatn = 0;
        double rel_violation_ind1 = 0;
        double rel_violation_ind2 = 0;

        for (int i = 0; i < ind1.numberOfConstraints(); ++i)
        {
            const double & constraint1 = ind1.getConstraint(i);
            const double & constraint2 = ind2.getConstraint(i);
            if (constraint1 > 0)
            {
                ++ind1_num_constr_violatn;
                if (constraint2 > 0)
                {
                    ++ind2_num_constr_violatn;
                    if (constraint1 > constraint2)
                    {
                        rel_violation_ind1 += 1;
                        rel_violation_ind2 += constraint2/constraint1;
                    }
                    else
                    {
                        rel_violation_ind2 += 1;
                        rel_violation_ind1 += constraint1/constraint2;
                    }

                }
                else
                {
                    rel_violation_ind1 += 1;
                }
            }
            else if (constraint2 > 0)
            {
                ++ind2_num_constr_violatn;
                rel_violation_ind2 += 1;
            }

        }

        // If both solutions infeasible, pick one which is has the smaller overall constraint violation
        if (ind1_num_constr_violatn != 0 && ind2_num_constr_violatn != 0)
        {
            if (ind1_num_constr_violatn < ind2_num_constr_violatn)
            {
                return (1);
            }
            else if (ind1_num_constr_violatn > ind2_num_constr_violatn)
            {
                return (2);
            }
            else
            {
                if (rel_violation_ind1 < rel_violation_ind2)
                {
                    return (1);
                }
                else if (rel_violation_ind1 > rel_violation_ind2)
                {
                    return (2);
                }
            }
        }
            // if one solution is feasible.....
//        else if (ind1_num_constr_violatn > 0 && ind2_num_constr_violatn == 0)
        else if (ind1_num_constr_violatn > 0)
        {
            return (2);
        }
//        else if (ind1_num_constr_violatn == 0 && ind2_num_constr_violatn > 0)
        else if (ind2_num_constr_violatn > 0)
        {
            return (1);
        }


            // if both solutions are feasible (or they have the same number and relative constraint values,) choose
            // the one which dominates via objective values.

            int flag1 = 0;
            int flag2 = 0;
            for (int j=0; j < ind1.numberOfObjectives(); ++j)
            {
                int which_is_better = compareObjective2(ind1, ind2, j);
                if (which_is_better == 1) flag1 = 1;
                if (which_is_better == 2) flag2 = 1;
            }
            if (flag1==1 && flag2==0)
            {
                return (1);
            }
            else if (flag1==0 && flag2==1)
            {

                return (2);
            }
            else
            {
                return (0);
            }


        return (3);
    }

    bool
    operator()(const IndividualSPtr ind1, const IndividualSPtr ind2)
    {
        return this->operator ()(*ind1, *ind2);
    }

    bool
    operator()(const Individual & ind1, const Individual & ind2)
    {
        if (whichDominates(ind1, ind2) == 1) return true;
        return false;
    }
};

class ObjectiveValueCompator
{
    int objective_index;

public:
    ObjectiveValueCompator(int _objective_index)
            : objective_index(_objective_index)
    {

    }

    inline bool operator()(const std::pair<IndividualSPtr, double> & first, const std::pair<IndividualSPtr, double> & second)
    {
        return (Comparator::compareObjective((first.first), (second.first), objective_index));
    }

    inline bool operator()(const IndividualSPtr first, const IndividualSPtr second)
    {
        return (Comparator::compareObjective(first, second, objective_index));
    }

    inline bool operator()(const Individual & first, const Individual & second)
    {
        return (Comparator::compareObjective(first, second, objective_index));
    }
};

#endif /* Comparator_h */

