//
//  Comparator.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 23/11/2015.
//
//

#ifndef Comparator_h
#define Comparator_h

class Comparator
{
public:
    
    static
    bool
    compareObjective(Individual & ind1, Individual & ind2, int index)
    {
        if (ind1.isMinimiseOrMaximise(index) == MINIMISATION)
        {
            return (ind1.getObjective(index) < ind2.getObjective(index));
        }
        return (ind1.getObjective(index) > ind2.getObjective(index));
    }
    
    static
    int
    whichDominates(Individual & ind1, Individual & ind2)
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
            else if (ind1_num_constr_violatn == ind2_num_constr_violatn)
            {
                if (rel_violation_ind1 < rel_violation_ind2)
                {
                    return (1);
                }
                else if (rel_violation_ind1 == rel_violation_ind2)
                {
                    return 0;
                }
                return (2);
                
            }
            return (2);
        }
        // if one solution is feasible, choose this one.
        else
        {
            if (ind1_num_constr_violatn > 0 && ind2_num_constr_violatn == 0)
            {
                return (2);
            }
            else if (ind1_num_constr_violatn == 0 && ind2_num_constr_violatn > 0)
            {
                return (1);
            }
            // if both solutions are feasible, choose the one which dominates via objective values.
            else
            {
                int flag1 = 0;
                int flag2 = 0;
                for (int j=0; j < ind1.numberOfObjectives(); ++j)
                {
                    if (compareObjective(ind1, ind2, j))
                    {
                        flag1 = 1;
                        
                    }
                    else
                    {
                        if (compareObjective(ind2, ind1, j))
                        {
                            flag2 = 1;
                        }
                    }
                }
                if (flag1==1 && flag2==0)
                {
                    return (1);
                }
                else
                {
                    if (flag1==0 && flag2==1)
                    {
                        return (2);
                    }
                    else
                    {
                        return (0);
                    }
                }
            }
        }
        return (3);
    }

    bool
    operator()(Individual & ind1, Individual & ind2)
    {
        if (whichDominates(ind1, ind2) == 1) return true;
        return false;
    }
};

#endif /* Comparator_h */
