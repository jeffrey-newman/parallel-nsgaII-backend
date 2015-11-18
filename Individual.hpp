//
//  Individual.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Individual_h
#define Individual_h

#include <vector>


class RealOrIntType {
    double real_val;
    int int_val;
    bool is_real;
    
public:
    RealOrIntType(double val):
    real_val(val), is_real(true)
    {
    }
    
    RealOrIntType(int val):
    int_val(val), is_real(false)
    {
    }
    
    RealOrIntType(RealOrIntType & copy):
    real_val(copy.real_val), int_val(copy.int_val), is_real(copy.is_real)
    {
    }
    
    RealOrIntType & operator=(double val)
    {
        real_val = val;
        is_real = true;
        return (*this);
    }
    
    RealOrIntType & operator=(int val)
    {
        int_val = val;
        is_real = false;
        return (*this);
    }
};

template <typename DecisionVariableType>
class Individual
{
private:
    std::vector<DecisionVariableType> decision_variables;
    std::vector<double> objectives;
    std::vector<double> constraints;
    int nondomination_rank;
    int crowding_score;
    
public:
    DecisionVariableType & operator[](const int index)
    {
        return (decision_variables[index]);
    }
    
    const double & getObjective(const int index) const
    {
        return (objectives[index]);
    }
    
    void setObjectives(const int index, const double & value)
    {
        objectives[index] = value;
    }
    
    const double & getConstraint(const int index) const
    {
        return (constraints[index]);
    }
    
    void setConstraint(const int index, const double & value)
    {
        constraints[index] = value;
    }
    
    int getNondominatedRank(void) const
    {
        return (nondomination_rank);
    }
    
    void setNondominatedRank(int rank)
    {
        nondomination_rank = rank;
    }
    
    int getCrowdingScore(void) const
    {
        return (crowding_score);
    }
    
    void setCrowdingScore(int score)
    {
        crowding_score = score;
    }
};


#endif /* Individual_h */
