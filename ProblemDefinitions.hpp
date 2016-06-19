//
//  ProblemDefinitions.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef ProblemDefinitions_h
#define ProblemDefinitions_h

#include <limits>


enum MinOrMaxType {
    MINIMISATION,
    MAXIMISATION
};


class ProblemDefinitions
{
public:
    std::vector<double> real_lowerbounds;
    std::vector<double> real_upperbounds;
    std::vector<int> int_lowerbounds;
    std::vector<int> int_upperbounds;
    std::vector<MinOrMaxType> minimise_or_maximise;
    int number_constraints;
    
    ProblemDefinitions()
    : real_lowerbounds(0), real_upperbounds(0), int_lowerbounds(0),int_upperbounds(0), minimise_or_maximise(1, MINIMISATION), number_constraints(0)
    {
        
    }
    
    ProblemDefinitions(int _number_of_objectives, int number_of_real_decision_variables, int number_of_int_decision_variables = 0, int num_constraints = 0)
    :
    real_lowerbounds(number_of_real_decision_variables, std::numeric_limits<double>::min()),
    real_upperbounds(number_of_real_decision_variables, std::numeric_limits<double>::max()),
    int_lowerbounds(number_of_int_decision_variables, std::numeric_limits<int>::min()),
    int_upperbounds(number_of_int_decision_variables, std::numeric_limits<int>::max()),
    minimise_or_maximise(_number_of_objectives, MINIMISATION), number_constraints(num_constraints)
    {
        
    }
    
    ProblemDefinitions(std::vector<double> & _real_lowerbounds,  std::vector<double> & _real_upperbounds,
                       std::vector<int> & _int_lowerbounds, std::vector<int> & _int_upperbounds, std::vector<MinOrMaxType> & _minimise_or_maximise, int num_constraints)
    : real_lowerbounds(_real_lowerbounds), real_upperbounds(_real_upperbounds),
    int_lowerbounds(_int_lowerbounds), int_upperbounds(_int_upperbounds),
    minimise_or_maximise(_minimise_or_maximise), number_constraints(num_constraints)
    {
        
    }
    
    ProblemDefinitions(int num_real_dvs, double real_min, double real_max, int num_int_dvs, int int_min, int int_max, int num_objectives, MinOrMaxType min_or_max, int num_constraints)
    :   real_lowerbounds(num_real_dvs, real_min), real_upperbounds(num_real_dvs, real_max),
        int_lowerbounds(num_int_dvs, int_min),
        int_upperbounds(num_int_dvs, int_max),
        minimise_or_maximise(num_objectives, min_or_max),
        number_constraints(num_constraints)
    {
        
    }

};

//ProblemDefinitions default_defs;
//
//ProblemDefinitions &
//makeProblemDefinitions(int _number_of_objectives, int number_of_real_decision_variables, int number_of_int_decision_variables = 0)
//{
//    default_defs = ProblemDefinitions(_number_of_objectives, number_of_real_decision_variables, number_of_int_decision_variables);
//    return default_defs;
//}
//
//ProblemDefinitions &
//makeProblemDefinitions(int num_real_dvs, double real_min, double real_max, int num_int_dvs, int int_min, int int_max, int num_objectives, MinOrMaxType min_or_max)
//{
//    default_defs = ProblemDefinitions(num_real_dvs, real_min, real_max, num_int_dvs, int_min, int_max, num_objectives, min_or_max);
//    return default_defs;
//}


#endif /* ProblemDefinitions_h */
