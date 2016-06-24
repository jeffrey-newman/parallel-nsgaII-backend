//
//  TestFON.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 2/12/2015.
//
//

#include <stdio.h>
#include <iostream>

#include "../SampleProblems/TestFunctions.hpp"

int main(int argc, char* argv[])
{
    // The optimisation problem
    FON test_problem;
    
    std::pair<std::vector<double>, std::vector<double> > results;
    std::vector<int> null_int_vars;
    std::vector<double> real_vars = {0, 0, 0};
    results = test_problem(real_vars, null_int_vars);
    std::cout << results.first[0] << " " << results.first[1] << std::endl;
}
