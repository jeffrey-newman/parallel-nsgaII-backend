//
//  Population.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Population_h
#define Population_h

#include "Individual.hpp"
#include <boost/shared_ptr.hpp>

template <typename DecisionVariableType>
class Population {
private:
    std::vector<Individual<DecisionVariableType> > population_vec;
    
public:
    std::vector<Individual<DecisionVariableType> > & operator[](int index)
    {
        return (population_vec[index]);
    }
};

template <typename DecisionVariableType>
using PopulationSPtr = boost::shared_ptr<Population<DecisionVariableType> >;

#endif /* Population_h */
