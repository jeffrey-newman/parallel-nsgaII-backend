//
//  Population.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Population_h
#define Population_h

#include <algorithm>
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "Individual.hpp"




class Population: public std::vector<Individual>
{
    
public:
    Population()
    {
        
    }
    
    Population(int population_size, ProblemDefinitions & defs)
    : std::vector<Individual>(population_size, Individual(defs))
    {
        
    }

    void append(const Population & appending_pop)
    {
        this->insert(this->end(), appending_pop.begin(), appending_pop.end());
    }
    
    unsigned long populationSize()
    {
        return (this->size());
    }
    
    Individual * getPointer2Member(int index)
    {
        return &(this->operator[](index));
    }
    
    std::vector<IndividualPtr>
    getVectorOfPointers()
    {
        std::vector<IndividualPtr> pop_ptr;
        BOOST_FOREACH(Individual & ind, *this)
        {
            pop_ptr.push_back(&ind);
        }
        return pop_ptr;

    }
};

typedef boost::shared_ptr<Population> PopulationSPtr;


template<typename RNG>
PopulationSPtr
intialisePopulationRandomDVAssignment(int population_size, ProblemDefinitions & defs, RNG & rng)
{
    PopulationSPtr pop(new Population(population_size, defs));
    
    
    for (int i = 0; i < defs.real_lowerbounds.size(); ++i)
    {
        std::uniform_real_distribution<double> uniform(defs.real_lowerbounds[i],defs.real_upperbounds[i]);
        
        BOOST_FOREACH(Individual & ind, *pop)
        {
            ind.setRealDV(i, uniform(rng));
        }
        
    }
    
    for (int i = 0; i < defs.int_lowerbounds.size(); ++i)
    {
        std::uniform_int_distribution<int> uniform(defs.int_lowerbounds[i],defs.int_upperbounds[i]);
        
        BOOST_FOREACH(Individual & ind, *pop)
        {
            ind.setIntDV(i, uniform(rng));
        }
    }
    return (pop);
                       
}

#endif /* Population_h */
