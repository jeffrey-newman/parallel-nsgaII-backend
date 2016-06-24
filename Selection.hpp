//
//  Selection.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 23/11/2015.
//
//

#ifndef Selection_h
#define Selection_h

#include <algorithm>
#include "Types.hpp"
#include "Comparator.hpp"

std::uniform_real_distribution<double> sel_uniform(0.0,1.0);
unsigned seed_selection = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 default_rng_selection(seed_selection);


template <typename RNG>
class TournamentSelection
{
    
    
private:
    RNG & random_number_gen;
    
    
    IndividualSPtr
    tournament(IndividualSPtr ind1, IndividualSPtr ind2)
    {
        int flag;
        flag = Comparator::whichDominates(ind1, ind2);
        if (flag==1)
        {
            return (ind1);
        }
        if (flag==2)
        {
            return (ind2);
        }
        if (ind1->getCrowdingScore() > ind2->getCrowdingScore())
        {
            return(ind1);
        }
        if (ind2->getCrowdingScore() > ind1->getCrowdingScore())
        {
            return(ind2);
        }
        if ( sel_uniform(random_number_gen) <= 0.5)
        {
            return(ind1);
        }
        else
        {
            return(ind2);
        }
    }
    
public:
    TournamentSelection(RNG & rng = default_rng_selection)
    : random_number_gen(rng)
    {
        
    }
    
    PopulationSPtr
    operator()(PopulationSPtr parent_pop)
    {
        std::vector<IndividualSPtr> a1, a2;
        for (int i = 0; i < parent_pop->populationSize(); ++i)
        {
            a1.push_back((*parent_pop)[i]);
            a2.push_back((*parent_pop)[i]);
        }
        
        std::shuffle(a2.begin(), a2.end(), random_number_gen);
        
        //Possible for a1[i] and a2[i] to be same indiviudal.
        PopulationSPtr child_pop(new Population);
        for (int i = 0; i < parent_pop->populationSize(); ++i)
        {
            child_pop->push_back(IndividualSPtr( new Individual(*(tournament(a1[i], a2[i])))));
        }
        
        return (child_pop);
    }
    
    
};

#endif /* Selection_h */
