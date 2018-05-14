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
#include <chrono>
#include <random>
#include "Comparator.hpp"
#include "Population.hpp"


template <typename RNG = std::mt19937>
class TournamentSelection
{
    
    
private:
    std::uniform_real_distribution<double> sel_uniform;
    unsigned seed_selection;
    RNG default_rng_selection;

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
    TournamentSelection()
            :
            sel_uniform(0.0,1.0),
            seed_selection(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_selection(seed_selection),
            random_number_gen(default_rng_selection)
    {

    }

    TournamentSelection(RNG & rng)
    :
            sel_uniform(0.0,1.0),
            seed_selection(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_selection(seed_selection),
            random_number_gen(rng)
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

template <typename RNG = std::mt19937>
class TournamentSelectionContinuousEvolution
{


private:
    std::uniform_real_distribution<double> sel_uniform;
    unsigned seed_selection;
    RNG default_rng_selection;

    RNG & random_number_gen;

    std::vector<IndividualSPtr> a1, a2;
    int pos;
    bool only_add_nondom_2_breed_pop;


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

    void
    resortBreedingPop()
    {
        std::shuffle(a1.begin(), a1.end(), random_number_gen);
        pos = 0;
    }

public:
    TournamentSelectionContinuousEvolution()
        :
        sel_uniform(0.0,1.0),
        seed_selection(std::chrono::system_clock::now().time_since_epoch().count()),
        default_rng_selection(seed_selection),
        random_number_gen(default_rng_selection),
        pos(0),
        only_add_nondom_2_breed_pop(true)
    {

    }

    TournamentSelectionContinuousEvolution(RNG & rng)
        :
        sel_uniform(0.0,1.0),
        seed_selection(std::chrono::system_clock::now().time_since_epoch().count()),
        default_rng_selection(seed_selection),
        random_number_gen(rng),
        pos(0),
        only_add_nondom_2_breed_pop(true)
    {

    }

    void
    resetBreedingPop(PopulationSPtr pop)
    {
        a1.clear();
        for (int i = 0; i < pop->populationSize(); ++i)
        {
            a1.push_back((*pop)[i]);
        }
        std::shuffle(a1.begin(), a1.end(), random_number_gen);
        pos = 0;
    }

    void
    add2BreedingPop(IndividualSPtr new_ind)
    {
        if (only_add_nondom_2_breed_pop)
        {
            //Test whether nondominated.
            for(IndividualSPtr current_ind: a1)
            {
                if (Comparator::whichDominates(new_ind, current_ind) == 1)
                {
                    //Then we add. then return.
                    std::uniform_int_distribution<int> uid(0, a1.size());
                    int loc =  uid(random_number_gen);
                    std::vector<IndividualSPtr>::iterator insert_it = a1.begin() + loc;
                    a1.insert(insert_it, new_ind);
                    if (pos <= loc) ++pos;
                    return;
                }
            }
        }
        else
        {
            std::uniform_int_distribution<int> uid(0, a1.size());
            int loc =  uid(random_number_gen);
            std::vector<IndividualSPtr>::iterator insert_it = a1.begin() + loc;
            a1.insert(insert_it, new_ind);
            if (pos <= loc) ++pos;
            return;
        }

    }

    PopulationSPtr
    operator()(int number_selected)
    {
        PopulationSPtr selected_pop(new Population);
        for (int i = 0; i < number_selected; ++i)
        {
            if (pos >= a1.size()) {resortBreedingPop(); pos = 0;}
            IndividualSPtr ind1 = a1[pos++];
            if (pos >= a1.size()) {resortBreedingPop(); pos = 0;}
            IndividualSPtr ind2 = a1[pos++];
            selected_pop->push_back(IndividualSPtr(new Individual(*(tournament(ind1, ind2)))));
        }

        return (selected_pop);
    }


};

#endif /* Selection_h */
