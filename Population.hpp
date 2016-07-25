//
//  Population.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Population_h
#define Population_h

#include <vector>
#include <fstream>
#include "Individual.hpp"

class Population;
typedef boost::shared_ptr<Population> PopulationSPtr;
typedef Population Front;
typedef PopulationSPtr FrontSPtr;
typedef std::vector<Front> Fronts;
typedef boost::shared_ptr<Fronts > FrontsSPtr;

class Population : public std::vector<IndividualSPtr>
{
private:

    FrontsSPtr front_sets;
    bool valid_obj_and_constraints;
    bool valid_fronts;

public:
    Population();

    FrontsSPtr
    getFronts();

    Population(int population_size, ProblemDefinitionsSPtr defs);

    void append(const Population & appending_pop);

    const unsigned long populationSize() const;

    void push_back(IndividualSPtr ind);

    void invalidate();
    void validateObjAndConstraints();

//    IndividualSPtr getSPtr2Member(int index)
//    {
//        return (this->operator[](index));
//    }

//    std::vector<IndividualPtr>
//    getVectorOfPointers()
//    {
//        std::vector<IndividualPtr> pop_ptr;
//        BOOST_FOREACH(Individual & ind, *this)
//        {
//            pop_ptr.push_back(&ind);
//        }
//        return pop_ptr;

//    }
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);

    friend std::ostream& operator<<(std::ostream& os, const Population& pop);
};

//typedef boost::shared_ptr<Population> PopulationSPtr;


#include <algorithm>
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include "DebsNondominatedSorting.hpp"

    Population::Population() :
    valid_obj_and_constraints(false), valid_fronts(false)
    {
        
    }

    FrontsSPtr
    Population::getFronts()
    {
        if (!valid_fronts)
        {
            front_sets = DebsNonDominatesSorting::sort(*this);
            valid_fronts = true;
        }
        return (front_sets);
    }
    
    Population::Population(int population_size, ProblemDefinitionsSPtr defs)
    {
        this->reserve(population_size);
        for (int i = 0; i < population_size; ++i)
        {
            std::vector<IndividualSPtr>::push_back(IndividualSPtr(new Individual(defs)));
        }
    }

    void
    Population::append(const Population & appending_pop)
    {
        this->insert(this->end(), appending_pop.begin(), appending_pop.end());
        valid_fronts = false;
    }
    
    const unsigned long
    Population::populationSize() const
    {
        return (this->size());
    }

    void
    Population::push_back(IndividualSPtr ind)
    {
        std::vector<IndividualSPtr>::push_back(ind);
        valid_fronts = false;
    }

    void
     Population::invalidate()
    {
        valid_obj_and_constraints = false;
        valid_fronts = false;
    }

    void
     Population::validateObjAndConstraints()
    {
        valid_obj_and_constraints = true;
    }
    
//    IndividualSPtr getSPtr2Member(int index)
//    {
//        return (this->operator[](index));
//    }
    
//    std::vector<IndividualPtr>
//    getVectorOfPointers()
//    {
//        std::vector<IndividualPtr> pop_ptr;
//        BOOST_FOREACH(Individual & ind, *this)
//        {
//            pop_ptr.push_back(&ind);
//        }
//        return pop_ptr;

//    }
    

    template<class Archive>
    void
    Population::serialize(Archive & ar, const unsigned int version)
    {
            ar & boost::serialization::make_nvp("PopVec", boost::serialization::base_object<std::vector<IndividualSPtr> >(*this) );
//            ar &  BOOST_SERIALIZATION_BASE_OBJECT_NVP(std::vector<Individual>);
    }



   std::ostream& operator<<(std::ostream& os, const Population& pop)
    {
        ObjectiveValueCompator obj_comparator(0);
        Population temp_pop(pop);
        std::sort(temp_pop.begin(), temp_pop.end(), obj_comparator);
        BOOST_FOREACH(IndividualSPtr ind, temp_pop)
        {
            os << ind << "\n";
        }
        return os;
    }

   std::ostream& operator<<(std::ostream& os, const PopulationSPtr pop)
    {
        os << *pop;
        return os;
    }



template<typename RNG>
PopulationSPtr
intialisePopulationRandomDVAssignment(int population_size, ProblemDefinitionsSPtr defs, RNG & rng)
{
    PopulationSPtr pop(new Population(population_size, defs));

    
    for (int i = 0; i < defs->real_lowerbounds.size(); ++i)
    {
        std::uniform_real_distribution<double> uniform(defs->real_lowerbounds[i],defs->real_upperbounds[i]);
        
        BOOST_FOREACH(IndividualSPtr ind, *pop)
        {
            ind->setRealDV(i, uniform(rng));
        }
        
    }
    
    for (int i = 0; i < defs->int_lowerbounds.size(); ++i)
    {
        std::uniform_int_distribution<int> uniform(defs->int_lowerbounds[i],defs->int_upperbounds[i]);
        
        BOOST_FOREACH(IndividualSPtr ind, *pop)
        {
            ind->setIntDV(i, uniform(rng));
        }
    }
    return (pop);
                       
}

void
restore_population(PopulationSPtr pop, const char * filename)
{
    // open the archive
    std::ifstream ifs(filename);
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(*pop);
}

#endif /* Population_h */
