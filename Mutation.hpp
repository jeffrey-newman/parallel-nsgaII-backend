//
//  Mutation.hpp
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 21/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Mutation_h
#define Mutation_h

#include <random>
#include <chrono>
#include "Types.hpp"
#include <boost/serialization/nvp.hpp>



std::uniform_real_distribution<double> mut_uniform(0.0,1.0);
double default_eta_m = 20.0;
double default_mutation_probability = 0.10;
unsigned seed_mutation = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 default_rng_mutation(seed_mutation);


template <typename RNG>
class DebsPolynomialMutation
{
    
    RNG & random_number_gen;
    double & eta_m;
    double & probability_mutation;
    
    
public:
    //Mutation is on a per chromosone basis
    DebsPolynomialMutation(RNG & rng = default_rng_mutation, double & eta = default_eta_m, double & _probability_mutation = default_mutation_probability) :
    random_number_gen(rng), eta_m(eta), probability_mutation(_probability_mutation)
    {
        
    }
    
    void
    mutation_implementation(Individual & individual)
    {
        
        for (unsigned long j=0; j < individual.numberOfRealDecisionVariables(); j++)
        {
            if (mut_uniform(random_number_gen) <= this->probability_mutation)
            {
                double delta1, delta2, xy, val, deltaq;
                const double & y = individual.getRealDV(j);
                const double yl=individual.getRealLowerBound(j);//lower limt of variable j of ind i
                const double yu=individual.getRealUpperBound(j);//upper limt of variable j of ind i

                delta1 = (y-yl)/(yu-yl);
                delta2 = (yu-y)/(yu-yl);
                double rnd = mut_uniform(random_number_gen);
                double mut_pow = 1.0/(this->eta_m+1.0);
                if (rnd <= 0.5)
                {
                    xy = 1.0-delta1;
                    val = 2.0*rnd+(1.0-2.0*rnd)*(pow(xy,(this->eta_m+1.0)));
                    deltaq =  pow(val,mut_pow) - 1.0;
                }
                else
                {
                    xy = 1.0-delta2;
                    val = 2.0*(1.0-rnd)+2.0*(rnd-0.5)*(pow(xy,(this->eta_m+1.0)));
                    deltaq = 1.0 - (pow(val,mut_pow));
                }
                double new_val = y + deltaq*(yu-yl);
                if (new_val<yl)
                    new_val = yl;
                if (new_val>yu)
                    new_val = yu;
                individual.setRealDV(j, new_val);
            }
        }
        return;
    }
    
    void
    operator()(PopulationSPtr & parent_pop)
    {
        for (int i = 0; i < parent_pop->size(); ++i)
        {
                mutation_implementation((*parent_pop)[i]);
        }
        
        
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(eta_m);
            ar & BOOST_SERIALIZATION_NVP(probability_mutation);
    }
};

#endif /* Mutation_h */
