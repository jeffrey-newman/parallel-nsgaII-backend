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
#include "Individual.hpp"
#include <boost/serialization/nvp.hpp>

double default_mutation_probability = 0.10;

class MutationBase
{
protected:
        double & probability_mutation;
public:
    MutationBase(double & _probability_mutation = default_mutation_probability)
    : probability_mutation(_probability_mutation)
    {
        
    }
    
    virtual void operator() (Individual & individual) = 0;
    
    void
    setMutationProbability(double _mutation_probability)
    {
        probability_mutation = _mutation_probability;
    }
    
    void
    setMutationInverseDVSize(IndividualSPtr _ind_sample)
    {
        this->setMutationProbability(1 / double(_ind_sample->numberOfIntDecisionVariables()));
    }
};


std::uniform_real_distribution<double> mut_uniform(0.0,1.0);
double default_eta_m = 20.0;
unsigned seed_mutation = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 default_rng_mutation(seed_mutation);


template <typename RNG = std::mt19937>
class DebsPolynomialMutation : public MutationBase
{
    
    RNG & random_number_gen;
    double & eta_m;
    int gene_m_count = 0;
    int gene_count = 0;
    
    
public:
    //Mutation is on a per chromosone basis
    DebsPolynomialMutation(RNG & rng = default_rng_mutation, double & eta = default_eta_m, double & _probability_mutation = default_mutation_probability) :
    MutationBase(_probability_mutation),
    random_number_gen(rng), eta_m(eta)
    {
        
    }
    
    void
    operator()(Individual & individual)
    {


        for (unsigned long j=0; j < individual.numberOfRealDecisionVariables(); j++)
        {
//            ++gene_count;
            if (mut_uniform(random_number_gen) <= this->probability_mutation)
            {
//                ++gene_m_count;
//                individual.mutated = true;
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
    
    DebsPolynomialMutation &
    setEtaM(double _eta_m)
    {
        eta_m = _eta_m;
        return (*this);
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(eta_m);
            ar & BOOST_SERIALIZATION_NVP(probability_mutation);
    }
};





template <typename RNG = std::mt19937>
class UniformIntMutation : public MutationBase
{
    
    RNG & random_number_gen;
    int gene_m_count = 0;
    int gene_count = 0;
    
    
public:
    //Mutation is on a per chromosone basis
    UniformIntMutation(RNG & rng = default_rng_mutation, double & _probability_mutation = default_mutation_probability) :
    MutationBase(_probability_mutation),
    random_number_gen(rng)
    {
        
    }
    
    void
    operator()(Individual & individual)
    {
        
        
        for (unsigned long j=0; j < individual.numberOfIntDecisionVariables(); j++)
        {
            
            
            //            ++gene_count;
            if (mut_uniform(random_number_gen) <= this->probability_mutation)
            {
                
                std::uniform_int_distribution<int> distribution(individual.getIntLowerBound(j),individual.getIntUpperBound(j));
                
                individual.setIntDV(j, distribution(random_number_gen));
                
            }
        }
        return;
    }
    
    

    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(probability_mutation);
    }
};


DebsPolynomialMutation<> default_real_mutation;
UniformIntMutation<> default_int_mutation;

template <typename RNG>
class CombinedRealIntMutation
{
    
    RNG & random_number_gen;
    MutationBase & real_mut;
    MutationBase & int_mut;
    int gene_m_count = 0;
    int gene_count = 0;
    
    
public:
    //Mutation is on a per chromosone basis
    CombinedRealIntMutation(RNG & rng = default_rng_mutation, MutationBase & _real_mut = default_real_mutation, MutationBase & _int_mut = default_int_mutation) :
    random_number_gen(rng), real_mut(_real_mut), int_mut(_int_mut)
    {
        
    }
    
    
    void
    operator()(PopulationSPtr & parent_pop)
    {
        //        int ind_count= 0;
        //        int m_count = 0;
        //        gene_m_count = 0;
        //        gene_count = 0;
        parent_pop->invalidate();
        for (int i = 0; i < parent_pop->size(); ++i)
        {
            //            ++ind_count;
            real_mut(*((*parent_pop)[i]));
            int_mut(*((*parent_pop)[i]));
            //            if ((*parent_pop)[i]->mutated == true ) ++m_count;
        }
        
        //        std::cout << "Mutation: " << m_count << " of " << ind_count << " ind underwent: " << 100 * double(m_count) / ind_count << "%\n";
        //        std::cout << "Mutation: " << gene_m_count << " of " << gene_count << " genes underwent: " << 100 * double(gene_m_count) / gene_count << "%\n";
        
        
    }
    
    MutationBase &
    getRealMutationOperator()
    {
        return (real_mut);
    }
    
    MutationBase &
    getIntMutationOperator()
    {
        return (int_mut);
    }
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(real_mut);
        ar & BOOST_SERIALIZATION_NVP(int_mut);
    }
};

#endif /* Mutation_h */
