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
#include "Population.hpp"
#include <boost/serialization/nvp.hpp>


template <typename RNG = std::mt19937>
class MutationBase
{
protected:
        double default_mutation_probability;
        double & probability_mutation;
        unsigned seed_mutation;
        RNG default_rng_mutation;
        RNG & random_number_gen;
        std::uniform_real_distribution<double> mut_uniform;

public:

    MutationBase()
            : default_mutation_probability(0.10),
              probability_mutation(default_mutation_probability),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(default_rng_mutation),
              mut_uniform(0.0, 1.0)
    {

    }

    MutationBase(RNG & rng)
            : default_mutation_probability(0.10),
              probability_mutation(default_mutation_probability),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(rng),
              mut_uniform(0.0, 1.0)
    {

    }

    MutationBase(double & _probability_mutation)
            : default_mutation_probability(0.10),
              probability_mutation(_probability_mutation),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(default_rng_mutation),
              mut_uniform(0.0, 1.0)
    {

    }

    MutationBase(RNG & rng, double & _probability_mutation)
            : default_mutation_probability(0.10),
              probability_mutation(_probability_mutation),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(rng),
              mut_uniform(0.0, 1.0)
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
        this->setMutationProbability(1 / double(_ind_sample->numberOfIntDecisionVariables() + _ind_sample->numberOfRealDecisionVariables()));
    }
};





template <typename RNG = std::mt19937>
class DebsPolynomialMutation : public MutationBase<RNG>
{
    



    double default_eta_m;
    double eta_m;
    int gene_m_count = 0;
    int gene_count = 0;
    
    
public:
    //Mutation is on a per chromosone basis
    DebsPolynomialMutation():

            default_eta_m(20.0),
            eta_m(default_eta_m)
    {

    }

    //Mutation is on a per chromosone basis
    DebsPolynomialMutation(RNG & rng):
            default_eta_m(20.0),
            MutationBase<RNG>(rng),
            eta_m(default_eta_m)
    {

    }

    DebsPolynomialMutation(RNG & rng, double & eta, double & _probability_mutation) :
    default_eta_m(20.0),
    MutationBase<RNG>(rng, _probability_mutation),
    eta_m(eta)
    {
        
    }
    
    void
    operator()(Individual & individual)
    {


        for (unsigned long j=0; j < individual.numberOfRealDecisionVariables(); j++)
        {
//            ++gene_count;
            if (this->mut_uniform(this->random_number_gen) <= this->probability_mutation)
            {
//                ++gene_m_count;
//                individual.mutated = true;
                double delta1, delta2, xy, val, deltaq;
                const double & y = individual.getRealDV(j);
                const double yl=individual.getRealLowerBound(j);//lower limt of variable j of ind i
                const double yu=individual.getRealUpperBound(j);//upper limt of variable j of ind i

                delta1 = (y-yl)/(yu-yl);
                delta2 = (yu-y)/(yu-yl);
                double rnd = this->mut_uniform(this->random_number_gen);
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
            ar & BOOST_SERIALIZATION_NVP(this->probability_mutation);
    }
};





template <typename RNG = std::mt19937>
class UniformIntMutation : public MutationBase<RNG>
{

    int gene_m_count = 0;
    int gene_count = 0;

    
public:
    //Mutation is on a per chromosone basis
    UniformIntMutation()
    {

    }

    UniformIntMutation(RNG & rng) :
            MutationBase<RNG>(rng)
    {

    }

    UniformIntMutation(RNG & rng, double & _probability_mutation) :
        MutationBase<RNG>(rng, _probability_mutation)
    {
        
    }
    
    void
    operator()(Individual & individual)
    {
        
        
        for (unsigned long j=0; j < individual.numberOfIntDecisionVariables(); j++)
        {
            
            
            //            ++gene_count;
            if (this->mut_uniform(this->random_number_gen) <= this->probability_mutation)
            {
                
                std::uniform_int_distribution<int> distribution(individual.getIntLowerBound(j),individual.getIntUpperBound(j));
                
                individual.setIntDV(j, distribution(this->random_number_gen));
                
            }
        }
        return;
    }
    
    

    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(this->probability_mutation);
    }
};




template <typename RNG = std::mt19937>
class CombinedRealIntMutation
{
    unsigned seed_mutation;
    RNG default_rng_mutation;
    RNG & random_number_gen;

    DebsPolynomialMutation<> default_real_mutation;
    UniformIntMutation<> default_int_mutation;

    MutationBase<RNG> & real_mut;
    MutationBase<RNG> & int_mut;
    
    
public:
    //Mutation is on a per chromosone basis
    CombinedRealIntMutation() :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(default_rng_mutation),
            default_real_mutation(random_number_gen),
            default_int_mutation(random_number_gen),
            real_mut(default_real_mutation),
            int_mut(default_int_mutation)
    {

    }

    CombinedRealIntMutation(RNG & rng) :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(rng),
            default_real_mutation(random_number_gen),
            default_int_mutation(random_number_gen),
            real_mut(default_real_mutation),
            int_mut(default_int_mutation)
    {

    }

    CombinedRealIntMutation(MutationBase<RNG> & _real_mut, MutationBase<RNG> & _int_mut) :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(default_rng_mutation),
            default_real_mutation(random_number_gen),
            default_int_mutation(random_number_gen),
            real_mut(_real_mut),
            int_mut(_int_mut)
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
    
    MutationBase<RNG> &
    getRealMutationOperator()
    {
        return (real_mut);
    }
    
    MutationBase<RNG> &
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
