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
        std::uniform_real_distribution<double> uniform_rand_distributn;

public:

    MutationBase()
            : default_mutation_probability(0.10),
              probability_mutation(default_mutation_probability),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(default_rng_mutation),
              uniform_rand_distributn(0.0, 1.0)
    {

    }

    MutationBase(RNG & rng)
            : default_mutation_probability(0.10),
              probability_mutation(default_mutation_probability),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(rng),
              uniform_rand_distributn(0.0, 1.0)
    {

    }

    MutationBase(double & _probability_mutation)
            : default_mutation_probability(0.10),
              probability_mutation(_probability_mutation),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(default_rng_mutation),
              uniform_rand_distributn(0.0, 1.0)
    {

    }

    MutationBase(RNG & rng, double & _probability_mutation)
            : default_mutation_probability(0.10),
              probability_mutation(_probability_mutation),
              seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
              default_rng_mutation(seed_mutation),
              random_number_gen(rng),
              uniform_rand_distributn(0.0, 1.0)
    {

    }

    virtual void operator() (Individual & individual) = 0;

	virtual MutationBase &
    setMutationProbability(double _mutation_probability)
    {
        probability_mutation = _mutation_probability;
		return (*this);
    }

    void
    setMutationInverseDVSize(IndividualSPtr _ind_sample)
    {
        setMutationProbability(1 / double(_ind_sample->numberOfIntDecisionVariables() + _ind_sample->numOfRealDVs()));
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


        for (Individual::RealDVsT::size_type j=0; j < individual.numOfRealDVs(); j++)
        {
//            ++gene_count;
            if (uniform_rand_distributn(random_number_gen) <= probability_mutation)
            {
//                ++gene_m_count;
//                individual.mutated = true;
                double delta1, delta2, xy, val, deltaq;
                const double & current_val = individual.getRealDV(j);
                const double lower_bound_val=individual.getRealLowerBound(j);//lower limt of variable j of ind i
                const double upper_bound_val=individual.getRealUpperBound(j);//upper limt of variable j of ind i

                delta1 = (current_val-lower_bound_val)/(upper_bound_val-lower_bound_val);
                delta2 = (upper_bound_val-current_val)/(upper_bound_val-lower_bound_val);
                double rand = uniform_rand_distributn(random_number_gen);
                double mut_pow = 1.0/(eta_m+1.0);
                if (rand <= 0.5)
                {
                    xy = 1.0-delta1;
                    val = 2.0*rand+(1.0-2.0*rand)*(pow(xy,(eta_m+1.0)));
                    deltaq =  pow(val,mut_pow) - 1.0;
                }
                else
                {
                    xy = 1.0-delta2;
                    val = 2.0*(1.0-rand)+2.0*(rand-0.5)*(pow(xy,(eta_m+1.0)));
                    deltaq = 1.0 - (pow(val,mut_pow));
                }
                double new_val = current_val + deltaq*(upper_bound_val-lower_bound_val);
                if (new_val<lower_bound_val)
                    new_val = lower_bound_val;
                if (new_val>upper_bound_val)
                    new_val = upper_bound_val;
                individual.setRealDV(j, new_val);
            }
        }
        return;
    }

	virtual DebsPolynomialMutation&
	setMutationProbability(double _mutation_probability)
	{
		probability_mutation = _mutation_probability;
		return (*this);
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
        
        
        for (unsigned long j=0; j < individual.numOfUnorderedDVs(); j++)
        {
            
            
            //            ++gene_count;
            if (uniform_rand_distributn(random_number_gen) <= probability_mutation)
            {
                
                std::uniform_int_distribution<int> distribution(individual.getUnorderedLowerBound(j),individual.getUnorderedUpperBound(j));
                
                individual.setUnorderedDV(j, distribution(random_number_gen));
                
            }
        }
        return;
    }
    
	virtual UniformIntMutation&
		setMutationProbability(double _mutation_probability)
	{
		probability_mutation = _mutation_probability;
		return (*this);
	}

    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(probability_mutation);
    }
};


template <typename RNG = std::mt19937>
class CreepMutation : public MutationBase<RNG>
{
public:

	CreepMutation()
	{

	}

	/**
	 *
	 * @param _epsilon the range for uniform nutation
	 * @param _p_change the probability to change a given coordinate
	 * @param _p_change_down probability of changing to a lower value
	 */
	CreepMutation(double _p_change = 0.05, double _p_change_down = 0.65)
		: MutationBase(_p_change), p_change_down(_p_change_down), count1(0), count2(0), 
	{
	}



	/**
	 * Do it!
	 * @param _eo The indi undergoing the mutation
	 */

	void
		operator()(Individual& individual)
	{
		//bool hasChanged = false;
			for (Individual::OrderedDVsT::size_type i = 0; i < individual.numOfOrderedDVs(); i++)
			{
				if (uniform_rand_distributn(random_number_gen) <= probability_mutation)
				{
					if (uniform_rand_distributn(random_number_gen) <= p_change_down)
					{
						individual.setOrderedDV(i,individual.getOrderedDV(i)-1);
						if (individual.getOrderedDV(i) < individual.getOrderedLowerBound(i))
						{
							individual.setOrderedDV(i, individual.getOrderedUpperBound(i));
						}
					}
					else
					{
						individual.setOrderedDV(i, individual.getOrderedDV(i) + 1);
						if (individual.getOrderedDV(i) > individual.getOrderedUpperBound(i))
						{
							individual.setOrderedDV(i, individual.getOrderedLowerBound(i));
						}
					}
					//hasChanged = true;
					count2++;
				}
			}
		
		count1++;
	}

	virtual CreepMutation&
	setMutationProbability(double _mutation_probability)
	{
		probability_mutation = _mutation_probability;
		return (*this);
	}

	CreepMutation&
	setCreepDownProbability(double _p_change_down)
	{
		p_change_down = _p_change_down;
		return (*this);
	}

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		ar& BOOST_SERIALIZATION_NVP(probability_mutation);
		ar& BOOST_SERIALIZATION_NVP(p_change_down);
		ar& BOOST_SERIALIZATION_NVP(count1);
		ar& BOOST_SERIALIZATION_NVP(count2);
	}

private:
	//double p_change; // the proba that each variable is modified
	double p_change_down; // the proba that each variable is changed down
	int count1;
	int count2;
};



template <typename RNG = std::mt19937>
class CombinedRealIntMutation
{
    unsigned seed_mutation;
    RNG default_rng_mutation;
    RNG & random_number_gen;

    DebsPolynomialMutation<> default_real_mutation;
    UniformIntMutation<> default_unordered_mutation;
	CreepMutation<> default_ordered_mutation;

    MutationBase<RNG> & real_mut;
    MutationBase<RNG> & unordered_mut;
	MutationBase<RNG> & ordered_mut;
    
    
public:
    //Mutation is on a per chromosone basis
    CombinedRealIntMutation() :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(default_rng_mutation),
            default_real_mutation(random_number_gen),
            default_unordered_mutation(random_number_gen),
            real_mut(default_real_mutation),
            unordered_mut(default_unordered_mutation),
			ordered_mut(default_ordered_mutation)
    {

    }

    CombinedRealIntMutation(RNG & rng) :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(rng),
            default_real_mutation(random_number_gen),
            default_unordered_mutation(random_number_gen),
            real_mut(default_real_mutation),
            unordered_mut(default_unordered_mutation),
			ordered_mut(default_ordered_mutation)
    {

    }

    CombinedRealIntMutation(MutationBase<RNG> & _real_mut, MutationBase<RNG> & _unordered_mut, MutationBase<RNG> & _ordered_mut) :
            seed_mutation(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_mutation(seed_mutation),
            random_number_gen(default_rng_mutation),
            default_real_mutation(random_number_gen),
            default_unordered_mutation(random_number_gen),
            real_mut(_real_mut),
            unordered_mut(_unordered_mut)
			ordered_mut(_ordered_mut)
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
		for (IndividualSPtr ind : parent_pop)
		{
			real_mut(*ind);
			unordered_mut(*ind);
			ordered_mut(*ind);

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
    getUnorderedMutationOperator()
    {
        return (unordered_mut);
    }

	MutationBase<RNG> &
	getOrderedMutationOperator()
	{
		return (ordered_mut);
	}
    
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(real_mut);
        ar & BOOST_SERIALIZATION_NVP(unordered_mut);
    }
};

#endif /* Mutation_h */
