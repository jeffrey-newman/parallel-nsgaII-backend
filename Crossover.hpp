//
//  Crossover.hpp
//  parallel-nsgaII-backend
//
//  Created by Jeffrey Newman on 19/11/2015.
//  Copyright © 2015 University of Adelaide and Bushfire and Natural Hazards CRC. All rights reserved.
//

/*this file contains the crossover operator, which apply one point crossover to
 the integer chromosome chrom1 and SBX to the real-coded chromsome chrom2.*/

// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.33.7291&rep=rep1&type=pdf

#ifndef Crossover_h
#define Crossover_h

#include <cmath>
#include <chrono>
#include "Population.hpp"

#include <boost/serialization/nvp.hpp>

template <typename RNG = std::mt19937>
class CrossoverBase
{
public:
    CrossoverBase():
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(default_rng_crossover)
            {

            }

    CrossoverBase(RNG & rng):
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(rng)
    {

    }

    virtual void operator()(Individual & individal_1, Individual & individal_2) = 0;

protected:
    std::uniform_real_distribution<double> cross_uniform;
    unsigned seed_crossover;
    RNG default_rng_crossover;
    RNG & random_number_gen;
};




template <typename RNG = std::mt19937>
class DebsSBXCrossover : public CrossoverBase<RNG>
{
    double default_eps;
    double default_eta;
    double eps;
    double eta_c;
    
    
public:

    DebsSBXCrossover() :
            default_eps(0.00001),
            default_eta(20.0),
            eps(default_eps),
            eta_c(default_eta)
    {

    }

    DebsSBXCrossover(RNG & rng) :
            CrossoverBase<RNG>(rng),
            default_eps(0.00001),
            default_eta(20.0),
            eps(default_eps),
            eta_c(default_eta)
    {

    }
    
    DebsSBXCrossover(RNG & rng, double & eta, double & _eps) :
            CrossoverBase<RNG>(rng),
            default_eps(0.00001),
            default_eta(20.0),
            eps(_eps),
            eta_c(eta)
    {
        
    }
    
    DebsSBXCrossover & setEps(double & _eps)
    {
        eps = _eps;
        return (*this);
    }
    
    DebsSBXCrossover & setEtaC(double & _eta_c)
    {
        eta_c = _eta_c;
        return (*this);
    }
    
    
    virtual void operator()(Individual & individal_1, Individual & individal_2)
    {
        //        individal_1.crossovered = true;
        //        individal_2.crossovered = true;
        for (int j=0;j < individal_1.numberOfRealDecisionVariables(); ++j)
        {//for each real in chrom
            
            const double & dvar_value_par1 = individal_1.getRealDV(j);
            const double & dvar_value_par2 = individal_2.getRealDV(j);
            const double & yl = individal_1.getRealLowerBound(j);//lower limt of variable j of ind i
            const double & yu = individal_1.getRealUpperBound(j);//upper limt of variable j of ind i
            
            if (this->cross_uniform(this->random_number_gen)<=0.5) //only half of the bits in chrom2 will be crossed
            {
                double y1, y2;
                if (fabs( dvar_value_par1 - dvar_value_par2) > eps) //values of two parents are different
                {
                    if (dvar_value_par2 > dvar_value_par1)
                    {
                        y2 = dvar_value_par2; //y2 is the parent that has larger value
                        y1 = dvar_value_par1;
                    }
                    else
                    {
                        y2 = dvar_value_par1;
                        y1 = dvar_value_par2;
                    }
                    
                    //for the first child
                    double beta = 1.0 + (2.0 * (y1 - yl) / (y2 - y1));
                    double alpha = 2.0 - pow(beta, -(eta_c + 1.0));
                    double betaq;
                    
                    double rand = this->cross_uniform(this->random_number_gen);
                    if (rand <= (1.0 / alpha))
                        betaq = pow((rand * alpha), (1.0 / (eta_c + 1.0)));
                    else
                        betaq=pow((1.0 / (2.0 - rand * alpha)), (1.0 / (eta_c + 1.0)));
                    
                    double dvar_val_child1 = 0.5 * ((y1 + y2) - betaq * (y2 - y1));
                    
                    //for the second child
                    beta = 1.0 + (2.0 * (yu - y2) / (y2 - y1));
                    alpha = 2.0 - pow(beta, -(eta_c+1.0));
                    if (rand <= (1.0 / alpha))
                        betaq = pow((rand * alpha), (1.0 / (eta_c + 1.0)));
                    else
                        betaq=pow((1.0 / (2.0 - rand * alpha)), (1.0 / (eta_c + 1.0)));
                    
                    double dvar_val_child2 = 0.5 * ((y1 + y2) + betaq * (y2 - y1));
                    
                    if (dvar_val_child1<yl) dvar_val_child1=yl;
                    if (dvar_val_child1>yu) dvar_val_child1=yu;
                    if (dvar_val_child2<yl) dvar_val_child2=yl;
                    if (dvar_val_child2>yu) dvar_val_child2=y2;
                    
                    if (this->cross_uniform(this->random_number_gen)<=0.5)
                    {
                        individal_1.setRealDV(j, dvar_val_child2);
                        individal_2.setRealDV(j, dvar_val_child1);
                    }
                    else
                    {
                        individal_1.setRealDV(j, dvar_val_child1);
                        individal_2.setRealDV(j, dvar_val_child2);
                    }
                }
            }
            
        }
        
    }
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(eps);
        ar & BOOST_SERIALIZATION_NVP(eta_c);

    }
};

template <typename RNG = std::mt19937>
class OnePointCrossover : public CrossoverBase<RNG>
{
private:

    
public:
    OnePointCrossover()
    {

    }
    
    OnePointCrossover(RNG & rng) :
            CrossoverBase<RNG>(rng)
    {
        
    }
    
    virtual void operator()(Individual & individal_1, Individual & individal_2)
    {
        //        individal_1.crossovered = true;
        //        individal_2.crossovered = true;
        //Determine location of crossover.
        int location = int(std::ceil(this->cross_uniform(this->random_number_gen) * (individal_1.numberOfIntDecisionVariables() - 1)));
        
        // To the right of location, the chromosones are crossovered.
        
        for (int j=location;j < individal_1.numberOfIntDecisionVariables(); ++j)
        {//for each real in chrom
            
//
                int val = individal_1.getIntDV(j);
                individal_1.setIntDV(j, individal_2.getIntDV(j));
                individal_2.setIntDV(j, val);                

        }
        
    }
    
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // no persistent information required to rebuild...
    }
};







template <typename RNG = std::mt19937>
class CombinedRealIntCrossover
{
    std::uniform_real_distribution<double> cross_uniform;
    unsigned seed_crossover;
    RNG default_rng_crossover;
    RNG & random_number_gen;

    double default_crossoverp;
    DebsSBXCrossover<> default_real_xover;
    OnePointCrossover<> default_int_xover;

    double probability_crossover;
    CrossoverBase<RNG> & real_xover;
    CrossoverBase<RNG> & int_xover;

public:

    CombinedRealIntCrossover():
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(default_rng_crossover),

            default_crossoverp(0.9),
            default_real_xover(random_number_gen),
            default_int_xover(random_number_gen),

            probability_crossover(default_crossoverp),
            real_xover(default_real_xover),
            int_xover(default_int_xover)
    {

    }

    CombinedRealIntCrossover(double & _probability_crossover):
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(default_rng_crossover),

            default_crossoverp(0.9),
            default_real_xover(random_number_gen),
            default_int_xover(random_number_gen),

            probability_crossover(_probability_crossover),
            real_xover(default_real_xover),
            int_xover(default_int_xover)
    {

    }

    CombinedRealIntCrossover(double & _probability_crossover, CrossoverBase<RNG> & _real_xover, CrossoverBase<RNG> & _int_xover) :
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(default_rng_crossover),

            default_crossoverp(0.9),
            default_real_xover(random_number_gen),
            default_int_xover(random_number_gen),

            probability_crossover(_probability_crossover),
            real_xover(_real_xover),
            int_xover(_int_xover)
    {

    }

    CombinedRealIntCrossover(RNG & rng, double & _probability_crossover):
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(rng),

            default_crossoverp(0.9),
            default_real_xover(random_number_gen),
            default_int_xover(random_number_gen),

            probability_crossover(_probability_crossover),
            real_xover(default_real_xover),
            int_xover(default_int_xover)
    {

    }

    CombinedRealIntCrossover(RNG & rng, double & _probability_crossover, CrossoverBase<RNG> & _real_xover, CrossoverBase<RNG> & _int_xover) :
            cross_uniform(0.0, 1.0),
            seed_crossover(std::chrono::system_clock::now().time_since_epoch().count()),
            default_rng_crossover(seed_crossover),
            random_number_gen(rng),

            default_crossoverp(0.9),
            default_real_xover(random_number_gen),
            default_int_xover(random_number_gen),

            probability_crossover(_probability_crossover),
            real_xover(_real_xover),
            int_xover(_int_xover)
    {

    }

    CombinedRealIntCrossover & setProbabilityCrossover(double & _probability_crossover)
    {
        probability_crossover = _probability_crossover;
        return (*this);
    }
    
    void
    operator()(PopulationSPtr pop)
    {
        pop->invalidate();
        std::shuffle(pop->begin(), pop->end(), this->random_number_gen);
        
//        int count = 0;
//        int x_count = 0;

        for (int i = 0; i <= pop->size()-2; i = i + 2)
        {
//            ++count;
            double rand = this->cross_uniform(this->random_number_gen);
//            std::cout << rand << "\n";
            if (rand <= probability_crossover)
            {
//                ++x_count;
                IndividualSPtr parent1 = (*pop)[i];
                IndividualSPtr parent2 = (*pop)[i+1];
                real_xover(*parent1, *parent2);
                int_xover(*parent1, *parent2);
            }
        }


//       std::cout << "Crossover: " << x_count << " of " << count << " iterations underwent: " << 100 * double(x_count) / count << "%\n";
        
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(probability_crossover);
        ar & BOOST_SERIALIZATION_NVP(real_xover);
        ar & BOOST_SERIALIZATION_NVP(int_xover);
    }
};







#endif /* Crossover_h */
