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
#include "Types.hpp"

#include <boost/serialization/nvp.hpp>


class CrossoverBase
{
public:
    virtual void operator()(Individual & individal_1, Individual & individal_2) = 0;
};

double default_eps = 0.00001;
double default_eta = 20.0;
std::uniform_real_distribution<double> cross_uniform(0.0,1.0);
unsigned seed_crossover = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 default_rng_crossover(seed_crossover);

template <typename RNG = std::mt19937>
class DebsSBXCrossover : public CrossoverBase
{
    RNG & random_number_gen;
    double & eps;
    double & eta_c;
    
    
public:
    
    DebsSBXCrossover(RNG & rng = default_rng_crossover, double & eta = default_eta, double & _eps = default_eps) :
    random_number_gen(rng), eps(_eps), eta_c(eta)
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
            
            if (cross_uniform(random_number_gen)<=0.5) //only half of the bits in chrom2 will be crossed
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
                    
                    double rand = cross_uniform(random_number_gen);
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
                    
                    if (cross_uniform(random_number_gen)<=0.5)
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
class OnePointCrossover : public CrossoverBase
{
    RNG & random_number_gen;
    
    
public:
    
    OnePointCrossover(RNG & rng = default_rng_crossover) :
    random_number_gen(rng)
    {
        
    }
    
    virtual void operator()(Individual & individal_1, Individual & individal_2)
    {
        //        individal_1.crossovered = true;
        //        individal_2.crossovered = true;
        //Determine location of crossover.
        int location = int(std::ceil(cross_uniform(random_number_gen) * (individal_1.numberOfIntDecisionVariables() - 1)));
        
        // To the right of location, the chromosones are crossovered.
        
        for (int j=0;j < individal_1.numberOfIntDecisionVariables(); ++j)
        {//for each real in chrom
            
//            if (j <= location) // no crossover
//            {
//                // Do nothing. Leave each chromosone as it is
//            }
            if (j > location)
            {
                int val = individal_1.getIntDV(j);
                individal_1.setIntDV(j, individal_2.getIntDV(j));
                individal_2.setIntDV(j, val);                
            }
        }
        
    }
    
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // no persistent information required to rebuild...
    }
};





//double default_proportion_crossed = 0.5;
double default_crossoverp = 0.9;
DebsSBXCrossover<> default_real_xover;
OnePointCrossover<> default_int_xover;

template <typename RNG = std::mt19937>
class CombinedRealIntCrossover
{
    RNG & random_number_gen;
    double & probability_crossover;
    CrossoverBase & real_xover;
    CrossoverBase & int_xover;
    
    
public:
    
    CombinedRealIntCrossover(RNG & rng = default_rng_crossover, double _probability_crossover = default_crossoverp, CrossoverBase & _real_xover = default_real_xover, CrossoverBase & _int_xover = default_int_xover) :
    random_number_gen(rng), probability_crossover(_probability_crossover), real_xover(_real_xover), int_xover(_int_xover)
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
        std::shuffle(pop->begin(), pop->end(), random_number_gen);
        
//        int count = 0;
//        int x_count = 0;

        for (int i = 0; i <= pop->size()-2; i = i + 2)
        {
//            ++count;
            double rand = cross_uniform(random_number_gen);
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
