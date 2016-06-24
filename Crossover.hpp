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


double default_eps = 0.00001;
double default_crossoverp = 0.7;
double default_eta = 20.0;
std::uniform_real_distribution<double> cross_uniform(0.0,1.0);
double default_proportion_crossed = 0.5;
unsigned seed_crossover = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 default_rng_crossover(seed_crossover);

template <typename RNG = std::mt19937>
class DebsSBXCrossover
{
    RNG & random_number_gen;
    double & probability_crossover;
    double & eps;
    double & eta_c;
    double & proportion_crossed;
    
    
public:
    
    DebsSBXCrossover(RNG & rng = default_rng_crossover, double & eta = default_eta, double & _probability_crossover = default_crossoverp, double & _eps = default_eps, double & _proportion_crossed = default_proportion_crossed) :
    random_number_gen(rng), probability_crossover(_probability_crossover), eps(_eps), eta_c(eta),  proportion_crossed(_proportion_crossed)
    {
        
    }
    
    
    void crossover_implementation(Individual & individal_1, Individual & individal_2)
    {
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
    
    void
    operator()(PopulationSPtr pop)
    {
        pop->invalidate();
        std::shuffle(pop->begin(), pop->end(), random_number_gen);
        
        for (int i = 0; i < pop->size() / 2; i = i + 2)
        {
            
            if (cross_uniform(random_number_gen) <= probability_crossover)
            {
                IndividualSPtr parent1 = (*pop)[i];
                IndividualSPtr parent2 = (*pop)[i+1];
                crossover_implementation(*parent1, *parent2);
            }
        }
        
        
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(probability_crossover);
            ar & BOOST_SERIALIZATION_NVP(eps);
            ar & BOOST_SERIALIZATION_NVP(eta_c);
            ar & BOOST_SERIALIZATION_NVP(proportion_crossed);
    }
};




#endif /* Crossover_h */
