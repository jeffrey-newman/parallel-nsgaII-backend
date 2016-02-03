//
//  EvaluationServer.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef EvaluationServer_h
#define EvaluationServer_h

#include <tuple>
#include <vector>
#include <boost/foreach.hpp>
#include "Types.hpp"


class EvaluatorBase
{
public:
    virtual
    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars) = 0;
};

class DummyEvaluator : public EvaluatorBase
{
private:
    std::pair<std::vector<double>, std::vector<double> > dummy_return;
    
public:
    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars)
    {
        return (dummy_return);
    }
};

class EvaluatePopulationBase
{
public:
    virtual void
    operator()(PopulationSPtr population) = 0;
};

class EvaluatePopulation : public EvaluatePopulationBase
{
    EvaluatorBase & eval;
    
public:
    EvaluatePopulation(EvaluatorBase & _eval)
    : eval(_eval)
    {
        
    }
    
    void
    operator()(PopulationSPtr population)
    {
        BOOST_FOREACH(Individual & ind, *population)
        {
            std::vector<double> objectives;
            std::vector<double> constraints;
            std::tie(objectives, constraints) = eval(ind.getRealDVVector(), ind.getIntDVVector());
            ind.setObjectives(objectives);
            ind.setConstraints(constraints);
            
            // Too much copying of data in this function...
        }
    }
};

#endif /* EvaluationServer_h */
