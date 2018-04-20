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
#include "Population.hpp"


class ObjectivesAndConstraintsBase
{
public:
    /**
     * Evaluate a solution.
     * @param real_decision_vars
     * @param int_decision_vars
     * @return
     */
    virtual
    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars) = 0;

    /**
     * Evaluate a solution, including saving details of the evaluation in save_dir.
     * @param real_decision_vars
     * @param int_decision_vars
     * @param save_dir Where the evaluation is saved to.
     * @return
     */
    virtual
    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars, const boost::filesystem::path & save_dir) = 0;
};

class DummyObjectivesAndConstraints : public ObjectivesAndConstraintsBase
{
private:
    std::pair<std::vector<double>, std::vector<double> > dummy_return;
    
public:
    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars)
    {
        return (dummy_return);
    }

    std::pair<std::vector<double>, std::vector<double> > &
    operator()(const std::vector<double> & real_decision_vars, const std::vector<int> & int_decision_vars, const boost::filesystem::path & save_dir)
    {
        return (dummy_return);
    };
};

class EvaluatePopulationBase
{
public:
    /**
     * Evaluation all memebers of a population
     * @param population
     */
    virtual void
    operator()(PopulationSPtr population) = 0;

    /**
     * Evaluate all memebers of a population, and save each memeber. Implementations of this
     * function will need to create directories for each member of the population to be
     * saved into.
     * @param population
     * @param save_dir where the members of population are saved to. Make subdirectories in this folder.
     */
    virtual void
    operator()(PopulationSPtr population, const boost::filesystem::path & save_dir) = 0;
};

class EvaluatePopulation : public EvaluatePopulationBase
{
    ObjectivesAndConstraintsBase & eval;
    
public:
    EvaluatePopulation(ObjectivesAndConstraintsBase & _eval)
    : eval(_eval)
    {
        
    }
    
    void
    operator()(PopulationSPtr population)
    {
        for(IndividualSPtr ind: *population)
        {
            ind->getMutableObjectivesAndConstraints() = eval(ind->getRealDVVector(), ind->getIntDVVector());
        }
    }

    void
    operator()(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        int i = 0;
        for(IndividualSPtr ind: *population)
                    {
                        boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(i++));
                        if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
                        ind->getMutableObjectivesAndConstraints() = eval(ind->getRealDVVector(), ind->getIntDVVector(), save_ind_dir);
                    }
    }
};

#endif /* EvaluationServer_h */
