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

	typedef ProblemDefinitions::ObjectivesAndConstraintsT ObjectivesAndConstraintsT;
	typedef ProblemDefinitions::RealDVsT RealDVsT;
	typedef ProblemDefinitions::UnorderedDVsT UnorderedDVsT;
	typedef ProblemDefinitions::OrderedDVsT OrderedDVsT;

    /**
     * Evaluate a solution.
     * @param real_decision_vars
     * @param int_decision_vars
     * @return
     */
    virtual
	ObjectivesAndConstraintsT
    operator()(const RealDVsT & real_decision_vars, const UnorderedDVsT & unordered_decision_vars, 
		const OrderedDVsT & ordered_dvs) = 0;

    /**
     * Evaluate a solution, including saving details of the evaluation in save_dir.
     * @param real_decision_vars
     * @param int_decision_vars
     * @param save_dir Where the evaluation is saved to.
     * @return
     */
    virtual
	ObjectivesAndConstraintsT
    operator()(const RealDVsT& real_decision_vars, const UnorderedDVsT& unordered_decision_vars, 
		const OrderedDVsT& ordered_dvs, const boost::filesystem::path & save_dir) = 0;
};

class DummyObjectivesAndConstraints : public ObjectivesAndConstraintsBase
{
private:
    //std::pair<std::vector<double>, std::vector<double> > dummy_return;
    
public:
	ObjectivesAndConstraintsT
    operator()(const RealDVsT& real_decision_vars, const UnorderedDVsT& unordered_decision_vars,
		const OrderedDVsT& ordered_dvs)
    {
        return (std::pair<std::vector<double>, std::vector<double> >());
    }

	ObjectivesAndConstraintsT
    operator()(const RealDVsT& real_decision_vars, const UnorderedDVsT& unordered_decision_vars, 
		const OrderedDVsT& ordered_dvs, const boost::filesystem::path& save_dir)
    {
        return (std::pair<std::vector<double>, std::vector<double> >());
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
            ind->setObjectivesAndConstraints(eval(ind->getRealDVVector(), ind->getUnorderedDVVector(), ind->getOrderedDVVector()));
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
						ind->setObjectivesAndConstraints(eval(ind->getRealDVVector(), ind->getUnorderedDVVector(), ind->getOrderedDVVector(), save_ind_dir));
                    }
    }
};

#endif /* EvaluationServer_h */
