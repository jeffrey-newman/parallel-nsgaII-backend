//
//  Individual.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Individual_h
#define Individual_h

#include <vector>
#include "ProblemDefinitions.hpp"
#include <boost/archive/tmpdir.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/foreach.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/home/support/common_terminals.hpp>

class Individual;
typedef Individual * IndividualPtr;
typedef boost::shared_ptr<Individual> IndividualSPtr;

class Population;
typedef boost::shared_ptr<Population> PopulationSPtr;
typedef Population Front;
typedef PopulationSPtr FrontSPtr;
typedef std::vector<Front> Fronts;
typedef boost::shared_ptr<Fronts > FrontsSPtr;
inline FrontsSPtr debNonDominatedSort(const Population& population_ref);
typedef std::vector<std::pair<IndividualSPtr, double > > IndividualsWithCrowdingDistanceVec;
inline IndividualsWithCrowdingDistanceVec calculateDebsCrowdingDistance(const Front& front_set);

class EvaluatePopulationBase;

class Individual
{
	friend FrontsSPtr debNonDominatedSort(const Population& population_ref);
	friend IndividualsWithCrowdingDistanceVec calculateDebsCrowdingDistance(const Front& front_set);
	friend class EvaluatePopulation;
	friend class ParallelEvaluatePopServerNonBlocking;
	template <typename RNG> friend class ParallelEvaluatePopServerNonBlockingContinuousEvolution;
	friend class ParallelEvaluatorServerBase;
	template <typename RNG> friend class CreepMutation;
	template <typename RNG> friend class UniformIntMutation;
	template <typename RNG> friend class DebsPolynomialMutation;
	template <typename RNG> friend class OnePointCrossover;
	template <typename RNG> friend class DebsSBXCrossover;

public:

	typedef ProblemDefinitions::RealDVsT RealDVsT;
	typedef ProblemDefinitions::UnorderedDVsT UnorderedDVsT;
	typedef ProblemDefinitions::OrderedDVsT OrderedDVsT;
	typedef ProblemDefinitions::ObjectivesT ObjectivesT;
	typedef ProblemDefinitions::ConstraintsT ConstraintsT;
	typedef ProblemDefinitions::MinOrMaxType MinOrMaxType;
	typedef ProblemDefinitions::ObjectivesDirectionT ObjectivesDirectionT;
	typedef ProblemDefinitions::ObjectivesAndConstraintsT ObjectivesAndConstraintsT;

private:
    
    ProblemDefinitionsSPtr definitions;
	RealDVsT real_decision_variables;
	UnorderedDVsT unordered_dvs;
	OrderedDVsT ordered_dvs;
	ObjectivesT objectives;
	ConstraintsT constraints;
    int rank;
    double crowding_score;
    
public:
    
//    Individual(DecisionVariableType dv_type, int number_of_decision_variables)
    
//    Individual(int number_of_real_decision_variables, int number_of_int_decision_variables = 0)
//    : definitions(number_of_real_decision_variables, number_of_int_decision_variables)
//    {
//        
//    }
//    
//    Individual(std::vector<double> & _real_lowerbounds,  std::vector<double> & _real_upperbounds,
//               std::vector<int> & _int_lowerbounds, std::vector<int> & _int_upperbounds, std::vector<MinOrMaxType> _min_or_max_vec)
//    : definitions(_real_lowerbounds, _real_upperbounds, _int_lowerbounds, _int_upperbounds, _min_or_max_vec)
//    {
//        
//    }
//    
//    Individual(std::vector<double> & _real_lowerbounds,  std::vector<double> & _real_upperbounds,
//               std::vector<int> & _int_lowerbounds, std::vector<int> & _int_upperbounds, std::vector<MinOrMaxType> _min_or_max_vec,
//               std::initializer_list<double> _init_real_dv_list, std::initializer_list<int> _init_int_dv_list)
//    : definitions(_real_lowerbounds, _real_upperbounds, _int_lowerbounds, _int_upperbounds, _min_or_max_vec),
//     real_decision_variables(_init_real_dv_list), int_decision_variables(_init_int_dv_list)
//    {
//        
//    }
    
    Individual(const Individual & cpy)
    : definitions(cpy.definitions), 
		real_decision_variables(cpy.real_decision_variables), 
		unordered_dvs(cpy.unordered_dvs), 
		ordered_dvs(cpy.ordered_dvs),
		objectives(cpy.objectives), 
		constraints(cpy.constraints), 
		rank(cpy.rank), 
		crowding_score(cpy.crowding_score)//, mutated(false), crossovered(false), child(false), parent(false)
    {
        
    }
    
    Individual(ProblemDefinitionsSPtr defs)
    : definitions(defs), 
		real_decision_variables(defs->real_lowerbounds.size()), 
		unordered_dvs(defs->unordered_lowerbounds.size()), 
		ordered_dvs(defs->ordered_lowerbounds.size()),
		objectives(defs->minimise_or_maximise.size()), 
		constraints(defs->number_constraints), 
		rank(std::numeric_limits<int>::max()), 
		crowding_score(std::numeric_limits<double>::min())//, mutated(false), crossovered(false), child(false), parent(false)
    {
        
    }

    Individual(std::string & s, ProblemDefinitionsSPtr defs)
            : definitions(defs), 
		real_decision_variables(defs->real_lowerbounds.size()), 
		unordered_dvs(defs->unordered_lowerbounds.size()), 
		objectives(defs->minimise_or_maximise.size()), 
		ordered_dvs(defs->ordered_lowerbounds.size()),
		constraints(defs->number_constraints), 
		rank(std::numeric_limits<int>::max()), 
		crowding_score(std::numeric_limits<double>::min())//, mutated(false), crossovered(false), child(false), parent(false)
    {
		parse(s, defs.get());
    }

    Individual(std::string & s, ProblemDefinitions * defs)
        : definitions(defs), 
		real_decision_variables(defs->real_lowerbounds.size()), 
		unordered_dvs(defs->unordered_lowerbounds.size()), 
		ordered_dvs(defs->ordered_lowerbounds.size()), 
		objectives(defs->minimise_or_maximise.size()), 
		constraints(defs->number_constraints), 
		rank(std::numeric_limits<int>::max()), 
		crowding_score(std::numeric_limits<double>::min())//, mutated(false), crossovered(false), child(false), parent(false)
    {
		parse(s, defs);
    }

	void
		parse(std::string& s, ProblemDefinitions* defs)
	{
		//Format of population seeding file:
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)
		//[int_dv1 int_dv2 ... ; real_dv1 real_dv2 ..... ]    -> (obj1 obj2 ...; cnstr1 cnstr2....)

		namespace qi = boost::spirit::qi;
		namespace ph = boost::phoenix;

		qi::rule<std::string::iterator, std::vector<int>(), qi::space_type> int_vec_parser = *qi::int_;
		qi::rule<std::string::iterator, std::vector<double>(), qi::space_type> real_vec_parser = *qi::double_;
		qi::rule<std::string::iterator, qi::space_type> ind_parser =
			qi::lit('[') >>
			int_vec_parser[ph::ref(unordered_dvs) = qi::_1] >>
			qi::lit(';') >>
			int_vec_parser[ph::ref(ordered_dvs) = qi::_1] >>
			qi::lit(';') >>
			real_vec_parser[ph::ref(real_decision_variables) = qi::_1] >>
			qi::lit(']')
			>> -(
				qi::lit("->")
				>> qi::lit('(')
				>> real_vec_parser[ph::ref(objectives) = qi::_1]
				>> qi::lit(';')
				>> real_vec_parser[ph::ref(constraints) = qi::_1]
				>> qi::lit(')')
				)
			>> -(qi::lit("Rank:") >> qi::int_[ph::ref(rank) = qi::_1])
			>> -(qi::lit("CrowdingDist:") >> qi::double_[ph::ref(crowding_score) = qi::_1])
			;
		ind_parser.name("individual_parser");
		int_vec_parser.name("int_vec_parser");
		real_vec_parser.name("real_vec_parser");
		//        qi::debug(int_vec_parser);
		//        qi::debug(real_vec_parser);
		//        qi::debug(ind_parser);
		std::string::iterator it = s.begin();
		std::string::iterator end = s.end();
		bool r = boost::spirit::qi::phrase_parse(it, end, ind_parser, qi::space);
		if (!(r && it == end))
		{
			std::string rest(it, end);
			std::cout << "-------------------------\n";
			std::cout << "Parsing failed\n";
			std::cout << "stopped at: \"" << rest << "\"\n";
			std::cout << "-------------------------\n";
		}

		if (real_decision_variables.size() < numOfRealDVs())
		{
			std::cerr << "Check specification - read in " << real_decision_variables.size() << "; However, problem specification indicated there would be " << numOfRealDVs() << " real decision variables." << std::endl;
			real_decision_variables.resize(numOfRealDVs());
		}
		if (unordered_dvs.size() < numOfUnorderedDVs())
		{
			std::cerr << "Check specification - read in " << unordered_dvs.size() << "; However, problem specification indicated there would be " << numOfUnorderedDVs() << " integer decision variables." << std::endl;
			unordered_dvs.resize(numOfUnorderedDVs());
		}
		if (ordered_dvs.size() < numOfOrderedDVs())
		{
			std::cerr << "Check specification - read in " << ordered_dvs.size() << "; However, problem specification indicated there would be " << numOfOrderedDVs() << " integer decision variables." << std::endl;
			ordered_dvs.resize(numOfOrderedDVs());
		}
		for (RealDVsT::size_type j = 0; j < real_decision_variables.size(); ++j)
		{
			if (real_decision_variables[j] < defs->real_lowerbounds[j])
			{
				std::cerr << "input real decision variable " << real_decision_variables[j] << " at place " << j << " out of bounds; setting to lower bound which is " << defs->real_lowerbounds[j] << std::endl;
				real_decision_variables[j] = defs->real_lowerbounds[j];
			}
			if (real_decision_variables[j] > defs->real_upperbounds[j])
			{
				std::cerr << "input real decision variable " << real_decision_variables[j] << " at place " << j << " out of bounds; setting to upper bound which is " << defs->real_upperbounds[j] << std::endl;
				real_decision_variables[j] = defs->real_upperbounds[j];
			}
		}
		for (UnorderedDVsT::size_type j = 0; j < unordered_dvs.size(); ++j)
		{
			if (unordered_dvs[j] < defs->unordered_lowerbounds[j])
			{
				std::cerr << "input unordered decision variable " << unordered_dvs[j] << " at place " << j << " out of bounds; setting to lower bound which is " << defs->unordered_lowerbounds[j] << std::endl;
				unordered_dvs[j] = defs->unordered_lowerbounds[j];
			}
			if (unordered_dvs[j] > defs->unordered_upperbounds[j])
			{
				std::cerr << "input unordered decision variable " << unordered_dvs[j] << " at place " << j << " out of bounds; setting to upper bound which is " << defs->unordered_upperbounds[j] << std::endl;
				unordered_dvs[j] = defs->unordered_upperbounds[j];
			}
		}
		for (OrderedDVsT::size_type j = 0; j < ordered_dvs.size(); ++j)
		{
			if (ordered_dvs[j] < defs->ordered_lowerbounds[j])
			{
				std::cerr << "input ordered decision variable " << ordered_dvs[j] << " at place " << j << " out of bounds; setting to lower bound which is " << defs->ordered_lowerbounds[j] << std::endl;
				ordered_dvs[j] = defs->ordered_lowerbounds[j];
			}
			if (ordered_dvs[j] > defs->ordered_upperbounds[j])
			{
				std::cerr << "input ordered decision variable " << ordered_dvs[j] << " at place " << j << " out of bounds; setting to upper bound which is " << defs->ordered_upperbounds[j] << std::endl;
				ordered_dvs[j] = defs->ordered_upperbounds[j];
			}
		}


		if (objectives.size() < numOfObjectives()) objectives.resize(numOfObjectives());
		if (constraints.size() < numOfConstraints()) constraints.resize(numOfConstraints());
	}

    Individual()
    {

    }

	// Should not be able to change problem definitions. THese should be set at construction and not changed, else behaviour could be weird?
    //void
    //setProblemDefinitions(ProblemDefinitionsSPtr defs)
    //{
    //    definitions.swap(defs);
    //}
    
    Individual &
    operator= ( const Individual & orig)
    {
        this->definitions = orig.definitions;
        this->real_decision_variables = orig.real_decision_variables;
        this->unordered_dvs = orig.unordered_dvs;
		this->ordered_dvs = orig.ordered_dvs;
        this->objectives = orig.objectives;
        this->constraints = orig.constraints;
        this->rank = orig.rank;
        this->crowding_score = orig.crowding_score;
        return (*this);
    }
    
    const RealDVsT &
    getRealDVVector() const
    {
        return (real_decision_variables);
    }
    
    const UnorderedDVsT &
    getUnorderedDVVector() const
    {
        return (unordered_dvs);
    }

	const OrderedDVsT &
	getOrderedDVVector() const
	{
		return (unordered_dvs);
	}
    
    const ObjectivesT &
    getObjectives() const
    {
        return objectives;
    }
    
    const ConstraintsT &
    getConstraints() const
    {
        return constraints;
    }
    
    const MinOrMaxType &
    isMinimiseOrMaximise(const ProblemDefinitions::ObjectivesDirectionT::size_type index) const
    {
        return (definitions->minimise_or_maximise[index]);
        
    }
    
    const double &
    getRealDV(const RealDVsT::size_type index) const
    {
        return (real_decision_variables[index]);
    }
    
    const int &
    getUnorderedDV(const UnorderedDVsT::size_type index) const
    {
        return (unordered_dvs[index]);
    }

	const int&
	getOrderedDV(const OrderedDVsT::size_type index) const
	{
		return (ordered_dvs[index]);
	}

    const double & getObjective(ObjectivesT::size_type index) const
    {
        return (objectives[index]);
    }
    
    const double & getConstraint(ConstraintsT::size_type index) const
    {
        return (constraints[index]);
    }
    
    const int getRank(void) const
    {
        return (rank);
    }
    
    const double getCrowdingScore(void) const
    {
        return (crowding_score);
    }
    
    const double & getRealUpperBound(RealDVsT::size_type index) const
    {
        return (definitions->real_upperbounds[index]);
    }
    
    const double & getRealLowerBound(const RealDVsT::size_type index) const
    {
        return (definitions->real_lowerbounds[index]);
    }
    
    const int getUnorderedUpperBound(const UnorderedDVsT::size_type index) const
    {
        return (definitions->unordered_upperbounds[index]);
    }
    
    const int getUnorderedLowerBound(const UnorderedDVsT::size_type index) const
    {
        return (definitions->unordered_lowerbounds[index]);
    }

	const int getOrderedUpperBound(const OrderedDVsT::size_type index) const
	{
		return (definitions->ordered_upperbounds[index]);
	}

	const int getOrderedLowerBound(const OrderedDVsT::size_type index) const
	{
		return (definitions->ordered_lowerbounds[index]);
	}
    
    const RealDVsT::size_type numOfRealDVs() const
    {
        return (definitions->real_lowerbounds.size());
    }
    
    const OrderedDVsT::size_type numOfOrderedDVs() const
    {
        return (definitions->ordered_lowerbounds.size());
    }
    
	const  UnorderedDVsT::size_type numOfUnorderedDVs() const
	{
		return (definitions->unordered_lowerbounds.size());
	}

    const ObjectivesDirectionT::size_type numOfObjectives() const
    {
        return (definitions->minimise_or_maximise.size());
    }
    
    const unsigned int numOfConstraints() const
    {
        return (definitions->number_constraints);
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(real_decision_variables);
            ar & BOOST_SERIALIZATION_NVP(unordered_dvs);
			ar& BOOST_SERIALIZATION_NVP(ordered_dvs);
            ar & BOOST_SERIALIZATION_NVP(objectives);
            ar & BOOST_SERIALIZATION_NVP(constraints);
            ar & BOOST_SERIALIZATION_NVP(rank);
            ar & BOOST_SERIALIZATION_NVP(crowding_score);
            ar & BOOST_SERIALIZATION_NVP(definitions);
    }

    friend std::ostream& operator<<(std::ostream& os, const Individual& Individual);


	private:

		//ObjectivesAndConstraintsT
		//	getMutableObjectivesAndConstraints()
		//{
		//	return ObjectivesAndConstraintsT(objectives, constraints);
		//};

		void setObjectivesAndConstraints(ObjectivesAndConstraintsT objs_and_constraints)
		{
			setObjectives(objs_and_constraints.first);
			setConstraints(objs_and_constraints.second);
		}

		void setObjective(const int index, const double& value)
		{
			objectives[index] = value;
		}

		void
			setObjectives(ProblemDefinitions::ObjectivesT objs)
		{
			objectives = objs;
		}

		void setConstraint(const int index, const double& value)
		{
			constraints[index] = value;
		}

		void
			setConstraints(ProblemDefinitions::ConstraintsT cons)
		{
			constraints = cons;
		}

		void
			setOrderedDV(const int index, const int& val)
		{
			ordered_dvs[index] = val;
		}

		void
			setOrderedDVs(ProblemDefinitions::OrderedDVsT _ordered_dvs)
		{
			ordered_dvs = _ordered_dvs;
		}

		void
			setUnorderedDV(const int index, const int& val)
		{
			unordered_dvs[index] = val;
		}

		void
			setUnorderedDVs(ProblemDefinitions::UnorderedDVsT _unordered_dvs)
		{
			unordered_dvs = _unordered_dvs;
		}

		void
			setRealDV(const int index, const double& val)
		{
			real_decision_variables[index] = val;
		}

		void
			setRealDVs(ProblemDefinitions::RealDVsT real_dvs)
		{
			real_decision_variables = real_dvs;
		}

		void setCrowdingScore(double score)
		{
			crowding_score = score;
		}

		void setRank(const int _rank)
		{
			rank = _rank;
		}
		
};

inline std::ostream&
operator<<(std::ostream& os, const IndividualSPtr ind)
{
    os << *ind;
    return os;
}

inline std::ostream&
operator<<(std::ostream& os, const Individual & ind)
{
    os << "[ ";

    for(const int & idv: ind.getUnorderedDVVector())
    {
        os << idv << " ";
    }
    os << "; ";
	for(const int& idv: ind.getOrderedDVVector())
	{
		os << idv << " ";
	}
	os << "; ";
	for (const double & ddv: ind.getRealDVVector())
    {
        os << ddv << " ";
    }
    os << "] -> ( ";
	for (const double & obj: ind.getObjectives())
    {
        os << obj << " ";
    }
    os << "; ";
	for (const double & cnstrnt: ind.getConstraints())
    {
        os << cnstrnt << " ";
    }
    os << ") ";
    os << "Rank: " << ind.getRank() << " CrowdingDist: " << ind.getCrowdingScore();
//    if (ind.mutated) os << "\tMutated";
//    if (ind.crossovered) os << "\tCrossovered" << std::endl;
//    if (ind.child) os << "\tFrom_Child";
//    if (ind.parent) os << "\tFrom_Parent" << std::endl;
    return os;
}






#endif /* Individual_h */
