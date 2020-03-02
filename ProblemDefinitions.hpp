//
//  ProblemDefinitions.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef ProblemDefinitions_h
#define ProblemDefinitions_h

#include <limits>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>






class ProblemDefinitions
{
public:

	typedef std::vector<double> RealDVsT;
	typedef std::vector<int> UnorderedDVsT;
	typedef std::vector<int> OrderedDVsT;
	typedef std::vector<double> ObjectivesT;
	typedef std::vector<double> ConstraintsT;
	enum MinOrMaxType {
		MINIMISATION,
		MAXIMISATION
	};
	typedef std::vector<MinOrMaxType> ObjectivesDirectionT;
	typedef std::pair<ObjectivesT, ConstraintsT> ObjectivesAndConstraintsT;

	RealDVsT real_lowerbounds;
	RealDVsT real_upperbounds;
	UnorderedDVsT unordered_lowerbounds;
	UnorderedDVsT unordered_upperbounds;
	OrderedDVsT ordered_lowerbounds;
	OrderedDVsT ordered_upperbounds;

	ObjectivesDirectionT minimise_or_maximise;
    unsigned int number_constraints;
    
    ProblemDefinitions()
    : real_lowerbounds(0), real_upperbounds(0), unordered_lowerbounds(0),unordered_upperbounds(0), ordered_lowerbounds(0), ordered_upperbounds(0), minimise_or_maximise(1, MINIMISATION), number_constraints(0)
    {
        
    }
    
    ProblemDefinitions(int _number_of_objectives, int number_of_real_decision_variables, int number_of_unordered_decision_variables = 0, int number_of_ordered_decision_variables = 0, int num_constraints = 0)
    :
    real_lowerbounds(number_of_real_decision_variables, std::numeric_limits<double>::min()),
    real_upperbounds(number_of_real_decision_variables, std::numeric_limits<double>::max()),
    unordered_lowerbounds(number_of_unordered_decision_variables, std::numeric_limits<int>::min()),
		unordered_upperbounds(number_of_unordered_decision_variables, std::numeric_limits<int>::max()),
		ordered_lowerbounds(number_of_ordered_decision_variables, std::numeric_limits<int>::min()),
		ordered_upperbounds(number_of_ordered_decision_variables, std::numeric_limits<int>::max()),
    minimise_or_maximise(_number_of_objectives, MINIMISATION), number_constraints(num_constraints)
    {
        
    }
    
    ProblemDefinitions(RealDVsT & _real_lowerbounds,  RealDVsT & _real_upperbounds,
		UnorderedDVsT& _unordered_lowerbounds, UnorderedDVsT & _unordered_upperbounds,
		OrderedDVsT& _ordered_lowerbounds, OrderedDVsT & _ordered_upperbounds,
		std::vector<MinOrMaxType> & _minimise_or_maximise, int num_constraints)
    : real_lowerbounds(_real_lowerbounds), real_upperbounds(_real_upperbounds),
    unordered_lowerbounds(_unordered_lowerbounds), unordered_upperbounds(_unordered_upperbounds),
		ordered_lowerbounds(_ordered_lowerbounds), ordered_upperbounds(_ordered_upperbounds),
    minimise_or_maximise(_minimise_or_maximise), number_constraints(num_constraints)
    {
        
    }
    
    ProblemDefinitions(int num_real_dvs, double real_min, double real_max, int num_unordered_dvs, int unordered_min, int unordered_max, int num_ordered_dvs, int ordered_min, int ordered_max, int num_objectives, MinOrMaxType min_or_max, int num_constraints)
    :   real_lowerbounds(num_real_dvs, real_min), real_upperbounds(num_real_dvs, real_max),
        unordered_lowerbounds(num_unordered_dvs, unordered_min),
		unordered_upperbounds(num_unordered_dvs, unordered_max),
		ordered_lowerbounds(num_ordered_dvs, ordered_min),
		ordered_upperbounds(num_ordered_dvs, ordered_max),
        minimise_or_maximise(num_objectives, min_or_max),
        number_constraints(num_constraints)
    {
        
    }
    
    ProblemDefinitions(int num_real_dvs, double real_min, double real_max, int num_unordered_dvs, int unordered_min, int unordered_max, int num_ordered_dvs, int ordered_min, int ordered_max, std::vector<MinOrMaxType> & _minimise_or_maximise, int num_constraints)
    :   real_lowerbounds(num_real_dvs, real_min), real_upperbounds(num_real_dvs, real_max),
		unordered_lowerbounds(num_unordered_dvs, unordered_min),
		unordered_upperbounds(num_unordered_dvs, unordered_max),
		ordered_lowerbounds(num_ordered_dvs, ordered_min),
		ordered_upperbounds(num_ordered_dvs, ordered_max),
    minimise_or_maximise(_minimise_or_maximise),
    number_constraints(num_constraints)
    {
        
    }



    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(real_lowerbounds);
            ar & BOOST_SERIALIZATION_NVP(real_upperbounds);
            ar & BOOST_SERIALIZATION_NVP(unordered_lowerbounds);
            ar & BOOST_SERIALIZATION_NVP(unordered_upperbounds);
			ar& BOOST_SERIALIZATION_NVP(ordered_lowerbounds);
			ar& BOOST_SERIALIZATION_NVP(ordered_upperbounds);
            ar & BOOST_SERIALIZATION_NVP(minimise_or_maximise);
            ar & BOOST_SERIALIZATION_NVP(number_constraints);
    }

};

typedef boost::shared_ptr<ProblemDefinitions> ProblemDefinitionsSPtr;

//ProblemDefinitions default_defs;
//
//ProblemDefinitions &
//makeProblemDefinitions(int _number_of_objectives, int number_of_real_decision_variables, int number_of_int_decision_variables = 0)
//{
//    default_defs = ProblemDefinitions(_number_of_objectives, number_of_real_decision_variables, number_of_int_decision_variables);
//    return default_defs;
//}
//
//ProblemDefinitions &
//makeProblemDefinitions(int num_real_dvs, double real_min, double real_max, int num_int_dvs, int int_min, int int_max, int num_objectives, MinOrMaxType min_or_max)
//{
//    default_defs = ProblemDefinitions(num_real_dvs, real_min, real_max, num_int_dvs, int_min, int_max, num_objectives, min_or_max);
//    return default_defs;
//}


#endif /* ProblemDefinitions_h */
