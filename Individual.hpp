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

class Individual
{
private:
    
    ProblemDefinitions & definitions;
    std::vector<double> real_decision_variables;
    std::vector<int> int_decision_variables;
    std::vector<double> objectives;
    std::vector<double> constraints;
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
    : definitions(cpy.definitions), real_decision_variables(cpy.real_decision_variables), int_decision_variables(cpy.int_decision_variables), objectives(cpy.objectives), constraints(cpy.constraints), rank(cpy.rank), crowding_score(cpy.crowding_score)
    {
        
    }
    
    Individual(ProblemDefinitions & defs)
    : definitions(defs), real_decision_variables(defs.real_lowerbounds.size()), int_decision_variables(defs.int_lowerbounds.size()), objectives(defs.minimise_or_maximise.size()), constraints(defs.number_constraints), rank(std::numeric_limits<int>::max()), crowding_score(std::numeric_limits<double>::min())
    {
        
    }
    
    Individual &
    operator= ( const Individual & orig)
    {
        this->definitions = orig.definitions;
        this->real_decision_variables = orig.real_decision_variables;
        this->int_decision_variables = orig.int_decision_variables;
        this->objectives = orig.objectives;
        this->constraints = orig.constraints;
        this->rank = orig.rank;
        this->crowding_score = orig.crowding_score;
        return (*this);
    }
    
    const std::vector<double> &
    getRealDVVector() const
    {
        return (real_decision_variables);
    }
    
    const std::vector<int> &
    getIntDVVector() const
    {
        return (int_decision_variables);
    }
    
    const std::vector<double> &
    getObjectives() const
    {
        return objectives;
    }

    void
    setObjectives(std::vector<double> objs)
    {
        objectives = objs;
    }
    
    const std::vector<double> &
    getConstraints() const
    {
        return constraints;
    }

    void
    setConstraints(std::vector<double> cons)
    {
        constraints = cons;
    }
    
    const MinOrMaxType &
    isMinimiseOrMaximise(const int index) const
    {
        return (definitions.minimise_or_maximise[index]);
        
    }
    
    const double &
    getRealDV(const int index) const
    {
        return (real_decision_variables[index]);
    }
    
    void
    setRealDV(const int index, const double & val)
    {
        real_decision_variables[index] = val;
    }
    
    void
    setRealDVs(std::vector<double> real_dvs)
    {
        real_decision_variables = real_dvs;
    }
    
    const int &
    getIntDV(const int index) const
    {
        return (int_decision_variables[index]);
    }
    
    void
    setIntDV(const int index, const int & val)
    {
        int_decision_variables[index] = val;
    }
    
    void
    setIntDVs(std::vector<int> int_dvs)
    {
        int_decision_variables = int_dvs;
    }
    
    const double & getObjective(const int index) const
    {
        return (objectives[index]);
    }
    
    void setObjective(const int index, const double & value)
    {
        objectives[index] = value;
    }
    
    const double & getConstraint(const int index) const
    {
        return (constraints[index]);
    }
    
    void setConstraint(const int index, const double & value)
    {
        constraints[index] = value;
    }
    
    const int getRank(void) const
    {
        return (rank);
    }
    
    void setRank(const int _rank)
    {
        rank = _rank;
    }
    
    const double getCrowdingScore(void) const
    {
        return (crowding_score);
    }
    
    void setCrowdingScore(double score)
    {
        crowding_score = score;
    }
    
    const double & getRealUpperBound(const int index) const
    {
        return (definitions.real_upperbounds[index]);
    }
    
    const double & getRealLowerBound(const int index) const
    {
        return (definitions.real_lowerbounds[index]);
    }
    
    const int getIntUpperBound(const int index) const
    {
        return (definitions.int_upperbounds[index]);
    }
    
    const int getIntLowerBound(const int index) const
    {
        return (definitions.int_lowerbounds[index]);
    }
    
    const unsigned long numberOfRealDecisionVariables() const
    {
        return (real_decision_variables.size());
    }
    
    const unsigned long numberOfIntDecisionVariables() const
    {
        return (int_decision_variables.size());
    }
    
    const unsigned long numberOfObjectives() const
    {
        return (objectives.size());
    }
    
    const unsigned long numberOfConstraints() const
    {
        return (constraints.size());
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(real_decision_variables);
            ar & BOOST_SERIALIZATION_NVP(int_decision_variables);
            ar & BOOST_SERIALIZATION_NVP(objectives);
            ar & BOOST_SERIALIZATION_NVP(constraints);
            ar & BOOST_SERIALIZATION_NVP(rank);
            ar & BOOST_SERIALIZATION_NVP(crowding_score);
    }

    friend std::ostream& operator<<(std::ostream& os, const Individual& Individual);
};

//typedef Individual * IndividualPtr;
typedef boost::shared_ptr<Individual> IndividualSPtr;

std::ostream& operator<<(std::ostream& os, const IndividualSPtr ind)
{
    os << *ind;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Individual & ind)
{
    os << "[ ";

    BOOST_FOREACH(const int & idv, ind.getIntDVVector())
    {
        os << idv << " ";
    }
    os << ";\t";
    BOOST_FOREACH(const double & ddv, ind.getRealDVVector())
    {
        os << ddv << " ";
    }
    os << "]\t->\t(";
    BOOST_FOREACH(const double & obj, ind.getObjectives())
    {
        os << obj << " ";
    }
    os << ";\t";
    BOOST_FOREACH(const double & cnstrnt, ind.getConstraints())
    {
        os << cnstrnt << " ";
    }
    os << ")\t";
    os << "Rank: " << ind.getRank() << "\tCrowdingDist: " << ind.getCrowdingScore();
    return os;
}



#endif /* Individual_h */
