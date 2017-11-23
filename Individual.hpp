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

class Individual
{
public:
//    bool mutated;
//    bool crossovered;
//    bool parent;
//    bool child;

private:
    
    ProblemDefinitionsSPtr definitions;
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
    : definitions(cpy.definitions), real_decision_variables(cpy.real_decision_variables), int_decision_variables(cpy.int_decision_variables), objectives(cpy.objectives), constraints(cpy.constraints), rank(cpy.rank), crowding_score(cpy.crowding_score)//, mutated(false), crossovered(false), child(false), parent(false)
    {
        
    }
    
    Individual(ProblemDefinitionsSPtr defs)
    : definitions(defs), real_decision_variables(defs->real_lowerbounds.size()), int_decision_variables(defs->int_lowerbounds.size()), objectives(defs->minimise_or_maximise.size()), constraints(defs->number_constraints), rank(std::numeric_limits<int>::max()), crowding_score(std::numeric_limits<double>::min())//, mutated(false), crossovered(false), child(false), parent(false)
    {
        
    }

    Individual(std::string & s, ProblemDefinitionsSPtr defs)
            : definitions(defs), real_decision_variables(defs->real_lowerbounds.size()), int_decision_variables(defs->int_lowerbounds.size()), objectives(defs->minimise_or_maximise.size()), constraints(defs->number_constraints), rank(std::numeric_limits<int>::max()), crowding_score(std::numeric_limits<double>::min())//, mutated(false), crossovered(false), child(false), parent(false)
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
        qi::rule<std::string::iterator,  qi::space_type> ind_parser = qi::lit('[')
                                                        >> int_vec_parser[ph::ref(this->int_decision_variables) = qi::_1]
                                                        >> qi::lit(';')
                                                        >> real_vec_parser[ph::ref(this->real_decision_variables) = qi::_1]
                                                        >> qi::lit(']')
                                                        >> qi::lit("->")
                                                        >> qi::lit('(')
                                                        >> real_vec_parser[ph::ref(this->objectives) = qi::_1]
                                                        >> qi::lit(';')
                                                        >> real_vec_parser[ph::ref(this->constraints) = qi::_1]
                                                        >> qi::lit(')')
                                                        - (qi::lit("Rank:") >> qi::int_[ph::ref(this->rank) = qi::_1])
                                                        - (qi::lit("CrowdingDist:") >> qi::double_[ph::ref(this->crowding_score) = qi::_1]);
        ind_parser.name("individual_parser");
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

        if (this->real_decision_variables.size() < this->numberOfRealDecisionVariables()) this->real_decision_variables.resize(this->numberOfRealDecisionVariables());
        if (this->int_decision_variables.size() < this->numberOfIntDecisionVariables()) this->int_decision_variables.resize(this->numberOfIntDecisionVariables());
        if (this->objectives.size() < this->numberOfObjectives()) this->objectives.resize(this->numberOfObjectives());
        if (this->constraints.size() < this->numberOfConstraints()) this->constraints.resize(this->numberOfConstraints());
    }

    Individual()
    {

    }

    void
    setProblemDefinitions(ProblemDefinitionsSPtr defs)
    {
        definitions.swap(defs);
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

    std::pair<std::vector<double> &, std::vector<double> &>
    getMutableObjectivesAndConstraints()
    {
        return std::make_pair(objectives, constraints);
    };
    
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
        return (definitions->minimise_or_maximise[index]);
        
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
        return (definitions->real_upperbounds[index]);
    }
    
    const double & getRealLowerBound(const int index) const
    {
        return (definitions->real_lowerbounds[index]);
    }
    
    const int getIntUpperBound(const int index) const
    {
        return (definitions->int_upperbounds[index]);
    }
    
    const int getIntLowerBound(const int index) const
    {
        return (definitions->int_lowerbounds[index]);
    }
    
    const unsigned long numberOfRealDecisionVariables() const
    {
        return (definitions->real_lowerbounds.size());
    }
    
    const unsigned long numberOfIntDecisionVariables() const
    {
        return (definitions->int_lowerbounds.size());
    }
    
    const unsigned long numberOfObjectives() const
    {
        return (definitions->minimise_or_maximise.size());
    }
    
    const unsigned long numberOfConstraints() const
    {
        return (definitions->number_constraints);
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
            ar & BOOST_SERIALIZATION_NVP(definitions);
    }

    friend std::ostream& operator<<(std::ostream& os, const Individual& Individual);
};

//typedef Individual * IndividualPtr;
typedef boost::shared_ptr<Individual> IndividualSPtr;

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
//    if (ind.mutated) os << "\tMutated";
//    if (ind.crossovered) os << "\tCrossovered" << std::endl;
//    if (ind.child) os << "\tFrom_Child";
//    if (ind.parent) os << "\tFrom_Parent" << std::endl;
    return os;
}






#endif /* Individual_h */
