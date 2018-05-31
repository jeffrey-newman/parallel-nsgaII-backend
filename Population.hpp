//
//  Population.h
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 18/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef Population_h
#define Population_h

#include <vector>
#include <fstream>
#include <random>

#include <boost/filesystem.hpp>
#include "Individual.hpp"
#include "Comparator.hpp"

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>


//class Population;
//typedef boost::shared_ptr<Population> PopulationSPtr;
//typedef Population Front;
//typedef PopulationSPtr FrontSPtr;
//typedef std::vector<Front> Fronts;
//typedef boost::shared_ptr<Fronts > FrontsSPtr;
//
//class Population : public std::vector<IndividualSPtr>
//{
//private:
//
//    FrontsSPtr front_sets;
//    bool valid_obj_and_constraints;
//    bool valid_fronts;
//
//public:
//    Population();
//
//    FrontsSPtr
//    getFronts();
//
//    Population(int population_size, ProblemDefinitionsSPtr defs);
//
//    Population(boost::filesystem::path file, ProblemDefinitionsSPtr defs);
//
//    void append(const Population & appending_pop);
//
//    const unsigned long populationSize() const;
//
//    void push_back(IndividualSPtr ind);
//
//    void invalidate();
//    void validateObjAndConstraints();
//
//    friend class boost::serialization::access;
//    template<class Archive>
//    void serialize(Archive & ar, const unsigned int version)
//    {
//        ar & boost::serialization::make_nvp("PopVec", boost::serialization::base_object<std::vector<IndividualSPtr> >(*this) );
////            ar &  BOOST_SERIALIZATION_BASE_OBJECT_NVP(std::vector<Individual>);
//    }
//
//    friend std::ostream& operator<<(std::ostream& os, const Population& pop);
//};
//
//
//
//PopulationSPtr
//restore_population(boost::filesystem::path filename);
//
//
//template<typename RNG>
//PopulationSPtr
//intialisePopulationRandomDVAssignment(int population_size, ProblemDefinitionsSPtr defs, RNG & rng)
//{
//    PopulationSPtr pop(new Population(population_size, defs));
//
//
//    for (int i = 0; i < defs->real_lowerbounds.size(); ++i)
//    {
//        std::uniform_real_distribution<double> uniform(defs->real_lowerbounds[i],defs->real_upperbounds[i]);
//
//        BOOST_FOREACH(IndividualSPtr ind, *pop)
//                    {
//                        ind->setRealDV(i, uniform(rng));
//                    }
//
//    }
//
//    for (int i = 0; i < defs->int_lowerbounds.size(); ++i)
//    {
//        std::uniform_int_distribution<int> uniform(defs->int_lowerbounds[i],defs->int_upperbounds[i]);
//
//        BOOST_FOREACH(IndividualSPtr ind, *pop)
//                    {
//                        ind->setIntDV(i, uniform(rng));
//                    }
//    }
//    return (pop);
//
//}
//
//
//std::ostream& operator<<(std::ostream& os, const PopulationSPtr pop);
//
//std::ostream& operator<<(std::ostream& os, const Population& pop);





///////////////////////////////////////////////////////////////////////////////////////////////////////



class Population;
typedef boost::shared_ptr<Population> PopulationSPtr;
typedef Population Front;
typedef PopulationSPtr FrontSPtr;
typedef std::vector<Front> Fronts;
typedef boost::shared_ptr<Fronts > FrontsSPtr;
inline FrontsSPtr debNonDominatedSort(const Population & population_ref);

class Population : public std::vector<IndividualSPtr>
{
private:

    FrontsSPtr front_sets;
    bool valid_obj_and_constraints;
    bool valid_fronts;

public:
    Population() :
            valid_obj_and_constraints(false), valid_fronts(false)
    {

    }

    void
    calcFronts()
    {
            front_sets = debNonDominatedSort(*this);
            valid_fronts = true;
    }

    FrontsSPtr
    getFronts()
    {
        if (!valid_fronts)
        {
            front_sets = debNonDominatedSort(*this);
            valid_fronts = true;
        }
        return (front_sets);
    }

    Population(int population_size, ProblemDefinitionsSPtr defs):
            valid_obj_and_constraints(false), valid_fronts(false)
    {
        this->reserve(population_size);
        for (int i = 0; i < population_size; ++i)
        {
            std::vector<IndividualSPtr>::push_back(IndividualSPtr(new Individual(defs)));
        }
    }

    Population(boost::filesystem::path file, ProblemDefinitionsSPtr defs):
            valid_obj_and_constraints(false), valid_fronts(false)
    {
        if (!boost::filesystem::exists(file))
        {
            std::cerr << "Cannot construct pop from " << file << "; File does not exisit\n";
        }
        else
        {
            std::ifstream fs(file.string().c_str());
            if (fs.is_open())
            {
                std::string line;

                while (std::getline(fs, line)) {
                    std::vector<IndividualSPtr>::push_back(IndividualSPtr(new Individual(line, defs)));
                }
            }
            else
            {
                std::cerr << "Cannot construct pop from " << file << "; File could not be openned\n";
            }
        }
    }

    void
    append(const Population & appending_pop)
    {
        this->insert(this->end(), appending_pop.begin(), appending_pop.end());
        valid_fronts = false;
    }

    const unsigned long
    populationSize() const
    {
        return (this->size());
    }

    void
    push_back(IndividualSPtr ind)
    {
        std::vector<IndividualSPtr>::push_back(ind);
        valid_fronts = false;
    }

    void
    invalidate()
    {
        valid_obj_and_constraints = false;
        valid_fronts = false;
    }

    void
    validateObjAndConstraints()
    {
        valid_obj_and_constraints = true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::make_nvp("PopVec", boost::serialization::base_object<std::vector<IndividualSPtr> >(*this) );
//            ar &  BOOST_SERIALIZATION_BASE_OBJECT_NVP(std::vector<Individual>);
    }

    friend std::ostream& operator<<(std::ostream& os, const Population& pop);
};



inline PopulationSPtr
restore_population(boost::filesystem::path filename, ProblemDefinitionsSPtr defs)
{
    PopulationSPtr pop(new Population);
    std::string extension = filename.extension().string();
    if (extension == ".txt")
    {
        pop.reset(new Population(filename, defs));
    }
    else if(extension == ".xml")
    {
        // open the archive
        std::ifstream ifs(filename.c_str());
        assert(ifs.good());
        assert(ifs.is_open());
        boost::archive::xml_iarchive ia(ifs);

        // restore the schedule from the archive
        ia >> BOOST_SERIALIZATION_NVP(pop);
    }
    else
    {
        std::cout << "Unrecognised file format for loading population from\n";
    }

    return (pop);
}


template<typename RNG>
PopulationSPtr
intialisePopulationRandomDVAssignment(int population_size, ProblemDefinitionsSPtr defs, RNG & rng)
{
    PopulationSPtr pop(new Population(population_size, defs));


    for (int i = 0; i < defs->real_lowerbounds.size(); ++i)
    {
        std::uniform_real_distribution<double> uniform(defs->real_lowerbounds[i],defs->real_upperbounds[i]);

        BOOST_FOREACH(IndividualSPtr ind, *pop)
                    {
                        ind->setRealDV(i, uniform(rng));
                    }

    }

    for (int i = 0; i < defs->int_lowerbounds.size(); ++i)
    {
        std::uniform_int_distribution<int> uniform(defs->int_lowerbounds[i],defs->int_upperbounds[i]);

        BOOST_FOREACH(IndividualSPtr ind, *pop)
                    {
                        ind->setIntDV(i, uniform(rng));
                    }
    }
    return (pop);

}


inline std::ostream&
operator<<(std::ostream& os, const PopulationSPtr pop)
{
    os << *pop;
    return os;
}

inline std::ostream&
operator<<(std::ostream& os, const Population& pop)
{
    ObjectiveValueCompator obj_comparator(0);
    Population temp_pop(pop);
    std::sort(temp_pop.begin(), temp_pop.end(), obj_comparator);
    BOOST_FOREACH(IndividualSPtr ind, temp_pop)
                {
                    os << ind << "\n";
                }
    return os;
}

inline void
print(const Population& pop, boost::filesystem::path file)
{
    std::string extension = file.extension().string();
    if (extension == ".txt")
    {
        std::ofstream of(file.string().c_str());
        if (of)
        {
            of << pop;
        }
    }
    else if(extension == ".xml")
    {
        std::ofstream ofs(file.string().c_str());
        if (ofs)
        {
            boost::archive::xml_oarchive oa(ofs);
            oa << boost::serialization::make_nvp("Population", pop);
        }

    }
    else
    {
        std::string filename = file.stem().string();
        if (filename == ".") filename = "population";
        boost::filesystem::path file_i_extn = file.parent_path() / (filename + ".txt");
        std::ofstream of(file_i_extn.string().c_str());
        if (of)
        {
            of << pop;
        }
        file_i_extn = file.parent_path() / (filename + ".xml");
        std::ofstream ofs(file_i_extn.string().c_str());
        if (ofs)
        {
            boost::archive::xml_oarchive oa(ofs);
            oa << boost::serialization::make_nvp("Population", pop);
        }
    }
}

inline void
print(const PopulationSPtr pop, boost::filesystem::path file)
{
    print(*pop, file);
}

#include "DebsNondominatedSorting.hpp"

#endif /* Population_h */
