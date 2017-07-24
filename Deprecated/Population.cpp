////
//// Created by a1091793 on 11/4/17.
////
//
////
////  Population.h
////  parallel-nsgaII-backend
////
////  Created by a1091793 on 18/11/2015.
////  Copyright © 2015 University of Adelaide. All rights reserved.
////
//
//#include "Population.hpp"
//
//#include <vector>
//#include <fstream>
//#include <boost/filesystem.hpp>
//#include "Individual.hpp"
//
//#include <algorithm>
//#include <random>
//#include <boost/shared_ptr.hpp>
//#include <boost/foreach.hpp>
//#include <boost/serialization/nvp.hpp>
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/shared_ptr.hpp>
//#include <boost/archive/xml_oarchive.hpp>
//#include <boost/archive/xml_iarchive.hpp>
//#include "DebsNondominatedSorting.hpp"
//#include "Comparator.hpp"
//
//
//Population::Population() :
//        valid_obj_and_constraints(false), valid_fronts(false)
//{
//
//}
//
//FrontsSPtr
//Population::getFronts()
//{
//    if (!valid_fronts)
//    {
//        front_sets = debNonDominatedSort(*this);
//        valid_fronts = true;
//    }
//    return (front_sets);
//}
//
//Population::Population(int population_size, ProblemDefinitionsSPtr defs):
//        valid_obj_and_constraints(false), valid_fronts(false)
//{
//    this->reserve(population_size);
//    for (int i = 0; i < population_size; ++i)
//    {
//        std::vector<IndividualSPtr>::push_back(IndividualSPtr(new Individual(defs)));
//    }
//}
//
//Population::Population(boost::filesystem::path file, ProblemDefinitionsSPtr defs):
//        valid_obj_and_constraints(false), valid_fronts(false)
//{
//    if (!boost::filesystem::exists(file))
//    {
//        std::cerr << "Cannot construct pop from " << file << "; File does not exisit\n";
//    }
//    else
//    {
//        std::ifstream fs(file.string().c_str());
//        if (fs.is_open())
//        {
//            std::string line;
//
//            while (std::getline(fs, line)) {
//                std::vector<IndividualSPtr>::push_back(IndividualSPtr(new Individual(line, defs)));
//            }
//        }
//        else
//        {
//            std::cerr << "Cannot construct pop from " << file << "; File could not be openned\n";
//        }
//    }
//}
//
//void
//Population::append(const Population & appending_pop)
//{
//    this->insert(this->end(), appending_pop.begin(), appending_pop.end());
//    valid_fronts = false;
//}
//
//const unsigned long
//Population::populationSize() const
//{
//    return (this->size());
//}
//
//void
//Population::push_back(IndividualSPtr ind)
//{
//    std::vector<IndividualSPtr>::push_back(ind);
//    valid_fronts = false;
//}
//
//void
//Population::invalidate()
//{
//    valid_obj_and_constraints = false;
//    valid_fronts = false;
//}
//
//void
//Population::validateObjAndConstraints()
//{
//    valid_obj_and_constraints = true;
//}
//
//std::ostream& operator<<(std::ostream& os, const Population& pop)
//{
//    ObjectiveValueCompator obj_comparator(0);
//    Population temp_pop(pop);
//    std::sort(temp_pop.begin(), temp_pop.end(), obj_comparator);
//    BOOST_FOREACH(IndividualSPtr ind, temp_pop)
//                {
//                    os << ind << "\n";
//                }
//    return os;
//}
//
//std::ostream& operator<<(std::ostream& os, const PopulationSPtr pop)
//{
//    os << *pop;
//    return os;
//}
//
//PopulationSPtr
//restore_population(boost::filesystem::path filename)
//{
//    // open the archive
//    std::ifstream ifs(filename.c_str());
//    assert(ifs.good());
//    assert(ifs.is_open());
//    boost::archive::xml_iarchive ia(ifs);
//
//    PopulationSPtr pop(new Population);
//    // restore the schedule from the archive
//    ia >> BOOST_SERIALIZATION_NVP(pop);
//    return (pop);
//}
//
