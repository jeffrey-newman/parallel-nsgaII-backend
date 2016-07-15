#ifndef HYPERVOLUME_HPP
#define HYPERVOLUME_HPP

#include "../Checkpoint.hpp"
#include "../Population.hpp"
#include "../Individual.hpp"
#include "MetricBase.hpp"
#include "hv.h"
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/map.hpp>
#include "../Serialization/SerializeBoostPath.hpp"




class Hypervolume : public MetricBase
{
public:

    enum DoTerminate {TERMINATION, NO_TERMINATION};
private:
    std::vector<double> ref_point;
    int dimension;
    double* ref_point_array;
    double volume;
    int dataNumber;
    std::map<int, double> hypervolume_log;
    boost::filesystem::path log_path;
    int generation;
    int gen_frequency;
    DoTerminate do_terminate;
    int gens_no_improvement;
    int max_gens_no_improvement;
    double best_hypervolume;


public:
    Hypervolume(std::vector<double> & _ref_point,
                boost::filesystem::path log_dir,
                int _gen_frequency = 1,
                DoTerminate _do_terminate = NO_TERMINATION,
                int _max_gen_no_improvement = 100)
        : ref_point(_ref_point), dimension(_ref_point.size()),
          ref_point_array(new double[dimension]), volume(0), dataNumber(0), generation(0),
          gen_frequency(_gen_frequency), do_terminate(_do_terminate), gens_no_improvement(0),
          max_gens_no_improvement(_max_gen_no_improvement),
          best_hypervolume(std::numeric_limits<double>::min())
    {
        for (int var = 0; var < ref_point.size(); ++var)
        {
            ref_point_array[var] = 0.0;
        }

        if (max_gens_no_improvement / gen_frequency < 3)
        {
            std::cout << "Warning: In Hypervolume checkpoint, termination "
                         "results based on less than three calculations of the hypervolume"
                      << std::endl;
        }
    }

    ~Hypervolume()
    {
        delete[] ref_point_array;
        boost::filesystem::path save_file = log_path / ("hypervolume.xml");
        std::ofstream ofs(save_file.c_str());
        assert(ofs.good());
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp("hypervolume", *this);
        ofs.close();
    }

    bool operator() (PopulationSPtr population)
    {
        // initialize volume
        ++generation;


        if (generation % gen_frequency == 0)
        {
            Front first_front = population->getFronts()->at(0);
            dataNumber = first_front.size();
            assert(population->at(0)->numberOfObjectives() == dimension);
            double* data = new double[dataNumber * dimension];
            int j = 0;
            BOOST_FOREACH(IndividualSPtr ind, first_front)
            {
                for (int i = 0; i < ind->numberOfObjectives(); ++i)
                {
                    if (ind->isMinimiseOrMaximise(i) == MINIMISATION)
                    {
                        data[j++] = ind->getObjective(i) - ref_point[i];
                    }
                    else
                    {
                        data[j++] = ref_point[i] - ind->getObjective(i);
                    }

                }
            }

//            std::cout << "Hypervolume transformation: \n";
//            for (int j = 0; j < (dataNumber * dimension); j = j + dimension)
//            {
//                for (int k = 0; k < dimension; ++k)
//                {
//                    std::cout << data[j+k] << " ";
//                }
//                std::cout << "\n";
//            }
//            std::cout << std::flush;

            volume = fpli_hv(data, dimension, dataNumber, ref_point_array);
            hypervolume_log.insert(std::make_pair(generation, volume));

            boost::filesystem::path save_file = log_path / ("hypervolume.xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);
            oa << boost::serialization::make_nvp("hypervolume", *this);
            ofs.close();
        }

        if (do_terminate == TERMINATION)
        {
            if (volume > best_hypervolume)
            {
                best_hypervolume = volume;
                gens_no_improvement = 0;
            }
            if (volume < best_hypervolume) ++gens_no_improvement;
            if (gens_no_improvement > max_gens_no_improvement)
            {
                std::cout << "Terminating. Hypervolume improvement stalled. No improvement in " << gens_no_improvement << " generations." << std::endl;
                return false;
            }
        }

        return true;
    }

    double
    getVal()
    {
        return volume;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::make_nvp("ref_point", ref_point);
        ar & boost::serialization::make_nvp("hypervolumes", hypervolume_log);
        //            ar & BOOST_SERIALIZATION_NVP(log_path);
    }
};

#endif // HYPERVOLUME_HPP





