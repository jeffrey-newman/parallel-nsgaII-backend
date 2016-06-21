#ifndef HYPERVOLUME_HPP
#define HYPERVOLUME_HPP

#include "../Checkpoint.hpp"
#include "../Population.hpp"
#include "../Individual.hpp"
#include "hv.h"
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/map.hpp>
#include "../Serialization/SerializeBoostPath.hpp"


class Hypervolume : public CheckpointBase
{
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


public:
    Hypervolume(std::vector<double> & _ref_point, boost::filesystem::path log_dir, int _gen_frequency = 1)
        : ref_point(_ref_point), dimension(_ref_point.size()), ref_point_array(new double[dimension]),
         volume(0), dataNumber(0), generation(0), gen_frequency(_gen_frequency)
    {
        for (int var = 0; var < ref_point.size(); ++var)
        {
            ref_point_array[var] = ref_point[var];
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
        dataNumber = population->size();
        assert(population->at(0).numberOfObjectives() == dimension);
        double* data = new double[dataNumber * dimension];
        int i = 0;
        BOOST_FOREACH(Individual & ind, *population)
        {
            BOOST_FOREACH(const double & obj, ind.getObjectives())
            {
                data[i++] = obj;
            }
        }
        volume = fpli_hv(data, dimension, dataNumber, ref_point_array);
        hypervolume_log.insert(std::make_pair(generation, volume));

        if (generation % gen_frequency == 0)
        {
            boost::filesystem::path save_file = log_path / ("hypervolume.xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);
            oa << boost::serialization::make_nvp("hypervolume", *this);
            ofs.close();
        }

        return true;
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





