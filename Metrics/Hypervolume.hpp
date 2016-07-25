#ifndef HYPERVOLUME_HPP
#define HYPERVOLUME_HPP

#include <fstream>
#include "../Checkpoint.hpp"
#include "../Population.hpp"
#include "../Individual.hpp"
#include "MetricBase.hpp"
#include "hv.h"
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/map.hpp>
#include "../Serialization/SerializeBoostPath.hpp"
#include <boost/archive/xml_oarchive.hpp>



std::vector<double> dummy_point {0,0};

class Hypervolume : public MetricBase
{
public:
    enum Log{OFF, LVL1, LVL2, LVL3};

public:

    enum DoTerminate {TERMINATION, NO_TERMINATION};
private:

    std::vector<double> ref_point;
    std::vector<double> unitize_point;
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
    Log do_log;
    std::reference_wrapper<std::ostream> log_stream;
    bool unitize;
    bool is_minimise_hvol;


public:
    Hypervolume(std::vector<double> & _ref_point,
                boost::filesystem::path log_dir,
                int _gen_frequency = 1,
                DoTerminate _do_terminate = NO_TERMINATION,
                int _max_gen_no_improvement = 100,
                std::vector<double> & _unitize_point = dummy_point, bool _is_minimise_hvol = true)
        : ref_point(_ref_point), dimension(_ref_point.size()),
          ref_point_array(new double[dimension]), volume(0), dataNumber(0), generation(0),
          gen_frequency(_gen_frequency), do_terminate(_do_terminate), gens_no_improvement(0),
          max_gens_no_improvement(_max_gen_no_improvement),
          best_hypervolume(std::numeric_limits<double>::min()), do_log(OFF), log_stream(std::cout), unitize_point(_unitize_point), is_minimise_hvol(_is_minimise_hvol)
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

        if (unitize_point == dummy_point)
        {
            unitize = false;
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

        if (population->getFronts()->size() ==0)
        {
            if (do_log > OFF) log_stream.get() << "Hypervolume: Front has no solutions\n";
            volume = 0;
        }
        else
        {

            if (generation % gen_frequency == 0)
            {

                Front first_front = population->getFronts()->at(0);
                dataNumber = first_front.size();
                assert(population->at(0)->numberOfObjectives() == dimension);
                double* data = new double[dataNumber * dimension];
                int j = 0;
                if (is_minimise_hvol)
                {
                    BOOST_FOREACH(IndividualSPtr ind, first_front)
                    {

                        if (do_log > OFF) log_stream.get() << "Hypervolume: Front point " << j << ": ";
                        for (int i = 0; i < ind->numberOfObjectives(); ++i)
                        {
                            if (do_log > OFF) log_stream.get() << ind->getObjective(i) << "->";
                            if (ind->isMinimiseOrMaximise(i) == MINIMISATION)
                            {
                                data[j] = ind->getObjective(i) - ref_point[i];
                                if (unitize)
                                {
                                    data[j] = data[j] / (unitize_point[i] - ref_point[i]);
                                }
                            }
                            else
                            {
                                data[j] = ref_point[i] - ind->getObjective(i);
                                if (unitize)
                                {
                                    data[j] = data[j] / (ref_point[i] - unitize_point[i] );
                                }
                            }
                            if (do_log > OFF) log_stream.get() << data[j] << "(ref: " << ref_point[i] <<  ")" << ", ";
                            ++j;

                        }
                    }
                }
                else
                {
                    BOOST_FOREACH(IndividualSPtr ind, first_front)
                    {

                        if (do_log > OFF) log_stream.get() << "Hypervolume: Front point " << j << ": ";
                        for (int i = 0; i < ind->numberOfObjectives(); ++i)
                        {
                            if (do_log > OFF) log_stream.get() << ind->getObjective(i) << "->";
                            if (ind->isMinimiseOrMaximise(i) == MAXIMISATION)
                            {
                                data[j] = ind->getObjective(i) - ref_point[i];
                                if (unitize)
                                {
                                    data[j] = data[j] / (unitize_point[i] - ref_point[i]);
                                }
                            }
                            else
                            {
                                data[j] = ref_point[i] - ind->getObjective(i);
                                if (unitize)
                                {
                                    data[j] = data[j] / (ref_point[i] - unitize_point[i] );
                                }
                            }
                            if (do_log > OFF) log_stream.get() << data[j] << "(ref: " << ref_point[i] <<  ")" << ", ";
                            ++j;

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
            }
            if (do_log > OFF) log_stream.get() << "Hypervolume: " << volume << "\n";
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

    void
    log(Log _val = LVL1, std::ostream & _stream = std::cout)
    {
        do_log = _val;
        if (do_log > OFF)
        {
            log_stream = _stream;
        }
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





