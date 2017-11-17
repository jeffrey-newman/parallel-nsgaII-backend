#ifndef HYPERVOLUME_HPP
#define HYPERVOLUME_HPP

#include <fstream>
#include <cmath>
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
#include <boost/date_time.hpp>
#include <boost/optional.hpp>
#include <boost/timer/timer.hpp>


std::vector<double> dummy_point {0,0};

class Hypervolume : public MetricBase
{
public:
    enum Log{OFF, LVL1, LVL2, LVL3};
    enum Better{MAXIMISE, MINIMISE};

public:

    enum DoTerminate {TERMINATION, NO_TERMINATION};
private:

    std::vector<double> ref_point;
    std::vector<double> unitize_point;
    std::vector<double> axis_lengths;
    int dimension;
    double* ref_point_array;
    double volume;
    int dataNumber;
    std::map<int, double> hypervolume_log;
    boost::filesystem::path save_dir;
    int generation;
    int gen_frequency;
    DoTerminate do_terminate;
    int gens_no_improvement;
    int max_gens_no_improvement;
    double best_hypervolume;
    Log do_log;
    std::ofstream logging_file;
    std::reference_wrapper<std::ostream> log_stream;
    bool unitize;
    Better is_maximize_hvol;
    bool calc_ref_first_gen;


public:
    Hypervolume(int _gen_frequency = 1,
                boost::optional<boost::filesystem::path> _save_dir = boost::none,
                DoTerminate _do_terminate = NO_TERMINATION,
                int _max_gen_no_improvement = 100,
                boost::optional<std::vector<double> > _ref_point = boost::none,
                boost::optional<std::vector<double> >  _unitize_point = boost::none,
                Better _is_maximise_hvol = MAXIMISE
                )
    :
    ref_point_array(new double[2]),
    volume(0),
    dataNumber(0),
    generation(0),
    gen_frequency(_gen_frequency),
    do_terminate(_do_terminate),
    gens_no_improvement(0),
    max_gens_no_improvement(_max_gen_no_improvement),
    log_stream(std::cout),
    is_maximize_hvol(_is_maximise_hvol)
    {
        
        
        if (_ref_point)
        {
            calc_ref_first_gen = false;
            ref_point = _ref_point.get();
            axis_lengths = std::vector<double>(ref_point.size(), 0.0);
            dimension = ref_point.size();
            delete[] ref_point_array;
            ref_point_array = new double[dimension];
            for (int var = 0; var < ref_point.size(); ++var)
            {
                ref_point_array[var] = ref_point[var];
            }
            
        }
        else
        {
            calc_ref_first_gen = true;
        }
        
        if (_unitize_point)
        {
            unitize = true;
            unitize_point = _unitize_point.get();
            if (_ref_point)
            {
                for (int i = 0; i < ref_point.size(); ++i)
                {
                    axis_lengths[i] = std::abs(unitize_point[i] - ref_point[i]);
                }
            }
        }
        else
        {
            unitize = false;
        }
        
        
        if (_is_maximise_hvol == MAXIMISE)
        {
            best_hypervolume = std::numeric_limits<double>::min();
        }
        else
        {
            best_hypervolume = std::numeric_limits<double>::max();
        }
        
        if(_save_dir)
        {
            do_log = LVL3;
            save_dir = _save_dir.get();
            std::string filename = "logHypervolume_" + boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time()) + ".log";
            boost::filesystem::path logfile =  save_dir / filename;

            logging_file.open(logfile.c_str(), std::ios_base::app);
            if (!logging_file.is_open())
            {
                do_log = OFF;
                std::cout << "attempt to log in hypervolume evaluator failed\n";
            }
            else
            {
                log_stream = logging_file;
            }
            
        }
        else
        {
            do_log = OFF;
        }

//        if (do_log > OFF) log_stream.get() << "Hello Hypervolume\n";
        
        
        if (max_gens_no_improvement / gen_frequency < 3)
        {
            std::cout << "Warning: In Hypervolume checkpoint, termination "
            "results based on less than three calculations of the hypervolume"
            << std::endl;
        }
        
        
    }
    
    
//    Hypervolume(std::vector<double> & _ref_point,
//                boost::filesystem::path log_dir,
//                int _gen_frequency = 1,
//                DoTerminate _do_terminate = NO_TERMINATION,
//                int _max_gen_no_improvement = 100,
//                
//                std::vector<double> & _unitize_point = dummy_point)
//        : ref_point(_ref_point),
//          unitize_point(_unitize_point),
//          axis_lengths(_ref_point.size(), 0.0),
//          dimension(_ref_point.size()),
//          ref_point_array(new double[dimension]),
//          volume(0),
//          dataNumber(0),
//          generation(0),
//          gen_frequency(_gen_frequency),
//          do_terminate(_do_terminate),
//          gens_no_improvement(0),
//          max_gens_no_improvement(_max_gen_no_improvement),
//          best_hypervolume(std::numeric_limits<double>::min()),
//          do_log(OFF),
//          log_stream(std::cout),
//          unitize(true),
//          is_maximize_hvol(_is_maximise_hvol)
//    {
//        for (int var = 0; var < ref_point.size(); ++var)
//        {
//            ref_point_array[var] = 0.0;
//        }
//
//        if (max_gens_no_improvement / gen_frequency < 3)
//        {
//            std::cout << "Warning: In Hypervolume checkpoint, termination "
//                         "results based on less than three calculations of the hypervolume"
//                      << std::endl;
//        }
//
//        if (unitize_point == dummy_point)
//        {
//            unitize = false;
//        }
//        else
//        {
//            for (int i = 0; i < ref_point.size(); ++i)
//            {
//                axis_lengths[i] = std::abs(unitize_point[i] - ref_point[i]);
//            }
//        }
//    }

    ~Hypervolume()
    {
        delete[] ref_point_array;
        if (do_log > OFF)
        {
            boost::filesystem::path save_file = save_dir / ("hypervolume.xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);
            oa << boost::serialization::make_nvp("hypervolume", *this);
            ofs.close();
        }
    }

    bool operator() (PopulationSPtr population)
    {
        // initialize volume
        
        if (calc_ref_first_gen)
        {
            if (population->getFronts()->size() ==0)
            {
                if (do_log > OFF) log_stream.get() << "Hypervolume: Front has no solutions\n";
                volume = 0;
            }
            else
            {
                Front first_front = population->getFronts()->at(0);
                dataNumber = first_front.size();
                dimension = population->at(0)->numberOfObjectives();
                
                ref_point.resize(dimension,0);
//                double* data = new double[dataNumber * dimension];
//                int j = 0;
                
                for (int i = 0; i < dimension; ++i)
                {
                    // if maximise objective, find minimum value of obj
                    if (population->at(0)->isMinimiseOrMaximise(i) == MAXIMISATION)
                    {
                        ref_point[i] = std::numeric_limits<double>::max();
                    }
                    
                    // in minimise objecttive, find max value of obj
                    if (population->at(0)->isMinimiseOrMaximise(i) == MINIMISATION)
                    {
                        ref_point[i] = std::numeric_limits<double>::min();
                    }
                }
                
                BOOST_FOREACH(IndividualSPtr ind, first_front)
                {
                    for (int i = 0; i < dimension; ++i)
                    {
                        // if maximise objective, find minimum value of obj
                        if ((ind->isMinimiseOrMaximise(i) == MAXIMISATION) && (ind->getObjective(i) < ref_point[i]))
                        {
                            ref_point[i] = ind->getObjective(i);
                        }
                        
                        // in minimise objecttive, find max value of obj
                        if ((ind->isMinimiseOrMaximise(i) == MINIMISATION)  && (ind->getObjective(i) > ref_point[i]))
                        {
                            ref_point[i] = ind->getObjective(i);
                        }
                    }
                }
                
                calc_ref_first_gen = false;
                axis_lengths = std::vector<double>(ref_point.size(), 0.0);
                delete[] ref_point_array;
                ref_point_array = new double[dimension];
                for (int var = 0; var < ref_point.size(); ++var)
                {
                    ref_point_array[var] = 0;
                }
            }
        }
        
        
        ++generation;


        // Fonseca's Hypervolume code assumes we are maximising the hypervolume, for minimisation objectives.

        if (generation % gen_frequency == 0)
        {
            if (population->getFronts()->size() ==0)
            {
                if (do_log > OFF) log_stream.get() << "Hypervolume: Front has no solutions\n";
                volume = 0;
            }
            else
            {


                Front first_front = population->getFronts()->at(0);
                dataNumber = first_front.size();
                assert(population->at(0)->numberOfObjectives() == dimension);
                double* data = new double[dataNumber * dimension];
                int j = 0;

                BOOST_FOREACH(IndividualSPtr ind, first_front)
                {

//                    if (do_log > OFF) log_stream.get() << "Hello Hypervolume again\n";
                    if (do_log > OFF) log_stream.get() << "Hypervolume: Front point " << j << ": ";
                    for (int i = 0; i < ind->numberOfObjectives(); ++i)
                    {
                        if (do_log > OFF) log_stream.get() << ind->getObjective(i) << " -> ";
                        if (ind->isMinimiseOrMaximise(i) == MINIMISATION)
                        {
                            data[j] = ind->getObjective(i) - ref_point[i];
                            if (unitize)
                            {
                                data[j] = data[j] / axis_lengths[i];
                            }
                        }
                        else
                        {
                            data[j] = ref_point[i] - ind->getObjective(i);
                            if (unitize)
                            {
                                data[j] = data[j] / axis_lengths[i];
                            }
                        }
                        if (do_log > OFF) log_stream.get() << data[j] << " (ref: " << ref_point[i] <<  " )" << ", ";
                        ++j;
                    }
                    if (do_log > OFF) log_stream.get() <<  "\n";
                }

                if (do_log > OFF) log_stream.get() <<  std::endl;

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

            if (do_log > OFF)
            {
                boost::filesystem::path save_file = save_dir / ("hypervolume.xml");
                std::ofstream ofs(save_file.c_str());
                assert(ofs.good());
                boost::archive::xml_oarchive oa(ofs);
                oa << boost::serialization::make_nvp("hypervolume", *this);
                ofs.close();
            }

            if (do_terminate == TERMINATION)
            {
                if (is_maximize_hvol == MAXIMISE)
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
                else
                {
                    if (volume < best_hypervolume)
                    {
                        best_hypervolume = volume;
                        gens_no_improvement = 0;
                    }
                    if (volume > best_hypervolume) ++gens_no_improvement;
                    if (gens_no_improvement > max_gens_no_improvement)
                    {
                        std::cout << "Terminating. Hypervolume improvement stalled. No improvement in " << gens_no_improvement << " generations." << std::endl;
                        return false;
                    }
                }

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





