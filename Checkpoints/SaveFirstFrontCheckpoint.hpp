#ifndef SAVEFIRSTFRONTCHECKPOINT_HPP
#define SAVEFIRSTFRONTCHECKPOINT_HPP


#include "../Checkpoint.hpp"
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <iostream>
#include <fstream>


class SaveFirstFrontCheckpoint : public CheckpointBase
{

    int gen_frequency;
    int gen_number;
    boost::filesystem::path save_path;
    bool txt_rep;

public:
    SaveFirstFrontCheckpoint(int _gen_frequency, boost::filesystem::path _save_path, bool _txt_rep = true)
    : gen_frequency(_gen_frequency), gen_number(0), save_path(_save_path), txt_rep(_txt_rep)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number % gen_frequency == 0)
        {
            boost::filesystem::path save_file = save_path / ("first_front_gen" + std::to_string(gen_number) + ".xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);

            Population & first_front = population->getFronts()->at(0);
            oa << BOOST_SERIALIZATION_NVP(first_front);

            if (txt_rep)
            {
                boost::filesystem::path save_file2 = save_path / ("first_front_gen" + std::to_string(gen_number) +  ".txt");
                std::ofstream ofs2(save_file2.c_str());
                assert(ofs2.good());
                ofs2 << population->getFronts()->at(0);
            }
        }
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(gen_frequency);
            ar & BOOST_SERIALIZATION_NVP(gen_number);
            ar & BOOST_SERIALIZATION_NVP(save_path);
    }
};




#endif // SAVEFIRSTFRONTCHECKPOINT_HPP
