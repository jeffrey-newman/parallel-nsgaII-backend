#ifndef SERIALISECHECKPOINT_HPP
#define SERIALISECHECKPOINT_HPP

#include "../Checkpoint.hpp"
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/export.hpp>
#include "../NSGAII.hpp"





template <typename SerializableObject>
class SerialiseCheckpoint : public CheckpointBase
{
    template<class Archive> friend void boost::serialization::save_construct_data(
            Archive & ar, const SerialiseCheckpoint<NSGAII<std::mt19937> > * t, const unsigned int file_version);
    template<class Archive> friend void boost::serialization::load_construct_data(
            Archive & ar, const SerialiseCheckpoint<NSGAII<std::mt19937> > * t, const unsigned int file_version);

    int gen_frequency;
    int gen_number;
    SerializableObject & obj_2_serialize;
    boost::filesystem::path save_path;

public:
    SerialiseCheckpoint(int _gen_frequency, SerializableObject & _obj_2_serialize, boost::filesystem::path _save_path)
    : gen_frequency(_gen_frequency), gen_number(0), obj_2_serialize(_obj_2_serialize), save_path(_save_path)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number % gen_frequency == 0)
        {
            boost::filesystem::path save_file = save_path / ("nsgaii_gen" + std::to_string(gen_number) + ".xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);
            oa << BOOST_SERIALIZATION_NVP(obj_2_serialize);
        }
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(gen_frequency);
            ar & BOOST_SERIALIZATION_NVP(gen_number);
    }
};

//namespace boost { namespace serialization {
//template<class Archive>
//inline void save_construct_data(
//    Archive & ar, const SerialiseCheckpoint<NSGAII<std::mt19937> > * t, const unsigned int file_version
//){
//    // save data required to construct instance
//    ar << t->gen_frequency;
//    ar << t->gen_number;
//    ar << t->obj_2_serialize;
//    ar << t->save_path;
//}

//template<class Archive>
//inline void load_construct_data(
//    Archive & ar, SerialiseCheckpoint<NSGAII<std::mt19937> > * t, const unsigned int file_version
//){
//    // retrieve data from archive required to construct new instance
//    int t_gen_frequency;
//    int t_gen_number;
//    NSGAII<std::mt19937> & obj_2_serialize;
//    boost::filesystem::path save_path;
//    ar >> t_gen_frequency;
//    ar >> t_gen_number;
//    ar >> obj_2_serialize;
//    ar >> save_path
//    // invoke inplace constructor to initialize instance of my_class
//    ::new(t)SerialiseCheckpoint<NSGAII<std::mt19937> >(t_gen_frequency, obj_2_serialize, save_path);
//    t->gen_number=t_gen_number;
//}
//}} // namespace ...


//BOOST_CLASS_EXPORT_KEY(SerialiseCheckpoint<NSGAII<std::mt19937> >);
//BOOST_CLASS_EXPORT_IMPLEMENT(SerialiseCheckpoint<NSGAII<std::mt19937> >);

#endif // SERIALISECHECKPOINT_HPP
