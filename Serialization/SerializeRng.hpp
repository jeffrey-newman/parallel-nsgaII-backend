#ifndef SERIALIZERNG_HPP
#define SERIALIZERNG_HPP

#include <random>
#include <sstream>

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif
#include <boost/config.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/serialization/level.hpp>
BOOST_CLASS_IMPLEMENTATION(std::mt19937, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive, class String, class Traits>
void serialize(Archive& ar, std::mt19937& rng,
                const unsigned int version)
{

     std::string s;
     if(Archive::is_saving::value)
     {
         std::stringstream ss;
         s << rng;
         ss = s.str();
     }

     ar & boost::serialization::make_nvp("rng_string_rep", s);

     if(Archive::is_loading::value)
     {
         std::stringstream ss(s);
         ss >> rng;
     }
}
}}



#endif // SERIALIZERNG_HPP
