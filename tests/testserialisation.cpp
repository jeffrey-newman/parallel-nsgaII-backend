#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include "../Individual.hpp"
#include "../SampleProblems/TestFunctions.hpp"

//#include <boost/config.hpp>
//#if defined(BOOST_NO_STDC_NAMESPACE)
//namespace std{
//    using ::remove;
//}
//#endif

#include <boost/archive/tmpdir.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>



int main(int argc, char *argv[])
{
    SUM demo_problem;
    Individual test_ind(demo_problem.getProblemDefinitions());
    for (int var = 0; var < test_ind.numberOfConstraints(); ++var)
    {
        test_ind.setConstraint(var, var);
    }
    for (int var = 0; var < test_ind.numberOfIntDecisionVariables(); ++var)
    {
        test_ind.setIntDV(var, var);
    }
    for (int var = 0; var < test_ind.numberOfRealDecisionVariables(); ++var)
    {
        test_ind.setRealDV(var, double(var) / test_ind.numberOfRealDecisionVariables());
    }
    for (int var = 0; var < test_ind.numberOfObjectives(); ++var)
    {
        test_ind.setObjective(var, var);
    }


    std::string filename(boost::archive::tmpdir());
    filename += "/demo.xml";
    std::ofstream ofs(filename);
    assert(ofs.good());
    boost::archive::xml_oarchive oa(ofs);
    oa << BOOST_SERIALIZATION_NVP(test_ind);


    return 0;
}
