//
//  TestDebsCrowdingDistance.cpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 4/12/2015.
//
//

#include <stdio.h>
#include "../DebsCrowdingDistance.hpp"
#include "../Types.hpp"
#include "../PlotFronts.hpp"

int main(int argc, char * argv[])
{
    ProblemDefinitions dummy_defs;
    PopulationSPtr myPop(new Population);
    
    Individual ind1(dummy_defs);
    std::vector<double> dvs1 {1.0, 0.0};
    ind1.setRealDVs(dvs1);
    ind1.setObjectives(std::vector<double>{1.0, 0.0});
    myPop->push_back(ind1);
    
    Individual ind2(dummy_defs);
    ind2.setRealDVs(std::vector<double>{0.7, 0.15});
    ind2.setObjectives(std::vector<double>{0.49, 0.0225});
    myPop->push_back(ind2);
    
    Individual ind3(dummy_defs);
    ind3.setRealDVs(std::vector<double>{0.5, 0.26});
    ind3.setObjectives(std::vector<double>{0.25, 0.0676});
    myPop->push_back(ind3);
    
    Individual ind4(dummy_defs);
    ind4.setRealDVs(std::vector<double>{0.44, 0.3});
    ind4.setObjectives(std::vector<double>{0.1936, 0.09});
    myPop->push_back(ind4);
    
    Individual ind5(dummy_defs);
    ind5.setRealDVs(std::vector<double>{0.4, 0.37});
    ind5.setObjectives(std::vector<double>{0.16, 0.1369});
    myPop->push_back(ind5);
    
    Individual ind6(dummy_defs);
    ind6.setRealDVs(std::vector<double>{0.2, 0.7});
    ind6.setObjectives(std::vector<double>{0.04, 0.49});
    myPop->push_back(ind6);

    Individual ind7(dummy_defs);
    ind7.setRealDVs(std::vector<double>{0.0, 1.0});
    ind7.setObjectives(std::vector<double>{0, 1});
    myPop->push_back(ind7);
    
    std::vector< std::vector<std::pair<IndividualPtr, double > > > values1 = DebsCrowdingDistance::calculate(myPop);
    
    typedef std::pair<IndividualPtr, double> IndDistPair;
    std::vector<std::vector<IndividualPtr> > values2(values1.size());
    for (int i = 0; i < values2.size(); ++i)
    {
        BOOST_FOREACH(IndDistPair ind, values1[i])
        {
            values2[i].push_back(ind.first);
        }
    }
    
    PlotFrontVTK plot;
    plot(values2);

}