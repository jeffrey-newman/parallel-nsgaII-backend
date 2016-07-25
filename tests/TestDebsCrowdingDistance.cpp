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
#include "../Checkpoints/PlotFronts.hpp"

int main(int argc, char * argv[])
{
    ProblemDefinitionsSPtr dummy_defs(new ProblemDefinitions(2,2,0,0));
    PopulationSPtr myPop(new Population);
    
    IndividualSPtr ind1(new Individual(dummy_defs));
    std::vector<double> dvs1 {1.0, 0.0};
    ind1->setRealDVs(dvs1);
    ind1->setObjectives(std::vector<double>{1.0, 0.0});
    myPop->push_back(ind1);
    
    IndividualSPtr ind2(new Individual(dummy_defs));
    ind2->setRealDVs(std::vector<double>{0.7, 0.15});
    ind2->setObjectives(std::vector<double>{0.49, 0.0225});
    myPop->push_back(ind2);
    
    IndividualSPtr ind3(new Individual(dummy_defs));
    ind3->setRealDVs(std::vector<double>{0.5, 0.26});
    ind3->setObjectives(std::vector<double>{0.25, 0.0676});
    myPop->push_back(ind3);
    
    IndividualSPtr ind4(new Individual(dummy_defs));
    ind4->setRealDVs(std::vector<double>{0.44, 0.3});
    ind4->setObjectives(std::vector<double>{0.1936, 0.09});
    myPop->push_back(ind4);
    
    IndividualSPtr ind5(new Individual(dummy_defs));
    ind5->setRealDVs(std::vector<double>{0.4, 0.37});
    ind5->setObjectives(std::vector<double>{0.16, 0.1369});
    myPop->push_back(ind5);
    
    IndividualSPtr ind6(new Individual(dummy_defs));
    ind6->setRealDVs(std::vector<double>{0.2, 0.7});
    ind6->setObjectives(std::vector<double>{0.04, 0.49});
    myPop->push_back(ind6);

    IndividualSPtr ind7(new Individual(dummy_defs));
    ind7->setRealDVs(std::vector<double>{0.0, 1.0});
    ind7->setObjectives(std::vector<double>{0, 1});
    myPop->push_back(ind7);

    IndividualSPtr ind1_(new Individual(dummy_defs));
    ind1_->setRealDVs(std::vector<double> {1.0, 0.0});
    ind1_->setObjectives(std::vector<double>{1.2, 0.2});
    myPop->push_back(ind1_);

    IndividualSPtr ind2_(new Individual(dummy_defs));
    ind2_->setRealDVs(std::vector<double>{0.7, 0.15});
    ind2_->setObjectives(std::vector<double>{0.69, 0.2225});
    myPop->push_back(ind2_);

    IndividualSPtr ind3_(new Individual(dummy_defs));
    ind3_->setRealDVs(std::vector<double>{0.5, 0.26});
    ind3_->setObjectives(std::vector<double>{0.45, 0.2676});
    myPop->push_back(ind3_);

    IndividualSPtr ind4_(new Individual(dummy_defs));
    ind4_->setRealDVs(std::vector<double>{0.44, 0.3});
    ind4_->setObjectives(std::vector<double>{0.3936, 0.29});
    myPop->push_back(ind4_);

    IndividualSPtr ind5_(new Individual(dummy_defs));
    ind5_->setRealDVs(std::vector<double>{0.4, 0.37});
    ind5_->setObjectives(std::vector<double>{0.36, 0.3369});
    myPop->push_back(ind5_);

    IndividualSPtr ind6_(new Individual(dummy_defs));
    ind6_->setRealDVs(std::vector<double>{0.2, 0.7});
    ind6_->setObjectives(std::vector<double>{0.24, 0.69});
    myPop->push_back(ind6_);

    IndividualSPtr ind7_(new Individual(dummy_defs));
    ind7_->setRealDVs(std::vector<double>{0.0, 1.0});
    ind7_->setObjectives(std::vector<double>{0.2, 1.2});
    myPop->push_back(ind7_);
    
    std::vector< std::vector<std::pair<IndividualSPtr, double > > > values1 = DebsCrowdingDistance::calculate(myPop);
    FrontsSPtr values2 = myPop->getFronts();
//    typedef std::pair<IndividualSPtr, double> IndDistPair;
//    FrontsSPtr values2(new Fronts(values1.size()));
//    boost::shared_ptr<std::vector<std::vector<IndividualSPtr> > > values2(new std::vector<std::vector<IndividualSPtr> >(values1.size()));
//    for (int i = 0; i < values2->size(); ++i)
//    {
//        BOOST_FOREACH(IndDistPair ind, values1[i])
//        {
//            (*values2)[i].push_back(ind.first);
//        }
//    }
    
#ifdef WITH_VTK
    PlotFrontVTK plot;
    plot(myPop);
#endif

}
