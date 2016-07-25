//
//  DebsSBXCrossoverTester.cpp
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 20/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//



// Test to get replication of figure in Deb's article "Self-Adaptation in Real-Parameter Genetic Algorithms with Simulated Binary Crossover" through montecarlo sampling

#ifdef WITH_VTK
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
#endif

#include <stdio.h>
#include <random>
#include <string>

#ifdef WITH_VTK
#include <vtkVersion.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkPlot.h>
#include <vtkTable.h>
#include <vtkIntArray.h>
#include <vtkCharArray.h>
#include <vtkDoubleArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkRenderWindowInteractor.h>
#endif

#ifdef WITH_VTK
#define VTK_CREATE(type, name) \
vtkSmartPointer<type> name = vtkSmartPointer<type>::New()
#endif

#include "../Mutation.hpp"

int main(int argc, char* argv[])
{
    typedef double DecisionVariable_t;
    typedef std::mt19937 RNG_t;
    unsigned int seed = 1;
    double eta = 10;
    double mutation_probability = 1.0;
    RNG_t rng(seed);
    
    DebsPolynomialMutation<RNG_t> mutation_tester(rng, eta, mutation_probability);
    
    int number_dvs = 1; //number of decision variables
    double min_value = -1.0;
    double max_value = 8.0;
    std::vector<DecisionVariable_t> lower_bounds(number_dvs, min_value);
    std::vector<DecisionVariable_t> upper_bounds(number_dvs, max_value);
    std::vector<int> lower_bounds_i;
    std::vector<int> upper_bounds_i;
    std::vector<MinOrMaxType> min_or_max(1, MINIMISATION);
    ProblemDefinitionsSPtr defs(new ProblemDefinitions(lower_bounds, upper_bounds,lower_bounds_i, upper_bounds_i, min_or_max, 0));
    //    std::vector<DecisionVariable_t> parent1_dv_values {2};
    //    std::vector<DecisionVariable_t> parent2_dv_values {5};
    Individual ind1(defs);

    
    std::vector<DecisionVariable_t> results;
    
    int num_samples = 1000000;
    for (int i = 0; i < num_samples; ++i)
    {
        Individual muted1(ind1);
        mutation_tester.mutation_implementation(muted1);
        results.push_back(muted1.getRealDV(0));
        //        std::cout << child1[0] << std::endl;
        //        std::cout << child2[0] << std::endl;
    }
    
#ifdef WITH_VTK
    // plot results.
    // Set up a 2D scene, add an XY chart to it
    VTK_CREATE(vtkContextView, view);
    view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
    view->GetRenderWindow()->SetSize(400, 300);
    VTK_CREATE(vtkChartXY, chart);
    view->GetScene()->AddItem(chart);
    
    // Create a table with some points in it...
    VTK_CREATE(vtkTable, table);
    
    
    VTK_CREATE(vtkIntArray, arrIBin);
    arrIBin->SetName("bin value");
    table->AddColumn(arrIBin);
    
    VTK_CREATE(vtkDoubleArray, arrDBin);
    arrDBin->SetName("decision variable value");
    table->AddColumn(arrDBin);
    
    VTK_CREATE(vtkIntArray, arrFrequency);
    arrFrequency->SetName("Frequency");
    table->AddColumn(arrFrequency);
    
    int num_bins = 50;
    table->SetNumberOfRows(num_bins);
    
    std::vector<int> frequency_count(num_bins);
    std::vector<int> bin_i_names(num_bins);
    std::vector<double> bin_d_names(num_bins);
//    std::vector<std::string> bin_names(num_bins);
    
    for (int i = 0; i < num_samples; ++i)
    {
        frequency_count[int((results[i] - min_value) / (max_value - min_value) * num_bins)]++;
    }
    
    for (int i = 0; i < num_bins; ++i)
    {
        //        bin_names[i] = std::to_string((double(i) / double(num_bins)) * (max_value-min_value) + min_value);
        bin_d_names[i] = ((double(i) / double(num_bins)) * (max_value-min_value) + min_value);
        bin_i_names[i] = i;
    }
    
    for (int i = 0; i < num_bins; i++)
    {
        table->SetValue(i,0,bin_i_names[i]);
        table->SetValue(i,1, bin_d_names[i]);
        table->SetValue(i,2,frequency_count[i]);
    }
    
    // Add multiple line plots, setting the colors etc
    vtkPlot *line = 0;
    
    line = chart->AddPlot(vtkChart::BAR);
#if VTK_MAJOR_VERSION <= 5
    line->SetInput(table, 1, 2);
#else
    line->SetInputData(table, 1, 2);
#endif
    line->SetColor(0, 255, 0, 255);
    
    //Finally render the scene and compare the image to a reference image
    view->GetRenderWindow()->SetMultiSamples(0);
    view->GetInteractor()->Initialize();
    view->GetInteractor()->Start();
#endif
    
}
