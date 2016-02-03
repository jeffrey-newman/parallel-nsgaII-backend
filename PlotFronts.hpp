//
//  PlotFronts.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 1/12/2015.
//
//

#ifndef PlotFronts_h
#define PlotFronts_h

#include <sstream>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>

#include <vtkChartXY.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkFloatArray.h>
#include <vtkStringArray.h>
#include <vtkPlotPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTable.h>

#include "colours.hpp"


class PlotFrontVTK
{
    vtkSmartPointer<vtkContextView> view;
    vtkSmartPointer<vtkChartXY> chart;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> window;
    std::vector<vtkSmartPointer<vtkTable> > front_tables;
    std::vector<vtkSmartPointer<vtkFloatArray> > obj_data;
    std::vector<vtkSmartPointer<vtkStringArray> > crowd_dist_labels;
    
    RandomColourGen my_colour_gen;
    
public:
    
    PlotFrontVTK() :
        view(vtkSmartPointer<vtkContextView>::New()),
        chart(vtkSmartPointer<vtkChartXY>::New()),
        renderer(vtkSmartPointer<vtkRenderer>::New())
    {
        
        view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
        view->GetRenderWindow()->SetSize(400, 300);
        view->GetScene()->AddItem(chart);
        chart->SetShowLegend(true);
        
    }
    
    ~PlotFrontVTK()
    {
//        view->GetInteractor()->Initialize();
        view->GetInteractor()->Start();
    }
    
    void
    operator()(std::vector<std::vector<IndividualPtr> > fronts)
    {
        front_tables.clear();
        obj_data.clear();
        crowd_dist_labels.clear();
        chart->ClearPlots();
        
        int num_objectives = fronts.front().front()->numberOfObjectives();
        int number_of_fronts = fronts.size();
        
        for (int i = 0; i < number_of_fronts; ++i)
        {
            int front_size = fronts[i].size();
            
            if (front_size > 0)
            {
                //make new table
                front_tables.push_back(vtkSmartPointer<vtkTable>::New());
                vtkTable * table = front_tables[i];
                
                // Add a column to the table for each objective
                for (int j = 0; j < num_objectives; ++j)
                {
                    obj_data.push_back(vtkSmartPointer<vtkFloatArray>::New());
                    vtkFloatArray * array = obj_data[i * num_objectives + j];
                    std::string name = std::string("Obj ").append(std::to_string(j+1));
                    array->SetName( name.c_str() );
                    table->AddColumn( array );
                }
                
                // Add a column with labels -- of the crowding distance.
                crowd_dist_labels.push_back(vtkSmartPointer<vtkStringArray>::New());
                vtkStringArray * crowd_dist_label = crowd_dist_labels[i];
                crowd_dist_label->SetName("Ind: ");
                table->AddColumn(crowd_dist_label);
                
                table->SetNumberOfRows(fronts[i].size());
                
                for (int j = 0; j < num_objectives; ++j)
                {
                    for (int k = 0; k < front_size; ++k)
                    {
                        table->SetValue(k, j, fronts[i][k]->getObjective(j));
                    }
                }
                
                for (int k = 0; k < front_size; ++k)
                {
                    std::stringstream label;
                    for (int l = 0; l < fronts[i][k]->numberOfRealDecisionVariables(); ++l)
                    {
                        label << "x" << std::to_string(l+1) << ": " << fronts[i][k]->getRealDV(l) << " ";
                    }
//                    label.precision(4);
                    label << "crowd_dist: " << fronts[i][k]->getCrowdingScore();
                    table->SetValue(k, num_objectives, label.str().c_str());
                }
            }
        }
        
//        vtkTable * table = front_tables[0];
//        // Add multiple scatter plots, setting the colors etc
//        vtkPlot *points = chart->AddPlot(vtkChart::POINTS);
//#if VTK_MAJOR_VERSION <= 5
//        points->SetInput(table, 0, 1);
//#else
//        points->SetInputData(table, 0, 1);
//#endif
//        points->SetColor(0, 0, 0, 255);
//        points->SetWidth(1.0);
//        vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CROSS);
        
        for (int i = 0; i < front_tables.size(); ++i)
        {
//            int alpha = (255 / front_tables.size()) * (i + 1);
//            front_tables[i]->Dump();
            // Add multiple scatter plots, setting the colors etc
            vtkPlot *points = chart->AddPlot(vtkChart::POINTS);
#if VTK_MAJOR_VERSION <= 5
            points->SetInput(front_tables[i], 0, 1);
#else
            points->SetInputData(front_tables[i], 0, 1);
#endif
            points->SetLabel(std::string("Front ").append(std::to_string(i+1)));
            rgb colour = my_colour_gen();
            points->SetColor(int(colour.r * 255), int(colour.g * 255), int(colour.b * 255), 255);
            points->SetWidth(1.0);
            vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CIRCLE);
            points->SetIndexedLabels(crowd_dist_labels[i].GetPointer());
            points->SetTooltipLabelFormat("%i from %l (%x, %y)");
            
            
        }
        
        //Finally render the scene
        view->GetRenderWindow()->SetMultiSamples(0);
        view->GetInteractor()->Initialize();
        view->GetInteractor()->Render();
//        view->GetRenderWindow()->Render();

    }
    

};

#endif /* PlotFronts_h */
