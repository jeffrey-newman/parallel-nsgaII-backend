//
//  PlotFronts.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 1/12/2015.
//
//

#ifndef PlotFronts_h
#define PlotFronts_h




#ifdef WITH_VTK
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

#include "../Population.hpp"
#include "../Checkpoint.hpp"



class PlotFrontVTK : public CheckpointBase
{
    vtkSmartPointer<vtkContextView> view_objs;
    vtkSmartPointer<vtkChartXY> chart_objs;
    vtkSmartPointer<vtkRenderer> renderer_objs;
    vtkSmartPointer<vtkRenderWindow> window_objs;
    std::vector<vtkSmartPointer<vtkTable> > front_tables_objs;
    std::vector<vtkSmartPointer<vtkFloatArray> > obj_data_objs;
    std::vector<vtkSmartPointer<vtkStringArray> > crowd_dist_labels_objs;
    
    RandomColourGen my_colour_gen_objs;

    vtkSmartPointer<vtkContextView> view_decs;
    vtkSmartPointer<vtkChartXY> chart_decs;
    vtkSmartPointer<vtkRenderer> renderer_decs;
    vtkSmartPointer<vtkRenderWindow> window_decs;
    std::vector<vtkSmartPointer<vtkTable> > front_tables_decs;
    std::vector<vtkSmartPointer<vtkFloatArray> > obj_data_decs;
    std::vector<vtkSmartPointer<vtkStringArray> > crowd_dist_labels_decs;

    RandomColourGen my_colour_gen_decs;
    
public:
    
    PlotFrontVTK() :
        view_objs(vtkSmartPointer<vtkContextView>::New()),
        chart_objs(vtkSmartPointer<vtkChartXY>::New()),
        renderer_objs(vtkSmartPointer<vtkRenderer>::New()),
        view_decs(vtkSmartPointer<vtkContextView>::New()),
        chart_decs(vtkSmartPointer<vtkChartXY>::New()),
        renderer_decs(vtkSmartPointer<vtkRenderer>::New())
    {
        
        view_objs->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
        view_objs->GetRenderWindow()->SetSize(400, 300);
        view_objs->GetScene()->AddItem(chart_objs);
        chart_objs->SetShowLegend(true);
        
        view_decs->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
        view_decs->GetRenderWindow()->SetSize(400, 300);
        view_decs->GetScene()->AddItem(chart_decs);
        chart_decs->SetShowLegend(true);
    }
    
    ~PlotFrontVTK()
    {
//        view_objs->GetInteractor()->Initialize();
        view_objs->GetInteractor()->Start();

//        view_decs->GetInteractor()->Initialize();
        view_decs->GetInteractor()->Start();
    }
    
    bool
    operator()(PopulationSPtr pop)
    {
        FrontsSPtr fronts = pop->getFronts();
        front_tables_objs.clear();
        obj_data_objs.clear();
        crowd_dist_labels_objs.clear();
        chart_objs->ClearPlots();
        
        int num_objectives = fronts->front().front()->numberOfObjectives();
        int number_of_fronts = fronts->size();
        
        for (int i = 0; i < number_of_fronts; ++i)
        {
            int front_size = (*fronts)[i].size();
            
            if (front_size > 0)
            {
                //make new table
                front_tables_objs.push_back(vtkSmartPointer<vtkTable>::New());
                vtkTable * table = front_tables_objs[i];
                
                // Add a column to the table for each objective
                for (int j = 0; j < num_objectives; ++j)
                {
                    obj_data_objs.push_back(vtkSmartPointer<vtkFloatArray>::New());
                    vtkFloatArray * array = obj_data_objs[i * num_objectives + j];
                    std::string name = std::string("Obj ").append(std::to_string(j+1));
                    array->SetName( name.c_str() );
                    table->AddColumn( array );
                }
                
                // Add a column with labels -- of the crowding distance.
                crowd_dist_labels_objs.push_back(vtkSmartPointer<vtkStringArray>::New());
                vtkStringArray * crowd_dist_label = crowd_dist_labels_objs[i];
                crowd_dist_label->SetName("Ind: ");
                table->AddColumn(crowd_dist_label);
                
                table->SetNumberOfRows((*fronts)[i].size());
                
                for (int j = 0; j < num_objectives; ++j)
                {
                    for (int k = 0; k < front_size; ++k)
                    {
                        table->SetValue(k, j, (*fronts)[i][k]->getObjective(j));
                    }
                }
                
                for (int k = 0; k < front_size; ++k)
                {
                    std::stringstream label;
                    for (int l = 0; l < (*fronts)[i][k]->numberOfRealDecisionVariables(); ++l)
                    {
                        label << "x" << std::to_string(l+1) << ": " << (*fronts)[i][k]->getRealDV(l) << " ";
                    }
//                    label.precision(4);
                    label << "crowd_dist: " << (*fronts)[i][k]->getCrowdingScore();
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
        
        for (int i = 0; i < front_tables_objs.size(); ++i)
        {
//            int alpha = (255 / front_tables.size()) * (i + 1);
//            front_tables[i]->Dump();
            // Add multiple scatter plots, setting the colors etc
            vtkPlot *points = chart_objs->AddPlot(vtkChart::POINTS);
#if VTK_MAJOR_VERSION <= 5
            points->SetInput(front_tables[i], 0, 1);
#else
            points->SetInputData(front_tables_objs[i], 0, 1);
#endif
            points->SetLabel(std::string("Front ").append(std::to_string(i+1)));
            rgb colour = my_colour_gen_objs();
            points->SetColor(int(colour.r * 255), int(colour.g * 255), int(colour.b * 255), 255);
            points->SetWidth(1.0);
            vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CIRCLE);
            points->SetIndexedLabels(crowd_dist_labels_objs[i].GetPointer());
            points->SetTooltipLabelFormat("%i from %l (%x, %y)");
            
            
        }
        
        //Finally render the scene
        view_objs->GetRenderWindow()->SetMultiSamples(0);
        view_objs->GetInteractor()->Initialize();
        view_objs->GetInteractor()->Render();
//        view->GetRenderWindow()->Render();
//        view->GetInteractor()->Start();

        front_tables_decs.clear();
        obj_data_decs.clear();
        crowd_dist_labels_decs.clear();
        chart_decs->ClearPlots();

        int num_dvs = fronts->front().front()->numberOfRealDecisionVariables();
        for (int i = 0; i < number_of_fronts; ++i)
        {
            int front_size = (*fronts)[i].size();

            if (front_size > 0)
            {
                //make new table
                front_tables_decs.push_back(vtkSmartPointer<vtkTable>::New());
                vtkTable * table_decs = front_tables_decs[i];

                // Add a column to the table for each objective
                for (int j = 0; j < num_dvs; ++j)
                {
                    obj_data_decs.push_back(vtkSmartPointer<vtkFloatArray>::New());
                    vtkFloatArray * array_decs = obj_data_decs[i * num_dvs + j];
                    std::string name = std::string("DV ").append(std::to_string(j+1));
                    array_decs->SetName( name.c_str() );
                    table_decs->AddColumn( array_decs );
                }

                // Add a column with labels -- of the crowding distance.
                crowd_dist_labels_decs.push_back(vtkSmartPointer<vtkStringArray>::New());
                vtkStringArray * crowd_dist_label = crowd_dist_labels_decs[i];
                crowd_dist_label->SetName("Ind: ");
                table_decs->AddColumn(crowd_dist_label);

                table_decs->SetNumberOfRows((*fronts)[i].size());

                for (int j = 0; j < num_dvs; ++j)
                {
                    for (int k = 0; k < front_size; ++k)
                    {
                        table_decs->SetValue(k, j, (*fronts)[i][k]->getRealDV(j));
                    }
                }

                for (int k = 0; k < front_size; ++k)
                {
                    std::stringstream label;
                    for (int l = 0; l < (*fronts)[i][k]->numberOfRealDecisionVariables(); ++l)
                    {
                        label << "x" << std::to_string(l+1) << ": " << (*fronts)[i][k]->getRealDV(l) << " ";
                    }
//                    label.precision(4);
                    label << "crowd_dist: " << (*fronts)[i][k]->getCrowdingScore();
                    table_decs->SetValue(k, num_dvs, label.str().c_str());
                }
            }
        }



        for (int i = 0; i < front_tables_decs.size(); ++i)
        {
//            int alpha = (255 / front_tables.size()) * (i + 1);
//            front_tables[i]->Dump();
            // Add multiple scatter plots, setting the colors etc
            vtkPlot *points = chart_decs->AddPlot(vtkChart::POINTS);
#if VTK_MAJOR_VERSION <= 5
            points->SetInput(front_tables[i], 0, 1);
#else
            points->SetInputData(front_tables_decs[i], 0, 1);
#endif
            points->SetLabel(std::string("Front ").append(std::to_string(i+1)));
            rgb colour = my_colour_gen_decs();
            points->SetColor(int(colour.r * 255), int(colour.g * 255), int(colour.b * 255), 255);
            points->SetWidth(1.0);
            vtkPlotPoints::SafeDownCast(points)->SetMarkerStyle(vtkPlotPoints::CIRCLE);
            points->SetIndexedLabels(crowd_dist_labels_decs[i].GetPointer());
            points->SetTooltipLabelFormat("%i from %l (%x, %y)");


        }

        //Finally render the scene
        view_decs->GetRenderWindow()->SetMultiSamples(0);
        view_decs->GetInteractor()->Initialize();
        view_decs->GetInteractor()->Render();
        return true;
    }
    

};

#else
class PlotFrontVTK : public CheckpointBase
{
    
public:
    
    PlotFrontVTK()
    {
        

        
    }
    
    ~PlotFrontVTK()
    {

    }
    
    bool
    operator()(PopulationSPtr pop)
    {
        return true;
    }
    
    
};
#endif

#endif /* PlotFronts_h */
