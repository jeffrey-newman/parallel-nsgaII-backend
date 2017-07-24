#ifndef METRICLINEPLOT_HPP
#define METRICLINEPLOT_HPP

#include "../Population.hpp"
#include "../Checkpoint.hpp"
#include "../Metrics/MetricBase.hpp"


#if defined(WITH_VTK)
#include <vtkVersion.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>

class MetricLinePlot : public CheckpointBase
{
private:
    vtkSmartPointer<vtkTable> table;
    vtkSmartPointer<vtkFloatArray> arrX;
    vtkSmartPointer<vtkFloatArray> arrM;
    vtkSmartPointer<vtkContextView> view;
    vtkSmartPointer<vtkChartXY> chart;
    int num_gens;
    MetricBaseSPtr metric;

public:
    MetricLinePlot(MetricBaseSPtr _metric) :
        table(vtkSmartPointer<vtkTable>::New()),
        arrX(vtkSmartPointer<vtkFloatArray>::New()),
        arrM(vtkSmartPointer<vtkFloatArray>::New()),
        view(vtkSmartPointer<vtkContextView>::New()),
        chart(vtkSmartPointer<vtkChartXY>::New()),
        num_gens(0),
        metric(_metric)
    {
        arrX->SetName("Generations");
        table->AddColumn(arrX);
        arrM->SetName("Metric");
        table->AddColumn(arrM);
        view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
        view->GetScene()->AddItem(chart);

    }

    ~MetricLinePlot()
    {
        view->GetInteractor()->Start();
    }

    bool
    operator ()(PopulationSPtr pop)
    {
        ++num_gens;
//        bool return_val = metric(pop);

        vtkIdType row_num = table->InsertNextBlankRow();
        table->SetValue(row_num, 0, num_gens);
        table->SetValue(row_num, 1, metric->getVal());

        if (table->GetNumberOfRows() > 2)
        {
            // Start interactor
            chart->ClearPlots();
            vtkPlot *line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
            line->SetInput(table, 0, 1);
#else
            line->SetInputData(table, 0, 1);
#endif
            line->SetColor(0, 255, 0, 255);
            line->SetWidth(1.0);
//            view->Render();
            //Finally render the scene
            view->GetRenderWindow()->SetMultiSamples(0);
            view->GetInteractor()->Initialize();
            view->GetInteractor()->Render();
        }
//        view->GetInteractor()->Initialize();

        return true;
    }
};


#else
class MetricLinePlot : public CheckpointBase
{
private:
    MetricBaseSPtr metric;

public:

    MetricLinePlot(MetricBaseSPtr _metric) :
        metric(_metric)
    {



    }

    ~MetricLinePlot()
    {

    }

    bool
    operator()(PopulationSPtr pop)
    {
        return true;
    }


};
#endif


#endif // METRICLINEPLOT_HPP
