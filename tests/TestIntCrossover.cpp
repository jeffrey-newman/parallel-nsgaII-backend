//
// Created by a1091793 on 23/3/17.
//

//
// Created by a1091793 on 10/3/17.
//

#include "../Crossover.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>

QT_CHARTS_USE_NAMESPACE

int
main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    typedef std::mt19937 RNG_t;
    unsigned int seed = 1;
    double eta = 10;
    double crossover_probability = 1.0;
    RNG_t rng(seed);

    OnePointCrossover<RNG_t> crossover_tester(rng);

    int number_dvs = 2; //number of decision variables
    int min_value = -1.0;
    int max_value = 8.0;
    std::vector<int> lower_bounds(number_dvs, min_value);
    std::vector<int> upper_bounds(number_dvs, max_value);
    std::vector<double> lower_bounds_d;
    std::vector<double> upper_bounds_d;
    std::vector<MinOrMaxType> min_or_max(1, MINIMISATION);
    ProblemDefinitionsSPtr defs(new ProblemDefinitions(lower_bounds_d, upper_bounds_d,lower_bounds, upper_bounds, min_or_max, 0));
    //    std::vector<DecisionVariable_t> parent1_dv_values {2};
    //    std::vector<DecisionVariable_t> parent2_dv_values {5};
    Individual ind1(defs);
    Individual ind2(defs);
    ind1.setIntDV(0,1); ind1.setIntDV(1,6);
    ind2.setIntDV(0,6); ind2.setIntDV(1,1);

//    std::vector<QBarSet> chart_data;
//    for(int i = min_value; i <= max_value; i++)
//    {
//        chart_data.push_back(QBarSet(std::to_string(i)));
//    }


    std::map<int,std::map<int, int> > counts;
    //Initialise counts to zero.
    for (int j = min_value; j <= max_value ; ++j)
    {
        for (int i = min_value; i <= max_value ; ++i)
        {
            counts[i][j] = 0;
        }
    }

    int num_samples = 100;
    for (int i = 0; i < num_samples; ++i)
    {
        Individual ind1_cpy = ind1;
        Individual ind2_cpy = ind2;
        crossover_tester(ind1_cpy, ind2_cpy);
        ++counts[ind1_cpy.getIntDV(0)][ind1_cpy.getIntDV(1)];
        ++counts[ind2_cpy.getIntDV(0)][ind2_cpy.getIntDV(1)];
    }

    // Create series for each point, so we can have different size of marker.
    std::vector<QScatterSeries *> points;
    for (int j = min_value; j <= max_value ; ++j)
    {
        for (int i = min_value; i <= max_value ; ++i)
        {
            QScatterSeries *series0 = new QScatterSeries();
            series0->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            series0->setMarkerSize(counts[i][j]);
            *series0 << QPointF(i,j);
            points.push_back(series0);
        }
    }

    QChart *chart = new QChart();

    BOOST_FOREACH(QScatterSeries * series, points)
                {
                    chart->addSeries(series);
                }


    chart->setTitle("Testing Integer Crossover");
//    chart->setAnimationOptions(QChart::SeriesAnimations);

//    QStringList categories;
//    for (int i = min_value; i <= max_value; i++)
//    {
//        categories << QString::number(i);
//    }
//    QBarCategoryAxis *axis = new QBarCategoryAxis();
//    axis->append(categories);
    chart->createDefaultAxes();
//    chart->setAxisX(axis, series);
//
//    chart->legend()->setVisible(true);
//    chart->legend()->setAlignment(Qt::AlignBottom);

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(420, 300);
    window.show();

    return a.exec();

}