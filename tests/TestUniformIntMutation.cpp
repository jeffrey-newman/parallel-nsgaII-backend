//
// Created by a1091793 on 10/3/17.
//

#include "../Mutation.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
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
    double mutation_probability = 1.0;
    RNG_t rng(seed);

    UniformIntMutation<RNG_t> mutation_tester(rng, mutation_probability);

    int number_dvs = 1; //number of decision variables
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

//    std::vector<QBarSet> chart_data;
//    for(int i = min_value; i <= max_value; i++)
//    {
//        chart_data.push_back(QBarSet(std::to_string(i)));
//    }


    QBarSet *set = new QBarSet("Frequency Counts");
    std::vector<int> counts(max_value-min_value);

    int num_samples = 20;
    for (int i = 0; i < num_samples; ++i)
    {
        Individual muted1(ind1);
        mutation_tester.operator()(muted1);
        counts[muted1.getIntDV(0)-min_value]++;
//        results.push_back(muted1.getRealDV(0));
        //        std::cout << child1[0] << std::endl;
        //        std::cout << child2[0] << std::endl;
    }

    BOOST_FOREACH(int count, counts)
                {
                    *set << count;
                }

    QBarSeries * series = new QBarSeries();
    series->append(set);

    QChart *chart = new QChart();
    chart->addSeries(series);
    chart->setTitle("Testing Uniform Integer Mutation");
    chart->setAnimationOptions(QChart::SeriesAnimations);

    QStringList categories;
    for (int i = min_value; i <= max_value; i++)
    {
        categories << QString::number(i);
    }
    QBarCategoryAxis *axis = new QBarCategoryAxis();
    axis->append(categories);
    chart->createDefaultAxes();
    chart->setAxisX(axis, series);

    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(420, 300);
    window.show();

    return a.exec();

}