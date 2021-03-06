//
//  DebsSBXCrossoverTester.cpp
//  parallel-nsgaII-backend
//
//  Created by a1091793 on 20/11/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//



// Test to get replication of figure in Deb's article "Self-Adaptation in Real-Parameter Genetic Algorithms with Simulated Binary Crossover" through montecarlo sampling



#include <stdio.h>
#include <random>
#include <string>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>


QT_CHARTS_USE_NAMESPACE



#include "../Mutation.hpp"

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

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

    
    QBarSet *set = new QBarSet("Frequency Counts");
    std::vector<int> counts(max_value-min_value);
//    std::vector<double> dv_vals_sample;
    
    int num_samples = 1000000;
    for (int i = 0; i < num_samples; ++i)
    {
        Individual muted1(ind1);
        mutation_tester.operator()(muted1);
        counts[int(muted1.getRealDV(0))-min_value]++;
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
    
}
