#ifndef GENETIC_OPTIMIZER_H
#define GENETIC_OPTIMIZER_H
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "agent.h"
#include "openga.hpp"
#
#define MAX_NUMBER_OF_THRUSTS 5

class genetic_optimizer;
struct MySolution;
struct MyMiddleCost;

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

class genetic_optimizer {
    public:
        int instance;
        string fileName;
        double max_fuel;
        int max_time[MAX_NUMBER_OF_THRUSTS];
        int nb_of_thrusts;
        int global_min_time = 0;
        executionT so_far_executed;
        bool verbose_chromosomes = false;
        agent* starting_point_agent = NULL;

        genetic_optimizer(int instance, int arg_nb_of_thrusts);
        ~genetic_optimizer();
        bool solve(int population_size);

    private:
        bool eval_solution(const MySolution& p, MyMiddleCost& c);
        void init_genes(MySolution& p, const std::function<double(void)>& rnd01);
        MySolution mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale);
        MySolution crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01);
        double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);
        void SO_report_generation(int generation_number,
                    const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
                    const MySolution& best_genes);
};

struct MySolution {
	int time[MAX_NUMBER_OF_THRUSTS];
	double speed_x[MAX_NUMBER_OF_THRUSTS];
	double speed_y[MAX_NUMBER_OF_THRUSTS];

	string to_string(genetic_optimizer *optimizer) const {
		std::ostringstream out;
		out.precision(20);
		out << "{ ";
		for (executionT::iterator it = optimizer->so_far_executed.begin();
			 it != optimizer->so_far_executed.end(); ++it)
			out << "map[" << std::to_string(it->first) << "] = "
				<< "Complex(" << std::to_string(real(it->second)) << ", "
				<< std::to_string(imag(it->second)) << "); ";
		for (int i = 0; i < optimizer->nb_of_thrusts; i++) {
			out << "map[" << std::to_string(time[i]) << "] = "
				<< "Complex(" << std::to_string(speed_x[i]) << ", "
				<< std::to_string(speed_y[i]) << "); ";
		}
		out << " }";
		return out.str();
	}
};

struct MyMiddleCost {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};
#endif