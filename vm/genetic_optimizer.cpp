// main.cpp

#include <fstream>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"

using std::cout;
using std::endl;
using std::string;

int gInstance;

// main.cpp

#include <fstream>
#include <iostream>
#include <string>
#include "openga.hpp"

using std::cout;
using std::endl;
using std::string;

struct MySolution {
	int time1;
	double speed1;
	double alpha1;

	string to_string() const {
		return string("{ [") + std::to_string(time1) + "]" +
			   "Complex(" + std::to_string(speed1) +
			   ", " + std::to_string(alpha1) + ") }";
	}
};

struct MyMiddleCost {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

void init_genes(MySolution& p, const std::function<double(void)>& rnd01) {
	// rnd01() gives a random number in 0~1
	p.time1 = 0.0 + 10 * rnd01();
	p.speed1 = -10000. + 20000. * rnd01();
	p.alpha1 = -1.0 + 2 * 1.0 * rnd01();
}

bool eval_solution(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent executeur(gInstance);

	execution[p.time1] = Complex(p.speed1, p.alpha1);
	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();

	return true;  // solution is accepted
}

MySolution mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale) {
	MySolution X_new;
	bool in_range;
	do {
		in_range = true;
		X_new = X_base;
		X_new.time1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.time1 >= 0.0 && X_new.time1 < 10);
		X_new.speed1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed1 >= -10000 && X_new.speed1 < 10000);
		X_new.alpha1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range =
			in_range && (X_new.alpha1 >= -1.0 && X_new.alpha1 < 1.0);
	} while (!in_range);
	return X_new;
}

MySolution crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01) {
	MySolution X_new;
	double r;
	r = rnd01();
	X_new.time1 = r * X1.time1 + (1.0 - r) * X2.time1;
	r = rnd01();
	X_new.speed1 = r * X1.speed1 + (1.0 - r) * X2.speed1;
	r = rnd01();
	X_new.alpha1 = r * X1.alpha1 + (1.0 - r) * X2.alpha1;
	r = rnd01();
	return X_new;
}

double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

std::ofstream output_file;

void SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "Generation [" << generation_number << "], "
		 << "Best=" << last_generation.best_total_cost << ", "
		 << "Average=" << last_generation.average_cost << ", "
		 << "Best genes=(" << best_genes.to_string() << ")"
		 << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;

	output_file << generation_number << "\t" << last_generation.average_cost
				<< "\t" << last_generation.best_total_cost << "\t"
				<< best_genes.to_string() << "\n";
	output_file.flush();
}

int main(int argc, char** argv) {
	if (argc < 2) {
		cerr << "please enter the scenario id" << endl;
		exit(-1);
	}
	gInstance = atoi(argv[1]);

	output_file.open("./results/" + to_string(gInstance) + ".txt");
	output_file << "step"
				<< "\t"
				<< "cost_avg"
				<< "\t"
				<< "cost_best"
				<< "\t"
				<< "solution_best"
				<< "\n";

	output_file.flush();
	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.idle_delay_us = 1;  // switch between threads quickly
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = 500;
	ga_obj.generation_max = 1000;
	ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
	ga_obj.init_genes = init_genes;
	ga_obj.eval_solution = eval_solution;
	ga_obj.mutate = mutate;
	ga_obj.crossover = crossover;
	ga_obj.SO_report_generation = SO_report_generation;
	ga_obj.crossover_fraction = 0.8;
	ga_obj.mutation_rate = 0.8;
	ga_obj.best_stall_max = 50;
	ga_obj.elite_count = 10;
	ga_obj.solve();

	cout << "The problem is optimized in " << timer.toc() << " seconds."
		 << endl;

	output_file.close();
	return 0;
}