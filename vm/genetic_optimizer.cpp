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

struct winning_trace {
	double speed1;
	double alpha1;
	int time1;
	double speed2;
	double alpha2;
	int time2;
	double speed3;
	double alpha3;
	int time3;
	double speed4;
	double alpha4;
	int time4;

	string to_string() const {
		return string("{") + "speed1:" + std::to_string(speed1) +
			   ", alpha1:" + std::to_string(alpha1) +
			   ", time1:" + std::to_string(time1) +
			   ", speed2:" + std::to_string(speed2) +
			   ", alpha2:" + std::to_string(alpha2) +
			   ", time2:" + std::to_string(time2) +
			   ", speed3:" + std::to_string(speed3) +
			   ", alpha3:" + std::to_string(alpha3) +
			   ", time3:" + std::to_string(time3) +
			   ", speed4:" + std::to_string(speed4) +
			   ", alpha4:" + std::to_string(alpha4) +
			   ", time4:" + std::to_string(time4) + "}";
	}
};

struct score {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};

typedef EA::Genetic<winning_trace, score> GA_Type;
typedef EA::GenerationType<winning_trace, score> Generation_Type;

void init_genes(winning_trace& p, const std::function<double(void)>& rnd01) {
	// rnd01() gives a random number in 0~1
	p.speed1 = 0.0 + 3 * rnd01();
	p.alpha1 = -1.0 + 2 * rnd01();
	p.time1 = 0.0 + 10000000 * rnd01();
	p.speed2 = 0.0 + 3 * rnd01();
	p.alpha2 = -1.0 + 10001 * rnd01();
	p.time2 = 0.0 + 10000000 * rnd01();
	p.speed3 = 0.0 + 3 * rnd01();
	p.alpha3 = -1.0 + 2 * rnd01();
	p.time3 = 0.0 + 10000000 * rnd01();
	p.speed4 = 0.0 + 3 * rnd01();
	p.alpha4 = -1.0 + 2 * rnd01();
	p.time4 = 0.0 + 10000000 * rnd01();
}

winning_trace mutate(const winning_trace& X_base,
					 const std::function<double(void)>& rnd01,
					 double shrink_scale) {
	winning_trace X_new;
	bool in_range;
	do {
		in_range = true;
		X_new = X_base;
		X_new.speed1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed1 >= 0.0 && X_new.speed1 < 3.0);
		X_new.alpha1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.alpha1 >= -1.0 && X_new.alpha1 < 1.0);
		X_new.time1 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.time1 >= 0.0 && X_new.time1 < 10e6);
		X_new.speed2 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed2 >= 0.0 && X_new.speed2 < 3.0);
		X_new.alpha2 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.alpha2 >= -1.0 && X_new.alpha2 < 10000);
		X_new.time2 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.time2 >= 0.0 && X_new.time2 < 10e6);
		X_new.speed3 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed3 >= 0.0 && X_new.speed3 < 3.0);
		X_new.alpha3 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.alpha3 >= -1.0 && X_new.alpha3 < 1.0);
		X_new.time3 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.time3 >= 0.0 && X_new.time3 < 10e6);
		X_new.speed4 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed4 >= 0.0 && X_new.speed4 < 3.0);
		X_new.alpha4 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.alpha4 >= -1.0 && X_new.alpha4 < 1.0);
		X_new.time4 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.time4 >= 0.0 && X_new.time4 < 10e6);
	} while (!in_range);
	return X_new;
}

winning_trace crossover(const winning_trace& X1, const winning_trace& X2,
						const std::function<double(void)>& rnd01) {
	winning_trace X_new;
	double r;
	r = rnd01();
	X_new.speed1 = r * X1.speed1 + (1.0 - r) * X2.speed1;
	r = rnd01();
	X_new.alpha1 = r * X1.alpha1 + (1.0 - r) * X2.alpha1;
	r = rnd01();
	X_new.time1 = r * X1.time1 + (1.0 - r) * X2.time1;
	r = rnd01();
	X_new.speed2 = r * X1.speed2 + (1.0 - r) * X2.speed2;
	r = rnd01();
	X_new.alpha2 = r * X1.alpha2 + (1.0 - r) * X2.alpha2;
	r = rnd01();
	X_new.time2 = r * X1.time2 + (1.0 - r) * X2.time2;
	r = rnd01();
	X_new.speed3 = r * X1.speed3 + (1.0 - r) * X2.speed3;
	r = rnd01();
	X_new.alpha3 = r * X1.alpha3 + (1.0 - r) * X2.alpha3;
	r = rnd01();
	X_new.time3 = r * X1.time3 + (1.0 - r) * X2.time3;
	r = rnd01();
	X_new.speed4 = r * X1.speed4 + (1.0 - r) * X2.speed4;
	r = rnd01();
	X_new.alpha4 = r * X1.alpha4 + (1.0 - r) * X2.alpha4;
	r = rnd01();
	X_new.time4 = r * X1.time4 + (1.0 - r) * X2.time4;
	return X_new;
}

bool eval_solution(const winning_trace& p, score& c) {
	executionT execution;
	execution[p.time1] = std::make_tuple(p.speed1, p.alpha1);
	execution[p.time1] = std::make_tuple(p.speed2, p.alpha2);
	execution[p.time2] = std::make_tuple(p.speed3, p.alpha3);
	execution[p.time3] = std::make_tuple(p.speed4, p.alpha4);
	// execution[p.time4] = std::make_tuple(p.speed1, p.alpha1);

	agent executeur(gInstance);
	c.objective1 = executeur.run(&execution);
	return true;  // solution is accepted
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
	const EA::GenerationType<winning_trace, score>& last_generation,
	const winning_trace& best_genes) {
	cout << "Generation [" << generation_number << "], "
		 << "Best=" << last_generation.best_total_cost << ", "
		 << "Average=" << last_generation.average_cost << ", "
		 << "Best genes=(" << best_genes.to_string() << ")"
		 << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;

	output_file << generation_number << "\t" << last_generation.average_cost
				<< "\t" << last_generation.best_total_cost << "\t"
				<< best_genes.to_string() << "\n";
}

int main(int argc, char** argv) {
	if (argc < 2) {
		cerr << "please enter the scenario id" << endl;
		exit(-1);
	}
	gInstance = atoi(argv[1]);

	output_file.open("results.txt");
	output_file << "step"
				<< "\t"
				<< "cost_avg"
				<< "\t"
				<< "cost_best"
				<< "\t"
				<< "solution_best"
				<< "\n";

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.idle_delay_us = 1;  // switch between threads quickly
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = 1000;
	ga_obj.generation_max = 1000;
	ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
	ga_obj.init_genes = init_genes;
	ga_obj.eval_solution = eval_solution;
	ga_obj.mutate = mutate;
	ga_obj.crossover = crossover;
	ga_obj.SO_report_generation = SO_report_generation;
	ga_obj.best_stall_max = 10;
	ga_obj.elite_count = 10;
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.2;
	ga_obj.best_stall_max = 10;
	ga_obj.elite_count = 10;
	ga_obj.solve();

	cout << "The problem is optimized in " << timer.toc() << " seconds."
		 << endl;

	output_file.close();
	return 0;
}