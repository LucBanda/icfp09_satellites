#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "agent.h"
#include "openga.hpp"
#include "ellipse.h"

using std::cout;
using std::endl;
using std::string;

int gInstance;
double max_fuel;
int max_time1;
int max_time2;
int nb_of_thrusts;
ellipse *reference_ellipse;

struct MySolution {
	int time1;
	double speed1_x;
	double speed1_y;
	int time2;
	double speed2_x;
	double speed2_y;

	string to_string() const {
		std::ostringstream out;
		out.precision(20);
		out << "{ map[" << std::to_string(time1) << "] = "
			   << "Complex(" << std::to_string(speed1_x)
			   << ", " << std::to_string(speed1_y) << "); map["
			   << (nb_of_thrusts >= 2 ? std::to_string(time2) : "-1") << "] = "
			   <<"Complex(" << std::to_string(speed2_x)
			   << ", " << std::to_string(speed2_y) << "); }";
		return out.str();
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
	p.time1 = 0 + max_time1 * rnd01();
	p.speed1_x = -max_fuel + 2 * max_fuel * rnd01();
	p.speed1_y = -max_fuel + 2 * max_fuel * rnd01();
	if (nb_of_thrusts >= 2) {
		p.time2 = 0 + max_time2 * rnd01();
		p.speed2_x = -max_fuel + 2 * max_fuel * rnd01();
		p.speed2_y = -max_fuel + 2 * max_fuel * rnd01();
	} else {
		p.time2 = -1;
		p.speed2_x =-1;
		p.speed2_y =-1;
	}
}

bool eval_solution1(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent1 executeur(gInstance);

	execution[p.time1] = Complex(p.speed1_x, p.speed1_y);
	if (nb_of_thrusts >= 2) {
		execution[p.time2] = Complex(p.speed2_x, p.speed2_y);
	}

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

bool eval_solution2(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent2 executeur(gInstance);

	execution[p.time1] = Complex(p.speed1_x, p.speed1_y);
	if (nb_of_thrusts >= 2) {
		execution[p.time2] = Complex(p.speed2_x, p.speed2_y);
	}

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

bool eval_solution3(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent3 executeur(gInstance);

	execution[p.time1] = Complex(p.speed1_x, p.speed1_y);
	if (nb_of_thrusts >= 2) {
		execution[p.time2] = Complex(p.speed2_x, p.speed2_y);
	}
	executeur.set_target_ellipse(reference_ellipse);

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
		in_range = in_range && (X_new.time1 >= 0 && X_new.time1 < max_time1);
		X_new.speed1_x += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed1_x >= -max_fuel && X_new.speed1_x < max_fuel);
		X_new.speed1_y += 0.2 * (rnd01() - rnd01()) * shrink_scale;
		in_range = in_range && (X_new.speed1_y >= -max_fuel && X_new.speed1_y < max_fuel);
		if (nb_of_thrusts >= 2) {
			X_new.time2 += 0.2 * (rnd01() - rnd01()) * shrink_scale;
			in_range = in_range && (X_new.time2 >= 0 && X_new.time2 < max_time2);
			X_new.speed2_x += 0.2 * (rnd01() - rnd01()) * shrink_scale;
			in_range = in_range && (X_new.speed2_x >= -max_fuel && X_new.speed2_x < max_fuel);
			X_new.speed2_y += 0.2 * (rnd01() - rnd01()) * shrink_scale;
			in_range = in_range && (X_new.speed2_y >= -max_fuel && X_new.speed2_y < max_fuel);
		}
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
	X_new.speed1_x = r * X1.speed1_x + (1.0 - r) * X2.speed1_x;
	r = rnd01();
	X_new.speed1_y = r * X1.speed1_y + (1.0 - r) * X2.speed1_y;
	r = rnd01();
	if (nb_of_thrusts >= 2) {
		X_new.time2 = r * X1.time2 + (1.0 - r) * X2.time2;
		r = rnd01();
		X_new.speed2_x = r * X1.speed2_x + (1.0 - r) * X2.speed2_x;
		r = rnd01();
		X_new.speed2_y = r * X1.speed2_y + (1.0 - r) * X2.speed2_y;
	}
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
	cout << "Problem Id:" << gInstance << ", "
		 << "Generation [" << generation_number << "], "
		 <<setprecision(10)<< "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "Best genes=(" << best_genes.to_string() << ")"
		 << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;

	output_file << generation_number << "   \t"
				<< std::setw( 11 ) << -last_generation.average_cost
				<< "   \t" << std::setw( 11 ) << -last_generation.best_total_cost << "\t"
				<< std::setw( 20 ) << best_genes.to_string() << "\n";
	output_file.flush();
}

int main(int argc, char** argv) {
	bool do_all;
	if (argc < 2) {
		cerr << "please enter the scenario id" << endl;
		exit(-1);
	}
	gInstance = atoi(argv[1]);

	if (strcmp(argv[1], "all") != 0) {
		gInstance = atoi(argv[1]);
		do_all = false;
	} else
		do_all = true;

	for (int i = 2; i < 4; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				gInstance = i * 1000 + j;
			}

			if (gInstance / 1000 == 1) {
				agent1 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 1;
				max_time2 = 5000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2001) {
				agent2 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 20000;
				max_time2 = 50000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2002) {
				agent2 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 10000;
				max_time2 = 25000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2003) {
				agent2 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 1000;
				max_time2 = 100000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2004) {
				agent2 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 50000;
				max_time2 = 100000;
				nb_of_thrusts = 1;
			}else if (gInstance == 3001) {
				agent3 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 20000;
				max_time2 = 20000;
				reference_ellipse = new ellipse(gInstance, 0);
				nb_of_thrusts = 1;
			} else if (gInstance == 3002) {
				agent3 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 30000;
				reference_ellipse = new ellipse(gInstance, 0);
				nb_of_thrusts = 1;
			} else if (gInstance == 3003) {
				agent3 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 50000;
				max_time2 = 50000;
				reference_ellipse = new ellipse(gInstance, 0);
				nb_of_thrusts = 1;
			} else if (gInstance == 3004) {
				agent3 ag(gInstance);
				max_fuel = ag.vm->get_fuel();
				max_time1 = 50000;
				max_time2 = 50000;
				reference_ellipse = new ellipse(gInstance, 0);
				nb_of_thrusts = 1;
			}

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
			ga_obj.population = 2000;
			ga_obj.generation_max = 1000;
			ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
			ga_obj.init_genes = init_genes;
			if (gInstance / 1000 == 1) {
				ga_obj.eval_solution = eval_solution1;
			} else if (gInstance / 1000 == 2) {
				ga_obj.eval_solution = eval_solution2;
			} else if (gInstance / 1000 == 3) {
				ga_obj.eval_solution = eval_solution3;
			}
			ga_obj.mutate = mutate;
			ga_obj.crossover = crossover;
			ga_obj.SO_report_generation = SO_report_generation;
			ga_obj.crossover_fraction = 0.7;
			ga_obj.mutation_rate = 0.8;
			ga_obj.best_stall_max = 50;
			ga_obj.elite_count = 20;
			ga_obj.solve();
			cout << "The problem is optimized in " << timer.toc() << " seconds."
				<< endl;

			output_file.close();
			if (reference_ellipse)
				delete reference_ellipse;

			if (!do_all) {
				return 0;
			}
		}
	}
	return 0;
}