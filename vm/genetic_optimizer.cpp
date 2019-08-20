#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "fileparser.h"
#include "openga.hpp"
#include "genetic_optimizer.h"
#include "functional"

using std::cout;
using std::endl;
using std::string;

void genetic_optimizer::init_genes(MySolution& p, const std::function<double(void)>& rnd01) {
	int i;
	int min_time = global_min_time;
	for (i = 0; i < nb_of_thrusts; i++) {
		p.time[i] = min_time + max_time[i] * rnd01();
		p.speed_x[i] = -max_fuel + 2 * max_fuel * rnd01();
		p.speed_y[i] = -max_fuel + 2 * max_fuel * rnd01();
		min_time = p.time[i];
	}
}

bool genetic_optimizer::eval_solution(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent *executeur = agent_factory(instance);

	execution = so_far_executed;
	for (int i = 0; i < nb_of_thrusts; i++)
		execution[p.time[i]] = Complex(p.speed_x[i], p.speed_y[i]);

	if (starting_point_agent != NULL) {
		executeur->set_resume_point(starting_point_agent);
	}
	executeur->set_execution_map(&execution);
	c.objective1 = -executeur->run();
	delete executeur;

	if (c.objective1 == -1) return false;
	return true;  // solution is accepted
}

MySolution genetic_optimizer::mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale) {
	MySolution X_new;
	bool in_range;
	int min_time = global_min_time;
	int damped_thrust = nb_of_thrusts * rnd01();
	for (int i = 0; i < nb_of_thrusts; i++) {
		if (i == damped_thrust) {
			do {
				in_range = true;
				X_new = X_base;
				X_new.time[damped_thrust] +=
					10 * (rnd01() - rnd01()) * shrink_scale;
				if (X_new.time[damped_thrust] < min_time)
					X_new.time[damped_thrust] = min_time;
				if (X_new.time[damped_thrust] >=
					(min_time + max_time[damped_thrust]))
					X_new.time[damped_thrust] =
						(min_time + max_time[damped_thrust]);
				X_new.speed_x[damped_thrust] +=
					.2 * (rnd01() - rnd01()) * shrink_scale;
				in_range =
					in_range && (X_new.speed_x[damped_thrust] >= -max_fuel &&
								 X_new.speed_x[damped_thrust] < max_fuel);
				X_new.speed_y[damped_thrust] +=
					.2 * (rnd01() - rnd01()) * shrink_scale;
				in_range =
					in_range && (X_new.speed_y[damped_thrust] >= -max_fuel &&
								 X_new.speed_y[damped_thrust] < max_fuel);

			} while (!in_range);
		}
		min_time = max(X_base.time[i], global_min_time);
	}
	return X_new;
}

MySolution genetic_optimizer::crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01) {
	MySolution X_new;
	double r;
	for (int i = 0; i < nb_of_thrusts; i++) {
		r = rnd01();
		X_new.time[i] = max((double)global_min_time,
							r * X1.time[i] + (1.0 - r) * X2.time[i]);
		r = rnd01();
		X_new.speed_x[i] = r * X1.speed_x[i] + (1.0 - r) * X2.speed_x[i];
		r = rnd01();
		X_new.speed_y[i] = r * X1.speed_y[i] + (1.0 - r) * X2.speed_y[i];
	}
	return X_new;
}

double genetic_optimizer::calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

std::ofstream output_file;

void genetic_optimizer::SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "Problem Id:" << instance << ", "
		 << "Generation [" << generation_number << "], " << setprecision(10)
		 << "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "Best genes=(" << best_genes.to_string(this) << ")"
		 << ", "
		 << "Exe_time=" << last_generation.exe_time << endl
		 << endl;

	if (verbose_chromosomes) {
		output_file.open("./debug/" + to_string(instance) + "/" +
						 to_string(generation_number) + ".txt");
		for (auto it = last_generation.chromosomes.begin();
			 it != last_generation.chromosomes.end(); ++it) {
			for (int i = 0; i < nb_of_thrusts; i++) {
				output_file << -it->total_cost << "\t" << it->genes.time[i]
							<< "\t" << it->genes.speed_x[i] << "\t"
							<< it->genes.speed_y[i] << endl;
			}
		}
		output_file.close();
	}
	output_file.open(fileName,
					 std::ofstream::app);
	output_file << generation_number << "   \t" << std::setw(11)
				<< -last_generation.average_cost << "   \t" << std::setw(11)
				<< -last_generation.best_total_cost << "\t" << std::setw(20)
				<< best_genes.to_string(this) << "\n";
	output_file.flush();
	output_file.close();
}

genetic_optimizer::genetic_optimizer(int arg_instance, int arg_nb_of_thrusts) {
	instance = arg_instance;
	fileName = "./results/" + to_string(instance) + ".txt";
	agent* base_agent = agent_factory(instance);
	ifstream file_to_test(fileName);
	so_far_executed.clear();
	global_min_time = 0;
	nb_of_thrusts = arg_nb_of_thrusts;
	if (!file_to_test.good()) {
		cout << "file do not exist" << endl;
		cout << "start from scratch" << endl;
		output_file.open("./results/" + to_string(instance) + ".txt");
		output_file << "step"
					<< "\t"
					<< "cost_avg"
					<< "\t"
					<< "cost_best"
					<< "\t"
					<< "solution_best"
					<< "\n";
		output_file.close();
		global_min_time = 0;
		so_far_executed.clear();
	} else {
		so_far_executed = parse_result(fileName);
		base_agent->set_execution_map(&so_far_executed);
		base_agent->run();
		global_min_time = base_agent->last_validated_time;

		// TODO generalize to other agents
		if (global_min_time > 0 && instance > 4000) {
			starting_point_agent = new agent4(instance);
			starting_point_agent->set_execution_map(
				&so_far_executed);
			starting_point_agent->run(global_min_time);
		}
		executionT::iterator target = so_far_executed.begin();
		while (target != so_far_executed.end()) {
			if (target->first > global_min_time) {
				target = so_far_executed.erase(target);
			} else {
				target++;
			}
		}
	}
	max_fuel = base_agent->vm->get_fuel_max() / 3.;
	max_time[0] = max(10000., global_min_time / 3.);

	cout << "searching time range : " << global_min_time << " -> "
			<< global_min_time + max_time[0] << endl;

	for (int k = 1; k < nb_of_thrusts; k++) max_time[k] = 50000;
	delete base_agent;
}
genetic_optimizer::~genetic_optimizer() {
	delete starting_point_agent;
}

bool genetic_optimizer::solve(int population_size) {
	using namespace std::placeholders;

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.idle_delay_us = 1;  // switch between threads quickly
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = population_size;
	ga_obj.generation_max = 5000;
	ga_obj.calculate_SO_total_fitness = std::bind(&genetic_optimizer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes = std::bind(&genetic_optimizer::init_genes, this, _1, _2);
	ga_obj.eval_solution = std::bind(&genetic_optimizer::eval_solution, this, _1, _2);
	ga_obj.mutate = std::bind(&genetic_optimizer::mutate, this, _1, _2, _3);
	ga_obj.crossover = std::bind(&genetic_optimizer::crossover, this, _1, _2, _3);
	ga_obj.SO_report_generation = std::bind(&genetic_optimizer::SO_report_generation, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = nb_of_thrusts * 20;
	ga_obj.average_stall_max = nb_of_thrusts * 20;
	ga_obj.elite_count = 10;
	EA::StopReason reason = ga_obj.solve();
	cout << "The problem is optimized in " << timer.toc()
			<< " seconds." << endl;
	cout << "cause: " << ga_obj.stop_reason_to_string(reason)
			<< endl;

	agent *base_agent = agent_factory(instance);
	so_far_executed = parse_result(fileName);
	base_agent->set_execution_map(&so_far_executed);
	base_agent->run();
	global_min_time = base_agent->last_validated_time;
	bool end = base_agent->non_validated_targets.size() == 0;
	delete base_agent;
	return end;
}


static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-l : load the best solution so far for this problem \n"
		"	-a : do all problem\n"
		"	-n number: number of thrust to optimize (default 1)\n"
		"	-f factor: divider of fuel max to limit thrust range (default is "
		"2)\n"
		"	-p population: population of each generation (default 2000)\n"
		"	-d : enable logging of chromosomes in a file\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int c;
	bool continue_after_stall = false;
	int population = 2000;
	int fuel_factor = 2.;
	int nb_of_thrusts = 1;
	int gInstance = 0;
	bool verbose_chromosomes = false;

	while ((c = getopt(argc, argv, "vdf:p:n:ahci:")) != -1) switch (c) {
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				break;
			case 'c':
				continue_after_stall = true;
				break;
			case 'n':
				nb_of_thrusts = atoi(optarg);
				break;
			case 'f':
				fuel_factor = atoi(optarg);
				break;
			case 'p':
				population = atoi(optarg);
				break;
			case 'v':
				verbose_chromosomes = true;
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}

	for (int i = 1; i < 5; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				gInstance = i * 1000 + j;
			}

			do {
				genetic_optimizer optimizer(gInstance, nb_of_thrusts);
				optimizer.verbose_chromosomes = verbose_chromosomes;

				bool solved = optimizer.solve(population);
				if (solved) break;

			} while (continue_after_stall);

			if (!do_all) {
				return 0;
			}
		}
	}
	return 0;
}