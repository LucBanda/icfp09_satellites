#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "agent.h"
#include "openga.hpp"
#include "fileparser.h"

using std::cout;
using std::endl;
using std::string;

#define MAX_NUMBER_OF_THRUSTS	20
int gInstance;
double max_fuel;
int max_time[MAX_NUMBER_OF_THRUSTS];
int nb_of_thrusts;
int global_min_time = 0;
executionT so_far_executed;

struct MySolution {
	int time[MAX_NUMBER_OF_THRUSTS];
	double speed_x[MAX_NUMBER_OF_THRUSTS];
	double speed_y[MAX_NUMBER_OF_THRUSTS];

	string to_string() const {
		std::ostringstream out;
		out.precision(20);
		out << "{ ";
			for (executionT::iterator it = so_far_executed.begin() ; it != so_far_executed.end(); ++it)
				out << "map[" << std::to_string(it->first) << "] = "
					<< "Complex(" << std::to_string(real(it->second))
					<< ", " << std::to_string(imag(it->second)) << "); ";
			for (int i = 0; i < nb_of_thrusts; i++) {
				out << "map[" << std::to_string(time[i]) << "] = "
			   		<< "Complex(" << std::to_string(speed_x[i])
			   		<< ", " << std::to_string(speed_y[i]) << "); ";
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

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

void init_genes(MySolution& p, const std::function<double(void)>& rnd01) {
	// rnd01() gives a random number in 0~1
	int i;
	for (i = 0; i < MAX_NUMBER_OF_THRUSTS; i++) {
		p.time[i] = -1;
		p.speed_x[i] =-1;
		p.speed_y[i] =-1;
	}
	int min_time = global_min_time;
	for (i = 0; i < nb_of_thrusts; i ++) {
		p.time[i] = min_time + max_time[i] * rnd01();
		p.speed_x[i] = -max_fuel + 2 * max_fuel * rnd01();
		p.speed_y[i] = -max_fuel + 2 * max_fuel * rnd01();
		min_time = p.time[i];
	}
}

bool eval_solution1(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent1 executeur(gInstance);

	for (int i = 0; i < nb_of_thrusts; i++)
		execution[p.time[i]] = Complex(p.speed_x[i], p.speed_y[i]);

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

bool eval_solution2(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent2 executeur(gInstance);

	for (int i = 0; i < nb_of_thrusts; i++)
		execution[p.time[i]] = Complex(p.speed_x[i], p.speed_y[i]);

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

bool eval_solution3(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent3 executeur(gInstance);

	for (int i = 0; i < nb_of_thrusts; i++)
		execution[p.time[i]] = Complex(p.speed_x[i], p.speed_y[i]);

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

bool eval_solution4(const MySolution& p, MyMiddleCost& c) {
	executionT execution;
	agent4 executeur(gInstance);

	execution = so_far_executed;
	for (int i = 0; i < nb_of_thrusts; i++)
		execution[p.time[i]] = Complex(p.speed_x[i], p.speed_y[i]);

	executeur.set_execution_map(&execution);
	c.objective1 = -executeur.run();
	return true;  // solution is accepted
}

MySolution mutate(const MySolution& X_base,
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
				X_new.time[damped_thrust] += 10. * (rnd01() - rnd01()) * shrink_scale;
				if (X_new.time[damped_thrust] < min_time) X_new.time[damped_thrust] = min_time;
				if(X_new.time[damped_thrust] >= (min_time + max_time[damped_thrust])) X_new.time[damped_thrust] = (min_time + max_time[damped_thrust]);
				X_new.speed_x[damped_thrust] += .02 * (rnd01() - rnd01()) * shrink_scale;
				in_range = in_range && (X_new.speed_x[damped_thrust] >= -max_fuel && X_new.speed_x[damped_thrust] < max_fuel);
				X_new.speed_y[damped_thrust] += .02 * (rnd01() - rnd01()) * shrink_scale;
				in_range = in_range && (X_new.speed_y[damped_thrust] >= -max_fuel && X_new.speed_y[damped_thrust] < max_fuel);

			} while (!in_range);
		}
		min_time = max(X_base.time[i], global_min_time);
	}
	return X_new;
}

MySolution crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01) {
	MySolution X_new;
	double r, rmin;
	for (int i = 0; i < nb_of_thrusts; i++) {
			r = rnd01();
			rmin = 1. - rnd01() / 10.;
			X_new.time[i] = max((double)global_min_time, rmin * r * X1.time[i] + (1.0 - r) * X2.time[i]);
			r = rnd01();
			X_new.speed_x[i] = r * X1.speed_x[i] + (1.0 - r) * X2.speed_x[i];
			r = rnd01();
			X_new.speed_y[i] = r * X1.speed_y[i] + (1.0 - r) * X2.speed_y[i];
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
		 << "Exe_time=" << last_generation.exe_time << endl << endl;

	output_file << generation_number << "   \t"
				<< std::setw( 11 ) << -last_generation.average_cost
				<< "   \t" << std::setw( 11 ) << -last_generation.best_total_cost << "\t"
				<< std::setw( 20 ) << best_genes.to_string() << "\n";
	output_file.flush();
}

static void print_help() {
	printf(
"options: \n \
	-h : this help \n \
	-i instance: instance of the problem to display \n \
	-l : load the best solution so far for this problem \n \
	-a : do all problem \
	-c : continue until solution is found\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	bool continue_optim = false;
	int c;
	bool continue_after_stall = false;

	while ((c = getopt(argc, argv, "cahli:")) != -1) switch (c) {
			case 'l':
				continue_optim = true;
				break;
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				break;
			case 'c':
				continue_after_stall = true;
				break;
			case 'h':
			default:
				print_help();
		}

	for (int i = 1; i < 4; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				gInstance = i * 1000 + j;
			}
			agent *base_agent = agent_factory(gInstance);

			//while (continue_after_stall && base_agent->)


			max_fuel = base_agent->vm->get_fuel() / 10.;
			if (continue_optim) {
				so_far_executed = parse_result(gInstance);
				base_agent->set_execution_map(&so_far_executed);
				base_agent->run();
				global_min_time = base_agent->last_validated_time;
			}
			executionT::iterator target = so_far_executed.begin();
			while (target != so_far_executed.end()) {
				if (target->first > global_min_time) {
					target = so_far_executed.erase(target);
				} else {
					target++;
				}
			}
			cout << "global min time = " << global_min_time << endl;
			if (gInstance / 1000 == 1) {
				max_time[0] = 1;
				nb_of_thrusts = 1;
			} else if (gInstance == 2001) {
				max_time[0] = 20000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2002) {
				max_time[0] = 10000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2003) {
				max_time[0] = 1000;
				nb_of_thrusts = 1;
			} else if (gInstance == 2004) {
				max_time[0] = 25000;
				nb_of_thrusts = 1;
			}else if (gInstance == 3001) {
				max_time[0] = 20000;
				nb_of_thrusts = 1;
			} else if (gInstance == 3002) {
				max_time[0] = 30000;
				nb_of_thrusts = 1;
			} else if (gInstance == 3003) {
				max_time[0] = 500;
				nb_of_thrusts = 1;
			} else if (gInstance == 3004) {
				max_time[0] = 10000;
				nb_of_thrusts = 1;
			} else {
				delete base_agent;
				base_agent = agent_factory(gInstance);
				max_fuel = base_agent->vm->get_fuel() / 2.;
				max_time[0] = 10000;
				nb_of_thrusts = 1;
				for (int k = 1; k < nb_of_thrusts; k ++)
					max_time[k] = 50000;
			}

			delete base_agent;

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
			ga_obj.population = 3000;
			ga_obj.generation_max = 5000;
			ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
			ga_obj.init_genes = init_genes;
			if (gInstance / 1000 == 1) {
				ga_obj.eval_solution = eval_solution1;
			} else if (gInstance / 1000 == 2) {
				ga_obj.eval_solution = eval_solution2;
			} else if (gInstance / 1000 == 3) {
				ga_obj.eval_solution = eval_solution3;
			} else if (gInstance / 1000 == 4) {
				ga_obj.eval_solution = eval_solution4;
			}
			ga_obj.mutate = mutate;
			ga_obj.crossover = crossover;
			ga_obj.SO_report_generation = SO_report_generation;
			ga_obj.crossover_fraction = 0.7;
			ga_obj.mutation_rate = 0.7;
			ga_obj.best_stall_max = 30;
			ga_obj.average_stall_max = 10;
			ga_obj.elite_count = 50;
			EA::StopReason reason = ga_obj.solve();
			cout << "The problem is optimized in " << timer.toc() << " seconds."
				<< endl;
			cout << "cause: " << ga_obj.stop_reason_to_string(reason) << endl;

			output_file.close();

			if (!do_all) {
				return 0;
			}
		}
	}
	return 0;
}