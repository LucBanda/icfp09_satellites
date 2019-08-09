#include <iomanip>
#include "agent.h"
#include "bin.h"
#include "common.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"

struct main_status {
	vm_state* vm;
	renderer* render;
	agent* ag;
};

bool idle(void* user_param) {
	struct main_status* status = (struct main_status*)user_param;
	vm_state* vm = status->ag->vm;
	agent* ag = status->ag;

	ag->step();

	if (vm->get_score()) {
		cout << "score = " << vm->get_score() << " fuel = " << vm->get_fuel()
			 << endl;
		return true;
	}
	return false;
}

std::string getLastLine(std::ifstream& in) {
	std::string line;
	while (in >> std::ws && std::getline(in, line))  // skip empty lines
		;

	return line;
}

executionT parse_result(string fileName) {
	executionT map;
	std::ifstream file(fileName);

	if (file) {
		std::string token;
		std::string line = getLastLine(file);
		cout << line << endl;
		// remove beginning
		std::string delimiter = "{ ";
		size_t pos = line.find(delimiter);
		line.erase(0, pos + delimiter.length());

		// remove end
		delimiter = " }";
		pos = line.find(delimiter);
		// token = line.substr(0, pos);
		line.erase(pos, line.npos);

		while (line.size() != 0) {
			std::string delimiter = "map[";
			size_t pos = line.find(delimiter);
			line.erase(0, pos + delimiter.length());

			// parse first time
			delimiter = "] = Complex(";
			pos = line.find(delimiter);
			token = line.substr(0, pos);
			line.erase(0, pos + delimiter.length());
			int time = stoi(token);

			// parse x1
			delimiter = ", ";
			pos = line.find(delimiter);
			token = line.substr(0, pos);
			line.erase(0, pos + delimiter.length());
			double x = stold(token);

			// parse y1
			delimiter = "); ";
			pos = line.find(delimiter);
			token = line.substr(0, pos);
			line.erase(0, pos + delimiter.length());
			double y = stod(token);
			map[time] = Complex(x, y);

			std::cout << "[" << time << "] = (" << setprecision(10) << x << ", "
				<< setprecision(10) << y << ")\n";
		}

	} else
		std::cout << "Unable to open file.\n";
	return map;
}

void print_help() {
	printf(
"options: \n \
	-h : this help \n \
	-i instance: instance of the problem to display \n \
	-l : load the best solution so far for this problem \n \
	-a : do all problem\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	vm_state* vm = NULL;
	struct main_status status;
	renderer* render;
	int c;
	bool load_result = false;
	uint32_t instance = 0;
	executionT map;

	while ((c = getopt(argc, argv, "ahli:")) != -1) switch (c) {
			case 'l':
				load_result = true;
				break;
			case 'i':
				instance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				break;
			case 'h':
			default:
				print_help();
		}

	if (do_all && instance != 0) {
		print_help();
		return -1;
	}

	for (int i = 1; i < 5; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				instance = i * 1000 + j;
			}

			agent* ag;
			render = new renderer();
			if (instance / 1000 == 1) {
				ag = new agent1(instance);
			} else if (instance / 1000 == 2) {
				ag = new agent2(instance);
			} else if (instance / 1000 == 3) {
				ag = new agent3(instance);
			} else {
				ag = new agent4(instance);
			}

			vm = ag->vm;
			if (load_result) {
				map = parse_result("./results/" + to_string(instance) + ".txt");
				ag->set_execution_map(&map);
			}

			status.vm = vm;
			status.render = render;
			render->set_vm(vm);
			status.ag = ag;

			render->idle = &idle;
			render->idle_param = &status;
			render->mainLoop(NULL);

			delete render;
			delete ag;

			if (!do_all) return 0;
		}
	}
	return 0;
}
