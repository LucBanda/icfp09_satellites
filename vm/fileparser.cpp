#include "fileparser.h"
#include "agent.h"

static std::string getLastLine(std::ifstream& in) {
	std::string line;
	while (in >> std::ws && std::getline(in, line))  // skip empty lines
		;

	return line;
}

executionT parse_result(int instance) {
	executionT map;
	string fileName = "./results/" + to_string(instance) + ".txt";
	std::ifstream file(fileName);
    if (!file.good()) return map;
	if (file) {
		std::string token;
		std::string line = getLastLine(file);
		if (line == "") {
			cout << "empty line at end of file" << endl;
			exit(-1);
		}
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
			delimiter = "map[";
			pos = line.find(delimiter);
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