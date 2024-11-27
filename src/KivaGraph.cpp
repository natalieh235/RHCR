#include "KivaGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool KivaGrid::load_map(std::string fname)
{
    std::size_t pos = fname.rfind('.');      // position of the file extension
    auto ext_name = fname.substr(pos, fname.size());     // get the name without extension
    if (ext_name == ".grid")
        return load_weighted_map(fname);
    else if (ext_name == ".map")
        return load_unweighted_map(fname);
    else
    {
        std::cout << "Map file name should end with either .grid or .map. " << std::endl;
        return false;
    }
}

bool KivaGrid::load_weighted_map(std::string fname)
{
	std::string line;
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open())
	{
		std::cout << "Map file " << fname << " does not exist. " << std::endl;
		return false;
	}

	std::cout << "*** Loading map ***" << std::endl;
	clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
	map_name = fname.substr(0, pos);     // get the name without extension
	getline(myfile, line); // skip the words "grid size"
	getline(myfile, line);
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	this->rows = atoi((*beg).c_str()); // read number of cols
	// std::cout << "Num rows: " << this->rows << std::endl;
	beg++;
	this->cols = atoi((*beg).c_str()); // read number of rows
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;

	getline(myfile, line); // skip the headers

	//read tyeps and edge weights
	this->types.resize(rows * cols);
	this->weights.resize(rows * cols);
	for (int i = 0; i < rows * cols; i++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++; // skip id
		this->types[i] = std::string(beg->c_str()); // read type
		beg++;
		if (types[i] == "Home")
			this->agent_home_locations.push_back(i);
		else if (types[i] == "Endpoint")
			this->endpoints.push_back(i);
		beg++; // skip x
		beg++; // skip y
		weights[i].resize(5);
		for (int j = 0; j < 5; j++) // read edge weights
		{
			if (std::string(beg->c_str()) == "inf")
				weights[i][j] = WEIGHT_MAX;
			else
				weights[i][j] = std::stod(beg->c_str());
			beg++;
		}
	}

	myfile.close();
	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Map size: " << rows << "x" << cols << " with ";
	cout << endpoints.size() << " endpoints and " <<
		agent_home_locations.size() << " home stations." << std::endl;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
	return true;
}


// load map
bool KivaGrid::load_unweighted_map(std::string fname)
{
	std::cout << "loading unweighted map" << std::endl;
    std::string line;
    std::ifstream myfile ((fname).c_str());
	if (!myfile.is_open())
    {
	    std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }
	
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline (myfile, line); 
	
	
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	rows = atoi((*beg).c_str()); // read number of rows
	beg++;
	cols = atoi((*beg).c_str()); // read number of cols
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;

	std::stringstream ss;
	getline(myfile, line);
	ss << line;
	int num_endpoints;
	ss >> num_endpoints;

	int agent_num;
	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> agent_num;

	ss.clear();
	getline(myfile, line);
	ss << line;
	int maxtime;
	ss >> maxtime;

	//this->agents.resize(agent_num);
	//endpoints.resize(num_endpoints + agent_num);
	types.resize(rows * cols);
	weights.resize(rows*cols);
	//DeliverGoal.resize(row*col, false);
	// read map
	//int ep = 0, ag = 0;
	for (int i = 0; i < rows; i++)
	{
		getline(myfile, line);
		for (int j = 0; j < cols; j++)
		{
			int id = cols * i + j;
			weights[id].resize(5, WEIGHT_MAX);
			if (line[j] == '@') // obstacle
			{
				types[id] = "Obstacle";
			}
			else if (line[j] == 'e') //endpoint
			{
				types[id] = "Endpoint";
				weights[id][4] = 1;
				endpoints.push_back(id);
			}
			else if (line[j] == 'r') //robot rest
			{
				types[id] = "Home";
				weights[id][4] = 1;
				agent_home_locations.push_back(id);
			}
			else
			{
				types[id] = "Travel";
				weights[id][4] = 1;
			}
		}
	}
	shuffle(agent_home_locations.begin(), agent_home_locations.end(), std::default_random_engine());
	for (int i = 0; i < cols * rows; i++)
	{
		if (types[i] == "Obstacle")
		{
			continue;
		}
		for (int dir = 0; dir < 4; dir++)
		{
			if (0 <= i + move[dir] && i + move[dir] < cols * rows && get_Manhattan_distance(i, i + move[dir]) <= 1 && types[i + move[dir]] != "Obstacle")
				weights[i][dir] = 1;
			else
				weights[i][dir] = WEIGHT_MAX;
		}
	}
	
	myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with ";
	cout << endpoints.size() << " endpoints and " <<
	agent_home_locations.size() << " home stations." << std::endl;		
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void KivaGrid::preprocessing(bool consider_rotation)
{
	std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	std::cout << "*** rotation ***" << this->consider_rotation << ::endl;
	std::string fname;
	if (consider_rotation)
		fname = map_name + "_rotation_heuristics_table.txt";
	else
		fname = map_name + "_heuristics_table.txt";
	std::ifstream myfile(fname.c_str());
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}
	if (!succ)
	{
		for (auto endpoint : endpoints)
		{
			// std::cout << "Preprocessing: computing heuristic for " << endpoint << std::endl;
			heuristics[endpoint] = compute_heuristics(endpoint);
			// for (auto h: heuristics[endpoint]) {
				// std::cout << "heuristic: " << h << std::endl;
			// }
		}
		for (auto home : agent_home_locations)
		{
			heuristics[home] = compute_heuristics(home);
		}
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

bool KivaGrid::valid_move(int loc, int dir) const
{
	std::vector<int> occupied_cells = get_occupied_cells(loc, dir);
	// std::cout << "Kiva grid valid move" << std::endl;
	// std::cout << "evaluating cells: " << std::endl;
	for (auto &cell: occupied_cells) {
		if (types[cell] == "Obstacle") {
			return false;
		}
	}

	return true;
}

std::vector<int> KivaGrid::get_occupied_cells(int location, int orientation) const
{
	// x, y is the center of the cell at index location in row-major
	double x = location % this->cols + 0.5; 
	double y = location / this->cols + 0.5;

	int grid_height = 1;
	int grid_width = 1;

	std::vector<int> occupied_cells;
	double orientation_radians = orientation * -M_PI / 2;

	std::vector<std::pair<double, double>> corners = {
		{ x - robot_width/2, y - robot_height/2 },                                   // Top-left
		{ x + robot_width/2, y - robot_height/2},                     // Top-right
		{ x - robot_width/2, y + robot_height/2 },      // Bottom-right
		{ x + robot_width/2, y + robot_height/2}                     // Bottom-left
	};

	// std::cout << "looking for occupied cells at: " << location << ", " << orientation << ", " << orientation_radians << std::endl;
	// std::cout << "x y is " << x << ", " << y << std::endl;
	// Rotate corners
	for (auto& corner : corners) {
		// std::cout << "old corner: " << corner.first << ", " << corner.second << std::endl;
		double x_rotated = x + (corner.first - x) * cos(orientation_radians) - (corner.second - y) * sin(orientation_radians);
		double y_rotated = y + (corner.first - x) * sin(orientation_radians) + (corner.second - y) * cos(orientation_radians);
		corner.first = x_rotated;
		corner.second = y_rotated;
		// std::cout << "new corner: " << corner.first << ", " << corner.second << std::endl;
	}

	double x_min = std::min({ corners[0].first, corners[1].first, corners[2].first, corners[3].first });
	double x_max = std::max({ corners[0].first, corners[1].first, corners[2].first, corners[3].first });
	double y_min = std::min({ corners[0].second, corners[1].second, corners[2].second, corners[3].second });
	double y_max = std::max({ corners[0].second, corners[1].second, corners[2].second, corners[3].second });

	// std::cout << "xmin " << x_min << "xmax " << x_max << "ymin " << y_min << "ymax " << y_max << std::endl;

	// Convert bounding box to occupied cells
    int start_col = static_cast<int>(std::floor(x_min));
    int end_col = static_cast<int>(std::ceil(x_max));
    int start_row = static_cast<int>(std::floor(y_min));
    int end_row = static_cast<int>(std::ceil(y_max));

	// std::cout << "startcol " << start_col << "endcol " << end_col << "start_row " << start_row << "endrow " << end_row << std::endl;

    // Check each cell in the bounding box
    for (int row = start_row; row < end_row; ++row) {
        for (int col = start_col; col < end_col; ++col) {
            // Check if the cell is within grid bounds
            if (row >= 0 && row < this->rows && col >= 0 && col < this->cols) {
				// std::cout << "found cell: " << row * this->cols + col << std::endl;
                occupied_cells.push_back(row * this->cols + col); // Add cell index to occupied cells
            }
        }
    }

	// std::cout << "occupied cells: " << occupied_cells << std::endl;
	// std::cout << "======" << std::endl;
	return occupied_cells;
}

// Helper function to check if a point is inside the rotated robot (as a polygon)
bool point_inside_robot(double x, double y, const std::vector<std::pair<double, double>>& corners) {
	int n = corners.size();
	bool inside = false;

	// Using the ray-casting algorithm to check if a point is inside a polygon
	for (int i = 0, j = n - 1; i < n; j = i++) {
		double xi = corners[i].first, yi = corners[i].second;
		double xj = corners[j].first, yj = corners[j].second;

		bool intersect = ((yi > y) != (yj > y)) &&
							(x < (xj - xi) * (y - yi) / (yj - yi) + xi);
		if (intersect) {
			inside = !inside;
		}
	}

	return inside;
}