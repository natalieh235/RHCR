#include "CBasicSystem.h"
#include <stdlib.h>
#include <boost/tokenizer.hpp>


CBasicSystem::CBasicSystem(const BasicGraph& G, CMAPFSolver& solver): G(G), solver(solver), num_of_tasks(0) {}

CBasicSystem::~CBasicSystem() {}


bool CBasicSystem::load_locations()
{
	string fname = G.map_name + "_rotation=" + std::to_string(consider_rotation) +
		"_" + std::to_string(num_of_drives) + ".agents";
    std::ifstream myfile (fname.c_str());
    if (!myfile.is_open())
		return false;

    string line;
    getline (myfile,line);
    boost::char_separator<char> sep(",");

    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The agent file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline (myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();
        // starts
        int start_loc = atoi((*beg).c_str());
        beg++;
        int start_orient = atoi((*beg).c_str());
        beg++;
        starts[k] = CState(start_loc, 0, start_orient);
        paths[k].emplace_back(std::make_tuple(starts[k], 0.0));
        finished_tasks[k].push_back(std::make_pair(start_loc, 0));
        // goals
        int goal = atoi((*beg).c_str());
        goal_locations[k].emplace_back(goal, 0);
    }
    myfile.close();
	return true;
}


void CBasicSystem::update_start_locations()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        // starts[k] = State(paths[k][timestep].location, 0, paths[k][timestep].orientation, paths[k][timestep].velocity);
        // starts[k] = CState(paths[k][timestep].state.location, 0, paths[k][timestep].state.orientation, paths[k][timestep].state.velocity);

        // paths are variable length, so use the last entry instead of timestep
        if (!paths[k].empty()) {
            starts[k] = std::get<0>(paths[k].back());
        }
    }
}


void CBasicSystem::update_paths(const std::vector<CPath*>& MAPF_paths, int max_timestep = INT_MAX)
{
    if (MAPF_paths.empty()) {
        return;
    }

    for (int k = 0; k < num_of_drives; k++)
    {

        if (MAPF_paths[k]->empty()) {
            continue;
        }

        int old_length = (int)paths[k].size();
        paths[k].resize(paths[k].size() + (int)MAPF_paths[k]->size());
        for (int t = 0; t < (int)MAPF_paths[k]->size(); t++)
        {
            // paths[k][timestep + t] = MAPF_paths[k]->at(t);
            paths[k][old_length + t] = (*MAPF_paths[k])[t];
            // paths[k][timestep + t].state.timestep = timestep + t;
        }
    }
}

void CBasicSystem::update_paths(const std::vector<CPath>& MAPF_paths, int max_timestep = INT_MAX)
{
    std::cout << "updating paths" << std::endl;
    // std::cout << "paths: " << MAPF_paths.size() << std::endl;
    // std::cout << "max timestep: " << max_timestep << std::endl;

    if (MAPF_paths.empty()) {
        return;
    }
    
    for (int k = 0; k < num_of_drives; k++)
    {
        // check for empty path
        if (MAPF_paths[k].empty())
            continue;

        int old_length = (int)paths[k].size();
        paths[k].resize(paths[k].size() + (int)MAPF_paths[k].size());
        for (int t = 0; t < (int)MAPF_paths[k].size(); t++)
        {
            paths[k][old_length + t] = MAPF_paths[k][t];
        }
    }
}

// void CBasicSystem::update_initial_paths(vector<CPath>& initial_paths) const
// {
//     initial_paths.clear();
//     initial_paths.resize(num_of_drives);
//     for (int k = 0; k < num_of_drives; k++)
//     {
//         // check whether the path traverse every goal locations
//         int i = (int)goal_locations[k].size() - 1;
//         int j = (int)paths[k].size() - 1;
//         while (i >= 0 && j >= 0)
//         {
//             while (j >= 0 && paths[k][j].state.location != goal_locations[k][i].first &&
// 			paths[k][j].state.timestep >= goal_locations[k][i].second)
//                 j--;
//             i--;
//         }
//         if (j < 0)
//             continue;
          
//         if ((int) paths[k].size() <= timestep + planning_window)
//             continue;

//         initial_paths[k].resize(paths[k].size() - timestep);
//         for (int t = 0; t < (int)initial_paths[k].size(); t++)
//         {
//             initial_paths[k][t] = paths[k][timestep + t];
//             initial_paths[k][t].state.timestep = t;
//         }
//     }
// }

// void CBasicSystem::update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const
// {
//     initial_constraints.clear();
//     for (int k = 0; k < num_of_drives; k++)
//     {
//         int prev_location = -1;
//         for (int t = timestep; t > max(0, timestep - k_robust); t--)
//         {
//             int curr_location = paths[k][t].state.location;
//             if (curr_location < 0)
//                 continue;
//             else if (curr_location != prev_location)
//             {
//                 initial_constraints.emplace_back(k, curr_location, t + k_robust + 1 - timestep);
//                 prev_location = curr_location;
//             }
//         }
//     }
// }


// bool CBasicSystem::check_collisions(const vector<CPath>& input_paths) const
// {
// 	for (int a1 = 0; a1 < (int)input_paths.size(); a1++)
// 	{
// 		for (int a2 = a1 + 1; a2 < (int)input_paths.size(); a2++)
// 		{
// 			// TODO: add k-robust
// 			size_t min_path_length = input_paths[a1].size() < input_paths[a2].size() ? input_paths[a1].size() : input_paths[a2].size();
// 			for (size_t timestep = 0; timestep < min_path_length; timestep++)
// 			{
// 				int loc1 = input_paths[a1].at(timestep).state.location;
// 				int loc2 = input_paths[a2].at(timestep).state.location;
// 				if (loc1 == loc2)
// 					return true;
// 				else if (timestep < min_path_length - 1
// 					&& loc1 == input_paths[a2].at(timestep + 1).state.location
// 					&& loc2 == input_paths[a1].at(timestep + 1).state.location)
// 					return true;
// 			}
// 			if ((hold_endpoints || useDummyPaths) && input_paths[a1].size() != input_paths[a2].size())
// 			{
// 				int a1_ = input_paths[a1].size() < input_paths[a2].size() ? a1 : a2;
// 				int a2_ = input_paths[a1].size() < input_paths[a2].size() ? a2 : a1;
// 				int loc1 = input_paths[a1_].back().state.location;
// 				for (size_t timestep = min_path_length; timestep < input_paths[a2_].size(); timestep++)
// 				{
// 					int loc2 = input_paths[a2_].at(timestep).state.location;
// 					if (loc1 == loc2)
// 						return true;
// 				}
// 			}
// 		}
// 	}
// 	return false;
// }

// bool CBasicSystem::congested() const
// {
// 	if (simulation_window <= 1)
// 		return false;
//     int wait_agents = 0;
//     for (const auto& path : paths)
//     {
//         int t = 0;
//         while (t < simulation_window && path[timestep].state.location == path[timestep + t].state.location &&
//                 path[timestep].state.orientation == path[timestep + t].state.orientation)
//             t++;
//         if (t == simulation_window)
//             wait_agents++;
//     }
//     return wait_agents > num_of_drives / 2;  // more than half of drives didn't make progress
// }

// move all agents from start_timestep to end_timestep
// return a list of finished tasks
list<tuple<int, int, int>> CBasicSystem::move()
{
    int start_timestep = timestep;
    int end_timestep = timestep + simulation_window;

    std::cout << "CBasicSystem: move, " << start_timestep << ", " << end_timestep << std::endl;
	list<tuple<int, int, int>> cur_finished_tasks; // <agent_id, location, timestep>

    // const std::vector<CPath>& MAPF_paths

    for (int k = 0; k < num_of_drives; k++) {
        if (solver.solution[k].empty()) {
            continue;
        }

        for (auto p : solver.solution[k]) {
            CState curr = std::get<0>(p);
            if (!goal_locations[k].empty() && 
                curr.timestep >= start_timestep && 
                curr.timestep < end_timestep && 
                curr.location == goal_locations[k].front().first &&
                curr.timestep >= goal_locations[k].front().second
                ) {
                goal_locations[k].erase(goal_locations[k].begin());
                cur_finished_tasks.emplace_back(k, curr.location, curr.timestep);
            }
        }
    }
    
    // for (int t = start_timestep; t <= end_timestep; t++)
    // {
    //     for (int k = 0; k < num_of_drives; k++)
    //     {
    //         State curr = paths[k][t].state;

    //         if (!goal_locations[k].empty() && 
	// 			curr.location == goal_locations[k].front().first &&
	// 			curr.timestep >= goal_locations[k].front().second) // the agent finish its current task
    //         {
    //             goal_locations[k].erase(goal_locations[k].begin());
	// 			finished_tasks.emplace_back(k, curr.location, t);
    //         }

    //     }
    // }
    return cur_finished_tasks;
}


void CBasicSystem::add_partial_priorities(const vector<CPath>& initial_paths, PriorityGraph& initial_priorities) const
{
    list<int> low_priorities;
    list<int> high_priorities;
    for (int k = 0; k < num_of_drives; k++)
    {
        if (initial_paths[k].empty())
            low_priorities.push_back(k);
        else
            high_priorities.push_back(k);
    }

    for (auto low : low_priorities)
    {
        for (auto high : high_priorities)
            initial_priorities.add(low, high);
    }
}

void CBasicSystem::save_results()
{
    std::cout << "saving results" << outfile << ", time " << timestep << std::endl;
	if (screen)
		std::cout << "*** Saving " << seed << " ***" << std::endl;
    clock_t t = std::clock();
    std::ofstream output;

    // settings
    output.open(outfile + "/config.txt", std::ios::out);
    output << "map: " << G.map_name << std::endl
        << "#drives: " << num_of_drives << std::endl
        << "seed: " << seed << std::endl
        << "solver: " << solver.get_name() << std::endl
        << "time_limit: " << time_limit << std::endl
        << "simulation_window: " << simulation_window << std::endl
        << "planning_window: " << planning_window << std::endl
        << "simulation_time: " << simulation_time << std::endl
        << "robust: " << k_robust << std::endl
        << "rotate: " << consider_rotation << std::endl
        << "use_dummy_paths: " << useDummyPaths << std::endl
        << "hold_endpoints: " << hold_endpoints << std::endl
        << "total runtime: " << solver.total_runtime << std::endl;

    output.close();

    for (auto path: paths) {
        std::cout << "results path: " << path <<  " , " << timestep << std::endl;
    }

    // tasks
    output.open(outfile + "/tasks.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        int prev = finished_tasks[k].front().first;
        for (auto task : finished_tasks[k])
        {
            output << task.first << "," << task.second << ",";
            if (task.second != 0)
                output << G.heuristics.at(task.first)[prev];
            output << ";";
            prev = task.first;
        }
        for (auto goal : goal_locations[k]) // tasks that have not been finished yet
        {
            output << goal.first << ",-1,;";
        }
        output << std::endl;
    }
    output.close();

    std::cout << "writing to paths file " << timestep << std::endl;
    // paths
    output.open(outfile + "/paths.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        output << "Agent-" << k << ";";
        for (auto p : paths[k])
        {
            if (std::get<0>(p).timestep <= timestep)
                output << std::get<0>(p) << ";";
        }
        output << std::endl;
    }
    output.close();
    saving_time = (std::clock() - t) / CLOCKS_PER_SEC;
	if (screen)
		std::cout << "Done! (" << saving_time << " s)" << std::endl;
}


// void CBasicSystem::update_travel_times(unordered_map<int, double>& travel_times)
// {
//     if (travel_time_window <= 0)
//         return;

//     travel_times.clear();
//     unordered_map<int, int> count;

//     int t_min = max(0, timestep - travel_time_window);
//     if (t_min >= timestep)
//         return;
//     for (auto path : paths)
//     {
//         int t = timestep;
//         while (t >= t_min)
//         {
//             int loc = path[t].state.location;
//             int dir = path[t].state.orientation;
//             int wait = 0;
//             while (t > wait && path[t - 1 - wait].state.location == loc && path[t - 1 - wait].state.orientation == dir)
//                 wait++;
//             auto it = travel_times.find(loc);
//             if (it == travel_times.end())
//             {
//                 travel_times[loc] = wait;
//                 count[loc] = 1;
//             }
//             else
//             {
//                 travel_times[loc] += wait;
//                 count[loc] += 1;
//             }
//             t = t - 1 - wait;
//         }
//     }

//     for (auto it : count)
//     {
//         if (it.second > 1)
//         {
//             travel_times[it.first] /= it.second;
//         }
//     }
// }


void CBasicSystem::solve()
{
    // std::cout << "basic system solve called" << std::endl;
    // LRA_called = false;
	// LRAStar lra(G, solver.path_planner);
	// lra.simulation_window = simulation_window;
	// lra.k_robust = k_robust;
	solver.clear();

    std::cout << "CBasicSystem.cpp: starting solve" << std::endl;
    bool sol = solver.run(starts, goal_locations, time_limit);
    std::cout << "high level solver returned " << sol << std::endl;
    if (sol)
    {
        if (log)
            solver.save_constraints_in_goal_node(outfile + "/goal_nodes/" + std::to_string(timestep) + ".gv");
        update_paths(solver.solution);
    }
    // else
    // {
    //     lra.resolve_conflicts(solver.solution);
    //     update_paths(lra.solution);
    // }
    if (log)
        solver.save_search_tree(outfile + "/search_trees/" + std::to_string(timestep) + ".gv");

    // }
    // solver.save_results(outfile + "/solver.csv", "Time: " + std::to_string(timestep) + "," 
    //                                 + "Num Drives: " + std::to_string(num_of_drives) + ", Seed: " + std::to_string(seed));
}

void CBasicSystem::initialize_solvers()
{
	solver.k_robust = k_robust;
	solver.window = planning_window;
	solver.hold_endpoints = hold_endpoints || useDummyPaths;
	solver.screen = screen;

	// solver.initial_rt.hold_endpoints = true;
	// solver.initial_rt.map_size = G.size();
	// solver.initial_rt.k_robust = k_robust;
	// solver.initial_rt.window = INT_MAX;
}

// bool CBasicSystem::load_records()
// {
// 	boost::char_separator<char> sep1(";");
// 	boost::char_separator<char> sep2(",");
//     boost::char_separator<char> sep3(":");
// 	string line;

// 	// load paths
// 	std::ifstream myfile(outfile + "/paths.txt");

// 	if (!myfile.is_open())
// 			return false;

// 	timestep = INT_MAX;
// 	getline(myfile, line);
// 	if (atoi(line.c_str()) != num_of_drives)
// 	{
// 		cout << "The path file does not match the settings." << endl;
// 		exit(-1);
// 	}
// 	for (int k = 0; k < num_of_drives; k++)
// 	{
//         // std::cout << "got hereee" << std::endl;
// 		getline(myfile, line);
// 		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
 
// 		for (auto it = std::next(tok1.begin()); it != tok1.end(); ++it)
// 		{
//             auto task = *it;
//             // std::cout << "print: " << task << std::endl;
// 			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
// 			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
//             // std::cout << "cur: " << *beg << std::endl;
// 			int loc = atoi((*beg).c_str());
// 			beg++;
// 			int orientation = atoi((*beg).c_str());
// 			beg++;
// 			double time = atoi((*beg).c_str());
//             std::cout << loc <<" , "  <<  time << ", " << orientation << std::endl;
// 			// paths[k].emplace_back(PathStep(State(loc, time, orientation)));
//             // paths[k].emplace
// 		}
// 		timestep = std::min(static_cast<double>(timestep), std::get<0>(paths[k].back()).timestep);
// 	}
// 	myfile.close();

// 	// pick the timestep
// 	timestep = int((timestep - 1) / simulation_window) * simulation_window; //int((timestep - 1) / simulation_window) * simulation_window;

// 	// load tasks
// 	myfile.open(outfile + "/tasks.txt");
// 	if (!myfile.is_open())
// 		return false;

// 	getline(myfile, line);
// 	if (atoi(line.c_str()) != num_of_drives)
// 	{
// 		cout << "The task file does not match the settings." << endl;
// 		exit(-1);
// 	}
// 	for (int k = 0; k < num_of_drives; k++)
// 	{
// 		getline(myfile, line);
// 		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
// 		for (auto task : tok1)
// 		{
// 			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
// 			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
// 			int loc = atoi((*beg).c_str());
// 			beg++;
// 			int time = atoi((*beg).c_str());
// 			if (time >= 0 && time <= timestep)
// 			{
// 				finished_tasks[k].emplace_back(loc, time);
// 				timestep = max(timestep, time);
// 			}
// 			else
// 			{
// 				goal_locations[k].emplace_back(loc, 0);
// 			}
// 		}
// 	}
// 	myfile.close();
// 	return true;
// }

