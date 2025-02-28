#include "PP.h"
#include "PathTable.h"
#include <numeric>   // For std::iota
#include <limits>    // For std::numeric_limits
#include <random>    // For random number generation
#include <iostream>




PP::PP(const BasicGraph &G, SingleAgentSolver &path_planner) : MAPFSolver(G, path_planner){};

PP::~PP(){};

void PP::clear()
{
    starts.clear();
    goal_locations.clear();
    runtime_rt = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    paths.clear();
    best_paths.clear();
    rt.clear();
}

string PP::vector_to_string(const vector<int>& v) {
    string result;
    for (int num : v) {
        result += std::to_string(num) + ",";
    }
    return result;
}

bool PP::find_path()
{
    double lowest_cost = INFINITY;
    vector<int> current_order(num_of_agents);
    std::iota(current_order.begin(), current_order.end(), 0);
    std::unordered_set<std::string> unique_orders; // To track generated orders
    best_paths.resize(num_of_agents, nullptr);
    // Random number generator for shuffling
    std::random_device rd;
    std::mt19937 g(rd());  

    for (int i = 0; i < num_order_sample; ++i) {
        // Generate a unique total order
        std::cout << "======================================================" << std::endl;
        std::cout << "GENERATING FOR SAMPLE: " << i << std::endl;
        
        rt.clear();
        // paths.clear();
        paths.resize(num_of_agents, nullptr);

        do {
            std::shuffle(current_order.begin(), current_order.end(), g);
        } while (unique_orders.find(vector_to_string(current_order)) != unique_orders.end());

        // Store the new unique order
        unique_orders.insert(vector_to_string(current_order));
        solution_cost = find_path_per_order(current_order, false);
        runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
        // Evaluate the score for this order
 
        if (std::isinf(solution_cost))
        {
            continue;
        }

        // Check if this is the lowest score so far
        if (solution_cost < lowest_cost) {
            lowest_cost = solution_cost;
            /* In the provided code snippet, `best_order` is a member variable of the `PP` class. It is
            used to store the current best order of agents for finding a path. During the execution
            of the `find_path` function, the algorithm iterates through different orders of agents
            and evaluates the cost of the path for each order. */
            // best_order = current_order;
            // best_paths = paths;// Store the best order
            for (size_t i = 0; i < paths.size(); ++i) 
            {
                if (paths[i] != nullptr) 
                {
                    // Create a new Path object and copy the content of the original Path
                    best_paths[i] = new Path(*paths[i]);  // Using the copy constructor of Path
                }
            }
        }
    }
    

    if (all_elements_nullptr(best_paths))
    {
        paths.resize(num_of_agents, nullptr);
        find_path_per_order(current_order, true);
        for (size_t i = 0; i < paths.size(); ++i) 
        {
            if (paths[i] != nullptr) 
            {
                // Create a new Path object and copy the content of the original Path
                best_paths[i] = new Path(*paths[i]);  // Using the copy constructor of Path
            }
        }
        
        return false;
    }
    else
    {
        return true;
    }

    
}



double PP::find_path_per_order(const std::vector<int> &total_order, bool fake_order)
{
    // std::cout << "Finding path for order: " << vector_to_string(total_order) << std::endl;
    // std::cout << "fake order: " << fake_order << std::endl;
    clock_t time = std::clock();
    double total_path_cost = 0 ;
    paths_list.clear();

    for (int i:total_order) {   
        unordered_set<int> higher_priority_agents;
        if (!fake_order) {
            for (int j:total_order) {
                if (j==i) {
                    break;
                }
                // std::cout << "inserting " << j << " into higher_priority_agents" << std::endl;
                higher_priority_agents.insert(j);
            }
        }

        
        Path path;
        double path_cost;
        int start_location = starts[i].location;
        clock_t t = std::clock();

        rt.build(paths, initial_constraints, higher_priority_agents, i, start_location);

        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        t = std::clock();
        path = path_planner.run(G, starts[i], goal_locations[i], rt);
        runtime_plan_paths += (double)(std::clock()-t) / CLOCKS_PER_SEC;
        path_cost = path_planner.path_cost;
        rt.clear();

        if (path.empty())
        {
            // std::cout <<"low level faild"<< std::endl;
            total_path_cost = INFINITY;
            return total_path_cost;
        }
        // dummy_start -> paths.emplace_back(i, path);
        paths_list.emplace_back(i, path);
        paths[i] = &paths_list.back().second;
        // paths[i] = &dummy_start->paths.back().second;
        // dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
        // dummy_start->g_val += path_cost;
        total_path_cost += path_cost;
        
    }

    std::cout << "done finding path for order " << vector_to_string(total_order) << std::endl;
    if (! validate_solution())
    {
        // std::cout <<"Current order invalid"<< std::endl;;
        total_path_cost = INFINITY;
        return total_path_cost;
    }
    if (screen == 2)
    {
        double runtime = (double)(std::clock() - time) / CLOCKS_PER_SEC;
        std::cout << "Current order valid(" << runtime << "s)" << std::endl;
    }
    return total_path_cost;
}


bool PP::validate_solution()
{
    list<Conflict> conflict;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
            find_conflicts(conflict, a1, a2);
            if (!conflict.empty())
            {
                int a1_, a2_, loc1, loc2, t;
                std::tie(a1_, a2_, loc1, loc2, t) = conflict.front();
                if (loc2 < 0)
                    std::cout << "Agents "  << a1 << " and " << a2 << " collides at " << loc1 <<
                    " at timestep " << t << std::endl;
                else
                    std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                              loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                return false;
            }
		}
	}
	return true;
}

void PP::find_conflicts(list<Conflict>& conflicts, int a1, int a2)
{
    clock_t t = clock();
    if (paths[a1] == nullptr || paths[a2] == nullptr)
        return;
	
		// TODO: add k-robust

    int size1 = min(window + 1, (int)paths[a1]->size());
    int size2 = min(window + 1, (int)paths[a2]->size());
    for (int timestep = 0; timestep < size1; timestep++)
    {
        if (size2 <= timestep - k_robust)
            break;
        int loc = paths[a1]->at(timestep).state.location;
        for (int i = max(0, timestep - k_robust); i <= min(timestep + k_robust, size2 - 1); i++)
        {
            if (loc == paths[a2]->at(i).state.location && G.types[loc] != "Magic")
            {
                conflicts.emplace_back(a1, a2, loc, -1, min(i, timestep)); // k-robust vertex conflict
                // runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }
        if (k_robust == 0 && timestep < size1 - 1 && timestep < size2 - 1) // detect edge conflicts
        {
            int loc1 = paths[a1]->at(timestep).state.location;
            int loc2 = paths[a2]->at(timestep).state.location;
            if (loc1 != loc2 && loc1 == paths[a2]->at(timestep + 1).state.location
                        && loc2 == paths[a1]->at(timestep + 1).state.location)
            {
                conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); // edge conflict
                // runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }

    }
    
	// runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PP::get_solution()
{
    // update_paths(best_node);
    solution.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        solution[k] = *best_paths[k];
    }

    //solution_cost  = 0;
    avg_path_length = 0;

    for (int k = 0; k < num_of_agents; k++)
    {
        avg_path_length += best_paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}


bool PP::run(const vector<State>& starts,
            const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
            int time_limit)
{

    std::cout << "PP called" << std::endl;
    clear();
    start = std::clock();

    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = starts.size();
    this->time_limit = time_limit;

    solution_cost = INFINITY;
    solution_found = false;

    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
	rt.hold_endpoints = hold_endpoints;
    path_planner.travel_times = travel_times;
	path_planner.hold_endpoints = hold_endpoints;
	path_planner.prioritize_start = prioritize_start;

    // best_paths.resize(num_of_agents,nullptr);


    solution_found = find_path();
    get_solution();
    
    if (!solution_found)
    {
        std::cout << "PP failed" << std::endl;
        return false;
    }
    
    min_sum_of_costs = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        int start_loc = starts[i].location;
        for (const auto& goal : goal_locations[i])
        {
            min_sum_of_costs += G.heuristics.at(goal.first)[start_loc];
            start_loc = goal.first;
        }
    }
	// if (screen > 0) // 1 or 2
	// 	print_results();
	return solution_found;
}