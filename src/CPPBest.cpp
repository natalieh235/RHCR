#include "CPPBest.h"
#include <numeric> // For std::iota
#include <limits>  // For std::numeric_limits
#include <random>  // For random number generation
#include <iostream>
#include <algorithm>

void CPPBest::clear()
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

string CPPBest::vector_to_string(const vector<int> &v)
{
    string result;
    for (int num : v)
    {
        result += std::to_string(num) + ",";
    }
    return result;
}

bool CPPBest::find_path()
{
    double lowest_cost = INFINITY;
    bool best_order_has_fallback = false;
    best_paths.resize(num_of_agents, nullptr);
    paths.resize(num_of_agents, nullptr);

    if (!current_order_group.empty())
    {
        for (const auto &order : current_order_group)
        {
            rt.clear();
            // Reset the paths for a new evaluation.
            paths.assign(num_of_agents, nullptr);

            bool order_has_fallback = false; // local flag for this order
            double cost = find_path_per_order(order, false, order_has_fallback);
            runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;

            // Even if cost is INFINITY, we consider this order as a candidate.
            if (cost < lowest_cost)
            {
                lowest_cost = cost;
                best_order = order;
                best_order_has_fallback = order_has_fallback;

                // Deep copy the current paths into best_paths.
                for (size_t i = 0; i < paths.size(); ++i)
                {
                    if (paths[i] != nullptr)
                    {
                        if (best_paths[i] != nullptr)
                        {
                            delete best_paths[i];
                        }
                        best_paths[i] = new CPath(*paths[i]);
                    }
                }
            }
        }
        // Return false if the best order used any fallback; otherwise return true.
        return !best_order_has_fallback;
    }
    else
    {
        std::cout << "  No groups of order provided for CPPBest, generating own" << std::endl;
        for (int i = 0; i < num_order_sample; ++i)
        {
            vector<int> current_order(num_of_agents);
            std::iota(current_order.begin(), current_order.end(), 0);
            std::unordered_set<std::string> unique_orders; // To track generated orders

            // Random number generator for shuffling
            std::random_device rd;
            std::mt19937 g(rd());

            // do
            // {
                std::shuffle(current_order.begin(), current_order.end(), g);
            // } while (unique_orders.find(vector_to_string(current_order)) != unique_orders.end());

            // unique_orders.insert(vector_to_string(current_order));
            bool order_has_fallback = false;
            solution_cost = find_path_per_order(current_order, false, order_has_fallback);

            // std::cout << "Solution cost: " << solution_cost << std::endl;

            // if ()
            return !std::isinf(solution_cost);
        }
        // exit(-1);
    }
}

double CPPBest::find_path_per_order(const std::vector<int> &total_order, bool fake_order, bool &order_has_fallback)
{
    clock_t time = std::clock();
    double total_path_cost = 0;
    order_has_fallback = false; // flag to indicate fallback occurred
    paths_list.clear();
    int num_failed_call = 0;

    for (int i : total_order)
    {
        unordered_set<int> higher_priority_agent;
        if (!fake_order)
        {
            for (int j : total_order)
            {
                if (j == i)
                    break;
                higher_priority_agent.insert(j);
            }
        }

        CPath path;
        double path_cost;
        int start_location = starts[i].location;
        clock_t t = std::clock();
        rt.build(paths, initial_constraints, higher_priority_agent, i, start_location);
        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;

        t = std::clock();
        path = path_planner.run_continuous(G, starts[i], goal_locations[i], rt);
        runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        path_cost = path_planner.path_cost;
        rt.clear();

        if (path.empty())
        {
            // Fallback: use the pre-computed shortest path.
            paths[i] = &shortest_paths[i];
            num_failed_call++;
            num_failed_order++;
            order_has_fallback = true; // mark that this order had a fallback
            // add path cost of the shortest path
            total_path_cost += shortest_path_costs[i];
        }
        else
        {
            // std::cout << "found path for agent " << i << " with cost " << path_cost << std::endl;
            paths_list.emplace_back(i, path);
            paths[i] = &paths_list.back().second;
            total_path_cost += path_cost;
        }
    }

    return total_path_cost + 10 * num_failed_call;
}

bool CPPBest::validate_solution()
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
                    std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << t << std::endl;
                else
                    std::cout << "Agents " << a1 << " and " << a2 << " collides at (" << loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                return false;
            }
        }
    }
    return true;
}

void CPPBest::find_conflicts(list<Conflict> &conflicts, int a1, int a2)
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
        // int loc = paths[a1]->at(timestep).location;
        int loc = std::get<0>(paths[a1]->at(timestep)).location;
        for (int i = max(0, timestep - k_robust); i <= min(timestep + k_robust, size2 - 1); i++)
        {
            if (loc == std::get<0>(paths[a2]->at(i)).location && G.types[loc] != "Magic")
            // if (loc == paths[a2]->at(i).location && G.types[loc] != "Magic")
            {
                conflicts.emplace_back(a1, a2, loc, -1, min(i, timestep)); // k-robust vertex conflict
                // runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }
        if (k_robust == 0 && timestep < size1 - 1 && timestep < size2 - 1) // detect edge conflicts
        {
            int loc1 = std::get<0>(paths[a1]->at(timestep)).location;
            int loc2 = std::get<0>(paths[a2]->at(timestep)).location;
            // int loc1 = paths[a1]->at(timestep).location;
            // int loc2 = paths[a2]->at(timestep).location;
            if (loc1 != loc2 && loc1 == std::get<0>(paths[a2]->at(timestep + 1)).location && loc2 == std::get<0>(paths[a1]->at(timestep + 1)).location)
            {
                // if (loc1 != loc2 && loc1 == paths[a2]->at(timestep + 1).location
                //             && loc2 == paths[a1]->at(timestep + 1).location)
                // {
                conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); // edge conflict
                // runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }
    }

    // runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void CPPBest::get_solution()
{
    // std::cout << "  CPPBest::get_solution() called" << std::endl;
    // update_paths(best_node);
    solution.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        solution[k] = *paths[k];
    }

    // solution_cost  = 0;
    avg_path_length = 0;

    for (int k = 0; k < num_of_agents; k++)
    {
        avg_path_length += paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}

// bool CPPBest::run(const vector<CState>& starts,
//             const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
//             double _time_limit)
bool CPPBest::run(const vector<CState> &starts,
                  const vector<vector<pair<int, int>>> &goal_locations, // an ordered list of pairs of <location, release time>
                  int time_limit)
{
    clear();
    start = std::clock();

    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = starts.size();
    this->time_limit = time_limit;

    solution_cost = INFINITY;
    solution_found = false;

    // rt.num_of_agents = num_of_agents;
    // rt.map_size = G.size();
    // rt.k_robust = k_robust;
    // rt.window = window;
    // rt.hold_endpoints = hold_endpoints;
    path_planner.travel_times = travel_times;
    path_planner.hold_endpoints = hold_endpoints;
    path_planner.prioritize_start = prioritize_start;

    // best_paths.resize(num_of_agents,nullptr);

    find_shortest_paths();

    std::cout << "CPPBest: done finding shortest paths" << std::endl;

    solution_found = find_path();
    get_solution();

    if (!solution_found)
    {
        if (screen > 0)
        {
            std::cout << "CPPBest failed" << std::endl;
        }
        return false;
    }

    min_sum_of_costs = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        int start_loc = starts[i].location;
        for (const auto &goal : goal_locations[i])
        {
            min_sum_of_costs += G.heuristics.at(goal.first)[start_loc];
            start_loc = goal.first;
        }
    }
    // if (screen > 0) // 1 or 2
    // 	print_results();
    return solution_found;
}