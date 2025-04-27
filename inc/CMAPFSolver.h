#pragma once
#include "PBSNode.h"
#include "SIPP.h"
#include "CSIPP.h"
#include <ctime>

// Base class for MAPF solvers
class CMAPFSolver
{
public:
	int k_robust;
	int window;
	bool hold_endpoints;

	double runtime;
	double total_runtime;
	int screen;

	bool solution_found;
	double solution_cost;
	double avg_path_length;
	double min_sum_of_costs;
	vector<CPath> solution;

	int num_failed_order = 0;

	vector<int> current_order;
	vector<vector<int>> current_order_group;
	vector<int> best_order;

	vector<CPath> shortest_paths;
	vector<int> shortest_path_costs;

	// initial data
	ContinuousReservationTable initial_rt;
	vector<CPath> initial_paths;
	list<tuple<int, int, int>> initial_constraints;	   // <agent, location, timestep>:
													   // only this agent can stay in this location before this timestep.
	list<const CPath *> initial_soft_path_constraints; // the paths that all agents try to avoid
	unordered_map<int, double> travel_times;

	SingleAgentSolver &path_planner;
	// Runs the algorithm until the problem is solved or time is exhausted
	virtual bool run(const vector<CState> &starts,
					 const vector<vector<pair<int, int>>> &goal_locations, // an ordered list of pairs of <location, release time>
					 int time_limit) = 0;

	CMAPFSolver(
		const BasicGraph &G,
		SingleAgentSolver &path_planner) : solution_found(false),
										   solution_cost(-2),
										   avg_path_length(-1),
										   G(G),
										   path_planner(path_planner),
										   initial_rt(G),
										   rt(G) {}
	~CMAPFSolver() {};

	// CMAPFSolver(const BasicGraph &G, SingleAgentSolver &path_planner);
	// ~CMAPFSolver();

	// Save results
	virtual void save_results(const std::string &fileName, const std::string &instanceName) const = 0;
	virtual void save_search_tree(const std::string &fileName) const = 0;
	virtual void save_constraints_in_goal_node(const std::string &fileName) const = 0;
	virtual void clear() = 0;

	virtual string get_name() const = 0;

	const BasicGraph &G;
	vector<CState> starts;
	vector<vector<pair<int, int>>> goal_locations;
	int num_of_agents;
	int time_limit;

	// validate
	bool validate_solution();
	void print_solution() const;

	void find_shortest_paths();

protected:
	vector<vector<bool>> cat; // conflict avoidance table
	vector<unordered_set<pair<int, int>>> constraint_table;
	ContinuousReservationTable rt;
};
