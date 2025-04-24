#pragma once
#include "CPBSNode.h"
#include "CSIPP.h"
// #include "MAPFSolver.h"
#include <ctime>

// TODO: add topological sorting

class CPBS:
	// public MAPFSolver
{
public:
    bool lazyPriority;
    bool prioritize_start = true;

	 // runtime breakdown
    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    double runtime_get_higher_priority_agents = 0;
    double runtime_copy_priorities = 0;
    double runtime_detect_conflicts = 0;
    double runtime_copy_conflicts = 0;
    double runtime_choose_conflict = 0;
    double runtime_find_consistent_paths = 0;
    double runtime_find_replan_agents = 0;

    // double total_runtime = 0;

	CPBSNode* dummy_start = nullptr;
	CPBSNode* best_node;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;


	double min_f_val = 0;

	// Runs the algorithm until the problem is solved or time is exhausted 
    bool run(const vector<CState>& starts,
            const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
            int time_limit);


    CPBS(const BasicGraph& G, SingleAgentSolver& path_planner);
	~CPBS();

    void update_paths(CPBSNode* curr);
	// Save results
	void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const;
	void save_constraints_in_goal_node(const std::string &fileName) const;

	string get_name() const {return "CPBS"; }

	void clear();

	void setRT(bool use_cat, bool prioritize_start)
	{
		rt.use_cat = use_cat;
		rt.prioritize_start = prioritize_start;
	}

private:

    // MAPF solver stuff
    ContinuousReservationTable rt;
    SingleAgentSolver& path_planner;
    vector<CPath> solution;

    std::vector<CPath* > paths;
    // std::vector< Path* > occupied_cells;
    list<CPBSNode*> allNodes_table;
    list<CPBSNode*> dfs;

   //  vector<State> starts;
    // vector< vector<int> > goal_locations;

    std::clock_t start = 0;

	// double focal_w = 1.0;
    unordered_set<pair<int, int>> nogood;

    // SingleAgentICBS astar;


    bool generate_root_node();
    void push_node(CPBSNode* node);
    CPBSNode* pop_node();

    // high level search
	bool find_path(CPBSNode*  node, int ag);
    bool find_consistent_paths(CPBSNode* node, int a); // find paths consistent with priorities
    static void resolve_conflict(const Conflict& conflict, CPBSNode* n1, CPBSNode* n2);
	bool generate_child(CPBSNode* child, CPBSNode* curr);

	// conflicts
    void remove_conflicts(list<Conflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<Conflict>& old_conflicts, list<Conflict> & new_conflicts, int new_agent);
	void find_conflicts(list<Conflict> & conflicts, int a1, int a2);
    void find_conflicts(list<Conflict> & new_conflicts, int new_agent);
    void find_conflicts(list<Conflict> & new_conflicts);

	void choose_conflict(CPBSNode &parent);
	void copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent);
    void copy_conflicts(const list<Conflict>& conflicts,
                       list<Conflict>& copy, const vector<bool>& excluded_agents);

    double get_path_cost(const Path& path) const;
	
    // update information
    //void collect_constraints(const boost::unordered_set<int>& agents, int current_agent);
    void get_solution();

    void update_CAT(int ex_ag); // update conflict avoidance table
	void update_focal_list();
	inline void release_closed_list();
    void update_best_node(CPBSNode* node);

	// print and save
	void print_paths() const;
	void print_results() const;
	static void print_conflicts(const CPBSNode &curr) ;


	// validate
	bool validate_solution();
    static bool validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G) ;


    // tools
    static bool wait_at_start(const Path& path, int start_location, int timestep) ;
    void find_replan_agents(CPBSNode* node, const list<Conflict>& conflicts,
            unordered_set<int>& replan);
};

