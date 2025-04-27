#pragma once
#include "CMAPFSolver.h"
// #include "PBSNode.h"
// #include "common.h"

class CPPBest:public CMAPFSolver 
{
public:
    CPPBest(const BasicGraph &G, SingleAgentSolver &path_planner): CMAPFSolver(G, path_planner){};
    ~CPPBest(){};

    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    int num_order_sample = 300;
    // vector<int> best_order;
    string get_name() const {return "Continuous PPBest"; };

    bool run(const vector<CState>& starts,
            const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
            double _time_limit);

    // void save_results(const std::string &fileName, const std::string &instanceName) const;
    void save_results(const std::string &fileName, const std::string &instanceName) const {};

	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}

    void clear();
    bool prioritize_start = false;

private:
    std::clock_t start = 0;
    vector< CPath* > paths;
    vector<CPath*> best_paths;
    list< pair<int, CPath> > paths_list;

    bool find_path();
    // vector<int> select_best_order();

    double find_path_per_order(const std::vector<int>& total_order, bool fake_order, bool &order_has_fallback);

    void find_conflicts(list<Conflict>& conflicts, int a1, int a2);
    bool validate_solution();
    string vector_to_string(const vector<int>& v);
    void get_solution();

    bool all_elements_nullptr(const std::vector<CPath*>& vec) {
        return std::all_of(vec.begin(), vec.end(), [](CPath* ptr) {
            return ptr == nullptr;
        });
    }
};