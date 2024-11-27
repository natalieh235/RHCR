#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(const BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    // std::cout << "computing h function for " << goal_id << std::endl;
    // std::cout << goal_location[goal_id].first << std::endl;
    // for (auto &g: goal_location) {
    //     std::cout << g.first << ", " << g.second << std::endl;
    // }

    // for (auto &h: G.heuristics) {
    //     std::cout << h.first << std::endl;
    // }
    double h = G.heuristics.at(goal_location[goal_id].first)[curr];
    // std::cout << h << std::endl;
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        h += G.heuristics.at(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
        goal_id++;
    }
    return h;
}
