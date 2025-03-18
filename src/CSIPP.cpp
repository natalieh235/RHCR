#include "CSIPP.h"
#include <cstdlib>


bool debug = true;
Path run(const BasicGraph& G, const State& start,
                const vector<pair<int, int> >& goal_location,
                ReservationTable& RT) 
{
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();

    double h_val = abs(G.get_xy(start.location).first - G.get_xy(goal_location[0].first).first) + 
        abs(G.get_xy(start.location).second - G.get_xy(goal_location[0].first).second);

	if (h_val > INT_MAX)
	{
		cout << "The start and goal locations are disconnected!" << endl;
		return Path();
	}

    Interval interval = rt.getFirstSafeInterval(start.location);

    // state, g_val, h_val, interval, parent, # conflicts, goal
    auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0, goal_location[0]);

    num_generated++;

    // reference to node in open_list (a fibonacci heap)
    node->open_handle = open_list.push(node);
    node->in_openlist = true;

    // unordered set of all nodes
    allNodes_table.insert(node);


    while (!open_list.empty()) {
        if ((double)(std::clock() - t) / CLOCKS_PER_SEC > 7) {
            cout << "CSIPP: TIME LIMIT EXCEEDED" << endl;
            releaseClosedListNodes();
            open_list.clear();
            focal_list.clear();
            return Path();
        }

        SIPPNode* curr = open_list.top(); open_list.pop();
        open_list.erase(curr->open_handle); // remove from open
        curr->in_openlist = false; // removed
        num_expanded++;

        // update goal id
        if (curr->state.location == goal_location[curr->goal_id].first &&
            curr->state.timestep >= goal_location[curr->goal_id].second) // reach the goal location after its release time
        {
            curr->goal_id++;

            // reset open, closed ,and focal list
            if (curr->goal_id == (int)goal_location.size())
            {
                // cout << "SIPP: reached goal location " << curr->goal_id - 1 << ", " << curr->state << endl;
                
                // TODO RETURN PATH
                releaseClosedListNodes();
                open_list.clear();
                // focal_list.clear();
                runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
                if (debug) {
                    cout << "           SIPP: returning path," << endl;
                    std::cout << "          Time elapsed: " << (std::clock() - t) * 1.0 / CLOCKS_PER_SEC << " seconds" << std::endl;
                }
                return path;
            }

            SIPPNode* new_node = new SIPPNode(curr->state, curr->g_val, curr->h_val, curr->interval, curr->parent, curr->conflicts, curr->goal);
            new_node->goal_id++;
            new_node->goal = goal_location[new_node->goal_id];
            open_list.clear();
            // focal_list.clear();

            new_node->open_handle = open_list.push(new_node);
            new_node->in_openlist = true;

            // unordered set of all nodes
            allNodes_table.insert(new_node);
        }

        for (int o = 0; o < 4; 0++) {
            
        }
    }
}

inline void CSIPP::releaseClosedListNodes()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}