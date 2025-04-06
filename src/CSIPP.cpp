#include "CSIPP.h"
#include <cstdlib>


CPath CSIPP::updatePath(const CSIPPNode* goal)
{
    CPath path;
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    const CSIPPNode* curr = goal;
    while (true)
    {
        if (curr->parent == nullptr) // root node
        {
            path.emplace_back(curr->state);
            break;
        }
        else {
            const CSIPPNode* prev = curr->parent;
            path.emplace_back(curr->state); // move to current location
            curr = prev;
        }
    }
    return path;
}

std::tuple<bool, CPath> CSIPP::update_goals(CSIPPNode* curr, const vector<pair<int, int> >& goal_locations) {
    // update goal id

    cout << "updating goals, " << curr->state.location << " , " << curr->goal_id << endl;
    if (curr->state.location == goal_locations[curr->goal_id].first &&
        curr->state.timestep >= goal_locations[curr->goal_id].second) // reach the goal location after its release time
    {
        curr->goal_id++;

        // reset open, closed ,and focal list
        if (curr->goal_id == (int)goal_locations.size())
        {
            // cout << "SIPP: reached goal location " << curr->goal_id - 1 << ", " << curr->state << endl;
            // return path;
            // return {true, Path()};
            return {true, updatePath(curr)};
        }

        CSIPPNode* new_node = new CSIPPNode(curr->state, curr->g_val, curr->h_val, curr->interval, curr->parent, curr->conflicts, curr->goal);
        new_node->goal_id++;
        new_node->goal = goal_locations[new_node->goal_id];
        open_list.clear();

        new_node->open_handle = open_list.push(new_node);
        new_node->in_openlist = true;

        // unordered set of all nodes
        allNodes_table.insert(new_node);
    }

    return {false, CPath()};
}

CPath CSIPP::run_continuous(const BasicGraph& G, const CState& start,
                const vector<pair<int, int> >& goal_locations,
                ReservationTable& rt) 
{

    cout << "running csipp" << endl;
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();


    double h_val = compute_h_value(G, start.location, 0, goal_locations);

	if (h_val > INT_MAX)
	{
		cout << "The start and goal locations are disconnected!" << endl;
		return CPath();
	}

    std::cout << "goal: " << goal_locations[0].first << ", " << goal_locations[0].second << std::endl;

    // state, g_val, h_val, interval, parent, # conflicts, goal
    auto node = new CSIPPNode(start, 0, h_val, rt.getFirstSafeInterval(start.location), nullptr, 0, goal_locations[0]);

    cout << "start node: " << node->state << endl;

    num_generated++;

    // reference to node in open_list (a fibonacci heap)
    node->open_handle = open_list.push(node);
    node->in_openlist = true;

    // unordered set of all nodes
    allNodes_table.insert(node);

    std::tuple<bool, CPath> res = {false, CPath()};

    while (!open_list.empty()) {
        if ((double)(std::clock() - t) / CLOCKS_PER_SEC > 7) {
            cout << "CSIPP: TIME LIMIT EXCEEDED" << endl;
            releaseClosedListNodes();
            open_list.clear();
            focal_list.clear();
            return CPath();
        }

        CSIPPNode* curr = open_list.top(); open_list.pop();

        cout << "cur state: " << curr->state << endl;

        int loc = curr->state.location;

        // open_list.erase(curr->open_handle); // remove from open
        // cout << "here" << endl;
        curr->in_openlist = false; // removed
        num_expanded++;

        res = update_goals(curr, goal_locations);

        cout << "updated goals" << endl;

        if (std::get<0>(res)) {
            releaseClosedListNodes();
            open_list.clear();
            // focal_list.clear();
            runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
            // if (debug) {
                cout << "           CSIPP: returning path," << endl;
                std::cout << "          Time elapsed: " << (std::clock() - t) * 1.0 / CLOCKS_PER_SEC << " seconds" << std::endl;
            // }

            return std::get<1>(res);
        }

        list<Interval> safe_ints = rt.getSafeIntervals(
            curr->state.location, 
            curr->state.timestep, 
            std::get<1>(curr->interval));

        cout << "start intervals: " << endl;
        for (auto &interval : safe_ints) {
            cout << interval << endl;
        }
        vector<Interval> time_intervals, tmp;
        
        time_intervals.assign(safe_ints.begin(), safe_ints.end());

        cout << "starting expansion" << endl;

        // for each possible turn + move 
        for (int orientation = 0; orientation < 4; orientation++) {

            cout << "trying orientation " << orientation << endl;
            cout << "move: " << G.move[orientation] << endl;

            // if you can't turn, stop
            if (!G.valid_move(loc, orientation)) {
                cout << "   cannot turn" << endl;
                continue;
            }

            // check this time stuff
            vector<Interval> tmp;
            int next_loc = loc + G.move[orientation];
            // int next_loc = loc;
            int turn_time = G.get_rotate_degree(curr->state.orientation, orientation);
            int dist = 0;
            // look through intervals and keep track of valid ones throughout all neighbors

            int cur_len = 0;
            // while (G.valid_move(next_loc, orientation)) {
            //     max_len += 1;
            //     next_loc += G.move[orientation];
            // }
            while (G.valid_move(next_loc, orientation)) {
            // for (int len = 1; len <= max_len, len++) {
                cout << "next loc: " << next_loc << endl;
                // cur_len += 1;

                // auto profile = motion_model.getProfile(cur_len);

                for (auto &interval : time_intervals) {
                    int start = std::get<0>(interval);
                    int end = std::get<1>(interval);

                    list<Interval> conflicts = rt.getConflictIntervals(next_loc, start, INTERVAL_MAX);
                    cout << "found num conflicts: " << conflicts.size() << endl;

                    if (conflicts.empty()) {
                        tmp.push_back(interval);
                    }
   
                    double t_d1 = 1; // sweeping time, constant right now
                    double t_d2 = dist + turn_time; // 

                    // for every reserved period, project back to [start, end)
                    for (auto &reserved : conflicts) {
                        cout << "   conflict: " << std::get<0>(reserved) << ", " << std::get<1>(reserved) << endl;
                        int tl = std::get<0>(reserved);
                        int tr = std::get<1>(reserved);

                        double c_start = tl - t_d1 - t_d2;
                        double c_end = tr - t_d2;

                        // remove unavailable period from timeIntervals
                        if (c_end < c_start) continue;

                        if (c_start > end || c_end < start) continue;

                        // conflict encompasses SI
                        if (c_start <= start && c_end >= end) continue;

                        // conflict starts before SI, ends during SI
                        if (c_start <= start && c_end < end) {
                            tmp.push_back({c_end, end, 0});
                        } else if (c_start > start && c_end >= end) { // conflict starts after SI, ends after SI
                            tmp.push_back({start, c_start, 0});
                        } else { // conflict starts/ends during SI
                            tmp.push_back({start, c_start, 0});
                            tmp.push_back({c_end, end, 0});
                        }
                    }
                }

                // swap timeIntervals with tmp
                time_intervals.swap(tmp);
                tmp.clear();

                // make a new node for each valid interval
                double h_val = compute_h_value(G, next_loc, curr->goal_id, goal_locations);

                for (auto &it: time_intervals) {
                    cout << "   interval: " << it << endl;
                    int sweeping_time = 1;
                    CState next_state = CState(next_loc, curr->state.timestep + sweeping_time, orientation, 0);
                    int conflicts = std::get<2>(it) + curr->conflicts;

                    CSIPPNode* new_node = new CSIPPNode(
                        next_state, 
                        next_state.timestep, // gval
                        h_val,  //hval
                        {std::get<0>(it) + sweeping_time, std::get<1>(it) + sweeping_time, 0},
                        curr, 
                        conflicts, 
                        goal_locations[curr->goal_id]
                    );

                    add_node(new_node);
                }

                // next_loc = get_next_location(next_loc, orientation); // Move forward
                next_loc = next_loc + G.move[orientation];
                dist += 1;
            }
        }
    }

    return CPath();
}

Path CSIPP::run(const BasicGraph& G, const State& start,
                const vector<pair<int, int> >& goal_locations,
                ReservationTable& RT) 
{
    return Path();
};

void CSIPP::add_node(CSIPPNode* next)
{
    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);

    // if this node does not exist in allNodes
    if (it == allNodes_table.end())
    {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    CSIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist)
    {  // if its in the open list
        //  the existing node has a higher cost than the new node
        // ties broken by number of conflicts
        // relax the edge
        if (existing_f_val > next->g_val + next->h_val ||
            (existing_f_val == next->g_val + next->h_val && existing_next->conflicts > next->conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
            bool update_open = false;
            if ((next->g_val + next->h_val) <= focal_bound)
            {  // if the new f-val qualify to be in FOCAL
                if (existing_f_val > focal_bound)
                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                else
                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
            }
            if (existing_f_val > next->g_val + next->h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = next->g_val;
            existing_next->h_val = next->h_val;
            existing_next->parent = next->parent;
            // existing_next->depth = next->depth;
            existing_next->conflicts = next->conflicts;

            if (update_open)
                open_list.increase(existing_next->open_handle);  // increase because f-val improved
            if (add_to_focal)
                existing_next->focal_handle = focal_list.push(existing_next);
            if (update_in_focal)
                focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
        }
    }
    else
    {  // if its in the closed list (reopen)
        if (existing_f_val > next->g_val + next->h_val ||
            (existing_f_val == next->g_val + next->h_val && existing_next->conflicts > next->conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            existing_next->state = next->state;
            existing_next->g_val = next->g_val;
            existing_next->h_val = next->h_val;
            existing_next->parent = next->parent;
            // existing_next->depth = next->depth;
            existing_next->conflicts = next->conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete(next);  // not needed anymore -- we already generated it before
}

inline void CSIPP::releaseClosedListNodes()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}