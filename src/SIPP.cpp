#include "SIPP.h"
#include <cstdlib>

Path SIPP::updatePath(const BasicGraph& G, const SIPPNode* goal)
{
    Path path(goal->state.timestep + 1);
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    const SIPPNode* curr = goal;
    while (true)
    {
        if (curr->parent == nullptr) // root node
        {
            int t = curr->state.timestep;
            path[t] = curr->state;
            t--;
            for (; t >= 0; t--)
            {
                path[t] = State(-1, -1); // dummy start states
            }
            break;
        }
        else
        {
            const SIPPNode* prev = curr->parent;
            int degree = G.get_rotate_degree(prev->state.orientation, curr->state.orientation);
            int t = prev->state.timestep + 1;

            // std::cout << "curr " << curr->state << " interval: " << curr->interval << std::endl;
            // std::cout << "prev: " << curr->parent->state << " interval: " << curr->parent->interval << std::endl;
            // if (degree == 1) // turn right or turn left
            // {
            //     path[t] = State(prev->state.location, t, curr->state.orientation);
            //     t++;
            // }
            // else if (degree == 2) // turn back
            // {
            //     path[t] = State(prev->state.location, t, (prev->state.orientation + 1) % 4); // turn right
            //     t++;
            //     path[t] = State(prev->state.location, t, curr->state.orientation); // turn right
            //     t++;
            // }
            while ( t < curr->state.timestep)
            {
                // std::cout << "waiting? " << curr->state << std::endl;
                // path[t] = State(prev->state.location, t, curr->state.orientation, prev->state.velocity); // wait at prev location
                path[t] = State(prev->state);
                t++;
            }
            path[curr->state.timestep] = State(curr->state); // move to current location
            curr = prev;
        }
    }
    return path;
}

// for grid agents
void SIPP::fill_primitives() {
    std::cout << "SIPP: filling primitives" << std::endl;
    float MAXV  = 2.0;
    float MAX_ACC = 1.0;
    int minimalTransitionCost = 1; // the minimum cost to go from one cell to next (0.5s = 5 timesteps).
    int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
    // int turn_dx[4] = {}
    
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        tmp.mvs = {Primitive::move(0, 0, 0, 1, (o + 1) % MXO, 1)}; // 1 second to rotate
        tmp.o = (o + 1) % MXO;
        tmp.v = 0;
        motion_primitives[o][0].emplace_back(tmp);
        tmp.mvs.clear();
        tmp.mvs = {Primitive::move(0, 0, 0, 1, (o + 3) % MXO, 1)};
        tmp.o = (o + 3) % MXO;
        motion_primitives[o][0].emplace_back(tmp);
    }
   

    bool bidirectional = true;
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        std::cout << "primitives for " << o << std::endl;
        // ACCELERATION
        // Primitive tmp;
        tmp.v = 1; // ending velocity 1
        tmp.o=o; // end orientation is the same

        tmp.mvs = {
            Primitive::move(dx[o]*0, dy[o]*0, 0, 1, o, 0), // time = 0, occupies first cell for 1 second
            Primitive::move(dx[o]*1, dy[o]*1, 1, 1, o, 1), // time = 1, displacement is 1 cell
            // Primitive::move(dx[o]*4, dy[o]*4, 1, 1, o, 0)
        };

        motion_primitives[o][0].push_back(tmp);

        int next_orientation = (o + 2) % MXO;

        if (bidirectional) {
            tmp.mvs.clear();
            tmp.mvs = {
                Primitive::move(dx[next_orientation]*0, dy[next_orientation]*0, 0, 1, o, 0), // time = 0, occupies first cell for 1 second
                Primitive::move(dx[next_orientation]*1, dy[next_orientation]*1, 1, 1, o, 1), // time = 1, displacement is 1 cell
            };
            tmp.o = next_orientation;
            motion_primitives[o][0].push_back(tmp);
        }
        

        std::cout << "acclereation: " << tmp << std::endl;

        // deceleration
        tmp.mvs.clear();
        tmp.v = 0;
        tmp.o = o;
        tmp.mvs = {
            Primitive::move(dx[o]*0, dy[o]*0, 0, 1, o, 0), 
            Primitive::move(dx[o]*1, dy[o]*1, 1, 1, o, 1)
            // Primitive::move(dx[o]*4, dy[o]*4, 1, 1, o, 0)
        };

        motion_primitives[o][1].push_back(tmp);

        if (bidirectional) {
            tmp.mvs.clear();
            tmp.mvs = {
                Primitive::move(dx[next_orientation]*0, dy[next_orientation]*0, 0, 1, o, 0), 
                Primitive::move(dx[next_orientation]*1, dy[next_orientation]*1, 1, 1, o, 1)
                // Primitive::move(dx[o]*4, dy[o]*4, 1, 1, o, 0)
            };
            tmp.o = next_orientation;
            motion_primitives[o][1].push_back(tmp);
        }


        std::cout << "decel: " << tmp << std::endl;

        // just go forward
        // goes forward with constant velocity in 1 timestep, can only do with vel=1
        // because robot has width 2.0, covers cells 0-2
        tmp.mvs = {Primitive::move(0, 0, 0, 0, o, 0), 
                    Primitive::move(dx[o], dy[o], 0, 1, o, 1)};
        tmp.o = o;
        tmp.v = 1;
        motion_primitives[o][1].emplace_back(tmp);

        // if (bidirectional) {
        //     tmp.mvs.clear();
        //     tmp.mvs = {Primitive::move(0, 0, 0, 0, o, 0), 
        //                 Primitive::move(dx[next_orientation], dy[next_orientation], 0, 1, o, 1)};
        //     motion_primitives[o][1].emplace_back(tmp);
        // }


        std::cout << "constant: " << tmp << std::endl;
    }
}

// // for large agents
// void SIPP::fill_primitives() {
//     std::cout << "SIPP: filling primitives" << std::endl;
//     float MAXV  = 2.0;
//     float MAX_ACC = 1.0;
//     int minimalTransitionCost = 1; // the minimum cost to go from one cell to next (0.5s = 5 timesteps).
//     int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
//     // int turn_dx[4] = {}
    
//     //  turn from 0 to 1
//     Primitive tmp;
//     tmp.mvs = {Primitive::move(0, 1, 1, 1, 0, 0), Primitive::move(-1, 1, 1, 1, 1, 0), 
//             Primitive::move(-1, 0, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 1;
//     motion_primitives[0][0].emplace_back(tmp);

//     tmp.mvs.clear();

//     // turn from 0 to 3
//     tmp.mvs = {Primitive::move(0, 1, 1, 1, 0, 0), Primitive::move(1, 1, 1, 1, 1, 0), 
//             Primitive::move(1, 0, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 3;
//     motion_primitives[0][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 1 to 2
//     tmp.mvs = {Primitive::move(-1, 0, 1, 1, 0, 0), Primitive::move(-1, -1, 1, 1, 1, 0), 
//             Primitive::move(0, -1, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 2;
//     motion_primitives[1][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 1 to 0
//     tmp.mvs = {Primitive::move(-1, 0, 1, 1, 0, 0), Primitive::move(-1, 1, 1, 1, 1, 0), 
//             Primitive::move(0, 1, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 0;
//     motion_primitives[1][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 2 to 3
//     tmp.mvs = {Primitive::move(0, -1, 1, 1, 0, 0), Primitive::move(1, -1, 1, 1, 1, 0), 
//             Primitive::move(1, 0, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 3;
//     motion_primitives[2][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 2 to 1
//     tmp.mvs = {Primitive::move(0, -1, 1, 1, 0, 0), Primitive::move(-1, -1, 1, 1, 1, 0), 
//             Primitive::move(-1, 0, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 1;
//     motion_primitives[2][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 3 to 0
//     tmp.mvs = {Primitive::move(1, 0, 1, 1, 0, 0), Primitive::move(1, 1, 1, 1, 1, 0), 
//             Primitive::move(0, 1, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 0;
//     motion_primitives[3][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     // 3 to 2
//     tmp.mvs = {Primitive::move(1, 0, 1, 1, 0, 0), Primitive::move(1, -1, 1, 1, 1, 0), 
//             Primitive::move(0, -1, 1, 1, 1, 0), Primitive::move(0, 0, 1, 1, 1, 1)};
//     tmp.v = 0;
//     tmp.o = 2;
//     motion_primitives[3][0].emplace_back(tmp);
//     tmp.mvs.clear();

//     for (int o = 0; o < MXO; ++o) {
//         std::cout << "primitives for " << o << std::endl;
//         // ACCELERATION
//         // Primitive tmp;
//         tmp.v = 1; // ending velocity 1
//         tmp.o=o; // end orientation is the same

//         // (0, 0)  (0, 1)
//         // (1, 0)
//         tmp.mvs = {
//             Primitive::move(dx[o]*0, dy[o]*0, 0, 1, o, 0), // time = 0, occupies first cell for 1 second
//             Primitive::move(dx[o]*1, dy[o]*1, 0, 1, o, 0), // time = 1, displacement is 1 cell
//             Primitive::move(dx[o]*2, dy[o]*2, 1, 1, o, 0),
//             Primitive::move(dx[o]*3, dy[o]*3, 1, 1, o, 1),
//             // Primitive::move(dx[o]*4, dy[o]*4, 1, 1, o, 0)
//         };

//         motion_primitives[o][0].push_back(tmp);

//         std::cout << "acclereation: " << tmp << std::endl;

//         // deceleration
//         tmp.mvs.clear();
//         tmp.v = 0;
//         tmp.o = o;
//         tmp.mvs = {
//             Primitive::move(dx[o]*0, dy[o]*0, 0, 1, o, 0), 
//             Primitive::move(dx[o]*1, dy[o]*1, 0, 1, o, 0), 
//             Primitive::move(dx[o]*2, dy[o]*2, 1, 1, o, 0),
//             Primitive::move(dx[o]*3, dy[o]*3, 1, 1, o, 1),
//             // Primitive::move(dx[o]*4, dy[o]*4, 1, 1, o, 0)
//         };

//         motion_primitives[o][1].push_back(tmp);

//         std::cout << "decel: " << tmp << std::endl;

//         // just go forward
//         // goes forward with constant velocity in 1 timestep, can only do with vel=1
//         // because robot has width 2.0, covers cells 0-2
//         tmp.mvs = {Primitive::move(0, 0, 0, 0, o, 0), 
//                     Primitive::move(dx[o], dy[o], 0, 1, o, 0), 
//                     Primitive::move(dx[o]*2, dy[o]*2, 0, 1, o, 0)};
//         tmp.o = o;
//         tmp.v = 1;
//         motion_primitives[o][1].emplace_back(tmp);

//         std::cout << "constant: " << tmp << std::endl;
//     }
// }

void SIPP::generate_successors(SIPPNode* curr, const BasicGraph &G, ReservationTable &rt, 
    int t_lower, int t_upper, const vector<pair<int, int> >& goal_location) {
    
    std::cout << "generating successors for " << curr->state << std::endl;
    // for possible primitives with that starting velocity
    for (auto &mp: motion_primitives[curr->state.orientation][curr->state.velocity]) {
        // std::cout << "applying primitive " << mp << std::endl;
        auto cur_xy = G.get_xy(curr->state.location);
        int next_location = G.get_location(mp.mvs.back().dx + cur_xy.first, mp.mvs.back().dy + cur_xy.second);
        double h_val = compute_h_value(G, next_location, curr->goal_id, goal_location);

        apply_primitive(curr, G, rt, t_lower, t_upper, mp, h_val);
    }
    std::cout << "========= end successors ========" << std::endl;
}

// Generate acceleration primitive
// Primitive generate_acceleration_primitive(double v0, double vf, double max_acc, int cur_o) {
//     vector<Primitive::move> moves;
//     int dx = 0, dy = 1; 

//     // Iterate over velocity increments
//     for (int v = v0 + 1; v <= vf; ++v) {
//         double t = (v - v0) / max_acc; // Time to reach vf
//         double s = v0 * t + 0.5 * max_acc * t * t;          // Distance traveled
//         int cells = static_cast<int>(s / grid_size);        // Convert to grid cells

//         // Add move to primitive
//         moves.emplace_back(dx * cells, dy * cells, static_cast<int>(t), static_cast<int>(t), cur_o, vf == max_v);
//     }

//     // Return the Primitive
//     return Primitive(moves, cur_o, max_v);
// }

// generate wait intervals (belongs to safe interval)
// node to extand: n = (v, [tl, tu])
// outputs set of time intervals st. each ti belongs to one of the safe intervals of the target
// intervals do not overlap
void SIPP::apply_primitive(SIPPNode* curr, const BasicGraph &G, ReservationTable &rt, 
    int t_lower, int t_upper, Primitive mp, double h_val) {

    std::cout << "SIPP: applying primitive: " << mp << std::endl;
    vector<pair<int, int>> time_intervals, tmp;
    time_intervals.push_back({t_lower, t_upper});
    bool endCellTouched = false;
    pair<int, int> xy = G.get_xy(curr->state.location);
    int x = xy.first, y = xy.second;
    int xx = xy.first, yy = xy.second, past_time = 0;
    int next_location;
    // std::cout << "initial xx " << xx << " initial yy" << yy << std::endl;

    // for each primitive
    for (auto mv:mp.mvs) {
        xx = x + mv.dx;
        yy = y + mv.dy;

        if (xx < 0 || xx >= G.get_rows() || yy < 0 || y >= G.get_cols()) {
            // std::cout << "  apply primitive generates is invalid" << std::endl;
            return;
        }

        next_location = G.get_location(xx, yy);
        // std::cout << "  apply primitive next loc: " << next_location << std::endl;
        if (!G.valid_move(next_location, mv.cur_o)) {
            // std::cout << "  apply primitive next loc invalid" << std::endl;
            return;
        }

        int completion_time = mv.ftt - past_time;

        for (auto &it: time_intervals) {
            // std::cout << "curr interval: " << it.first << ", " << it.second << std::endl;
            
            int t_l = min(INTERVAL_MAX, it.first + completion_time); // time needed to complete this one move in the primitive
            int t_u = min(INTERVAL_MAX, it.second + completion_time);

            // std::cout << "starting: " << t_l << ", " << t_u << ", next location " << next_location << std::endl;

            auto vertex_intervals = rt.getSafeIntervals(next_location, t_l, t_u);

            for (auto &safe_it: vertex_intervals) {
                // std::cout << "safe interval: " << std::get<0>(safe_it) << ", " << std::get<1>(safe_it) << std::endl;
                // how do i generate new projected intervals?
                int new_tl = max(t_l, std::get<0>(safe_it));
                int new_tu = min(t_u, std::get<1>(safe_it) - mv.swt);

                // std::cout << "new bounds " << new_tl << ", " << new_tu << std::endl;
 
                // if waiting is allowed at the target vertex, extend the safe interval
                // to the upper bound of the SI at the target vertex
                if (new_tl <= new_tu && (mv.isEndCell && !endCellTouched) && mp.v == 0) {
                    // std::cout << "adjusting, " << std::get<0>(safe_it) - mv.swt << std::endl;
                    new_tu = std::get<1>(safe_it) - mv.swt;
                }

                if (new_tl <= new_tu) {
                    // std::cout << "found interval " << new_tl << ", " << new_tu << std::endl;
                    tmp.push_back({new_tl, new_tu});
                }
            }
        }

        time_intervals = tmp;
        tmp.clear();
        past_time = mv.ftt;
        if(mv.isEndCell){
            endCellTouched = true;
        }

        // std::cout << "done with move " << std::endl;
    }

    vector<pair<int, int>> intervals;
    
    for (auto it: time_intervals) {
        int adjusted_tl = std::get<0>(it) + mp.mvs.back().swt;
        int adjusted_tu = std::get<1>(it)+mp.mvs.back().swt;
        State next_state = State(next_location, adjusted_tl, mp.o, mp.v);
        generate_node({adjusted_tl, adjusted_tu, 0}, curr, next_state, G, 
            t_lower+mp.mvs.back().swt + mp.mvs.back().ftt, h_val);
        std::cout << "          generating new node with " << next_state << std::endl;
        // std::cout << "generating node with tl " << std::get<0>(it) << " + " << mp.mvs.back().swt
        //     << ", tu " << std::get<1>(it) << " + " << mp.mvs.back().swt << ", min timestep " 
        //     << t_lower+mp.mvs.back().swt + mp.mvs.back().ftt << " state: " << next_state << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
// after max_timestep, switch from time-space A* search to normal A* search
Path SIPP::run(const BasicGraph& G, const State& start,
               const vector<pair<int, int> >& goal_location,
               ReservationTable& rt)
{
    // std::cout << "SIPP: running SIPP" << std::endl;
    // ROBOT_HEIGHT = 1.5
    // ROBOT_WIDTH = 3
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();
    

    // heuristic from start to end
	double h_val = compute_h_value(G, start.location, 0, goal_location);

	if (h_val > INT_MAX)
	{
		cout << "The start and goal locations are disconnected!" << endl;
		return Path();
	}

    // Start at the first safe interval
    // beginning state is the start.location @ first safe interval
    Interval interval = rt.getFirstSafeInterval(start.location);

    // if first safe interval starts at 0
    if (std::get<0>(interval) == 0)
    {
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;

        // reference to node in open_list (a fibonacci heap)
        node->open_handle = open_list.push(node);
        node->in_openlist = true;

        // unordered set of all nodes
        allNodes_table.insert(node);

        // track min f_val
        min_f_val = node->getFVal();
        focal_bound = min_f_val * suboptimal_bound;
        node->focal_handle = focal_list.push(node);
    }
    else if(prioritize_start) // the agent has the highest priority at its start location
    {
        // This is correct only when k_robust <= 1. Otherwise, agents might not be able to
        // wait at its start locations due to initial constraints caused by the previous actions
        // of other agents.
        Interval interval = make_tuple(0, INTERVAL_MAX, 0);
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        allNodes_table.insert(node);
        min_f_val = node->getFVal();
        focal_bound = min_f_val;
        node->focal_handle = focal_list.push(node);
    }
	int earliest_holding_time = 0;
	if (hold_endpoints)
		earliest_holding_time = rt.getHoldingTimeFromSIT(goal_location.back().first);

    // std::cout << "starting focal list, " << focal_list.size() << std::endl;
    // take from focal list
    while (!focal_list.empty())
    {
        SIPPNode* curr = focal_list.top(); focal_list.pop();
        // std::cout << "focal size, " << focal_list.size() << std::endl;
        open_list.erase(curr->open_handle); // remove from open
        curr->in_openlist = false; // removed
        num_expanded++;

        // update goal id
        if (curr->state.location == goal_location[curr->goal_id].first &&
            curr->state.timestep >= goal_location[curr->goal_id].second) // reach the goal location after its release time
        {
            cout << "SIPP: reached goal location " << curr->goal_id << ", " << curr->state << endl;
            curr->goal_id++;
            if (curr->goal_id == (int)goal_location.size() &&
                earliest_holding_time > curr->state.timestep)
                curr->goal_id--;
        }
        // check if agent has reached all goal locations
        if (curr->goal_id == (int)goal_location.size())
        {
            Path path = updatePath(G, curr);
            releaseClosedListNodes();
            open_list.clear();
            focal_list.clear();
            runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
            cout << "SIPP: returning path" << curr->goal_id << "path: " << path << endl;
            return path;
        }

        bool use_primitives = true;
        if (use_primitives) {
            generate_successors(curr, G, rt, 
            std::get<0>(curr->interval), std::get<1>(curr->interval),
            goal_location);
        } else {
            std::cout << "=========" << std::endl;
            std::cout << "SIPP: curr state: " << curr->state << std::endl;
            std::cout << "SIPP: current interval: " << curr->interval << std::endl;
            
            // expand the nodes
            for (int orientation = 0; orientation < 4; orientation++) // move
            {
                if (!G.valid_move(curr->state.location, orientation)) // the edge is blocked
                    continue;
                int degree;
                if (curr->state.orientation < 0)
                    degree = 0;
                else
                    degree = G.get_rotate_degree(curr->state.orientation, orientation);

                if (degree > std::get<1>(curr->interval) - curr->state.timestep) // don't have enough time to turn
                    continue;

                int location = curr->state.location + G.move[orientation];

                if (!G.valid_move(location, orientation)) {
                    continue;
                }

                double h_val = compute_h_value(G, location, curr->goal_id, goal_location);
                if (h_val > INT_MAX)   // This vertex cannot reach the goal vertex
                    continue;
                int min_timestep = curr->state.timestep + degree + 1;
                for (auto interval : rt.getSafeIntervals(curr->state.location, location, min_timestep, std::get<1>(curr->interval) + 1))
                {
                    if (curr->state.orientation < 0) {
                        State next_state = State(location, min_timestep, -1, 0);
                        generate_node(interval, curr, next_state, G, min_timestep, h_val);
                    }
                    else {
                        State next_state = State(location, min_timestep, orientation, 0);
                        std::cout << "successor: " << next_state << std::endl;
                        generate_node(interval, curr, next_state, G, min_timestep, h_val);
                    }
                }

            }  // end for loop that generates successors
        }

        

        // update FOCAL if min f-val increased
        if (open_list.empty())  // in case OPEN is empty, no path found
        {
            // std::cout << "SIPP: open list empty" << std::endl;
            if(prioritize_start) // the agent has the highest priority at its start location
            {
                // std::cout << "prioritize start" << std::endl;
                // This is correct only when k_robust <= 1. Otherwise, agents might not be able to
                // wait at its start locations due to initial constraints caused by the previous actions
                // of other agents.
                Interval interval = rt.getFirstSafeInterval(start.location);
                Interval interval2 = make_tuple(std::get<1>(interval), INTERVAL_MAX, 0);
                if (std::get<1>(interval) == INTERVAL_MAX) {
                    break;
                }
                double h_val = compute_h_value(G, start.location, 0, goal_location);
                auto node2 = new SIPPNode(start, 0, h_val, interval2, nullptr, 0);
                num_generated++;
                node2->open_handle = open_list.push(node2);
                node2->in_openlist = true;
                allNodes_table.insert(node2);
                min_f_val = node2->getFVal();
                focal_bound = min_f_val;
                node2->focal_handle = focal_list.push(node2);
            }
            else
            {
                break;
            }
        }
        else // add to focal list
        {
            SIPPNode* open_head = open_list.top();
            if (open_head->getFVal() > min_f_val)
            {
                double new_min_f_val = open_head->getFVal();
                double new_focal_bound = new_min_f_val * suboptimal_bound;
                for (SIPPNode* n : open_list)
                {
                    if (n->getFVal() > focal_bound && n->getFVal() <= new_focal_bound) {
                        // std::cout << "adding node from open to focal, " << n->state << std::endl;
                        n->focal_handle = focal_list.push(n);
                    }
                }
                min_f_val = new_min_f_val;
                focal_bound = new_focal_bound;
            }
        }
        // std::cout << "focal size at end, " << focal_list.size() << std::endl;
    }  // end while loop

    // no path found
    releaseClosedListNodes();
    open_list.clear();
    focal_list.clear();
    return Path();
}

void SIPP::generate_node(const Interval& interval, SIPPNode* curr, State next_state, const BasicGraph& G,
        int min_timestep, double h_val)
{
    // max(interval_start, action_end)
    int timestep  = max(std::get<0>(interval), min_timestep);
    int wait_time = timestep - curr->state.timestep - 1; // inlcude move time
    double g_val = curr->g_val + wait_time * G.get_weight(curr->state.location, curr->state.location)
                   + G.get_weight(curr->state.location, next_state.location);

    // interval is start, end, has_conflict
    int conflicts = std::get<2>(interval) + curr->conflicts;

    // generate (maybe temporary) node
    auto next = new SIPPNode(next_state,
                             g_val, h_val, interval, curr, conflicts);

    
    // std::cout << "generate node: new state is " << next->state << std::endl;

    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);

    // if this node does not exist in allNodes
    if (it == allNodes_table.end())
    {
        // std::cout << "node doesn't exist" << next->state << std::endl;
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist)
    {  // if its in the open list
        //  the existing node has a higher cost than the new node
        // ties broken by number of conflicts
        // relax the edge
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
            bool update_open = false;
            if ((g_val + h_val) <= focal_bound)
            {  // if the new f-val qualify to be in FOCAL
                if (existing_f_val > focal_bound)
                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                else
                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
            }
            if (existing_f_val > g_val + h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            // existing_next->move = next->move;

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
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts > conflicts))
        {
            // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete(next);  // not needed anymore -- we already generated it before
}

inline void SIPP::releaseClosedListNodes()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}




