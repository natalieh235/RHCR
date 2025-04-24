#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"
#include "CStates.h"
// #include "MotionModel.h"
#include "CReservationTable.h"

class CSIPPNode {
public:
	CSIPPNode* parent;
    CInterval interval; // continuous interval

    CState state;

    Path node_path;
    // std::string primitive_name;

    std::pair<int, int> goal;

    double g_val;
    double h_val;
    double wait_time;
    int conflicts;
    int goal_id;

    bool in_openlist;
    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const CSIPPNode* n1, const CSIPPNode* n2) const
        {
            // if fvals the same
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                if (n1->g_val == n2->g_val) {
                    return std::get<0>(n1->interval) >= std::get<0>(n2->interval);
                }
                return n1->g_val <= n2->g_val;
            }
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }

    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

    // the following is used to comapre nodes in the FOCAL list
    struct secondary_compare_node
    {
        bool operator()(const CSIPPNode* n1, const CSIPPNode* n2) const // returns true if n1 > n2
        {
            if (n1->conflicts == n2->conflicts)
            {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val; // break ties towards smaller f_vals
            }
            return n1->conflicts >= n2->conflicts;  // n1 > n2 if it has more conflicts
        }
    };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


    // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
    fibonacci_heap< CSIPPNode*, compare<CSIPPNode::compare_node> >::
    handle_type open_handle;
    fibonacci_heap< CSIPPNode*, compare<CSIPPNode::secondary_compare_node> >::
    handle_type focal_handle;

    inline double getFVal() const { return g_val + h_val; }
    CSIPPNode(): parent(nullptr), in_openlist(false), g_val(0), h_val(0), goal_id(0), conflicts(0) {}

    CSIPPNode(const CState& state, double g_val, double h_val, const Interval& interval,
            CSIPPNode* parent, int conflicts, std::pair<int, int> goal, double wait_time):
            state(state), parent(parent), interval(interval), goal(goal), in_openlist(false), g_val(g_val), h_val(h_val), conflicts(conflicts), wait_time(wait_time) {}
    // The following is used to  check whether two nodes are equal
    // we say that two nodes are equal iff
    // both agree on the id and timestep
    struct EqNode
    {
        bool operator() (const CSIPPNode* n1, const CSIPPNode* n2) const
        {
            return (n1 == n2) ||
                  (n1 && n2 && n1->state.location == n2->state.location &&
                  n1->state.orientation == n2->state.orientation &&
                  n1->state.velocity == n2->state.velocity &&
                  n1->interval == n2->interval &&
                  n1->goal_id == n2->goal_id
                  );
        }
    };

    struct Hasher
    {
        std::size_t operator()(const CSIPPNode* n) const
        {
            return CState::Hasher()(n->state);
        }
    };
};
class CSIPP: public SingleAgentSolver
{
    public:
        CPath run_continuous(const BasicGraph& G, const CState& start,
                const vector<pair<int, int> >& goal_locations,
                ContinuousReservationTable& RT);

        Path run(const BasicGraph& G, const State& start,
                const vector<pair<int, int> >& goal_locations,
                ReservationTable& RT);

        string getName() const { return "CSIPP"; }
        CSIPP(): SingleAgentSolver() {}
        CSIPP(MotionModel motion_model): SingleAgentSolver() {
            this->motion_model = motion_model;
        }

    private:
        MotionModel motion_model;
        fibonacci_heap< CSIPPNode*, compare<CSIPPNode::compare_node> > open_list;
        fibonacci_heap< CSIPPNode*, compare<CSIPPNode::secondary_compare_node> > focal_list;
        unordered_set< CSIPPNode*, CSIPPNode::Hasher, CSIPPNode::EqNode> allNodes_table;
        inline void releaseClosedListNodes();
        void generate_node(const CInterval& interval, CSIPPNode* curr, State next_state, const BasicGraph& G,
                        int min_timestep, double h_val, std::pair<int, int> goal, std::string primitive_name);
        // Updates the path
        std::tuple<bool, CPath> update_goals(CSIPPNode* curr, const vector<pair<int, int> >& goal_locations); 
        void generate_successors(CSIPPNode* curr, const BasicGraph &G, 
            ReservationTable &rt, int t_lower, int t_upper, 
            const vector<pair<int, int> >& goal_location);
        void add_node(CSIPPNode* next);
        CPath updatePath(const CSIPPNode* goal);
};