#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"

class CSIPP: public SingleAgentSolver
{
    public:
        Path run(const BasicGraph& G, const State& start,
                const vector<pair<int, int> >& goal_location,
                ReservationTable& RT);

        string getName() const { return "SIPP"; }
        CSIPP(): SingleAgentSolver() {
            fill_primitives();
        }

    private:
        fibonacci_heap< SIPPNode*, compare<SIPPNode::compare_node> > open_list;
        fibonacci_heap< SIPPNode*, compare<SIPPNode::secondary_compare_node> > focal_list;
        unordered_set< SIPPNode*, SIPPNode::Hasher, SIPPNode::EqNode> allNodes_table;
        inline void releaseClosedListNodes();


        void generate_node(const Interval& interval, SIPPNode* curr, State next_state, const BasicGraph& G,
                        int min_timestep, double h_val, std::pair<int, int> goal, std::string primitive_name);
        // Updates the path
        Path updatePath(const BasicGraph& G, const SIPPNode* goal);

        void generate_successors(SIPPNode* curr, const BasicGraph &G, 
            ReservationTable &rt, int t_lower, int t_upper, 
            const vector<pair<int, int> >& goal_location);

};